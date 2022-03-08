use bytemuck::cast_slice;
use usb_device::{
    class_prelude::*,
    endpoint::{Endpoint, Out, In}
};

use crate::hal::pac;

use crate::uac::{descriptors::{AudioControlAllocator, descriptor_type::CS_INTERFACE, feature_unit_control_selector::{FU_MUTE_CONTROL, FU_VOLUME_CONTROL}, clock_source_control_selectors}, request::{RequestCode, Target}};

use super::{descriptors::{
    self,
    audio_function_class,
    audio_function_subclass,
    audio_function_protocol,
    audio_interface_class::{self, AUDIO},
    audio_interface_subclass::{self, AUDIOSTREAMING},
    audio_interface_protocol::{self, IP_VERSION_02_00},
    AudioControlInterfaceDescriptorWriter,
    ClockAttributes,
    ClockControls,
    terminal_type,
    ChannelConfig,
    InputTerminalControls,
    FeatureUnitControls,
    as_interface_descriptor_subtype::{AS_GENERAL, FORMAT_TYPE},
    format_type_codes::FORMAT_TYPE_I,
    audio_format_type_1_bit_allocations::PCM,
    descriptor_type::CS_ENDPOINT,
    endpoint_descriptor_subtypes::EP_GENERAL,
    audiostreaming_interface_control_selectors::{
        AS_VAL_ALT_SETTINGS_CONTROL,
        AS_ACT_ALT_SETTING_CONTROL
    },
    EntityId
}, request::ControlRequest, ClockCounter, StreamingState};

pub struct SimpleStereoOutput<'a, B: UsbBus> {
    if_audio_control: InterfaceNumber,
    if_audio_stream: InterfaceNumber,
    ep_audio_data: EndpointOut<'a, B>,
    ep_feedback: EndpointIn<'a, B>,

    #[allow(dead_code)]
    sample_rate: u32,
    #[allow(dead_code)]
    audio_subframe_size: usize,
    #[allow(dead_code)]
    audio_bit_resolution: usize,

    audio_data_buffer_size: usize,

    clock_source: EntityId,
    input_terminal: EntityId,
    feature_unit: EntityId,
    output_terminal: EntityId,

    pub mute: bool,
    pub volume: i16,

    alt_setting: u8,
    pub audio_data_available: bool,
    pub audio_feedback_needed: bool,
    pub enable_disable: Option<StreamingState>,
}

impl<'a, B> SimpleStereoOutput<'a, B>
where
    B: UsbBus
{
    const CHANNELS: usize = 2;

    pub fn new(
        alloc: &'a UsbBusAllocator<B>,
        sample_rate: u32,
        audio_subframe_size: usize,
        audio_bit_resolution: usize,
    ) -> SimpleStereoOutput<'a, B> {
        let if_audio_control = alloc.interface();
        let if_audio_stream = alloc.interface();

        let audio_data_buffer_size: usize = (sample_rate as usize / 1000 + 1) * audio_subframe_size * Self::CHANNELS;
        assert!(audio_data_buffer_size < 1023);
        let ep_audio_data: Endpoint<'a, B, Out> = alloc.isochronous(
            usb_device::endpoint::IsochronousSynchronizationType::Asynchronous,
            usb_device::endpoint::IsochronousUsageType::Data,
            audio_data_buffer_size as u16,
            0x01,
        );
        let ep_feedback: Endpoint<'a, B, In> = alloc.isochronous(
            usb_device::endpoint::IsochronousSynchronizationType::NoSynchronization,
            usb_device::endpoint::IsochronousUsageType::Feedback,
            3,
            // Per USB Audio Class 2.0 spec section 3.16.2.2, the feedback model
            // is contained in the USB Specification sections 5.12.4.2 and 9.6.6
            //
            // Feedback frequency (F_f) is available once every 2^(K - P) frames
            // K = 10 for USB full-speed interfaces with frames running at 1kHz
            //
            // Assuming we're running at F_m = 256 * F_s = 2^8 * F_s:
            // F_m = F_s * 2^P
            // P = 8
            // K = 10
            //
            // K - P = 10 - 8 = 2
            //
            // therefore feedback is available every 2^2 frames
            //
            // per USB Spec 9.6.6,
            // > for full-/high-speed isochronous endpoints, [bInterval] must be
            // > in the range from 1 to 16. The bInterval value is used as the
            // > exponent for a 2^(bInterval - 1) value; e.g. a bInterval of 4 means
            // > a period of 8 frames (2^4-1).
            //
            // 2^2 = 2^(3-1) therefore bInterval = 3
            // 3,
            // HACK tinyusb sets this to 1; let's give it a go.
            1,
        );

        let mut entity_allocator = AudioControlAllocator::new();
        let clock_source = entity_allocator.alloc_entity();
        let input_terminal = entity_allocator.alloc_entity();
        let feature_unit = entity_allocator.alloc_entity();
        let output_terminal = entity_allocator.alloc_entity();

        Self {
            if_audio_control,
            if_audio_stream,
            ep_audio_data,
            ep_feedback,
            sample_rate,
            audio_subframe_size,
            audio_bit_resolution,
            audio_data_buffer_size,
            clock_source,
            input_terminal,
            feature_unit,
            output_terminal,
            mute: false,
            volume: 0x0000,
            alt_setting: 0,
            audio_data_available: false,
            audio_feedback_needed: false,
            enable_disable: None,
        }
    }

    pub fn read_audio_data(&mut self, buffer: &mut [u8]) -> Result<usize, UsbError> {
        assert!(buffer.len() >= self.audio_data_buffer_size);
        assert!(self.audio_data_available);
        self.audio_data_available = false;
        self.ep_audio_data.read(buffer)
    }

    pub fn write_audio_feedback(&mut self, counter: &ClockCounter) -> Result<usize, UsbError> {
        let fractional_value = counter.current_rate();
        let buffer: &[u8] = &fractional_value.to_le_bytes()[0..3];
        self.audio_feedback_needed = false;
        self.ep_feedback.write(buffer)
    }

    pub fn write_raw_feedback(&mut self, value: u32) -> Result<usize, UsbError> {
        defmt::debug!("usb audio feedback :: frame {:x}", unsafe {
            let otg_device = &*pac::OTG_HS_DEVICE::ptr();
            otg_device.dsts.read().fnsof().bits()
        });

        let buffer: &[u8] = &value.to_ne_bytes()[0..3];
        defmt::debug!("usb audio feedback :: value {:x}", buffer);
        self.audio_feedback_needed = false;
        self.ep_feedback.write(buffer)
    }

    fn set_alt(&mut self, alt: u8) {
        if alt != self.alt_setting {
            if alt == 0 { self.disable_stream(); }
            if alt == 1 { self.enable_stream(); }
        }
        self.alt_setting = alt;
    }

    fn enable_stream(&mut self) {
        defmt::info!("usb audio :: Enabling audio stream");
        self.enable_disable.replace(StreamingState::Enabled);
        self.audio_feedback_needed = true;
    }

    fn disable_stream(&mut self) {
        defmt::info!("usb audio :: Disabling audio stream");
        self.enable_disable.replace(StreamingState::Disabled);
        self.audio_feedback_needed = false;
    }
}

impl<B: UsbBus> UsbClass<B> for SimpleStereoOutput<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()> {
        // Configure the two-channel audio output function
        //
        // Input Terminal -> Feature Unit -> Output Terminal
        //
        // The Input Terminal is configured for "USB Streaming"
        // The Feature Unit has mute & volume controls
        // The Output Terminal is configured as "Headphones"
        //
        // There is a single clock source, associated with the Output Terminal
        // since the clock domain of this whole function is tied to the I2S
        // clock

        defmt::info!("usb audio :: get configuration descriptors");

        defmt::debug!("usb audio :: iad");
        // Interface Association Descriptor
        writer.iad(
            self.if_audio_control,
            2, // 2 Interfaces: 1 AudioControl, 1 AudioStreaming
            audio_function_class::AUDIO_FUNCTION,
            audio_function_subclass::FUNCTION_SUBCLASS_UNDEFINED,
            audio_function_protocol::AF_VERSION_02_00
        )?;
        // Standard AC Interface Descriptor
        defmt::debug!("usb audio :: interface");
        writer.interface(
            self.if_audio_control,
            audio_interface_class::AUDIO,
            audio_interface_subclass::AUDIOCONTROL,
            audio_interface_protocol::IP_VERSION_02_00
        )?;
        // Class-Specific AC Interface Descriptor
        defmt::debug!("usb audio :: cs interface");
        let mut buf: [u8; 64] = [0; 64];
        let mut ac_interface_descriptor_writer = AudioControlInterfaceDescriptorWriter::new(
            &mut buf,
        );
        ac_interface_descriptor_writer.ac_interface_clock_source(
            self.clock_source,
            ClockAttributes::build(descriptors::ClockType::InternalProgrammable, false),
            ClockControls::new(),
            Some(self.output_terminal),
        )?;
        ac_interface_descriptor_writer.ac_interface_input_terminal(
            self.input_terminal,
            terminal_type::USB_STREAMING.to_le_bytes(),
            None,
            Some(self.clock_source),
            ChannelConfig::new(None).front_left().front_right(),
            InputTerminalControls::new(),
            None,
        )?;
        ac_interface_descriptor_writer.ac_interface_feature_unit(
            self.feature_unit,
            self.input_terminal,
            FeatureUnitControls::new()
                .mute(descriptors::ControlCapabilities::HostProgrammable)
                .volume(descriptors::ControlCapabilities::HostProgrammable),
            None,
        )?;
        ac_interface_descriptor_writer.ac_interface_output_terminal(
            self.output_terminal,
            terminal_type::OUTPUT_HEADPHONES.to_le_bytes(),
            None,
            self.feature_unit,
            self.clock_source,
            0x00, // No output terminal controls
            None,
        )?;
        defmt::debug!("usb audio :: cs interface write_into");
        ac_interface_descriptor_writer.write_into(writer)?;
        defmt::debug!("usb audio :: streaming interface");
        writer.interface(
            self.if_audio_stream,
            audio_interface_class::AUDIO,
            audio_interface_subclass::AUDIOSTREAMING,
            audio_interface_protocol::IP_VERSION_02_00
        )?;
        defmt::debug!("usb audio :: streaming interface alt");
        writer.interface_alt(self.if_audio_stream, 1, AUDIO, AUDIOSTREAMING, IP_VERSION_02_00, None)?;
        let bm_formats: u32 = PCM;
        let channels = ChannelConfig::new(None).front_left().front_right().channels();
        defmt::debug!("usb audio :: streaming interface cs");
        writer.write(CS_INTERFACE, &[
            AS_GENERAL,
            self.input_terminal.into(),
            0x00, // TODO set these controls properly
            FORMAT_TYPE_I, // Format Type
            (bm_formats & 0xff) as u8, // Formats
            (bm_formats >> 8 & 0x0ff) as u8, // ...
            (bm_formats >> 16 & 0xff) as u8, // ...
            (bm_formats >> 24 & 0xff) as u8, // ...
            2, // Channels
            channels[0], // ChannelConfig
            channels[1], // ...
            channels[2], // ...
            channels[3], // ...
            0x00, // Channel Names
        ])?;
        // 24-bit audio samples in 4-byte subslots
        defmt::debug!("usb audio :: streaming interface format");
        writer.write(CS_INTERFACE, &[
            FORMAT_TYPE,
            FORMAT_TYPE_I,
            4,
            24,
        ]).unwrap();
        defmt::debug!("usb audio :: endpoint audio data");
        writer.endpoint(&self.ep_audio_data)?;
        writer.write(CS_ENDPOINT, &[
            EP_GENERAL,
            0x00, // bit 7 = 0: allow packets shorter than max
            0x00, // no controls (TODO maybe add over-/under-run indicators)
            0x00, // lock delay units is ignored
            0x00, 0x00 // lock delay is also ignored
        ])?;
        defmt::debug!("usb audio :: endpoint feedback");
        writer.endpoint(&self.ep_feedback)?;

        defmt::info!("usb audio :: get configuration descriptors DONE");

        Ok(())
    }

    fn get_string(&self, index: StringIndex, lang_id: u16) -> Option<&str> {
        let _ = (index, lang_id);
        None
    }

    fn reset(&mut self) {
        let otg_device = unsafe { &*pac::OTG_HS_DEVICE::ptr() };
        let otg_global = unsafe { &*pac::OTG_HS_GLOBAL::ptr() };

        // Unmask a couple extra interrupts:
        // * IISOIXFRM - incomplete isochronous IN transfers
        // * EOPF - end of periodic frame
        otg_global.gintmsk.modify(|_,w| {
            w
                .iisoixfrm().set_bit()
                //.eopfm().set_bit()
        });
    }

    fn poll(&mut self) {
        // The synopsys crate (and usb-device above that) doesn't support
        // reporting incomplete isochronous IN transfers. We've got to trap them
        // ourselves and follow the procedure from the reference manual to reset
        // the endpoint. Once that's done, we can immediately re-attempt to
        // transmit.
        //
        // Tokens    Interrupts        Action
        //
        // SOF
        // ...
        // IN 1
        //           EP1 TXFE
        // ...
        //           EOPF &
        //           IISOIXFR          Disable & reset endpoint;
        //                             Fill EP1 TX FIFO (write to endpoint)
        // ...
        // SOF
        // ...
        // IN                          Core transmits from TX FIFO
        //           XFRC              Fill EP1 TX FIFO (write to endpoint)
        // ...
        // SOF
        // ...
        // IN                          Core transmits from TX FIFO
        //           XFRC              Fill EP1 TX FIFO (write to endpoint)
        // ...
        //
        let otg_device = unsafe { &*pac::OTG_HS_DEVICE::ptr() };
        let otg_global = unsafe { &*pac::OTG_HS_GLOBAL::ptr() };
        if otg_global.gintsts.read().iisoixfr().bit_is_set() {
            defmt::warn!("IISOIXFR: {:b}", otg_device.diepint1.read().bits());
            otg_global.gintsts.write(|w| w.iisoixfr().set_bit());

            if otg_device.diepint1.read().nak().bit_is_set() {
                // Set the endpoint to NAK mode
                otg_device.diepctl1.modify(|_,w| w.snak().set_bit());
                while otg_device.diepint1.read().inepne().bit_is_clear() {}

                // Disable the endpoint
                otg_device.diepctl1.modify(|_,w| w
                                            .snak().set_bit()
                                            .epdis().set_bit()
                );
                while otg_device.diepint1.read().epdisd().bit_is_clear() {}
                otg_device.diepint1.modify(|_,w| w.epdisd().set_bit());
                assert!(otg_device.diepctl1.read().epena().bit_is_clear());
                assert!(otg_device.diepctl1.read().epdis().bit_is_clear());

                // Flush the TX FIFO
                otg_global.grstctl.modify(|_,w| unsafe {
                    w.txfflsh().set_bit().txfnum().bits(0x01)
                });
                while otg_global.grstctl.read().txfflsh().bit_is_set() {}

                // We're ready to retry sending the feedback packet again
                self.audio_feedback_needed = true;
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let request = xfer.request();
        defmt::debug!("usb :: {:?}", defmt::Debug2Format(request));

        match request.request_type {
            control::RequestType::Standard => {
                match request.request {
                    control::Request::SET_INTERFACE => {
                        if request.index as u8 == self.if_audio_stream.into() {
                            match request.value {
                                0 => xfer.accept().map(|_| self.set_alt(0) ).unwrap(),
                                1 => xfer.accept().map(|_| self.set_alt(1) ).unwrap(),
                                _ => { xfer.reject().unwrap(); },
                            }
                            defmt::info!("SET_INTERFACE {}", self.alt_setting);
                        }
                    },
                    _ => { defmt::warn!("Unknown standard request {:?}", defmt::Debug2Format(request)); },
                }
            },
            control::RequestType::Class => {
                let cs_request = match ControlRequest::parse(request) {
                    Ok(r) => r,
                    Err(_) => {
                        defmt::warn!("usb audio :: Failed to parse class-specific request");
                        return xfer.reject().unwrap();
                    }
                };

                defmt::debug!("usb audio :: control out cs :: {:?}", cs_request);

                match cs_request.target {
                    Target::Interface(interface_number, entity_id) => {
                        if interface_number == self.if_audio_control.into() {
                            defmt::debug!("usb audio :: control out cs :: audio control interface");
                            match entity_id {
                                None => {
                                    // Control request directed at the interface itself
                                    return xfer.reject().unwrap();
                                },
                                Some(entity_id) => {
                                    defmt::debug!("usb audio :: control out cs :: audio control entity {:?}", entity_id);
                                    if entity_id == self.feature_unit.into() {
                                        if cs_request.control_selector == FU_MUTE_CONTROL {
                                            defmt::info!("usb audio :: Set Mute = {:?}", xfer.data());
                                            self.mute = xfer.data()[0] != 0;
                                            return xfer.accept().unwrap();
                                        } else if cs_request.control_selector == FU_VOLUME_CONTROL {
                                            defmt::info!("usb audio :: Set Volume = {:?}", xfer.data());
                                            let data = xfer.data();
                                            self.volume = i16::from_le_bytes([data[0], data[1]]);
                                            return xfer.accept().unwrap();
                                        }
                                    }
                                }
                            }
                        }
                    },
                    Target::Endpoint(endpoint_number) => {
                        defmt::debug!("usb audio :: control out cs :: endpoint number {:?}", endpoint_number);
                        return xfer.accept().unwrap();
                    }
                }
            },
            _ => { return; }
        }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let request = xfer.request();
        defmt::debug!("usb :: {:?}", defmt::Debug2Format(request));

        match request.request_type {
            control::RequestType::Vendor |
            control::RequestType::Reserved => { return; }

            control::RequestType::Standard => {
                match request.request {
                    control::Request::GET_INTERFACE => {
                        if request.index as u8 == self.if_audio_stream.into() {
                            xfer.accept_with(&[self.alt_setting]).unwrap();
                        }
                    },
                    _ => { defmt::debug!("Unknown standard request {:?}", defmt::Debug2Format(request)); },
                }
            },

            control::RequestType::Class => {
                let cs_request = match ControlRequest::parse(request) {
                    Ok(r) => r,
                    Err(_) => {
                        defmt::warn!("usb audio :: Failed to parse class-specific request");
                        return xfer.reject().unwrap();
                    }
                };

                defmt::debug!("usb audio :: control in cs :: {:?}", cs_request);

                match cs_request.target {
                    super::request::Target::Interface(interface_number, entity_id) => {
                        if interface_number == self.if_audio_control.into() {
                            defmt::debug!("usb audio :: control in cs :: audio control interface");
                            match entity_id {
                                None => {
                                    // Control request directed at the interface itself
                                    return;
                                },
                                Some(entity_id) => {
                                    defmt::debug!("usb audio :: control in cs :: audio control entity {:?}", entity_id);
                                    if entity_id == self.feature_unit.into() {
                                        if cs_request.control_selector == descriptors::feature_unit_control_selector::FU_MUTE_CONTROL {
                                            if cs_request.request_code == RequestCode::Cur {
                                                defmt::info!("usb audio :: GET MUTE Cur");
                                                return xfer.accept_with(&[self.mute as u8]).unwrap();
                                            }
                                        } else if cs_request.control_selector == descriptors::feature_unit_control_selector::FU_VOLUME_CONTROL {
                                            if cs_request.request_code == RequestCode::Cur {
                                                defmt::info!("usb audio :: GET VOLUME Cur");
                                                return xfer.accept_with(&self.volume.to_le_bytes()).unwrap();
                                            } else if cs_request.request_code == RequestCode::Range {
                                                defmt::info!("usb audio :: GET VOLUME Range");
                                                return xfer.accept_with(&[
                                                    0x01, 0x00, // 1 sub-range
                                                    0x00, 0x82, // -127 dB min
                                                    0x00, 0x00, // +0   dB max
                                                    0x00, 0x01, // 1    dB resolution
                                                ]).unwrap();
                                            }
                                        }
                                    } else if entity_id == self.clock_source.into() {
                                        if cs_request.control_selector == clock_source_control_selectors::CS_SAM_FREQ_CONTROL {
                                            match cs_request.request_code {
                                                RequestCode::Cur => {
                                                    return xfer.accept_with(&(96000_u32.to_le_bytes())).unwrap();
                                                },
                                                RequestCode::Range => {
                                                    let sample_rate = 96000_u32.to_le_bytes();
                                                    return xfer.accept_with(&[
                                                        0x01, 0x00, // 1 sub-range
                                                        sample_rate[0], sample_rate[1], sample_rate[2], sample_rate[3],
                                                        sample_rate[0], sample_rate[1], sample_rate[2], sample_rate[3],
                                                        0x01, 0x00, 0x00, 0x00,
                                                    ]).unwrap();
                                                },
                                                RequestCode::Mem => { return xfer.reject().unwrap(); }
                                            }
                                        }
                                    }
                                }
                            }
                        } else if interface_number == self.if_audio_stream.into() && entity_id == None {
                            if cs_request.control_selector == AS_ACT_ALT_SETTING_CONTROL {
                                defmt::info!("GET ALT_SETTING_CONTROL {}", self.alt_setting);
                                return xfer.accept_with(&[self.alt_setting]).unwrap();
                            } else if cs_request.control_selector == AS_VAL_ALT_SETTINGS_CONTROL {
                                return xfer.accept_with(&[
                                    // Length
                                    0x01,
                                    // Bitmask of current valid alternate settings
                                    0b00000011,
                                ]).unwrap();
                            }
                        } else {
                            return xfer.reject().unwrap();
                        }
                    },
                    super::request::Target::Endpoint(_endpoint_number) => {

                    },
                }
            }
        }
    }

    fn endpoint_setup(&mut self, addr: EndpointAddress) {
        let _ = addr;
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr == self.ep_audio_data.address() {
            self.audio_data_available = true;
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        defmt::debug!("Endpoint IN! {:?}", defmt::Debug2Format(&addr.index()));
        self.audio_feedback_needed = true;
    }
}
