use core::u8;

use usb_device::{
    class_prelude::*,
    endpoint::{Endpoint, Out, In},
    Result as UsbResult,
};

use crate::uac::{descriptors::{AudioControlAllocator, descriptor_type::CS_INTERFACE, feature_unit_control_selector::{FU_MUTE_CONTROL, FU_VOLUME_CONTROL}}, request::{RequestCode, Target}};

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
    EntityId, clock_source_control_selectors::CS_SAM_FREQ_CONTROL
}, request::{ControlRequest, Target::Interface}, ClockCounter, StreamingState};

pub struct SimpleStereoOutput<'a, B: UsbBus> {
    ep_audio_data: EndpointOut<'a, B>,
    ep_feedback: EndpointIn<'a, B>,

    audio_control: AudioControlInterface,
    audio_streaming: AudioStremingInterface,

    #[allow(dead_code)]
    sample_rate: u32,
    #[allow(dead_code)]
    audio_subframe_size: usize,
    #[allow(dead_code)]
    audio_bit_resolution: usize,
    #[allow(dead_code)]
    audio_data_buffer_size: usize,

    input_terminal: EntityId,
    output_terminal: EntityId,

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

        let audio_control = AudioControlInterface{
            if_number: if_audio_control,
            clock_source: ClockSource { entity_id: clock_source, sample_rate: 96000 },
            feature_unit: FeatureUnit { entity_id: feature_unit, mute: true, volume: 0 }
        };

        let audio_streaming = AudioStremingInterface {
            if_number: if_audio_stream,
            state: StreamingState::Disabled,
        };

        Self {
            audio_control,
            audio_streaming,
            ep_audio_data,
            ep_feedback,
            sample_rate,
            audio_subframe_size,
            audio_bit_resolution,
            audio_data_buffer_size,
            input_terminal,
            output_terminal,
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
        let buffer: &[u8] = &value.to_ne_bytes()[0..3];
        defmt::debug!("usb audio feedback :: value {:x}", buffer);
        self.audio_feedback_needed = false;
        self.ep_feedback.write(buffer)
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

    pub fn volume(&self) -> i16 {
        self.audio_control.feature_unit.volume
    }

    pub fn mute(&self) -> bool {
        self.audio_control.feature_unit.mute
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
            self.audio_control.if_number,
            2, // 2 Interfaces: 1 AudioControl, 1 AudioStreaming
            audio_function_class::AUDIO_FUNCTION,
            audio_function_subclass::FUNCTION_SUBCLASS_UNDEFINED,
            audio_function_protocol::AF_VERSION_02_00
        )?;
        // Standard AC Interface Descriptor
        defmt::debug!("usb audio :: interface");
        writer.interface(
            self.audio_control.if_number,
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
            self.audio_control.clock_source.entity_id,
            ClockAttributes::build(descriptors::ClockType::InternalProgrammable, false),
            ClockControls::new(),
            Some(self.output_terminal),
        )?;
        ac_interface_descriptor_writer.ac_interface_input_terminal(
            self.input_terminal,
            terminal_type::USB_STREAMING.to_le_bytes(),
            None,
            Some(self.audio_control.clock_source.entity_id),
            ChannelConfig::new(None).front_left().front_right(),
            InputTerminalControls::new(),
            None,
        )?;
        ac_interface_descriptor_writer.ac_interface_feature_unit(
            self.audio_control.feature_unit.entity_id,
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
            self.audio_control.feature_unit.entity_id,
            self.audio_control.clock_source.entity_id,
            0x00, // No output terminal controls
            None,
        )?;
        defmt::debug!("usb audio :: cs interface write_into");
        ac_interface_descriptor_writer.write_into(writer)?;
        defmt::debug!("usb audio :: streaming interface");
        writer.interface(
            self.audio_streaming.if_number,
            audio_interface_class::AUDIO,
            audio_interface_subclass::AUDIOSTREAMING,
            audio_interface_protocol::IP_VERSION_02_00
        )?;
        defmt::debug!("usb audio :: streaming interface alt");
        writer.interface_alt(self.audio_streaming.if_number, 1, AUDIO, AUDIOSTREAMING, IP_VERSION_02_00, None)?;
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
    }

    fn poll(&mut self) {
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let request = xfer.request();
        defmt::debug!("usb :: {:?}", defmt::Debug2Format(request));

        match request.request_type {
            control::RequestType::Standard => {
                match request.request {
                    control::Request::SET_INTERFACE => {
                        if request.index as u8 == self.audio_streaming.if_number.into() {
                            self.audio_streaming.control_out(xfer).unwrap()
                        }
                    },
                    control::Request::SET_ADDRESS => defmt::debug!("USB :: Set address {:?}", request.value),
                    control::Request::SET_CONFIGURATION => defmt::debug!("USB :: Set configuration {:?}", request.value),
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
                    Target::Interface(interface_number, Some(_)) if interface_number == self.audio_control.if_number.into() => self.audio_control.control_out(&cs_request, xfer).unwrap(),
                    Target::Endpoint(endpoint_number) => {
                        defmt::debug!("usb audio :: control out cs :: endpoint number {:?}", endpoint_number);
                        return xfer.accept().unwrap();
                    },
                    _ => return,
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
                        // TODO implement standard get interface control request on AudioStreamingInterface
                        if request.index as u8 == self.audio_streaming.if_number.into() {
                            xfer.accept_with(&[self.audio_streaming.state as u8]).unwrap();
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
                    Interface(interface_number, Some(_)) if interface_number == self.audio_control.if_number.into() => self.audio_control.cs_control_in(&cs_request, xfer).unwrap(),
                    Interface(interface_number, None) if interface_number == self.audio_streaming.if_number.into() => self.audio_streaming.cs_control_in(&cs_request, xfer).unwrap(),
                    _ => xfer.reject().unwrap()
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

struct AudioStremingInterface {
    if_number: InterfaceNumber,
    state: StreamingState,
}

impl AudioStremingInterface {
    fn cs_control_in<B: UsbBus>(&self, req: &ControlRequest, xfer: ControlIn<B>) -> UsbResult<()> {
        match req.control_selector {
            AS_ACT_ALT_SETTING_CONTROL => xfer.accept_with(&[match self.state {
                StreamingState::Disabled => 0,
                StreamingState::Enabled => 1,
            }]),
            AS_VAL_ALT_SETTINGS_CONTROL => xfer.accept_with(&[
                // Length
                0x01,
                // Bitmask of valid alternate settings
                0b00000011,
            ]),
            _ => xfer.reject()
        }
    }

    fn control_out<B: UsbBus>(&mut self, xfer: ControlOut<B>) -> UsbResult<()> {
        match xfer.request().value {
            0 => { self.state = StreamingState::Disabled; xfer.accept() },
            1 => { self.state = StreamingState::Enabled; xfer.accept() },
            _ => xfer.reject(),
        }
    }
}

struct AudioControlInterface {
    if_number: InterfaceNumber,
    feature_unit: FeatureUnit,
    clock_source: ClockSource,
}

impl AudioControlInterface {
    fn cs_control_in<B: UsbBus>(&self, req: &ControlRequest, xfer: ControlIn<B>) -> UsbResult<()> {
        match req.target {
            Interface(_, Some(entity_id)) if entity_id == self.feature_unit.entity_id.into() => self.feature_unit.cs_control_in(req, xfer),
            Interface(_, Some(entity_id)) if entity_id == self.clock_source.entity_id.into() => self.clock_source.cs_control_in(req, xfer),
            _ => Ok(())
        }
    }

    fn control_out<B: UsbBus>(&mut self, req: &ControlRequest, xfer: ControlOut<B>) -> UsbResult<()> {
        match req.target {
            Interface(_, Some(entity_id)) if entity_id == self.feature_unit.entity_id.into() => self.feature_unit.cs_control_out(req, xfer),
            Interface(_, Some(entity_id)) if entity_id == self.clock_source.entity_id.into() => self.clock_source.cs_control_out(req, xfer),
            _ => Ok(())
        }
    }
}

struct FeatureUnit{
    pub entity_id: EntityId,
    pub mute: bool,
    pub volume: i16,
}

impl FeatureUnit {
    fn cs_control_in<B: UsbBus>(&self, req: &ControlRequest, xfer: ControlIn<B>) -> UsbResult<()> {
        match (req.control_selector, req.request_code) {
            (FU_MUTE_CONTROL, RequestCode::Cur) => xfer.accept_with(&[self.mute as u8]),
            (FU_VOLUME_CONTROL, RequestCode::Cur) => xfer.accept_with(&self.volume.to_le_bytes()),
            (FU_VOLUME_CONTROL, RequestCode::Range) => xfer.accept_with(&[
                0x01, 0x00, // 1 sub-range
                0x00, 0xC4, // -60 dB min
                0x00, 0x00, // +0  dB max
                0x00, 0x01, // 1 dB resolution
            ]),
            (_, _) => Ok(())
        }
    }
    fn cs_control_out<B: UsbBus>(&mut self, req: &ControlRequest, xfer: ControlOut<B>) -> UsbResult<()> {
        match (req.control_selector, req.length) {
            (FU_MUTE_CONTROL, _) => {
                self.mute = xfer.data()[0] != 0;
                xfer.accept()
            },
            (FU_VOLUME_CONTROL, 2) => {
                self.volume = i16::from_le_bytes(xfer.data().try_into().unwrap());
                xfer.accept()
            },
            (_, _) => Ok(())
        }
    }
}

struct ClockSource{
    entity_id: EntityId,
    sample_rate: u32,
}

const SAMPLE_RATE: u32 = 96000;

impl ClockSource {
    fn cs_control_in<B: UsbBus>(&self, req: &ControlRequest, xfer: ControlIn<B>) -> UsbResult<()> {
        match (req.control_selector, req.request_code) {
            (CS_SAM_FREQ_CONTROL, RequestCode::Cur) => xfer.accept_with(&(self.sample_rate.to_le_bytes())),
            (CS_SAM_FREQ_CONTROL, RequestCode::Range) => xfer.accept_with(&[
                // 1 Sub-Range
                0x01, 0x00,
                // MIN = 96 kHz = 96000 = 0x17700
                0x00, 0x77, 0x01, 0x00,
                // MAX = 96 kHz
                0x00, 0x77, 0x01, 0x00,
                // RES = 0 (single value)
                0x00, 0x00, 0x00, 0x00,
            ]),
            (_, _) => xfer.reject()
        }
    }

    fn cs_control_out<B: UsbBus>(&mut self, req: &ControlRequest, xfer: ControlOut<B>) -> UsbResult<()> {
        match (req.control_selector, req.request_code, req.length) {
            (CS_SAM_FREQ_CONTROL, RequestCode::Cur, 4) => {
                self.sample_rate = u32::from_le_bytes(xfer.data().try_into().unwrap());
                xfer.accept()
            },
            (_, _, _) => Ok(())
        }
    }
}