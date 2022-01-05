use as_slice::AsMutSlice;
use usb_device::{class_prelude::*, descriptor::descriptor_type::{INTERFACE, ENDPOINT}};
use super::descriptors::{self, audio_function_class, audio_function_subclass, audio_function_protocol, audio_interface_class::{self, AUDIO}, audio_interface_subclass::{self, AUDIOSTREAMING}, audio_interface_protocol::{self, IP_VERSION_02_00}, AudioControlInterfaceDescriptorWriter, ClockAttributes, ClockControls, terminal_type, ChannelConfig, InputTerminalControls, FeatureUnitControls, as_interface_descriptor_subtype::{AS_GENERAL, FORMAT_TYPE}, format_type_codes::{self, FORMAT_TYPE_I}, audio_format_type_1_bit_allocations::PCM, descriptor_type::CS_ENDPOINT, endpoint_descriptor_subtypes::EP_GENERAL};

const CHANNELS: u8 = 2;

pub struct SimpleStereoOutput<'a, B: UsbBus> {
    if_audio_control: InterfaceNumber,
    if_audio_stream: InterfaceNumber,
    ep_audio_data: EndpointOut<'a, B>,
    ep_feedback: EndpointIn<'a, B>,

    sample_rate: u32,
    audio_subframe_size: usize,
    audio_bit_resolution: usize,

    audio_data_buffer_size: usize,

    mute: bool,
    volume: u8,
}

impl<'a, B> SimpleStereoOutput<'a, B>
where
    B: UsbBus
{
    pub fn new(
        alloc: &'a UsbBusAllocator<B>,
        sample_rate: u32,
        audio_subframe_size: usize,
        audio_bit_resolution: usize,
    ) -> SimpleStereoOutput<'a, B> {
        let if_audio_control = alloc.interface();
        let if_audio_stream = alloc.interface();

        let audio_data_buffer_size: usize = (sample_rate as usize / 1000 + 1) * audio_subframe_size * CHANNELS as usize;
        assert!(audio_data_buffer_size < 1023);
        let ep_audio_data = alloc.isochronous(
            usb_device::endpoint::IsochronousSynchronizationType::Asynchronous,
            usb_device::endpoint::IsochronousUsageType::Data,
            audio_data_buffer_size as u16,
            0x01,
        );
        let ep_feedback = alloc.isochronous(
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
            3,
        );

        Self {
            if_audio_control,
            if_audio_stream,
            ep_audio_data,
            ep_feedback,
            sample_rate,
            audio_subframe_size,
            audio_bit_resolution,
            audio_data_buffer_size,
            mute: false,
            volume: 100,
        }
    }

    pub fn read_audio_data(&mut self, buffer: &mut [u8]) -> Result<usize, UsbError> {
        assert!(buffer.len() >= self.audio_data_buffer_size);
        self.ep_audio_data.read(buffer)
    }
}

impl<B: UsbBus> UsbClass<B> for SimpleStereoOutput<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()> {
        // Interface Association Descriptor
        writer.iad(
            self.if_audio_control,
            2, // 2 Interfaces: 1 AudioControl, 1 AudioStreaming
            audio_function_class::AUDIO_FUNCTION,
            audio_function_subclass::FUNCTION_SUBCLASS_UNDEFINED,
            audio_function_protocol::AF_VERSION_02_00
        )?;
        // Standard AC Interface Descriptor
        writer.interface(
            self.if_audio_control,
            audio_interface_class::AUDIO,
            audio_interface_subclass::AUDIOCONTROL,
            audio_interface_protocol::IP_VERSION_02_00
        )?;
        // Class-Specific AC Interface Descriptor
        let mut buf: [u8; 64] = [0; 64];
        let mut ac_interface_descriptor_writer = AudioControlInterfaceDescriptorWriter::new(
            &mut buf,
        );
        let clock_source = ac_interface_descriptor_writer.alloc_entity()?;
        let input_terminal = ac_interface_descriptor_writer.alloc_entity()?;
        let feature_unit = ac_interface_descriptor_writer.alloc_entity()?;
        let output_terminal = ac_interface_descriptor_writer.alloc_entity()?;
        ac_interface_descriptor_writer.ac_interface_clock_source(
            clock_source,
            ClockAttributes::build(descriptors::ClockType::InternalProgrammable, false),
            ClockControls::new(),
            Some(output_terminal),
        );
        ac_interface_descriptor_writer.ac_interface_input_terminal(
            input_terminal,
            terminal_type::USB_STREAMING.to_le_bytes(),
            None,
            Some(clock_source),
            ChannelConfig::new(None).front_left().front_right(),
            InputTerminalControls::new(),
            None,
        );
        ac_interface_descriptor_writer.ac_interface_feature_unit(
            feature_unit,
            input_terminal,
            FeatureUnitControls::new()
                .mute(descriptors::ControlCapabilities::HostProgrammable)
                .volume(descriptors::ControlCapabilities::HostProgrammable),
            None,
        );
        ac_interface_descriptor_writer.ac_interface_output_terminal(
            output_terminal,
            terminal_type::OUTPUT_HEADPHONES.to_le_bytes(),
            None,
            feature_unit,
            clock_source,
            0x00, // No output terminal controls
            None,
        );
        ac_interface_descriptor_writer.write_into(writer);
        writer.interface(
            self.if_audio_stream,
            audio_interface_class::AUDIO,
            audio_interface_subclass::AUDIOSTREAMING,
            audio_interface_protocol::IP_VERSION_02_00
        );
        writer.interface_alt(self.if_audio_stream, 1, AUDIO, AUDIOSTREAMING, IP_VERSION_02_00, None);
        let bm_formats: u32 = PCM;
        let channels = ChannelConfig::new(None).front_left().front_right().channels();
        writer.write(INTERFACE, &[
            AS_GENERAL,
            input_terminal.into(),
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
        ]);
        // 24-bit audio samples in 4-byte subslots
        writer.write(INTERFACE, &[
            FORMAT_TYPE,
            FORMAT_TYPE_I,
            4,
            24,
        ]);
        writer.endpoint(&self.ep_audio_data);
        writer.write(ENDPOINT, &[
            CS_ENDPOINT,
            EP_GENERAL,
            0x00, // bit 7 = 0: allow packets shorter than max
            0x00, // no controls (TODO maybe add over-/under-run indicators)
            0x00, // lock delay units is ignored
            0x00, 0x00 // lock delay is also ignored
        ]);
        writer.endpoint(&self.ep_feedback);
        Ok(())
    }

    fn get_string(&self, index: StringIndex, lang_id: u16) -> Option<&str> {
        let _ = (index, lang_id);
        None
    }

    fn reset(&mut self) {}

    fn poll(&mut self) {}

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let _ = xfer;
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let _ = xfer;
    }

    fn endpoint_setup(&mut self, addr: EndpointAddress) {
        let _ = addr;
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        let _ = addr;
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        let _ = addr;
    }
}
