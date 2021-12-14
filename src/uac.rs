use usb_device::{class_prelude::*, descriptor::descriptor_type};

mod sizes {
    pub const CONTROL_BUFFER: usize = 256;
    pub const AUDIO_STREAM_BUFFER: usize = 96000 * 2 * 4 / 1000 + 1;
}

mod consts {
    pub const USB_AUDIO_CLASS: u8 = 0x01;
    pub const USB_AUDIO_CLASS_SUBCLASS_UNDEFINED: u8 = 0x00;
    pub const USB_AUDIO_CLASS_SUBCLASS_AUDIOCONTROL: u8 = 0x01;
    pub const USB_AUDIO_CLASS_SUBCLASS_AUDIOSTREAMING: u8 = 0x02;
    pub const USB_AUDIO_CLASS_SUBCLASS_MIDISTREAMING: u8 = 0x03;
}

pub struct UsbAudioClass<'a, B: UsbBus> {
    iface_audio_control: InterfaceNumber,
    iface_audio_stream: InterfaceNumber,
    ep_audio_stream: EndpointOut<'a, B>,
    ep_audio_stream_fb: EndpointIn<'a, B>,
    control_buf: [u8; sizes::CONTROL_BUFFER],
    audio_stream_buf: &'a mut [u8],
}

impl<'a, B: UsbBus> UsbAudioClass<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>, audio_stream_buf: &'a mut [u8]) -> UsbAudioClass<'a, B> {
        Self {
            iface_audio_control: alloc.interface(),
            iface_audio_stream: alloc.interface(),
            ep_audio_stream: alloc.isochronous(
                usb_device::endpoint::IsochronousSynchronizationType::Asynchronous,
                usb_device::endpoint::IsochronousUsageType::Data,
                sizes::AUDIO_STREAM_BUFFER as u16,
                0x01
            ),
            ep_audio_stream_fb: alloc.isochronous(
                usb_device::endpoint::IsochronousSynchronizationType::NoSynchronization,
                usb_device::endpoint::IsochronousUsageType::Feedback,
                // Per the UAC1.0 spec, audio feedback packets are 3 bytes long
                0x03,
                // Update the feedback value as frequently as possible (once per frame)
                0x01
            ),
            control_buf: [0; sizes::CONTROL_BUFFER],
            audio_stream_buf,
        }
    }
}

impl<B: UsbBus> UsbClass<B> for UsbAudioClass<'_, B> {
    fn reset(&mut self) {
    }

    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<(), UsbError> {
        // Interface Association Descriptor
        // Groups the audio control & streaming interfaces into one "function"
        writer.iad(
            self.iface_audio_control,
            0x02,
            consts::USB_AUDIO_CLASS, consts::USB_AUDIO_CLASS_SUBCLASS_AUDIOCONTROL, 0x00
        )?;
        // Control interface; uses the default 0 endpoint for audio control requests
        writer.interface(
            self.iface_audio_control,
            consts::USB_AUDIO_CLASS,
            consts::USB_AUDIO_CLASS_SUBCLASS_AUDIOCONTROL,
            0x00
        )?;
        writer.write(descriptor_type::INTERFACE, &[
            0x01, // HEADER
            0x01, 0x00, // Revision of class specification - 1.0
            0x00, 0x1E, // Total size of class-specific descriptors
            0x01,   // 1 streaming interface
            0x01,   // AudioStreaming interface 1 belongs to this AudioControl interface
        ])?;
        writer.write(descriptor_type::INTERFACE, &[
            0x02, // INPUT_TERMINAL
            0x01, // Terminal ID = 1
            0x01, 0x01, // USB Streaming
            0x00, // No associated terminal
            0x02, // Two channels
            0b00000000, 0b00000011, // Left Front, Right Front
            0x00, // No custom channel names
            0x00, // No text description
        ])?;
        writer.write(descriptor_type::INTERFACE, &[
            0x03, // OUTPUT_TERMINAL
            0x02, // Terminal ID = 2
            0x03, 0x02, // Headphones
            0x00, // No associated terminal
            0x01, // From input terminal
            0x00, // No text description
        ])?;
        // Audio streaming interface with no endpoints, used when the source isn't sending any data
        writer.interface(
            self.iface_audio_stream,
            consts::USB_AUDIO_CLASS,
            consts::USB_AUDIO_CLASS_SUBCLASS_AUDIOSTREAMING,
            0x00
        )?;
        // Audio streaming interface with an asynchronous isochronous data endpoint
        writer.interface_alt(
            self.iface_audio_stream,
            1,
            consts::USB_AUDIO_CLASS,
            consts::USB_AUDIO_CLASS_SUBCLASS_AUDIOSTREAMING,
            0x00,
            None
        )?;
        writer.write(descriptor_type::INTERFACE, &[
            0x01, // GENERAL
            0x01, // Input terminal 1
            0x01, // 1 frame of delay
            0x00, 0x01 // PCM Format
        ])?;
        writer.write(descriptor_type::INTERFACE, &[
            0x02, // FORMAT_TYPE subtype
            0x01, // FORMAT_TYPE_I
            0x02, // Two channels
            0x04, // Four bytes per audio subframe
            0x18, // 24 bits per sample
            0x01, // One frequency supported
            0x01, 0x77, 0x00, // 96_000 Hz
        ])?;
        writer.endpoint(&self.ep_audio_stream)?;
        writer.write(descriptor_type::ENDPOINT, &[
            0x01, // TODO EP_GENERAL
            0b00000000, // No control over anything for this endpoint
            0x00, // Lock Delay is unused for asynchronous endpoints
            0x00, 0x00 // Lock Delay is unused for asynchronous endpoints
        ])?;
        writer.endpoint(&self.ep_audio_stream_fb)?;

        Ok(())
    }

    fn poll(&mut self) {

    }
}