use core::{ops::DerefMut, pin::Pin};

use heapless::Deque;
use usb_device::{class_prelude::*, descriptor::descriptor_type, control::Recipient};
use as_slice::AsMutSlice;

use self::sizes::AUDIO_STREAM_BUFFER;

const CHANNELS: u32 = 2;
const SAMPLING_RATE: u32 = 96_000;
const BITS_PER_SAMPLE: u32 = 24;
const BYTES_PER_SAMPLE: u32 = 4;
const USB_FRAME_FREQUENCY: u32 = 1_000;
const EXPECTED_SAMPLES_PER_FRAME: u32 = SAMPLING_RATE / USB_FRAME_FREQUENCY;

mod sizes {
    use super::{SAMPLING_RATE, USB_FRAME_FREQUENCY, BYTES_PER_SAMPLE, CHANNELS};

    pub const CONTROL_BUFFER: usize = 256;
    pub const AUDIO_STREAM_BUFFER: usize =
        ((SAMPLING_RATE / USB_FRAME_FREQUENCY + 1) * BYTES_PER_SAMPLE * CHANNELS) as usize;
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

    pub audio_data_available: bool,
    //control_buf: [u8; sizes::CONTROL_BUFFER],
    //audio_stream_buf: &'a mut [u8],
}

struct DeviceStrings {
    interface_alt: StringIndex,
}

impl<'a, B> UsbAudioClass<'a, B>
where
    B: UsbBus
{
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> UsbAudioClass<'a, B> {
        defmt::debug!("Allocating audio control interface");
        let iface_audio_control = alloc.interface();
        defmt::debug!("Success");

        defmt::debug!("Allocating audio stream interface");
        let iface_audio_stream = alloc.interface();
        defmt::debug!("Success");

        defmt::debug!("Allocating audio OUT endpoint");
        let ep_audio_out = alloc.isochronous(
                usb_device::endpoint::IsochronousSynchronizationType::Asynchronous,
                usb_device::endpoint::IsochronousUsageType::Data,
                sizes::AUDIO_STREAM_BUFFER as u16,
                0x01
            );
        defmt::debug!("Success");

        defmt::debug!("Allocating audio IN endpoint");
        let ep_audio_fb_in = alloc.isochronous(
                usb_device::endpoint::IsochronousSynchronizationType::NoSynchronization,
                usb_device::endpoint::IsochronousUsageType::Feedback,
                // Per the UAC1.0 spec, audio feedback packets are 3 bytes long
                0x03,
                // Per USB Audio Class 1.0 3.7.2.2:
                //
                // Feedback frequency (F_f) is available once every 2^(K - P) frames
                // K = 10 for UAC 1.0 on USB FS interfaces with frames running at 1kHz
                //
                // Assuming we're running at F_m = 256 * F_s = 2^8 * F_s:
                // F_m = F_s * 2^(P - 1)
                // P = 9
                // K = 10
                //
                // bRefresh value is the exponent, which is 10 - P = 10 - 9 = 1
                0x01
            );
        defmt::debug!("Success");

        Self {
            iface_audio_control,
            iface_audio_stream,
            ep_audio_stream: ep_audio_out,
            ep_audio_stream_fb: ep_audio_fb_in,
            //control_buf: [0; sizes::CONTROL_BUFFER],
            audio_data_available: false,
        }
    }

    pub fn read_audio_stream(&self, buffer: &mut [u8]) -> Result<usize, UsbError>
    {
        assert!(buffer.len() >= sizes::AUDIO_STREAM_BUFFER);
        assert!(self.audio_data_available);
        self.ep_audio_stream.read(buffer.as_mut_slice()).map(|bytes_received| {
            bytes_received
        })
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
        writer.write(0x24, &[
            0x01, // HEADER
            0x00, 0x01, // Revision of class specification - 1.0
            0x27, 0x00, // Total size of class-specific descriptors
            0x01,   // 1 streaming interface
            //0x01,   // AudioStreaming interface 1 belongs to this AudioControl interface
            self.iface_audio_stream.into(),   // AudioStreaming interface 1 belongs to this AudioControl interface
        ])?;
        writer.write(0x24, &[
            0x02, // INPUT_TERMINAL
            0x01, // Terminal ID = 1
            0x01, 0x01, // USB Streaming
            0x00, // No associated terminal
            0x02, // Two channels
            0b00000011, 0b00000000, // Left Front, Right Front
            0x00, // No custom channel names
            0x00, // No text description
        ])?;
        writer.write(0x24, &[
            0x06, // FEATURE UNIT
            0x02, // STREAMING CONTROL
            0x01, // Source ID
            0x01, // Control Size
            0x01, 0x00, // mute
            0x00, // No associated terminal
        ])?;
        writer.write(0x24, &[
            0x03, // OUTPUT_TERMINAL
            0x03, // Terminal ID = 2
            0x02, 0x03, // Headphones
            0x00, // No associated terminal
            0x02, // From input terminal
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
        writer.write(0x24, &[
            0x01, // GENERAL
            0x01, // Input terminal 1
            0x01, // 1 frame of delay
            0x01, 0x00 // PCM Format
        ])?;
        writer.write(0x24, &[
            0x02, // FORMAT_TYPE subtype
            0x01, // FORMAT_TYPE_I
            0x02, // Two channels
            0x04, // Four bytes per audio subframe
            0x18, // 24 bits per sample
            0x01, // One frequency supported
            0x00, 0x77, 0x01, // 96_000 Hz
        ])?;
        writer.endpoint(&self.ep_audio_stream)?;
        writer.write(0x25, &[
            0x01, // TODO EP_GENERAL
            0b00000000, // No control over anything for this endpoint
            0x00, // Lock Delay is unused for asynchronous endpoints
            0x00, 0x00 // Lock Delay is unused for asynchronous endpoints
        ])?;
        writer.endpoint(&self.ep_audio_stream_fb)?;

        Ok(())
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr == self.ep_audio_stream.address() {
            defmt::debug!("Received audio stream packet");
            self.audio_data_available = true;
            // self.ep_audio_stream.read(&mut self.audio_stream_buf).map_or_else(|err| {
            //     defmt::warn!("Error in audio stream packet read");
            // }, |size| {
            //     defmt::debug!("Received {=usize} bytes of audio data", size);
            // });
        }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        defmt::debug!("Received control IN");
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        defmt::debug!("Received control OUT");
        let request = xfer.request();

        match request.request_type {
            control::RequestType::Standard => {
                defmt::debug!("  Standard Request, type {}", request.request);
                match request.request {
                    control::Request::SET_INTERFACE => {
                        defmt::debug!("  SET_INTERFACE {}",  request.value);
                        xfer.accept().unwrap();
                    },
                    _ => {
                        defmt::debug!("  other request type");
                    }
                }
            },
            _ => { defmt::debug!("Not handling control OUT"); }
        }
        //defmt::debug!("{:?}", request);
    }
}
