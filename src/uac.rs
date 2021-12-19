use core::{ops::DerefMut, pin::Pin};

use heapless::Deque;
use usb_device::{class_prelude::*, descriptor::descriptor_type};
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
    //control_buf: [u8; sizes::CONTROL_BUFFER],
    audio_stream_buf: &'a mut [u8],
    overflow_buf: [u32; 4],
    extra_samples: heapless::Deque<u8, {(2 * BYTES_PER_SAMPLE * CHANNELS) as usize}>,
}

impl<'a, B> UsbAudioClass<'a, B>
where
    B: UsbBus
{
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
            ),
            //control_buf: [0; sizes::CONTROL_BUFFER],
            audio_stream_buf,
            overflow_buf: [0; 4],
            extra_samples: Deque::new(),
        }
    }

    fn read_audio_stream(mut self, buffer: &mut [u8; AUDIO_STREAM_BUFFER]) -> Result<usize, UsbError>
    {
        self.ep_audio_stream.read(buffer.as_mut_slice()).map(|bytes_received| {
            let samples_received = bytes_received as u32 / BYTES_PER_SAMPLE / CHANNELS;
            let extra_samples = samples_received - EXPECTED_SAMPLES_PER_FRAME;
            if extra_samples > 0 {
                let (expected, extra) = buffer.split_at(buffer.len() - (extra_samples * BYTES_PER_SAMPLE * CHANNELS) as usize);
                for x in extra.iter() {
                    self.extra_samples.push_back(*x);
                }
            }
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

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr == self.ep_audio_stream.address() {
            defmt::debug!("Received audio stream packet");
            self.ep_audio_stream.read(&mut self.audio_stream_buf).map_or_else(|err| {
                defmt::warn!("Error in audio stream packet read");
            }, |size| {
                defmt::debug!("Received {=usize} bytes of audio data", size);
            });
        }
    }
}
