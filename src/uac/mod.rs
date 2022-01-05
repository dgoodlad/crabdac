use usb_device::class_prelude::*;
use as_slice::AsMutSlice;

mod descriptors;
mod simple_stereo_output;

const CHANNELS: u32 = 2;
const SAMPLING_RATE: u32 = 96_000;
const BITS_PER_SAMPLE: u32 = 24;
const BYTES_PER_SAMPLE: u32 = 4;
const USB_FRAME_FREQUENCY: u32 = 1_000;

mod sizes {
    use super::{SAMPLING_RATE, USB_FRAME_FREQUENCY, BYTES_PER_SAMPLE, CHANNELS};

    pub const AUDIO_STREAM_BUFFER: usize =
        ((SAMPLING_RATE / USB_FRAME_FREQUENCY + 1) * BYTES_PER_SAMPLE * CHANNELS) as usize;
}

mod consts {
    pub const USB_AUDIO_CLASS: u8 = 0x01;
    pub const USB_AUDIO_CLASS_SUBCLASS_UNDEFINED: u8 = 0x00;
    pub const USB_AUDIO_CLASS_SUBCLASS_AUDIOCONTROL: u8 = 0x01;
    pub const USB_AUDIO_CLASS_SUBCLASS_AUDIOSTREAMING: u8 = 0x02;
    // pub const USB_AUDIO_CLASS_SUBCLASS_MIDISTREAMING: u8 = 0x03;

    pub const AF_VERSION_02_00: u8 = 0x20;
}

#[repr(u8)]
#[derive(PartialEq)]
pub enum StreamingState {
    Enabled,
    Disabled,
}

pub struct UsbAudioClass<'a, B: UsbBus> {
    iface_audio_control: InterfaceNumber,
    iface_audio_stream: InterfaceNumber,
    ep_audio_stream: EndpointOut<'a, B>,
    ep_audio_stream_fb: EndpointIn<'a, B>,

    pub audio_data_available: bool,
    pub audio_feedback_needed: bool,
    pub enable_disable: Option<StreamingState>,

    mute: bool,

    printed: u8,
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
        defmt::info!("Allocating audio control interface");
        let iface_audio_control = alloc.interface();
        defmt::info!("Success");

        defmt::info!("Allocating audio stream interface");
        let iface_audio_stream = alloc.interface();
        defmt::info!("Success");

        defmt::info!("Allocating audio OUT endpoint");
        let ep_audio_out = alloc.isochronous(
                usb_device::endpoint::IsochronousSynchronizationType::Asynchronous,
                usb_device::endpoint::IsochronousUsageType::Data,
                sizes::AUDIO_STREAM_BUFFER as u16,
                0x01
            );
        defmt::info!("Success");

        defmt::info!("Allocating audio IN endpoint");
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
                // F_m = F_s * 2^P
                // P = 8
                // K = 10
                //
                // bRefresh value is the exponent, which is 10 - P = 10 - 8 = 2
                0x02
            );
        defmt::info!("Success");

        Self {
            iface_audio_control,
            iface_audio_stream,
            ep_audio_stream: ep_audio_out,
            ep_audio_stream_fb: ep_audio_fb_in,
            //control_buf: [0; sizes::CONTROL_BUFFER],
            audio_data_available: false,
            audio_feedback_needed: false,
            enable_disable: None,
            mute: false,
            printed: 0,
        }
    }

    pub fn read_audio_stream(&mut self, buffer: &mut [u8]) -> Result<usize, UsbError> {
        assert!(buffer.len() >= sizes::AUDIO_STREAM_BUFFER);
        assert!(self.audio_data_available);
        self.ep_audio_stream.read(buffer.as_mut_slice()).map(|bytes_received| {
            self.audio_data_available = false;
            if self.printed < 5 && (buffer[0] > 0x00 || buffer[4] > 0) {
                self.printed += 1;
                defmt::info!("Sending i2s data: {:#x}", buffer[0..bytes_received]);
            }
            bytes_received
        })
    }

    // TODO Feedback value should be:
    //   (sample1 + sample2) << 9
    // e.g (24599 + 24600) << 9 ~= 96.09 samples/frame
    pub fn write_audio_feedback(&mut self, counter: &ClockCounter) -> Result<usize, UsbError> {
        let fractional_value = counter.current_rate();
        let buffer = &fractional_value.to_le_bytes()[0..3];
        defmt::info!("usb audio :: feedback {:?} {:#x}", fractional_value, buffer);
        self.ep_audio_stream_fb.write(buffer).and_then(|x| {
            self.audio_feedback_needed = false;
            Ok(x)
        })
    }

    fn enable_stream(&mut self) {
        defmt::info!("Enabling audio stream");
        self.enable_disable.replace(StreamingState::Enabled);
        self.audio_feedback_needed = true;
    }

    fn disable_stream(&mut self) {
        defmt::info!("Disabling audio stream");
        self.enable_disable.replace(StreamingState::Disabled);
        self.audio_feedback_needed = false;
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
            consts::USB_AUDIO_CLASS,
            consts::USB_AUDIO_CLASS_SUBCLASS_UNDEFINED,
            consts::AF_VERSION_02_00,
        )?;
        // Control interface; uses the default 0 endpoint for audio control requests
        writer.interface(
            self.iface_audio_control,
            consts::USB_AUDIO_CLASS,
            consts::USB_AUDIO_CLASS_SUBCLASS_AUDIOCONTROL,
            0x20
        )?;
        writer.write(0x24, &[
            0x01, // HEADER
            0x00, 0x02, // Revision of class specification - 2.0
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
            defmt::trace!("Received audio stream packet");
            self.audio_data_available = true;
        }
    }


    fn control_in(&mut self, xfer: ControlIn<B>) {
        let request = xfer.request();

        match request.request_type {
            control::RequestType::Vendor |
            control::RequestType::Reserved => { return; },

            control::RequestType::Standard => {
                defmt::info!("Control IN Standard Request {}", request.request);

                match request.request {
                    control::Request::GET_INTERFACE => {
                        defmt::info!("GET_INTERFACE iface{}", request.index);
                        if request.index as u8 == self.iface_audio_stream.into() {
                            xfer.accept_with(&[0_u8]).unwrap();
                        }
                    },
                    _ => { return; }
                }
            },

            control::RequestType::Class => {
                match request.request {
                    0b00100001 => {
                        // Audio Control Request - CUR - Interface Entity
                        // Likely MUTE
                        defmt::info!("Control IN - Audio Control - CUR - Entity - {:#x} {:#x}", request.index, request.value);
                        if request.index == 0x02 && ((request.value & 0xff00) >> 8) == 0x01 {
                            // entity id 0x02 is our feature unit
                            // CS (high byte of value) is 1 == MUTE
                            xfer.accept_with(&[self.mute as u8]).unwrap();
                        } else {
                            xfer.reject().unwrap();
                        }
                    },
                    0b00100010 => {
                        // Audio Control Request - CUR - Endpoint
                        defmt::info!("Control IN - Audio Control - CUR - Endpoint - {:#x} {:#x}", request.index, request.value);
                        xfer.reject().unwrap();
                    },
                    _ => { xfer.reject().unwrap(); }
                }
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let request = xfer.request();

        match request.request_type {
            control::RequestType::Vendor |
            control::RequestType::Reserved => { return; },

            control::RequestType::Standard => {
                match request.request {
                    control::Request::SET_INTERFACE => {
                        defmt::info!("SET_INTERFACE iface{}->{}", request.index, request.value);
                        if request.index as u8 == self.iface_audio_stream.into() {
                            match request.value {
                                0 => xfer.accept().map(|_| self.disable_stream()).unwrap(),
                                1 => xfer.accept().map(|_| self.enable_stream()).unwrap(),
                                // We only have 0/1 alternate interfaces for the
                                // audio stream, so any other value is invalid
                                _ => { xfer.reject().unwrap(); },
                            }
                        }
                    },
                    _ => {},
                }
            },

            control::RequestType::Class => {
                defmt::info!("Class-specific request: {}, {:#b} {:#x} {:#x}, {:#x}", request.recipient as u8, request.request, request.value, request.index, request.length);
                match request.request {
                    0b00001 => {
                        // Directed at an entity in an interface of the audio function
                        let control_selector = request.value >> 8;
                        let channel_number = request.value & 0xff;

                        let entity_id = request.index >> 8;
                        let interface_number = request.index & 0xff;

                        let length = request.length as usize;

                        let data = xfer.data();
                        if entity_id == 0x02 { // hardcoded feature unit id TODO
                            if interface_number as u8 == self.iface_audio_control.into() {
                                if control_selector == 0x01 { // hardcoded control mute TODO
                                    if channel_number == 0x00 { // hardcoded channel number TODO
                                        assert!(data.len() == length);
                                        assert!(length == 1);
                                        let value = data[0];
                                        self.mute = value == 1;
                                        xfer.accept().unwrap();
                                        defmt::info!("Toggled mute: {}", self.mute);
                                    }
                                }
                            }
                        }
                    },
                    0b00010 => {
                        // Directed at the isochronous endpoint of the audio streaming interface
                    },
                    _ => { defmt::info!("Unsupported class-specific request"); }
                }
            }

        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        defmt::info!("Endpoint IN complete {}", addr.index());
        self.audio_feedback_needed = true;
    }
}

#[derive(Debug)]
pub struct ClockCounter {
    ticks: u32,
    frames: u8,
    mck_to_fs_ratio: u8,
}

impl ClockCounter {
    pub fn new(mck_to_fs_ratio: u8) -> Self {
        Self { ticks: 0, frames: 0, mck_to_fs_ratio }
    }

    pub fn clear(&mut self) {
        self.ticks = 0;
        self.frames = 0;
    }

    pub fn add(&mut self, ticks: u32) {
        self.ticks += ticks;
        self.frames += 1;
    }

    pub fn current_rate(&self) -> u32 {
        self.ticks << (14 - self.mck_to_fs_ratio - (self.frames - 1))
    }
}
