use usb_device::{
    class_prelude::{DescriptorWriter, InterfaceNumber, StringIndex},
    descriptor, Result, UsbError,
};

#[allow(dead_code)]
pub mod descriptor_type {
    pub const CS_UNDEFINED: u8 = 0x20;
    pub const CS_DEVICE: u8 = 0x21;
    pub const CS_CONFIGURATION: u8 = 0x22;
    pub const CS_STRING: u8 = 0x23;
    pub const CS_INTERFACE: u8 = 0x24;
    pub const CS_ENDPOINT: u8 = 0x25;
}

#[allow(dead_code)]
pub mod ac_interface_descriptor_subtype {
    pub const AC_UNDEFINED: u8 = 0x00;
    pub const HEADER: u8 = 0x01;
    pub const INPUT_TERMINAL: u8 = 0x02;
    pub const OUTPUT_TERMINAL: u8 = 0x03;
    pub const MIXER_UNIT: u8 = 0x04;
    pub const SELECTOR_UNIT: u8 = 0x05;
    pub const FEATURE_UNIT: u8 = 0x06;
    pub const EFFECT_UNIT: u8 = 0x07;
    pub const PROCESSING_UNIT: u8 = 0x08;
    pub const EXTENSION_UNIT: u8 = 0x09;
    pub const CLOCK_SOURCE: u8 = 0x0A;
    pub const CLOCK_SELECTOR: u8 = 0x0B;
    pub const CLOCK_MULTIPLIER: u8 = 0x0C;
    pub const SAMPLE_RATE_CONVERTER: u8 = 0x0D;
}

#[allow(dead_code)]
pub mod as_interface_descriptor_subtype {
    pub const AS_DESCRIPTOR_UNDEFINED: u8 = 0x00;
    pub const AS_GENERAL: u8 = 0x01;
    pub const FORMAT_TYPE: u8 = 0x02;
    pub const ENCODER: u8 = 0x03;
    pub const DECODER: u8 = 0x04;
}

#[allow(dead_code)]
pub mod feature_unit_control_selector {
    pub const FU_CONTROL_UNDEFINED: u8 = 0x00;
    pub const FU_MUTE_CONTROL: u8 = 0x01;
    pub const FU_VOLUME_CONTROL: u8 = 0x02;
    // TODO add the rest
}

#[allow(dead_code)]
pub mod endpoint_descriptor_subtype {
    pub const DESCRIPTOR_UNDEFINED: u8 = 0x00;
    pub const EP_GENERAL: u8 = 0x01;
}

#[allow(dead_code)]
pub mod audio_device_class_release_numbers {
    pub const BCD_100: u16 = 0x1000;
    pub const BCD_200: u16 = 0x2000;
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub struct TerminalId(u8);

impl From<TerminalId> for u8 {
    fn from(n: TerminalId) -> u8 {
        n.0
    }
}

pub struct ChannelConfig {
    channels: u32,
    num_channels: u8,
    channel_names: Option<StringIndex>,
}

// TODO add all the rest of the possible channels
impl ChannelConfig {
    pub fn new(channel_names: Option<StringIndex>) -> ChannelConfig {
        ChannelConfig {
            channels: 0,
            num_channels: 0,
            channel_names,
        }
    }

    pub fn channels(&self) -> [u8; 4] {
        self.channels.to_le_bytes()
    }

    pub fn num_channels(&self) -> u8 {
        self.num_channels
    }

    fn add_channel(&mut self, n: u8) -> &mut Self {
        if self.channels & 1 << n > 0 {
            return self;
        }

        self.num_channels += 1;
        self.channels |= 1 << n;
        self
    }

    pub fn front_left(&mut self) -> &mut Self {
        self.add_channel(0)
    }
    pub fn front_right(&mut self) -> &mut Self {
        self.add_channel(1)
    }
}

#[repr(u16)]
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum ControlCapabilities {
    NotPresent = 0b00,
    ReadOnly = 0b01,
    HostProgrammable = 0b11,
}

pub struct Controls {
    bitmap: u16,
}

impl Controls {
    pub fn new() -> Controls {
        Controls { bitmap: 0 }
    }

    pub fn bitmap(&self) -> [u8; 2] {
        self.bitmap.to_le_bytes()
    }

    fn set_control(
        &mut self,
        bit_offset: usize,
        capabilities: ControlCapabilities,
    ) -> &mut Controls {
        self.bitmap |= (capabilities as u16) << bit_offset;
        self
    }

    pub fn copy_protect(&mut self, capabilities: ControlCapabilities) -> &mut Controls {
        self.set_control(0, capabilities)
    }

    pub fn connector(&mut self, capabilities: ControlCapabilities) -> &mut Controls {
        self.set_control(2, capabilities)
    }

    pub fn overload(&mut self, capabilities: ControlCapabilities) -> &mut Controls {
        self.set_control(4, capabilities)
    }

    pub fn cluster(&mut self, capabilities: ControlCapabilities) -> &mut Controls {
        self.set_control(6, capabilities)
    }

    pub fn underflow(&mut self, capabilities: ControlCapabilities) -> &mut Controls {
        self.set_control(8, capabilities)
    }

    pub fn overflow(&mut self, capabilities: ControlCapabilities) -> &mut Controls {
        self.set_control(10, capabilities)
    }
}

pub struct AudioControlInterfaceDescriptorWriter<'a> {
    buf: &'a mut [u8],
    position: usize,
    audio_class_version: u16,
    streaming_interfaces: &'a [InterfaceNumber],
    next_terminal_id: u8,
}

const MAX_STREAMING_INTERFACES: usize = 8;

impl<'a> AudioControlInterfaceDescriptorWriter<'a> {
    pub fn new(
        buf: &'a mut [u8],
        streaming_interfaces: &'a [InterfaceNumber],
    ) -> AudioControlInterfaceDescriptorWriter<'a> {
        AudioControlInterfaceDescriptorWriter {
            buf,
            position: 0,
            audio_class_version: 0x0100, // TODO un-hardcode this
            streaming_interfaces,
            next_terminal_id: 1,
        }
    }

    pub fn alloc_terminal(&mut self) -> Result<TerminalId> {
        if self.next_terminal_id == 255 {
            return Err(UsbError::Unsupported);
        }

        let terminal_id = TerminalId(self.next_terminal_id);
        self.next_terminal_id += 1;
        Ok(terminal_id)
    }

    fn write_into(&self, writer: &mut DescriptorWriter) {
        let header_len = 9;
        let length = (self.position + header_len).to_le_bytes();
        let bcd_revision = self.audio_class_version.to_le_bytes();
        let mut buf: [u8; 6 + MAX_STREAMING_INTERFACES] = [0; 6 + MAX_STREAMING_INTERFACES];

        buf[0] = ac_interface_descriptor_subtype::HEADER;
        buf[1..=2].copy_from_slice(&bcd_revision);
        buf[3..=4].copy_from_slice(&length);
        buf[5] = self.streaming_interfaces.len() as u8;

        // TODO this is gross, why isn't there an easy way to map this?
        //buf[6..].copy_from_slice(self.streaming_interfaces.iter().map(|i| u8::from(*i)).collect())
        let mut pos: usize = 6;
        for i in self.streaming_interfaces {
            buf[pos] = (*i).into();
            pos += 1;
        }

        writer.write(descriptor_type::CS_INTERFACE, &buf);
    }

    fn write(&mut self, descriptor_subtype: u8, descriptor: &[u8]) -> Result<()> {
        let leading_bytes: usize = 3;

        let length = descriptor.len();

        if (self.position + leading_bytes + length) > self.buf.len() || (length + 3) > 255 {
            return Err(UsbError::BufferOverflow);
        }

        self.buf[self.position] = (length + leading_bytes) as u8;
        self.buf[self.position + 1] = descriptor::descriptor_type::INTERFACE;
        self.buf[self.position + 2] = descriptor_subtype;

        let start = self.position + leading_bytes;

        self.buf[start..start + length].copy_from_slice(descriptor);

        self.position = start + length;

        Ok(())
    }

    fn ac_interface_clock_source(&mut self) {
        todo!()
    }

    fn ac_interface_clock_multiplier(&mut self) {
        todo!()
    }

    fn ac_interface_input_terminal(
        &mut self,
        terminal_type: [u8; 2],
        assoc_terminal_id: Option<TerminalId>,
        clock_source_id: Option<TerminalId>,
        channels: ChannelConfig,
        controls: Controls,
        name: Option<StringIndex>,
    ) -> Result<TerminalId> {
        let terminal_id = self.alloc_terminal()?;
        let channel_bytes = channels.channels();
        let controls_bytes = controls.bitmap();
        self.write(
            ac_interface_descriptor_subtype::INPUT_TERMINAL,
            &[
                terminal_id.into(),
                terminal_type[0],
                terminal_type[1],
                assoc_terminal_id.map_or(0, |id| id.into()),
                clock_source_id.map_or(0, |id| id.into()),
                channels.num_channels(),
                channel_bytes[0],
                channel_bytes[1],
                channel_bytes[2],
                channel_bytes[3],
                channels.channel_names.map_or(0, |id| id.into()),
                controls_bytes[0],
                controls_bytes[1],
                name.map_or(0, |id| id.into()),
            ],
        )?;
        Ok(terminal_id)
    }

    fn ac_interface_output_terminal(
        &mut self,
        terminal_type: [u8; 2],
        assoc_terminal_id: Option<TerminalId>,
        source_terminal_id: TerminalId,
        name: Option<StringIndex>,
    ) -> Result<TerminalId> {
        let terminal_id = self.alloc_terminal()?;
        self.write(
            ac_interface_descriptor_subtype::OUTPUT_TERMINAL,
            &[
                terminal_id.into(),
                terminal_type[0],
                terminal_type[1],
                assoc_terminal_id.map_or(0, |id| id.into()),
                source_terminal_id.into(),
                name.map_or(0, |id| id.into()),
            ],
        )?;
        Ok(terminal_id)
    }

    fn ac_interface_mixer_unit(&mut self) {
        todo!()
    }

    fn ac_interface_selector_unit(&mut self) {
        todo!()
    }

    fn ac_interface_feature_unit(
        &mut self,
        source_terminal_id: TerminalId,
        master_channel_controls: Controls,
        logical_channel_controls: &[Controls],
    ) {
        let terminal_id = self.alloc_terminal()?;
        self.write(
            ac_interface_descriptor_subtype::FEATURE_UNIT,
            &[
                terminal_id.into(),
                source_terminal_id.into(),
                0x01 + logical_channel_controls.len() as u8,
                master_channel_controls.into(),
            ],
        )
    }

    fn ac_interface_effect_unit(&mut self) {
        todo!()
    }

    fn ac_interface_processing_unit(&mut self) {
        todo!()
    }

    fn ac_interface_extension_unit(&mut self) {
        todo!()
    }
}
