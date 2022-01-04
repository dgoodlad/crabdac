use usb_device::{
    class_prelude::{DescriptorWriter, InterfaceNumber, StringIndex},
    descriptor, Result, UsbError,
};

/// Audio Function Class Code
/// USB Audio 2.0 Spec Table A-1
#[allow(dead_code)]
pub mod audio_function_class {
    pub const AUDIO_FUNCTION: u8 = super::audio_interface_class::AUDIO;
}

/// Audio Function Subclass Codes
/// USB Audio 2.0 Spec Table A-2
#[allow(dead_code)]
pub mod audio_function_subclass {
    pub const FUNCTION_SUBCLASS_UNDEFINED: u8 = 0x00;
}

/// Audio Function Protocol Codes
/// USB Audio 2.0 Spec Table A-3
#[allow(dead_code)]
pub mod audio_function_protocol {
    pub const FUNCTION_PROTOCOL_UNDEFINED: u8 = 0x00;
    pub const AF_VERSION_02_00: u8 = super::audio_interface_protocol::IP_VERSION_02_00;
}

/// Audio Interface Class Code
/// USB Audio 2.0 Spec Table A-4
#[allow(dead_code)]
pub mod audio_interface_class {
    pub const AUDIO: u8 = 0x01;
}

/// Audio Interface Subclass Codes
/// USB Audio 2.0 Spec Table A-5
#[allow(dead_code)]
pub mod audio_interface_subclass {
    pub const INTERFACE_SUBCLASS_UNDEFINED: u8 = 0x00;
    pub const AUDIOCONTROL: u8 = 0x01;
    pub const AUDIOSTREAMING: u8 = 0x02;
    pub const MIDISTREAMING: u8 = 0x03;
}

/// Audio Interface Protocol Codes
/// USB Audio 2.0 Spec Table A-6
#[allow(dead_code)]
pub mod audio_interface_protocol {
    pub const INTERFACE_PROTOCOL_UNDEFINED: u8 = 0x00;
    pub const IP_VERSION_02_00: u8 = 0x20;
}

/// Audio Function Category Codes
/// USB Audio 2.0 Spec Table A-7
#[allow(dead_code)]
pub mod audio_function_category {
    pub const FUNCTION_SUBCLASS_UNDEFINED: u8 = 0x00;
    pub const DESKTOP_SPEAKER: u8 = 0x01;
    pub const HOME_THEATER: u8 = 0x02;
    // TODO add the rest of the codes from from Audio20 A.7
    pub const OTHER: u8 = 0xff;
}

/// Audio Class-Specific Descriptor Types
/// USB Audio 2.0 Spec Table A-8
#[allow(dead_code)]
pub mod descriptor_type {
    pub const CS_UNDEFINED: u8 = 0x20;
    pub const CS_DEVICE: u8 = 0x21;
    pub const CS_CONFIGURATION: u8 = 0x22;
    pub const CS_STRING: u8 = 0x23;
    pub const CS_INTERFACE: u8 = 0x24;
    pub const CS_ENDPOINT: u8 = 0x25;
}

/// Audio Class-Specific AC Interface Descriptor Subtypes
/// USB Audio 2.0 Spec Table A-9
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

/// Audio Class-Specific AS Interface Descriptor Subtypes
/// USB Audio 2.0 Spec Table A-10
#[allow(dead_code)]
pub mod as_interface_descriptor_subtype {
    pub const AS_DESCRIPTOR_UNDEFINED: u8 = 0x00;
    pub const AS_GENERAL: u8 = 0x01;
    pub const FORMAT_TYPE: u8 = 0x02;
    pub const ENCODER: u8 = 0x03;
    pub const DECODER: u8 = 0x04;
}

/// Audio Class-Specific Endpoint Descriptor Subtypes
/// USB Audio 2.0 Spec Table A-13
#[allow(dead_code)]
pub mod endpoint_descriptor_subtypes {
    pub const DESCRIPTOR_UNDEFINED: u8 = 0x00;
    pub const EP_GENERAL: u8 = 0x01;
}

/// Audio Class-Specific Request Codes
/// USB Audio 2.0 Spec Table A-14
#[allow(dead_code)]
pub mod request_codes {
    pub const REQUEST_CODE_UNDEFINED: u8 = 0x00;
    pub const CUR: u8 = 0x01;
    pub const RANGE: u8 = 0x02;
    pub const MEM: u8 = 0x03;
}

/// Clock Source Control Selectors
/// Table A-17
#[allow(dead_code)]
pub mod clock_source_control_selectors {
    pub const CS_CONTROL_UNDEFINED: u8 = 0x00;
    pub const CS_SAM_FREQ_CONTROL: u8 = 0x01;
    pub const CS_CLOCK_VALID_CONTROL: u8 = 0x02;
}

/// Feature Unit Control Selectors
/// Table A-23
#[allow(dead_code)]
pub mod feature_unit_control_selector {
    pub const FU_CONTROL_UNDEFINED: u8 = 0x00;
    pub const FU_MUTE_CONTROL: u8 = 0x01;
    pub const FU_VOLUME_CONTROL: u8 = 0x02;
    // TODO add the rest
}

/// AudioStreaming Interface Control Selectors
/// Table A-32
#[allow(dead_code)]
pub mod audiostreaming_interface_control_selectors {
    pub const AS_CONTROL_UNDEFINED: u8 = 0x00;
    pub const AS_ACT_ALT_SETTING_CONTROL: u8 = 0x01;
    pub const AS_VAL_ALT_SETTINGS_CONTROL: u8 = 0x02;
    pub const AS_AUDIO_DATA_FORMAT_CONTROL: u8 = 0x03;
}

/// Endpoint Control Selectors
/// Table A-38
#[allow(dead_code)]
pub mod endpoint_control_selectors {
    pub const EP_CONTROL_UNDEFINED: u8 = 0x00;
    // TODO add the rest
}

#[allow(dead_code)]
pub mod terminal_type {
    pub const USB_UNDEFINED: u16 = 0x0100;
    pub const USB_STREAMING: u16 = 0x0101;
    pub const USB_VENDOR_SPECIFIC: u16 = 0x01FF;

    pub const INPUT_UNDEFINED: u16 = 0x0200;
    pub const INPUT_MICROPHONE: u16 = 0x0201;
    pub const INPUT_DESKTOP_MICROPHONE: u16 = 0x0202;
    pub const INPUT_PERSONAL_MICROPHONE: u16 = 0x0203;
    pub const INPUT_OMNIDIRECTIONAL_MICROPHONE: u16 = 0x0204;
    pub const INPUT_MICROPHONE_ARRAY: u16 = 0x0205;
    pub const INPUT_PROCESSING_MICROPHONE_ARRAY: u16 = 0x0206;

    pub const OUTPUT_UNDEFINED: u16 = 0x0300;
    pub const OUTPUT_SPEAKER: u16 = 0x0301;
    pub const OUTPUT_HEADPHONES: u16 = 0x0302;
    pub const OUTPUT_HEAD_MOUNTED_DISPLAY_AUDIO: u16 = 0x0303;
    pub const OUTPUT_DESKTOP_SPEAKER: u16 = 0x0304;
    pub const OUTPUT_ROOM_SPEAKER: u16 = 0x0305;
    pub const OUTPUT_COMMUNICATION_SPEAKER: u16 = 0x0306;
    pub const OUTPUT_LFE_SPEAKER: u16 = 0x0307;

    pub const BIDIRECTIONAL_UNDEFINED: u16 = 0x0400;
    pub const BIDIRECTIONAL_HANDSET: u16 = 0x0401;
    pub const BIDIRECTIONAL_HEADSET: u16 = 0x0402;
    pub const BIDIRECTIONAL_SPEAKERPHONE_NO_ECHO_REDUCTION: u16 = 0x0403;
    pub const BIDIRECTIONAL_SPEAKERPHONE_ECHO_SUPPRESSING: u16 = 0x0404;
    pub const BIDIRECTIONAL_SPEAKERPHONE_ECHO_CANCELING: u16 = 0x0405;

    pub const TELEPHONY_UNDEFINED: u16 = 0x0500;
    pub const TELEPHONY_PHONE_LINE: u16 = 0x0501;
    pub const TELEPHONY_TELEPHONE: u16 = 0x0502;
    pub const TELEPHONY_DOWN_LINE_PHONE: u16 = 0x0503;

    pub const EXTERNAL_UNDEFINED: u16 = 0x0600;
    pub const EXTERNAL_ANALOG_CONNECTOR: u16 = 0x0601;
    pub const EXTERNAL_DIGITAL_AUDIO_INTERFACE: u16 = 0x0602;
    pub const EXTERNAL_LINE_CONNECTOR: u16 = 0x0603;
    pub const EXTERNAL_LEGACY_AUDIO_CONNECTOR: u16 = 0x0604;
    pub const EXTERNAL_SPDIF_INTERFACE: u16 = 0x0605;
    pub const EXTERNAL_1394_DA_STREAM: u16 = 0x0606;
    pub const EXTERNAL_1394_DV_STREAM_SOUNDTRACK: u16 = 0x0607;
    pub const EXTERNAL_ADAT_LIGHTPIPE: u16 = 0x0608;
    pub const EXTERNAL_TDIF: u16 = 0x0609;
    pub const EXTERNAL_MADI: u16 = 0x060a;

    //TODO embedded function terminal types (0x07XX)
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub struct EntityId(u8);

impl From<EntityId> for u8 {
    fn from(n: EntityId) -> u8 {
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

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum ControlCapabilities {
    NotPresent = 0b00,
    ReadOnly = 0b01,
    HostProgrammable = 0b11,
}

pub struct InputTerminalControls {
    bitmap: u16,
}

impl InputTerminalControls {
    pub fn new() -> InputTerminalControls {
        InputTerminalControls { bitmap: 0 }
    }

    pub fn bitmap(&self) -> [u8; 2] {
        self.bitmap.to_le_bytes()
    }

    fn set_control(
        &mut self,
        bit_offset: usize,
        capabilities: ControlCapabilities,
    ) -> &mut InputTerminalControls {
        self.bitmap |= (capabilities as u16) << bit_offset;
        self
    }

    pub fn copy_protect(&mut self, capabilities: ControlCapabilities) -> &mut InputTerminalControls {
        self.set_control(0, capabilities)
    }

    pub fn connector(&mut self, capabilities: ControlCapabilities) -> &mut InputTerminalControls {
        self.set_control(2, capabilities)
    }

    pub fn overload(&mut self, capabilities: ControlCapabilities) -> &mut InputTerminalControls {
        self.set_control(4, capabilities)
    }

    pub fn cluster(&mut self, capabilities: ControlCapabilities) -> &mut InputTerminalControls {
        self.set_control(6, capabilities)
    }

    pub fn underflow(&mut self, capabilities: ControlCapabilities) -> &mut InputTerminalControls {
        self.set_control(8, capabilities)
    }

    pub fn overflow(&mut self, capabilities: ControlCapabilities) -> &mut InputTerminalControls {
        self.set_control(10, capabilities)
    }
}

pub struct FeatureUnitControls(u32);

impl FeatureUnitControls {
    pub fn new() -> FeatureUnitControls {
        FeatureUnitControls(0)
    }

    pub fn bitmap(&self) -> [u8; 4] {
        self.0.to_le_bytes()
    }

    fn set_control(
        &mut self,
        bit_offset: usize,
        capabilities: ControlCapabilities
    ) -> &mut Self {
        self.0 |= (capabilities as u32) << bit_offset;
        self
    }

    pub fn mute(&mut self, capabilities: ControlCapabilities) -> &mut Self {
        self.set_control(0, capabilities)
    }

    pub fn volume(&mut self, capabilities: ControlCapabilities) -> &mut Self {
        self.set_control(2, capabilities)
    }

    // TODO add the remaining controls from Table 4-13 offset 5
}

#[repr(u8)]
pub enum ClockType {
    External = 0b00,
    InternalFixed = 0b01,
    InternalVariable = 0b10,
    InternalProgrammable = 0b11,
}

pub struct ClockAttributes(u8);

impl ClockAttributes {
    pub fn build(clock_type: ClockType, sync_to_sof: bool) -> ClockAttributes {
        ClockAttributes(
            clock_type as u8 |
            (sync_to_sof as u8) << 2
        )
    }
}

pub struct ClockControls(u8);

impl ClockControls {
    pub fn new() -> ClockControls {
        ClockControls(0)
    }

    fn set_control(
        &mut self,
        bit_offset: usize,
        capabilities: ControlCapabilities,
    ) -> &mut Self {
        self.0 |= (capabilities as u8) << bit_offset;
        self
    }

    pub fn clock_frequency_control(&mut self, capabilities: ControlCapabilities) -> &mut ClockControls {
        self.set_control(0, capabilities)
    }

    pub fn clock_validity_control(&mut self, capabilities: ControlCapabilities) -> &mut ClockControls {
        self.set_control(2, capabilities)
    }
}

pub struct AudioControlInterfaceDescriptorWriter<'a> {
    buf: &'a mut [u8],
    position: usize,
    audio_class_version: u16,
    streaming_interfaces: &'a [InterfaceNumber],
    next_entity_id: u8,
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
            next_entity_id: 1,
        }
    }

    pub fn alloc_entity(&mut self) -> Result<EntityId> {
        if self.next_entity_id == 255 {
            return Err(UsbError::Unsupported);
        }

        let entity_id = EntityId(self.next_entity_id);
        self.next_entity_id += 1;
        Ok(entity_id)
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

    fn ac_interface_clock_source(
        &mut self,
        attributes: ClockAttributes,
        controls: ClockControls,
        assoc_terminal_id: Option<EntityId>,
    ) -> Result<EntityId> {
        let clock_id = self.alloc_entity()?;
        self.write(
            ac_interface_descriptor_subtype::CLOCK_SOURCE,
            &[
                clock_id.into(),
                attributes.0,
                controls.0,
                assoc_terminal_id.map_or(0, Into::into),
                0,
            ]
        )?;
        Ok(clock_id)
    }

    fn ac_interface_clock_selector(&mut self) {
        todo!()
    }

    fn ac_interface_clock_multiplier(&mut self) {
        todo!()
    }

    fn ac_interface_input_terminal(
        &mut self,
        terminal_type: [u8; 2],
        assoc_terminal_id: Option<EntityId>,
        clock_source_id: Option<EntityId>,
        channels: ChannelConfig,
        controls: InputTerminalControls,
        name: Option<StringIndex>,
    ) -> Result<EntityId> {
        let terminal_id = self.alloc_entity()?;
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
        assoc_terminal_id: Option<EntityId>,
        source_id: EntityId,
        clock_source_id: EntityId,
        // TODO make this into a controls type
        controls: u8,
        name: Option<StringIndex>,
    ) -> Result<EntityId> {
        let terminal_id = self.alloc_entity()?;
        self.write(
            ac_interface_descriptor_subtype::OUTPUT_TERMINAL,
            &[
                terminal_id.into(),
                terminal_type[0],
                terminal_type[1],
                assoc_terminal_id.map_or(0, |id| id.into()),
                source_id.into(),
                clock_source_id.into(),
                controls,
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
        source_id: EntityId,
        master_channel_controls: FeatureUnitControls,
        //logical_channel_controls: &[FeatureUnitControls],
        name: Option<StringIndex>,
    ) -> Result<EntityId> {
        let unit_id = self.alloc_entity()?;
        let master_controls: [u8; 4] = master_channel_controls.bitmap();
        let mut buf: [u8; 2 + 4 + 4 * 2 + 1] = [0; 2 + 4 + 4 * 2 + 1];
        buf[0] = unit_id.into();
        buf[1] = source_id.into();
        buf[2..6].copy_from_slice(&master_controls);
        // TODO don't assume two channels without logical channel controls
        buf[6..10].copy_from_slice(&[0,0,0,0]);
        buf[10..14].copy_from_slice(&[0,0,0,0]);
        buf[14] = name.map_or(0, Into::into);
        self.write(
            ac_interface_descriptor_subtype::FEATURE_UNIT,
            &buf,
        )?;
        Ok(unit_id)
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
