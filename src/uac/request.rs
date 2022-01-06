use core::convert::TryFrom;

use defmt::Format;
use usb_device::{UsbDirection, control::{Recipient, Request}, UsbError, class_prelude::InterfaceNumber};

use super::descriptors::{request_codes::{CUR, RANGE, MEM}, EntityId};

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Format)]
pub enum GetOrSet {
    Set = 0b00000000,
    Get = 0b10000000,
}

impl From<UsbDirection> for GetOrSet {
    fn from(dir: UsbDirection) -> Self {
        match dir {
            UsbDirection::In => GetOrSet::Get,
            UsbDirection::Out => GetOrSet::Set,
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Format)]
pub enum Target {
    Interface(u8, Option<u8>),
    Endpoint(u8),
}

impl Target {
    pub fn from(recipient: Recipient, index: u16) -> Result<Target, UsbError> {
        let index_bytes = index.to_le_bytes();
        match recipient {
            Recipient::Interface => {
                let entity_id = if index_bytes[1] == 0 { None } else { Some(index_bytes[1]) };
                Ok(Target::Interface(index_bytes[0], entity_id))
            },
            Recipient::Endpoint => {
                Ok(Target::Endpoint(index_bytes[0]))
            },
            _ => { Err(UsbError::ParseError) }
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Format)]
pub enum RequestCode {
    Cur = super::descriptors::request_codes::CUR,
    Range = super::descriptors::request_codes::RANGE,
    Mem = super::descriptors::request_codes::MEM,
}

impl TryFrom<u8> for RequestCode {
    type Error = UsbError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            CUR => Ok(RequestCode::Cur),
            RANGE => Ok(RequestCode::Range),
            MEM => Ok(RequestCode::Mem),
            _ => Err(UsbError::ParseError),
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Format)]
pub struct ControlRequest {
    pub direction: GetOrSet,
    pub target: Target,
    pub request_code: RequestCode,
    pub control_selector: u8,
    pub channel_number: u8,
    pub length: u16,
}

impl ControlRequest {
    pub fn parse(request: &Request) -> Result<ControlRequest, UsbError> {
        let direction = GetOrSet::from(request.direction);
        let target = Target::from(request.recipient, request.index)?;
        let request_code = RequestCode::try_from(request.request)?;
        let control_selector = (request.value >> 8) as u8;
        let channel_number = (request.value & 0xff) as u8;

        Ok(ControlRequest {
            direction,
            target,
            request_code,
            control_selector,
            channel_number,
            length: request.length,
        })
    }
}
