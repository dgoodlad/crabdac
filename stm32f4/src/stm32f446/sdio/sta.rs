#[doc = "Register `STA` reader"]
pub struct R(crate::R<STA_SPEC>);
impl core::ops::Deref for R {
    type Target = crate::R<STA_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<crate::R<STA_SPEC>> for R {
    #[inline(always)]
    fn from(reader: crate::R<STA_SPEC>) -> Self {
        R(reader)
    }
}
#[doc = "Command response received (CRC check failed). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CCRCFAIL_A {
    #[doc = "0: Command response received, crc check passed"]
    NOTFAILED = 0,
    #[doc = "1: Command response received, crc check failed"]
    FAILED = 1,
}
impl From<CCRCFAIL_A> for bool {
    #[inline(always)]
    fn from(variant: CCRCFAIL_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `CCRCFAIL` reader - Command response received (CRC check failed). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct CCRCFAIL_R(crate::FieldReader<bool, CCRCFAIL_A>);
impl CCRCFAIL_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        CCRCFAIL_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> CCRCFAIL_A {
        match self.bits {
            false => CCRCFAIL_A::NOTFAILED,
            true => CCRCFAIL_A::FAILED,
        }
    }
    #[doc = "Checks if the value of the field is `NOTFAILED`"]
    #[inline(always)]
    pub fn is_not_failed(&self) -> bool {
        **self == CCRCFAIL_A::NOTFAILED
    }
    #[doc = "Checks if the value of the field is `FAILED`"]
    #[inline(always)]
    pub fn is_failed(&self) -> bool {
        **self == CCRCFAIL_A::FAILED
    }
}
impl core::ops::Deref for CCRCFAIL_R {
    type Target = crate::FieldReader<bool, CCRCFAIL_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Data block sent/received (CRC check failed). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DCRCFAIL_A {
    #[doc = "0: No Data block sent/received crc check fail"]
    NOTFAILED = 0,
    #[doc = "1: Data block sent/received crc failed"]
    FAILED = 1,
}
impl From<DCRCFAIL_A> for bool {
    #[inline(always)]
    fn from(variant: DCRCFAIL_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `DCRCFAIL` reader - Data block sent/received (CRC check failed). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct DCRCFAIL_R(crate::FieldReader<bool, DCRCFAIL_A>);
impl DCRCFAIL_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        DCRCFAIL_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> DCRCFAIL_A {
        match self.bits {
            false => DCRCFAIL_A::NOTFAILED,
            true => DCRCFAIL_A::FAILED,
        }
    }
    #[doc = "Checks if the value of the field is `NOTFAILED`"]
    #[inline(always)]
    pub fn is_not_failed(&self) -> bool {
        **self == DCRCFAIL_A::NOTFAILED
    }
    #[doc = "Checks if the value of the field is `FAILED`"]
    #[inline(always)]
    pub fn is_failed(&self) -> bool {
        **self == DCRCFAIL_A::FAILED
    }
}
impl core::ops::Deref for DCRCFAIL_R {
    type Target = crate::FieldReader<bool, DCRCFAIL_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Command response timeout. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. The Command Timeout period has a fixed value of 64 SDMMC_CK clock periods.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CTIMEOUT_A {
    #[doc = "0: No Command timeout"]
    NOTIMEOUT = 0,
    #[doc = "1: Command timeout"]
    TIMEOUT = 1,
}
impl From<CTIMEOUT_A> for bool {
    #[inline(always)]
    fn from(variant: CTIMEOUT_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `CTIMEOUT` reader - Command response timeout. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. The Command Timeout period has a fixed value of 64 SDMMC_CK clock periods."]
pub struct CTIMEOUT_R(crate::FieldReader<bool, CTIMEOUT_A>);
impl CTIMEOUT_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        CTIMEOUT_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> CTIMEOUT_A {
        match self.bits {
            false => CTIMEOUT_A::NOTIMEOUT,
            true => CTIMEOUT_A::TIMEOUT,
        }
    }
    #[doc = "Checks if the value of the field is `NOTIMEOUT`"]
    #[inline(always)]
    pub fn is_no_timeout(&self) -> bool {
        **self == CTIMEOUT_A::NOTIMEOUT
    }
    #[doc = "Checks if the value of the field is `TIMEOUT`"]
    #[inline(always)]
    pub fn is_timeout(&self) -> bool {
        **self == CTIMEOUT_A::TIMEOUT
    }
}
impl core::ops::Deref for CTIMEOUT_R {
    type Target = crate::FieldReader<bool, CTIMEOUT_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Data timeout. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DTIMEOUT_A {
    #[doc = "0: No data timeout"]
    NOTIMEOUT = 0,
    #[doc = "1: Data timeout"]
    TIMEOUT = 1,
}
impl From<DTIMEOUT_A> for bool {
    #[inline(always)]
    fn from(variant: DTIMEOUT_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `DTIMEOUT` reader - Data timeout. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct DTIMEOUT_R(crate::FieldReader<bool, DTIMEOUT_A>);
impl DTIMEOUT_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        DTIMEOUT_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> DTIMEOUT_A {
        match self.bits {
            false => DTIMEOUT_A::NOTIMEOUT,
            true => DTIMEOUT_A::TIMEOUT,
        }
    }
    #[doc = "Checks if the value of the field is `NOTIMEOUT`"]
    #[inline(always)]
    pub fn is_no_timeout(&self) -> bool {
        **self == DTIMEOUT_A::NOTIMEOUT
    }
    #[doc = "Checks if the value of the field is `TIMEOUT`"]
    #[inline(always)]
    pub fn is_timeout(&self) -> bool {
        **self == DTIMEOUT_A::TIMEOUT
    }
}
impl core::ops::Deref for DTIMEOUT_R {
    type Target = crate::FieldReader<bool, DTIMEOUT_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Transmit FIFO underrun error or IDMA read transfer error. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TXUNDERR_A {
    #[doc = "0: No transmit FIFO underrun error"]
    NOUNDERRUN = 0,
    #[doc = "1: Transmit FIFO underrun error"]
    UNDERRUN = 1,
}
impl From<TXUNDERR_A> for bool {
    #[inline(always)]
    fn from(variant: TXUNDERR_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `TXUNDERR` reader - Transmit FIFO underrun error or IDMA read transfer error. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct TXUNDERR_R(crate::FieldReader<bool, TXUNDERR_A>);
impl TXUNDERR_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        TXUNDERR_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> TXUNDERR_A {
        match self.bits {
            false => TXUNDERR_A::NOUNDERRUN,
            true => TXUNDERR_A::UNDERRUN,
        }
    }
    #[doc = "Checks if the value of the field is `NOUNDERRUN`"]
    #[inline(always)]
    pub fn is_no_underrun(&self) -> bool {
        **self == TXUNDERR_A::NOUNDERRUN
    }
    #[doc = "Checks if the value of the field is `UNDERRUN`"]
    #[inline(always)]
    pub fn is_underrun(&self) -> bool {
        **self == TXUNDERR_A::UNDERRUN
    }
}
impl core::ops::Deref for TXUNDERR_R {
    type Target = crate::FieldReader<bool, TXUNDERR_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Received FIFO overrun error or IDMA write transfer error. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RXOVERR_A {
    #[doc = "0: No FIFO overrun error"]
    NOOVERRUN = 0,
    #[doc = "1: Receive FIFO overrun error"]
    OVERRUN = 1,
}
impl From<RXOVERR_A> for bool {
    #[inline(always)]
    fn from(variant: RXOVERR_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `RXOVERR` reader - Received FIFO overrun error or IDMA write transfer error. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct RXOVERR_R(crate::FieldReader<bool, RXOVERR_A>);
impl RXOVERR_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        RXOVERR_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> RXOVERR_A {
        match self.bits {
            false => RXOVERR_A::NOOVERRUN,
            true => RXOVERR_A::OVERRUN,
        }
    }
    #[doc = "Checks if the value of the field is `NOOVERRUN`"]
    #[inline(always)]
    pub fn is_no_overrun(&self) -> bool {
        **self == RXOVERR_A::NOOVERRUN
    }
    #[doc = "Checks if the value of the field is `OVERRUN`"]
    #[inline(always)]
    pub fn is_overrun(&self) -> bool {
        **self == RXOVERR_A::OVERRUN
    }
}
impl core::ops::Deref for RXOVERR_R {
    type Target = crate::FieldReader<bool, RXOVERR_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Command response received (CRC check passed, or no CRC). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CMDREND_A {
    #[doc = "0: Command not done"]
    NOTDONE = 0,
    #[doc = "1: Command response received (CRC check passed)"]
    DONE = 1,
}
impl From<CMDREND_A> for bool {
    #[inline(always)]
    fn from(variant: CMDREND_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `CMDREND` reader - Command response received (CRC check passed, or no CRC). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct CMDREND_R(crate::FieldReader<bool, CMDREND_A>);
impl CMDREND_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        CMDREND_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> CMDREND_A {
        match self.bits {
            false => CMDREND_A::NOTDONE,
            true => CMDREND_A::DONE,
        }
    }
    #[doc = "Checks if the value of the field is `NOTDONE`"]
    #[inline(always)]
    pub fn is_not_done(&self) -> bool {
        **self == CMDREND_A::NOTDONE
    }
    #[doc = "Checks if the value of the field is `DONE`"]
    #[inline(always)]
    pub fn is_done(&self) -> bool {
        **self == CMDREND_A::DONE
    }
}
impl core::ops::Deref for CMDREND_R {
    type Target = crate::FieldReader<bool, CMDREND_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Command sent (no response required). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CMDSENT_A {
    #[doc = "0: Command not sent"]
    NOTSENT = 0,
    #[doc = "1: Command sent (no response required)"]
    SENT = 1,
}
impl From<CMDSENT_A> for bool {
    #[inline(always)]
    fn from(variant: CMDSENT_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `CMDSENT` reader - Command sent (no response required). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct CMDSENT_R(crate::FieldReader<bool, CMDSENT_A>);
impl CMDSENT_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        CMDSENT_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> CMDSENT_A {
        match self.bits {
            false => CMDSENT_A::NOTSENT,
            true => CMDSENT_A::SENT,
        }
    }
    #[doc = "Checks if the value of the field is `NOTSENT`"]
    #[inline(always)]
    pub fn is_not_sent(&self) -> bool {
        **self == CMDSENT_A::NOTSENT
    }
    #[doc = "Checks if the value of the field is `SENT`"]
    #[inline(always)]
    pub fn is_sent(&self) -> bool {
        **self == CMDSENT_A::SENT
    }
}
impl core::ops::Deref for CMDSENT_R {
    type Target = crate::FieldReader<bool, CMDSENT_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Data transfer ended correctly. (data counter, DATACOUNT is zero and no errors occur). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DATAEND_A {
    #[doc = "1: Data end (DCOUNT, is zero)"]
    DONE = 1,
    #[doc = "0: Not done"]
    NOTDONE = 0,
}
impl From<DATAEND_A> for bool {
    #[inline(always)]
    fn from(variant: DATAEND_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `DATAEND` reader - Data transfer ended correctly. (data counter, DATACOUNT is zero and no errors occur). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct DATAEND_R(crate::FieldReader<bool, DATAEND_A>);
impl DATAEND_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        DATAEND_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> DATAEND_A {
        match self.bits {
            true => DATAEND_A::DONE,
            false => DATAEND_A::NOTDONE,
        }
    }
    #[doc = "Checks if the value of the field is `DONE`"]
    #[inline(always)]
    pub fn is_done(&self) -> bool {
        **self == DATAEND_A::DONE
    }
    #[doc = "Checks if the value of the field is `NOTDONE`"]
    #[inline(always)]
    pub fn is_not_done(&self) -> bool {
        **self == DATAEND_A::NOTDONE
    }
}
impl core::ops::Deref for DATAEND_R {
    type Target = crate::FieldReader<bool, DATAEND_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Data block sent/received. (CRC check passed) and DPSM moves to the READWAIT state. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DBCKEND_A {
    #[doc = "0: Data block not sent/received (CRC check failed)"]
    NOTTRANSFERRED = 0,
    #[doc = "1: Data block sent/received (CRC check passed)"]
    TRANSFERRED = 1,
}
impl From<DBCKEND_A> for bool {
    #[inline(always)]
    fn from(variant: DBCKEND_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `DBCKEND` reader - Data block sent/received. (CRC check passed) and DPSM moves to the READWAIT state. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct DBCKEND_R(crate::FieldReader<bool, DBCKEND_A>);
impl DBCKEND_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        DBCKEND_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> DBCKEND_A {
        match self.bits {
            false => DBCKEND_A::NOTTRANSFERRED,
            true => DBCKEND_A::TRANSFERRED,
        }
    }
    #[doc = "Checks if the value of the field is `NOTTRANSFERRED`"]
    #[inline(always)]
    pub fn is_not_transferred(&self) -> bool {
        **self == DBCKEND_A::NOTTRANSFERRED
    }
    #[doc = "Checks if the value of the field is `TRANSFERRED`"]
    #[inline(always)]
    pub fn is_transferred(&self) -> bool {
        **self == DBCKEND_A::TRANSFERRED
    }
}
impl core::ops::Deref for DBCKEND_R {
    type Target = crate::FieldReader<bool, DBCKEND_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Data transfer aborted by CMD12. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CMDACT_A {
    #[doc = "0: Command transfer not in progress"]
    NOTINPROGRESS = 0,
    #[doc = "1: Command tranfer in progress"]
    INPROGRESS = 1,
}
impl From<CMDACT_A> for bool {
    #[inline(always)]
    fn from(variant: CMDACT_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `CMDACT` reader - Data transfer aborted by CMD12. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct CMDACT_R(crate::FieldReader<bool, CMDACT_A>);
impl CMDACT_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        CMDACT_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> CMDACT_A {
        match self.bits {
            false => CMDACT_A::NOTINPROGRESS,
            true => CMDACT_A::INPROGRESS,
        }
    }
    #[doc = "Checks if the value of the field is `NOTINPROGRESS`"]
    #[inline(always)]
    pub fn is_not_in_progress(&self) -> bool {
        **self == CMDACT_A::NOTINPROGRESS
    }
    #[doc = "Checks if the value of the field is `INPROGRESS`"]
    #[inline(always)]
    pub fn is_in_progress(&self) -> bool {
        **self == CMDACT_A::INPROGRESS
    }
}
impl core::ops::Deref for CMDACT_R {
    type Target = crate::FieldReader<bool, CMDACT_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Data path state machine active, i.e. not in Idle state. This is a hardware status flag only, does not generate an interrupt.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TXACT_A {
    #[doc = "0: Data transmit is not in progress"]
    NOTINPROGRESS = 0,
    #[doc = "1: Data transmit in progress"]
    INPROGRESS = 1,
}
impl From<TXACT_A> for bool {
    #[inline(always)]
    fn from(variant: TXACT_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `TXACT` reader - Data path state machine active, i.e. not in Idle state. This is a hardware status flag only, does not generate an interrupt."]
pub struct TXACT_R(crate::FieldReader<bool, TXACT_A>);
impl TXACT_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        TXACT_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> TXACT_A {
        match self.bits {
            false => TXACT_A::NOTINPROGRESS,
            true => TXACT_A::INPROGRESS,
        }
    }
    #[doc = "Checks if the value of the field is `NOTINPROGRESS`"]
    #[inline(always)]
    pub fn is_not_in_progress(&self) -> bool {
        **self == TXACT_A::NOTINPROGRESS
    }
    #[doc = "Checks if the value of the field is `INPROGRESS`"]
    #[inline(always)]
    pub fn is_in_progress(&self) -> bool {
        **self == TXACT_A::INPROGRESS
    }
}
impl core::ops::Deref for TXACT_R {
    type Target = crate::FieldReader<bool, TXACT_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Command path state machine active, i.e. not in Idle state. This is a hardware status flag only, does not generate an interrupt.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RXACT_A {
    #[doc = "0: Data receive not in progress"]
    NOTINPROGRESS = 0,
    #[doc = "1: Data receive in progress"]
    INPROGRESS = 1,
}
impl From<RXACT_A> for bool {
    #[inline(always)]
    fn from(variant: RXACT_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `RXACT` reader - Command path state machine active, i.e. not in Idle state. This is a hardware status flag only, does not generate an interrupt."]
pub struct RXACT_R(crate::FieldReader<bool, RXACT_A>);
impl RXACT_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        RXACT_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> RXACT_A {
        match self.bits {
            false => RXACT_A::NOTINPROGRESS,
            true => RXACT_A::INPROGRESS,
        }
    }
    #[doc = "Checks if the value of the field is `NOTINPROGRESS`"]
    #[inline(always)]
    pub fn is_not_in_progress(&self) -> bool {
        **self == RXACT_A::NOTINPROGRESS
    }
    #[doc = "Checks if the value of the field is `INPROGRESS`"]
    #[inline(always)]
    pub fn is_in_progress(&self) -> bool {
        **self == RXACT_A::INPROGRESS
    }
}
impl core::ops::Deref for RXACT_R {
    type Target = crate::FieldReader<bool, RXACT_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Transmit FIFO half empty At least half the number of words can be written into the FIFO. This bit is cleared when the FIFO becomes half+1 full.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TXFIFOHE_A {
    #[doc = "0: Transmit FIFO not half empty"]
    NOTHALFEMPTY = 0,
    #[doc = "1: Transmit FIFO half empty. At least 8 words can be written into the FIFO"]
    HALFEMPTY = 1,
}
impl From<TXFIFOHE_A> for bool {
    #[inline(always)]
    fn from(variant: TXFIFOHE_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `TXFIFOHE` reader - Transmit FIFO half empty At least half the number of words can be written into the FIFO. This bit is cleared when the FIFO becomes half+1 full."]
pub struct TXFIFOHE_R(crate::FieldReader<bool, TXFIFOHE_A>);
impl TXFIFOHE_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        TXFIFOHE_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> TXFIFOHE_A {
        match self.bits {
            false => TXFIFOHE_A::NOTHALFEMPTY,
            true => TXFIFOHE_A::HALFEMPTY,
        }
    }
    #[doc = "Checks if the value of the field is `NOTHALFEMPTY`"]
    #[inline(always)]
    pub fn is_not_half_empty(&self) -> bool {
        **self == TXFIFOHE_A::NOTHALFEMPTY
    }
    #[doc = "Checks if the value of the field is `HALFEMPTY`"]
    #[inline(always)]
    pub fn is_half_empty(&self) -> bool {
        **self == TXFIFOHE_A::HALFEMPTY
    }
}
impl core::ops::Deref for TXFIFOHE_R {
    type Target = crate::FieldReader<bool, TXFIFOHE_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Receive FIFO half full There are at least half the number of words in the FIFO. This bit is cleared when the FIFO becomes half+1 empty.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RXFIFOHF_A {
    #[doc = "0: Receive FIFO not half full"]
    NOTHALFFULL = 0,
    #[doc = "1: Receive FIFO half full. At least 8 words in the FIFO"]
    HALFFULL = 1,
}
impl From<RXFIFOHF_A> for bool {
    #[inline(always)]
    fn from(variant: RXFIFOHF_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `RXFIFOHF` reader - Receive FIFO half full There are at least half the number of words in the FIFO. This bit is cleared when the FIFO becomes half+1 empty."]
pub struct RXFIFOHF_R(crate::FieldReader<bool, RXFIFOHF_A>);
impl RXFIFOHF_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        RXFIFOHF_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> RXFIFOHF_A {
        match self.bits {
            false => RXFIFOHF_A::NOTHALFFULL,
            true => RXFIFOHF_A::HALFFULL,
        }
    }
    #[doc = "Checks if the value of the field is `NOTHALFFULL`"]
    #[inline(always)]
    pub fn is_not_half_full(&self) -> bool {
        **self == RXFIFOHF_A::NOTHALFFULL
    }
    #[doc = "Checks if the value of the field is `HALFFULL`"]
    #[inline(always)]
    pub fn is_half_full(&self) -> bool {
        **self == RXFIFOHF_A::HALFFULL
    }
}
impl core::ops::Deref for RXFIFOHF_R {
    type Target = crate::FieldReader<bool, RXFIFOHF_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Transmit FIFO full This is a hardware status flag only, does not generate an interrupt. This bit is cleared when one FIFO location becomes empty.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TXFIFOF_A {
    #[doc = "0: Transmit FIFO not full"]
    NOTFULL = 0,
    #[doc = "1: Transmit FIFO full"]
    FULL = 1,
}
impl From<TXFIFOF_A> for bool {
    #[inline(always)]
    fn from(variant: TXFIFOF_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `TXFIFOF` reader - Transmit FIFO full This is a hardware status flag only, does not generate an interrupt. This bit is cleared when one FIFO location becomes empty."]
pub struct TXFIFOF_R(crate::FieldReader<bool, TXFIFOF_A>);
impl TXFIFOF_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        TXFIFOF_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> TXFIFOF_A {
        match self.bits {
            false => TXFIFOF_A::NOTFULL,
            true => TXFIFOF_A::FULL,
        }
    }
    #[doc = "Checks if the value of the field is `NOTFULL`"]
    #[inline(always)]
    pub fn is_not_full(&self) -> bool {
        **self == TXFIFOF_A::NOTFULL
    }
    #[doc = "Checks if the value of the field is `FULL`"]
    #[inline(always)]
    pub fn is_full(&self) -> bool {
        **self == TXFIFOF_A::FULL
    }
}
impl core::ops::Deref for TXFIFOF_R {
    type Target = crate::FieldReader<bool, TXFIFOF_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Receive FIFO full This bit is cleared when one FIFO location becomes empty.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RXFIFOF_A {
    #[doc = "0: Transmit FIFO not full"]
    NOTFULL = 0,
    #[doc = "1: Receive FIFO full. When HW Flow Control is enabled, RXFIFOF signals becomes activated 2 words before the FIFO is full."]
    FULL = 1,
}
impl From<RXFIFOF_A> for bool {
    #[inline(always)]
    fn from(variant: RXFIFOF_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `RXFIFOF` reader - Receive FIFO full This bit is cleared when one FIFO location becomes empty."]
pub struct RXFIFOF_R(crate::FieldReader<bool, RXFIFOF_A>);
impl RXFIFOF_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        RXFIFOF_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> RXFIFOF_A {
        match self.bits {
            false => RXFIFOF_A::NOTFULL,
            true => RXFIFOF_A::FULL,
        }
    }
    #[doc = "Checks if the value of the field is `NOTFULL`"]
    #[inline(always)]
    pub fn is_not_full(&self) -> bool {
        **self == RXFIFOF_A::NOTFULL
    }
    #[doc = "Checks if the value of the field is `FULL`"]
    #[inline(always)]
    pub fn is_full(&self) -> bool {
        **self == RXFIFOF_A::FULL
    }
}
impl core::ops::Deref for RXFIFOF_R {
    type Target = crate::FieldReader<bool, RXFIFOF_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Transmit FIFO empty This bit is cleared when one FIFO location becomes full.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TXFIFOE_A {
    #[doc = "0: Transmit FIFO not empty"]
    NOTEMPTY = 0,
    #[doc = "1: Transmit FIFO empty. When HW Flow Control is enabled, TXFIFOE signals becomes activated when the FIFO contains 2 words."]
    EMPTY = 1,
}
impl From<TXFIFOE_A> for bool {
    #[inline(always)]
    fn from(variant: TXFIFOE_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `TXFIFOE` reader - Transmit FIFO empty This bit is cleared when one FIFO location becomes full."]
pub struct TXFIFOE_R(crate::FieldReader<bool, TXFIFOE_A>);
impl TXFIFOE_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        TXFIFOE_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> TXFIFOE_A {
        match self.bits {
            false => TXFIFOE_A::NOTEMPTY,
            true => TXFIFOE_A::EMPTY,
        }
    }
    #[doc = "Checks if the value of the field is `NOTEMPTY`"]
    #[inline(always)]
    pub fn is_not_empty(&self) -> bool {
        **self == TXFIFOE_A::NOTEMPTY
    }
    #[doc = "Checks if the value of the field is `EMPTY`"]
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        **self == TXFIFOE_A::EMPTY
    }
}
impl core::ops::Deref for TXFIFOE_R {
    type Target = crate::FieldReader<bool, TXFIFOE_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Receive FIFO empty This is a hardware status flag only, does not generate an interrupt. This bit is cleared when one FIFO location becomes full.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RXFIFOE_A {
    #[doc = "0: Receive FIFO not empty"]
    NOTEMPTY = 0,
    #[doc = "1: Receive FIFO empty"]
    EMPTY = 1,
}
impl From<RXFIFOE_A> for bool {
    #[inline(always)]
    fn from(variant: RXFIFOE_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `RXFIFOE` reader - Receive FIFO empty This is a hardware status flag only, does not generate an interrupt. This bit is cleared when one FIFO location becomes full."]
pub struct RXFIFOE_R(crate::FieldReader<bool, RXFIFOE_A>);
impl RXFIFOE_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        RXFIFOE_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> RXFIFOE_A {
        match self.bits {
            false => RXFIFOE_A::NOTEMPTY,
            true => RXFIFOE_A::EMPTY,
        }
    }
    #[doc = "Checks if the value of the field is `NOTEMPTY`"]
    #[inline(always)]
    pub fn is_not_empty(&self) -> bool {
        **self == RXFIFOE_A::NOTEMPTY
    }
    #[doc = "Checks if the value of the field is `EMPTY`"]
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        **self == RXFIFOE_A::EMPTY
    }
}
impl core::ops::Deref for RXFIFOE_R {
    type Target = crate::FieldReader<bool, RXFIFOE_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Inverted value of SDMMC_D0 line (Busy), sampled at the end of a CMD response and a second time 2 SDMMC_CK cycles after the CMD response. This bit is reset to not busy when the SDMMCD0 line changes from busy to not busy. This bit does not signal busy due to data transfer. This is a hardware status flag only, it does not generate an interrupt.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TXDAVL_A {
    #[doc = "0: Data not available in transmit FIFO"]
    NOTAVAILABLE = 0,
    #[doc = "1: Data available in transmit FIFO"]
    AVAILABLE = 1,
}
impl From<TXDAVL_A> for bool {
    #[inline(always)]
    fn from(variant: TXDAVL_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `TXDAVL` reader - Inverted value of SDMMC_D0 line (Busy), sampled at the end of a CMD response and a second time 2 SDMMC_CK cycles after the CMD response. This bit is reset to not busy when the SDMMCD0 line changes from busy to not busy. This bit does not signal busy due to data transfer. This is a hardware status flag only, it does not generate an interrupt."]
pub struct TXDAVL_R(crate::FieldReader<bool, TXDAVL_A>);
impl TXDAVL_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        TXDAVL_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> TXDAVL_A {
        match self.bits {
            false => TXDAVL_A::NOTAVAILABLE,
            true => TXDAVL_A::AVAILABLE,
        }
    }
    #[doc = "Checks if the value of the field is `NOTAVAILABLE`"]
    #[inline(always)]
    pub fn is_not_available(&self) -> bool {
        **self == TXDAVL_A::NOTAVAILABLE
    }
    #[doc = "Checks if the value of the field is `AVAILABLE`"]
    #[inline(always)]
    pub fn is_available(&self) -> bool {
        **self == TXDAVL_A::AVAILABLE
    }
}
impl core::ops::Deref for TXDAVL_R {
    type Target = crate::FieldReader<bool, TXDAVL_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "end of SDMMC_D0 Busy following a CMD response detected. This indicates only end of busy following a CMD response. This bit does not signal busy due to data transfer. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RXDAVL_A {
    #[doc = "0: Data not available in receive FIFO"]
    NOTAVAILABLE = 0,
    #[doc = "1: Data available in receive FIFO"]
    AVAILABLE = 1,
}
impl From<RXDAVL_A> for bool {
    #[inline(always)]
    fn from(variant: RXDAVL_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `RXDAVL` reader - end of SDMMC_D0 Busy following a CMD response detected. This indicates only end of busy following a CMD response. This bit does not signal busy due to data transfer. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct RXDAVL_R(crate::FieldReader<bool, RXDAVL_A>);
impl RXDAVL_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        RXDAVL_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> RXDAVL_A {
        match self.bits {
            false => RXDAVL_A::NOTAVAILABLE,
            true => RXDAVL_A::AVAILABLE,
        }
    }
    #[doc = "Checks if the value of the field is `NOTAVAILABLE`"]
    #[inline(always)]
    pub fn is_not_available(&self) -> bool {
        **self == RXDAVL_A::NOTAVAILABLE
    }
    #[doc = "Checks if the value of the field is `AVAILABLE`"]
    #[inline(always)]
    pub fn is_available(&self) -> bool {
        **self == RXDAVL_A::AVAILABLE
    }
}
impl core::ops::Deref for RXDAVL_R {
    type Target = crate::FieldReader<bool, RXDAVL_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "SDIO interrupt received. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR.\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SDIOIT_A {
    #[doc = "0: SDIO interrupt not receieved"]
    NOTRECEIVED = 0,
    #[doc = "1: SDIO interrupt received"]
    RECEIVED = 1,
}
impl From<SDIOIT_A> for bool {
    #[inline(always)]
    fn from(variant: SDIOIT_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `SDIOIT` reader - SDIO interrupt received. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
pub struct SDIOIT_R(crate::FieldReader<bool, SDIOIT_A>);
impl SDIOIT_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        SDIOIT_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> SDIOIT_A {
        match self.bits {
            false => SDIOIT_A::NOTRECEIVED,
            true => SDIOIT_A::RECEIVED,
        }
    }
    #[doc = "Checks if the value of the field is `NOTRECEIVED`"]
    #[inline(always)]
    pub fn is_not_received(&self) -> bool {
        **self == SDIOIT_A::NOTRECEIVED
    }
    #[doc = "Checks if the value of the field is `RECEIVED`"]
    #[inline(always)]
    pub fn is_received(&self) -> bool {
        **self == SDIOIT_A::RECEIVED
    }
}
impl core::ops::Deref for SDIOIT_R {
    type Target = crate::FieldReader<bool, SDIOIT_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl R {
    #[doc = "Bit 0 - Command response received (CRC check failed). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn ccrcfail(&self) -> CCRCFAIL_R {
        CCRCFAIL_R::new((self.bits & 0x01) != 0)
    }
    #[doc = "Bit 1 - Data block sent/received (CRC check failed). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn dcrcfail(&self) -> DCRCFAIL_R {
        DCRCFAIL_R::new(((self.bits >> 1) & 0x01) != 0)
    }
    #[doc = "Bit 2 - Command response timeout. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR. The Command Timeout period has a fixed value of 64 SDMMC_CK clock periods."]
    #[inline(always)]
    pub fn ctimeout(&self) -> CTIMEOUT_R {
        CTIMEOUT_R::new(((self.bits >> 2) & 0x01) != 0)
    }
    #[doc = "Bit 3 - Data timeout. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn dtimeout(&self) -> DTIMEOUT_R {
        DTIMEOUT_R::new(((self.bits >> 3) & 0x01) != 0)
    }
    #[doc = "Bit 4 - Transmit FIFO underrun error or IDMA read transfer error. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn txunderr(&self) -> TXUNDERR_R {
        TXUNDERR_R::new(((self.bits >> 4) & 0x01) != 0)
    }
    #[doc = "Bit 5 - Received FIFO overrun error or IDMA write transfer error. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn rxoverr(&self) -> RXOVERR_R {
        RXOVERR_R::new(((self.bits >> 5) & 0x01) != 0)
    }
    #[doc = "Bit 6 - Command response received (CRC check passed, or no CRC). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn cmdrend(&self) -> CMDREND_R {
        CMDREND_R::new(((self.bits >> 6) & 0x01) != 0)
    }
    #[doc = "Bit 7 - Command sent (no response required). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn cmdsent(&self) -> CMDSENT_R {
        CMDSENT_R::new(((self.bits >> 7) & 0x01) != 0)
    }
    #[doc = "Bit 8 - Data transfer ended correctly. (data counter, DATACOUNT is zero and no errors occur). Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn dataend(&self) -> DATAEND_R {
        DATAEND_R::new(((self.bits >> 8) & 0x01) != 0)
    }
    #[doc = "Bit 10 - Data block sent/received. (CRC check passed) and DPSM moves to the READWAIT state. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn dbckend(&self) -> DBCKEND_R {
        DBCKEND_R::new(((self.bits >> 10) & 0x01) != 0)
    }
    #[doc = "Bit 11 - Data transfer aborted by CMD12. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn cmdact(&self) -> CMDACT_R {
        CMDACT_R::new(((self.bits >> 11) & 0x01) != 0)
    }
    #[doc = "Bit 12 - Data path state machine active, i.e. not in Idle state. This is a hardware status flag only, does not generate an interrupt."]
    #[inline(always)]
    pub fn txact(&self) -> TXACT_R {
        TXACT_R::new(((self.bits >> 12) & 0x01) != 0)
    }
    #[doc = "Bit 13 - Command path state machine active, i.e. not in Idle state. This is a hardware status flag only, does not generate an interrupt."]
    #[inline(always)]
    pub fn rxact(&self) -> RXACT_R {
        RXACT_R::new(((self.bits >> 13) & 0x01) != 0)
    }
    #[doc = "Bit 14 - Transmit FIFO half empty At least half the number of words can be written into the FIFO. This bit is cleared when the FIFO becomes half+1 full."]
    #[inline(always)]
    pub fn txfifohe(&self) -> TXFIFOHE_R {
        TXFIFOHE_R::new(((self.bits >> 14) & 0x01) != 0)
    }
    #[doc = "Bit 15 - Receive FIFO half full There are at least half the number of words in the FIFO. This bit is cleared when the FIFO becomes half+1 empty."]
    #[inline(always)]
    pub fn rxfifohf(&self) -> RXFIFOHF_R {
        RXFIFOHF_R::new(((self.bits >> 15) & 0x01) != 0)
    }
    #[doc = "Bit 16 - Transmit FIFO full This is a hardware status flag only, does not generate an interrupt. This bit is cleared when one FIFO location becomes empty."]
    #[inline(always)]
    pub fn txfifof(&self) -> TXFIFOF_R {
        TXFIFOF_R::new(((self.bits >> 16) & 0x01) != 0)
    }
    #[doc = "Bit 17 - Receive FIFO full This bit is cleared when one FIFO location becomes empty."]
    #[inline(always)]
    pub fn rxfifof(&self) -> RXFIFOF_R {
        RXFIFOF_R::new(((self.bits >> 17) & 0x01) != 0)
    }
    #[doc = "Bit 18 - Transmit FIFO empty This bit is cleared when one FIFO location becomes full."]
    #[inline(always)]
    pub fn txfifoe(&self) -> TXFIFOE_R {
        TXFIFOE_R::new(((self.bits >> 18) & 0x01) != 0)
    }
    #[doc = "Bit 19 - Receive FIFO empty This is a hardware status flag only, does not generate an interrupt. This bit is cleared when one FIFO location becomes full."]
    #[inline(always)]
    pub fn rxfifoe(&self) -> RXFIFOE_R {
        RXFIFOE_R::new(((self.bits >> 19) & 0x01) != 0)
    }
    #[doc = "Bit 20 - Inverted value of SDMMC_D0 line (Busy), sampled at the end of a CMD response and a second time 2 SDMMC_CK cycles after the CMD response. This bit is reset to not busy when the SDMMCD0 line changes from busy to not busy. This bit does not signal busy due to data transfer. This is a hardware status flag only, it does not generate an interrupt."]
    #[inline(always)]
    pub fn txdavl(&self) -> TXDAVL_R {
        TXDAVL_R::new(((self.bits >> 20) & 0x01) != 0)
    }
    #[doc = "Bit 21 - end of SDMMC_D0 Busy following a CMD response detected. This indicates only end of busy following a CMD response. This bit does not signal busy due to data transfer. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn rxdavl(&self) -> RXDAVL_R {
        RXDAVL_R::new(((self.bits >> 21) & 0x01) != 0)
    }
    #[doc = "Bit 22 - SDIO interrupt received. Interrupt flag is cleared by writing corresponding interrupt clear bit in SDMMC_ICR."]
    #[inline(always)]
    pub fn sdioit(&self) -> SDIOIT_R {
        SDIOIT_R::new(((self.bits >> 22) & 0x01) != 0)
    }
}
#[doc = "The SDMMC_STAR register is a read-only register. It contains two types of flag:Static flags (bits \\[29,21,11:0\\]): these bits remain asserted until they are cleared by writing to the SDMMC interrupt Clear register (see SDMMC_ICR)Dynamic flags (bits \\[20:12\\]): these bits change state depending on the state of the underlying logic (for example, FIFO full and empty flags are asserted and de-asserted as data while written to the FIFO)\n\nThis register you can [`read`](crate::generic::Reg::read). See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [sta](index.html) module"]
pub struct STA_SPEC;
impl crate::RegisterSpec for STA_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [sta::R](R) reader structure"]
impl crate::Readable for STA_SPEC {
    type Reader = R;
}
#[doc = "`reset()` method sets STA to value 0"]
impl crate::Resettable for STA_SPEC {
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        0
    }
}
