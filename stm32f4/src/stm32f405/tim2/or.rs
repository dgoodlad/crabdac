#[doc = "Register `OR` reader"]
pub struct R(crate::R<OR_SPEC>);
impl core::ops::Deref for R {
    type Target = crate::R<OR_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<crate::R<OR_SPEC>> for R {
    #[inline(always)]
    fn from(reader: crate::R<OR_SPEC>) -> Self {
        R(reader)
    }
}
#[doc = "Register `OR` writer"]
pub struct W(crate::W<OR_SPEC>);
impl core::ops::Deref for W {
    type Target = crate::W<OR_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl core::ops::DerefMut for W {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl From<crate::W<OR_SPEC>> for W {
    #[inline(always)]
    fn from(writer: crate::W<OR_SPEC>) -> Self {
        W(writer)
    }
}
#[doc = "Timer Input 4 remap\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum ITR1_RMP_A {
    #[doc = "0: TIM8 trigger output is connected to TIM2_ITR1 input"]
    TIM8_TRGOUT = 0,
    #[doc = "1: Ethernet PTP clock is connected to TIM2_ITR1 input"]
    PTP = 1,
    #[doc = "2: OTG FS SOF is connected to the TIM2_ITR1 input"]
    OTG_FS_SOF = 2,
    #[doc = "3: OTG HS SOF is connected to the TIM2_ITR1 input"]
    OTG_HS_SOF = 3,
}
impl From<ITR1_RMP_A> for u8 {
    #[inline(always)]
    fn from(variant: ITR1_RMP_A) -> Self {
        variant as _
    }
}
#[doc = "Field `ITR1_RMP` reader - Timer Input 4 remap"]
pub struct ITR1_RMP_R(crate::FieldReader<u8, ITR1_RMP_A>);
impl ITR1_RMP_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        ITR1_RMP_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> ITR1_RMP_A {
        match self.bits {
            0 => ITR1_RMP_A::TIM8_TRGOUT,
            1 => ITR1_RMP_A::PTP,
            2 => ITR1_RMP_A::OTG_FS_SOF,
            3 => ITR1_RMP_A::OTG_HS_SOF,
            _ => unreachable!(),
        }
    }
    #[doc = "Checks if the value of the field is `TIM8_TRGOUT`"]
    #[inline(always)]
    pub fn is_tim8_trgout(&self) -> bool {
        **self == ITR1_RMP_A::TIM8_TRGOUT
    }
    #[doc = "Checks if the value of the field is `PTP`"]
    #[inline(always)]
    pub fn is_ptp(&self) -> bool {
        **self == ITR1_RMP_A::PTP
    }
    #[doc = "Checks if the value of the field is `OTG_FS_SOF`"]
    #[inline(always)]
    pub fn is_otg_fs_sof(&self) -> bool {
        **self == ITR1_RMP_A::OTG_FS_SOF
    }
    #[doc = "Checks if the value of the field is `OTG_HS_SOF`"]
    #[inline(always)]
    pub fn is_otg_hs_sof(&self) -> bool {
        **self == ITR1_RMP_A::OTG_HS_SOF
    }
}
impl core::ops::Deref for ITR1_RMP_R {
    type Target = crate::FieldReader<u8, ITR1_RMP_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `ITR1_RMP` writer - Timer Input 4 remap"]
pub struct ITR1_RMP_W<'a> {
    w: &'a mut W,
}
impl<'a> ITR1_RMP_W<'a> {
    #[doc = r"Writes `variant` to the field"]
    #[inline(always)]
    pub fn variant(self, variant: ITR1_RMP_A) -> &'a mut W {
        self.bits(variant.into())
    }
    #[doc = "TIM8 trigger output is connected to TIM2_ITR1 input"]
    #[inline(always)]
    pub fn tim8_trgout(self) -> &'a mut W {
        self.variant(ITR1_RMP_A::TIM8_TRGOUT)
    }
    #[doc = "Ethernet PTP clock is connected to TIM2_ITR1 input"]
    #[inline(always)]
    pub fn ptp(self) -> &'a mut W {
        self.variant(ITR1_RMP_A::PTP)
    }
    #[doc = "OTG FS SOF is connected to the TIM2_ITR1 input"]
    #[inline(always)]
    pub fn otg_fs_sof(self) -> &'a mut W {
        self.variant(ITR1_RMP_A::OTG_FS_SOF)
    }
    #[doc = "OTG HS SOF is connected to the TIM2_ITR1 input"]
    #[inline(always)]
    pub fn otg_hs_sof(self) -> &'a mut W {
        self.variant(ITR1_RMP_A::OTG_HS_SOF)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x03 << 10)) | ((value as u32 & 0x03) << 10);
        self.w
    }
}
impl R {
    #[doc = "Bits 10:11 - Timer Input 4 remap"]
    #[inline(always)]
    pub fn itr1_rmp(&self) -> ITR1_RMP_R {
        ITR1_RMP_R::new(((self.bits >> 10) & 0x03) as u8)
    }
}
impl W {
    #[doc = "Bits 10:11 - Timer Input 4 remap"]
    #[inline(always)]
    pub fn itr1_rmp(&mut self) -> ITR1_RMP_W {
        ITR1_RMP_W { w: self }
    }
    #[doc = "Writes raw bits to the register."]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.0.bits(bits);
        self
    }
}
#[doc = "TIM5 option register\n\nThis register you can [`read`](crate::generic::Reg::read), [`write_with_zero`](crate::generic::Reg::write_with_zero), [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`modify`](crate::generic::Reg::modify). See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [or](index.html) module"]
pub struct OR_SPEC;
impl crate::RegisterSpec for OR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [or::R](R) reader structure"]
impl crate::Readable for OR_SPEC {
    type Reader = R;
}
#[doc = "`write(|w| ..)` method takes [or::W](W) writer structure"]
impl crate::Writable for OR_SPEC {
    type Writer = W;
}
#[doc = "`reset()` method sets OR to value 0"]
impl crate::Resettable for OR_SPEC {
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        0
    }
}
