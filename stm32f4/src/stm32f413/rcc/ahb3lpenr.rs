#[doc = "Register `AHB3LPENR` reader"]
pub struct R(crate::R<AHB3LPENR_SPEC>);
impl core::ops::Deref for R {
    type Target = crate::R<AHB3LPENR_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<crate::R<AHB3LPENR_SPEC>> for R {
    #[inline(always)]
    fn from(reader: crate::R<AHB3LPENR_SPEC>) -> Self {
        R(reader)
    }
}
#[doc = "Register `AHB3LPENR` writer"]
pub struct W(crate::W<AHB3LPENR_SPEC>);
impl core::ops::Deref for W {
    type Target = crate::W<AHB3LPENR_SPEC>;
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
impl From<crate::W<AHB3LPENR_SPEC>> for W {
    #[inline(always)]
    fn from(writer: crate::W<AHB3LPENR_SPEC>) -> Self {
        W(writer)
    }
}
#[doc = "Flexible memory controller module clock enable during Sleep mode\n\nValue on reset: 1"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FSMCLPEN_A {
    #[doc = "0: Selected module is disabled during Sleep mode"]
    DISABLEDINSLEEP = 0,
    #[doc = "1: Selected module is enabled during Sleep mode"]
    ENABLEDINSLEEP = 1,
}
impl From<FSMCLPEN_A> for bool {
    #[inline(always)]
    fn from(variant: FSMCLPEN_A) -> Self {
        variant as u8 != 0
    }
}
#[doc = "Field `FSMCLPEN` reader - Flexible memory controller module clock enable during Sleep mode"]
pub struct FSMCLPEN_R(crate::FieldReader<bool, FSMCLPEN_A>);
impl FSMCLPEN_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        FSMCLPEN_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> FSMCLPEN_A {
        match self.bits {
            false => FSMCLPEN_A::DISABLEDINSLEEP,
            true => FSMCLPEN_A::ENABLEDINSLEEP,
        }
    }
    #[doc = "Checks if the value of the field is `DISABLEDINSLEEP`"]
    #[inline(always)]
    pub fn is_disabled_in_sleep(&self) -> bool {
        **self == FSMCLPEN_A::DISABLEDINSLEEP
    }
    #[doc = "Checks if the value of the field is `ENABLEDINSLEEP`"]
    #[inline(always)]
    pub fn is_enabled_in_sleep(&self) -> bool {
        **self == FSMCLPEN_A::ENABLEDINSLEEP
    }
}
impl core::ops::Deref for FSMCLPEN_R {
    type Target = crate::FieldReader<bool, FSMCLPEN_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `FSMCLPEN` writer - Flexible memory controller module clock enable during Sleep mode"]
pub struct FSMCLPEN_W<'a> {
    w: &'a mut W,
}
impl<'a> FSMCLPEN_W<'a> {
    #[doc = r"Writes `variant` to the field"]
    #[inline(always)]
    pub fn variant(self, variant: FSMCLPEN_A) -> &'a mut W {
        self.bit(variant.into())
    }
    #[doc = "Selected module is disabled during Sleep mode"]
    #[inline(always)]
    pub fn disabled_in_sleep(self) -> &'a mut W {
        self.variant(FSMCLPEN_A::DISABLEDINSLEEP)
    }
    #[doc = "Selected module is enabled during Sleep mode"]
    #[inline(always)]
    pub fn enabled_in_sleep(self) -> &'a mut W {
        self.variant(FSMCLPEN_A::ENABLEDINSLEEP)
    }
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !0x01) | (value as u32 & 0x01);
        self.w
    }
}
#[doc = "QUADSPI memory controller module clock enable during Sleep mode"]
pub type QSPILPEN_A = FSMCLPEN_A;
#[doc = "Field `QSPILPEN` reader - QUADSPI memory controller module clock enable during Sleep mode"]
pub type QSPILPEN_R = FSMCLPEN_R;
#[doc = "Field `QSPILPEN` writer - QUADSPI memory controller module clock enable during Sleep mode"]
pub struct QSPILPEN_W<'a> {
    w: &'a mut W,
}
impl<'a> QSPILPEN_W<'a> {
    #[doc = r"Writes `variant` to the field"]
    #[inline(always)]
    pub fn variant(self, variant: QSPILPEN_A) -> &'a mut W {
        self.bit(variant.into())
    }
    #[doc = "Selected module is disabled during Sleep mode"]
    #[inline(always)]
    pub fn disabled_in_sleep(self) -> &'a mut W {
        self.variant(QSPILPEN_A::DISABLEDINSLEEP)
    }
    #[doc = "Selected module is enabled during Sleep mode"]
    #[inline(always)]
    pub fn enabled_in_sleep(self) -> &'a mut W {
        self.variant(QSPILPEN_A::ENABLEDINSLEEP)
    }
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 1)) | ((value as u32 & 0x01) << 1);
        self.w
    }
}
impl R {
    #[doc = "Bit 0 - Flexible memory controller module clock enable during Sleep mode"]
    #[inline(always)]
    pub fn fsmclpen(&self) -> FSMCLPEN_R {
        FSMCLPEN_R::new((self.bits & 0x01) != 0)
    }
    #[doc = "Bit 1 - QUADSPI memory controller module clock enable during Sleep mode"]
    #[inline(always)]
    pub fn qspilpen(&self) -> QSPILPEN_R {
        QSPILPEN_R::new(((self.bits >> 1) & 0x01) != 0)
    }
}
impl W {
    #[doc = "Bit 0 - Flexible memory controller module clock enable during Sleep mode"]
    #[inline(always)]
    pub fn fsmclpen(&mut self) -> FSMCLPEN_W {
        FSMCLPEN_W { w: self }
    }
    #[doc = "Bit 1 - QUADSPI memory controller module clock enable during Sleep mode"]
    #[inline(always)]
    pub fn qspilpen(&mut self) -> QSPILPEN_W {
        QSPILPEN_W { w: self }
    }
    #[doc = "Writes raw bits to the register."]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.0.bits(bits);
        self
    }
}
#[doc = "AHB3 peripheral clock enable in low power mode register\n\nThis register you can [`read`](crate::generic::Reg::read), [`write_with_zero`](crate::generic::Reg::write_with_zero), [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`modify`](crate::generic::Reg::modify). See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [ahb3lpenr](index.html) module"]
pub struct AHB3LPENR_SPEC;
impl crate::RegisterSpec for AHB3LPENR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [ahb3lpenr::R](R) reader structure"]
impl crate::Readable for AHB3LPENR_SPEC {
    type Reader = R;
}
#[doc = "`write(|w| ..)` method takes [ahb3lpenr::W](W) writer structure"]
impl crate::Writable for AHB3LPENR_SPEC {
    type Writer = W;
}
#[doc = "`reset()` method sets AHB3LPENR to value 0x03"]
impl crate::Resettable for AHB3LPENR_SPEC {
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        0x03
    }
}
