#[doc = "Register `VHBPCR` reader"]
pub struct R(crate::R<VHBPCR_SPEC>);
impl core::ops::Deref for R {
    type Target = crate::R<VHBPCR_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<crate::R<VHBPCR_SPEC>> for R {
    #[inline(always)]
    fn from(reader: crate::R<VHBPCR_SPEC>) -> Self {
        R(reader)
    }
}
#[doc = "Register `VHBPCR` writer"]
pub struct W(crate::W<VHBPCR_SPEC>);
impl core::ops::Deref for W {
    type Target = crate::W<VHBPCR_SPEC>;
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
impl From<crate::W<VHBPCR_SPEC>> for W {
    #[inline(always)]
    fn from(writer: crate::W<VHBPCR_SPEC>) -> Self {
        W(writer)
    }
}
#[doc = "Field `HBP` reader - Horizontal Back-Porch duration"]
pub struct HBP_R(crate::FieldReader<u16, u16>);
impl HBP_R {
    #[inline(always)]
    pub(crate) fn new(bits: u16) -> Self {
        HBP_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for HBP_R {
    type Target = crate::FieldReader<u16, u16>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `HBP` writer - Horizontal Back-Porch duration"]
pub struct HBP_W<'a> {
    w: &'a mut W,
}
impl<'a> HBP_W<'a> {
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub unsafe fn bits(self, value: u16) -> &'a mut W {
        self.w.bits = (self.w.bits & !0x1fff) | (value as u32 & 0x1fff);
        self.w
    }
}
impl R {
    #[doc = "Bits 0:12 - Horizontal Back-Porch duration"]
    #[inline(always)]
    pub fn hbp(&self) -> HBP_R {
        HBP_R::new((self.bits & 0x1fff) as u16)
    }
}
impl W {
    #[doc = "Bits 0:12 - Horizontal Back-Porch duration"]
    #[inline(always)]
    pub fn hbp(&mut self) -> HBP_W {
        HBP_W { w: self }
    }
    #[doc = "Writes raw bits to the register."]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.0.bits(bits);
        self
    }
}
#[doc = "DSI Host Video HBP Configuration Register\n\nThis register you can [`read`](crate::generic::Reg::read), [`write_with_zero`](crate::generic::Reg::write_with_zero), [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`modify`](crate::generic::Reg::modify). See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [vhbpcr](index.html) module"]
pub struct VHBPCR_SPEC;
impl crate::RegisterSpec for VHBPCR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [vhbpcr::R](R) reader structure"]
impl crate::Readable for VHBPCR_SPEC {
    type Reader = R;
}
#[doc = "`write(|w| ..)` method takes [vhbpcr::W](W) writer structure"]
impl crate::Writable for VHBPCR_SPEC {
    type Writer = W;
}
#[doc = "`reset()` method sets VHBPCR to value 0"]
impl crate::Resettable for VHBPCR_SPEC {
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        0
    }
}
