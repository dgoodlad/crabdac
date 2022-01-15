#[doc = "Register `LPMCCR` reader"]
pub struct R(crate::R<LPMCCR_SPEC>);
impl core::ops::Deref for R {
    type Target = crate::R<LPMCCR_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<crate::R<LPMCCR_SPEC>> for R {
    #[inline(always)]
    fn from(reader: crate::R<LPMCCR_SPEC>) -> Self {
        R(reader)
    }
}
#[doc = "Register `LPMCCR` writer"]
pub struct W(crate::W<LPMCCR_SPEC>);
impl core::ops::Deref for W {
    type Target = crate::W<LPMCCR_SPEC>;
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
impl From<crate::W<LPMCCR_SPEC>> for W {
    #[inline(always)]
    fn from(writer: crate::W<LPMCCR_SPEC>) -> Self {
        W(writer)
    }
}
#[doc = "Field `LPSIZE` reader - Largest Packet Size"]
pub struct LPSIZE_R(crate::FieldReader<u8, u8>);
impl LPSIZE_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        LPSIZE_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for LPSIZE_R {
    type Target = crate::FieldReader<u8, u8>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `LPSIZE` writer - Largest Packet Size"]
pub struct LPSIZE_W<'a> {
    w: &'a mut W,
}
impl<'a> LPSIZE_W<'a> {
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0xff << 16)) | ((value as u32 & 0xff) << 16);
        self.w
    }
}
#[doc = "Field `VLPSIZE` reader - VACT Largest Packet Size"]
pub struct VLPSIZE_R(crate::FieldReader<u8, u8>);
impl VLPSIZE_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        VLPSIZE_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for VLPSIZE_R {
    type Target = crate::FieldReader<u8, u8>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `VLPSIZE` writer - VACT Largest Packet Size"]
pub struct VLPSIZE_W<'a> {
    w: &'a mut W,
}
impl<'a> VLPSIZE_W<'a> {
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !0xff) | (value as u32 & 0xff);
        self.w
    }
}
impl R {
    #[doc = "Bits 16:23 - Largest Packet Size"]
    #[inline(always)]
    pub fn lpsize(&self) -> LPSIZE_R {
        LPSIZE_R::new(((self.bits >> 16) & 0xff) as u8)
    }
    #[doc = "Bits 0:7 - VACT Largest Packet Size"]
    #[inline(always)]
    pub fn vlpsize(&self) -> VLPSIZE_R {
        VLPSIZE_R::new((self.bits & 0xff) as u8)
    }
}
impl W {
    #[doc = "Bits 16:23 - Largest Packet Size"]
    #[inline(always)]
    pub fn lpsize(&mut self) -> LPSIZE_W {
        LPSIZE_W { w: self }
    }
    #[doc = "Bits 0:7 - VACT Largest Packet Size"]
    #[inline(always)]
    pub fn vlpsize(&mut self) -> VLPSIZE_W {
        VLPSIZE_W { w: self }
    }
    #[doc = "Writes raw bits to the register."]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.0.bits(bits);
        self
    }
}
#[doc = "DSI Host Low-power Mode Current Configuration Register\n\nThis register you can [`read`](crate::generic::Reg::read), [`write_with_zero`](crate::generic::Reg::write_with_zero), [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`modify`](crate::generic::Reg::modify). See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [lpmccr](index.html) module"]
pub struct LPMCCR_SPEC;
impl crate::RegisterSpec for LPMCCR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [lpmccr::R](R) reader structure"]
impl crate::Readable for LPMCCR_SPEC {
    type Reader = R;
}
#[doc = "`write(|w| ..)` method takes [lpmccr::W](W) writer structure"]
impl crate::Writable for LPMCCR_SPEC {
    type Writer = W;
}
#[doc = "`reset()` method sets LPMCCR to value 0"]
impl crate::Resettable for LPMCCR_SPEC {
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        0
    }
}
