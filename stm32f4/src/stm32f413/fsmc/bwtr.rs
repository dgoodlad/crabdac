#[doc = "Register `BWTR%s` reader"]
pub struct R(crate::R<BWTR_SPEC>);
impl core::ops::Deref for R {
    type Target = crate::R<BWTR_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<crate::R<BWTR_SPEC>> for R {
    #[inline(always)]
    fn from(reader: crate::R<BWTR_SPEC>) -> Self {
        R(reader)
    }
}
#[doc = "Register `BWTR%s` writer"]
pub struct W(crate::W<BWTR_SPEC>);
impl core::ops::Deref for W {
    type Target = crate::W<BWTR_SPEC>;
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
impl From<crate::W<BWTR_SPEC>> for W {
    #[inline(always)]
    fn from(writer: crate::W<BWTR_SPEC>) -> Self {
        W(writer)
    }
}
#[doc = "Access mode\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum ACCMOD_A {
    #[doc = "0: Access mode A"]
    A = 0,
    #[doc = "1: Access mode B"]
    B = 1,
    #[doc = "2: Access mode C"]
    C = 2,
    #[doc = "3: Access mode D"]
    D = 3,
}
impl From<ACCMOD_A> for u8 {
    #[inline(always)]
    fn from(variant: ACCMOD_A) -> Self {
        variant as _
    }
}
#[doc = "Field `ACCMOD` reader - Access mode"]
pub struct ACCMOD_R(crate::FieldReader<u8, ACCMOD_A>);
impl ACCMOD_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        ACCMOD_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> ACCMOD_A {
        match self.bits {
            0 => ACCMOD_A::A,
            1 => ACCMOD_A::B,
            2 => ACCMOD_A::C,
            3 => ACCMOD_A::D,
            _ => unreachable!(),
        }
    }
    #[doc = "Checks if the value of the field is `A`"]
    #[inline(always)]
    pub fn is_a(&self) -> bool {
        **self == ACCMOD_A::A
    }
    #[doc = "Checks if the value of the field is `B`"]
    #[inline(always)]
    pub fn is_b(&self) -> bool {
        **self == ACCMOD_A::B
    }
    #[doc = "Checks if the value of the field is `C`"]
    #[inline(always)]
    pub fn is_c(&self) -> bool {
        **self == ACCMOD_A::C
    }
    #[doc = "Checks if the value of the field is `D`"]
    #[inline(always)]
    pub fn is_d(&self) -> bool {
        **self == ACCMOD_A::D
    }
}
impl core::ops::Deref for ACCMOD_R {
    type Target = crate::FieldReader<u8, ACCMOD_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `ACCMOD` writer - Access mode"]
pub struct ACCMOD_W<'a> {
    w: &'a mut W,
}
impl<'a> ACCMOD_W<'a> {
    #[doc = r"Writes `variant` to the field"]
    #[inline(always)]
    pub fn variant(self, variant: ACCMOD_A) -> &'a mut W {
        self.bits(variant.into())
    }
    #[doc = "Access mode A"]
    #[inline(always)]
    pub fn a(self) -> &'a mut W {
        self.variant(ACCMOD_A::A)
    }
    #[doc = "Access mode B"]
    #[inline(always)]
    pub fn b(self) -> &'a mut W {
        self.variant(ACCMOD_A::B)
    }
    #[doc = "Access mode C"]
    #[inline(always)]
    pub fn c(self) -> &'a mut W {
        self.variant(ACCMOD_A::C)
    }
    #[doc = "Access mode D"]
    #[inline(always)]
    pub fn d(self) -> &'a mut W {
        self.variant(ACCMOD_A::D)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x03 << 28)) | ((value as u32 & 0x03) << 28);
        self.w
    }
}
#[doc = "Field `DATAST` reader - Data-phase duration"]
pub struct DATAST_R(crate::FieldReader<u8, u8>);
impl DATAST_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        DATAST_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for DATAST_R {
    type Target = crate::FieldReader<u8, u8>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `DATAST` writer - Data-phase duration"]
pub struct DATAST_W<'a> {
    w: &'a mut W,
}
impl<'a> DATAST_W<'a> {
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0xff << 8)) | ((value as u32 & 0xff) << 8);
        self.w
    }
}
#[doc = "Field `ADDHLD` reader - Address-hold phase duration"]
pub struct ADDHLD_R(crate::FieldReader<u8, u8>);
impl ADDHLD_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        ADDHLD_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for ADDHLD_R {
    type Target = crate::FieldReader<u8, u8>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `ADDHLD` writer - Address-hold phase duration"]
pub struct ADDHLD_W<'a> {
    w: &'a mut W,
}
impl<'a> ADDHLD_W<'a> {
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x0f << 4)) | ((value as u32 & 0x0f) << 4);
        self.w
    }
}
#[doc = "Field `ADDSET` reader - Address setup phase duration"]
pub struct ADDSET_R(crate::FieldReader<u8, u8>);
impl ADDSET_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        ADDSET_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for ADDSET_R {
    type Target = crate::FieldReader<u8, u8>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `ADDSET` writer - Address setup phase duration"]
pub struct ADDSET_W<'a> {
    w: &'a mut W,
}
impl<'a> ADDSET_W<'a> {
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !0x0f) | (value as u32 & 0x0f);
        self.w
    }
}
#[doc = "Field `BUSTURN` reader - Bus turnaround phase duration"]
pub struct BUSTURN_R(crate::FieldReader<u8, u8>);
impl BUSTURN_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        BUSTURN_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for BUSTURN_R {
    type Target = crate::FieldReader<u8, u8>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `BUSTURN` writer - Bus turnaround phase duration"]
pub struct BUSTURN_W<'a> {
    w: &'a mut W,
}
impl<'a> BUSTURN_W<'a> {
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x0f << 16)) | ((value as u32 & 0x0f) << 16);
        self.w
    }
}
impl R {
    #[doc = "Bits 28:29 - Access mode"]
    #[inline(always)]
    pub fn accmod(&self) -> ACCMOD_R {
        ACCMOD_R::new(((self.bits >> 28) & 0x03) as u8)
    }
    #[doc = "Bits 8:15 - Data-phase duration"]
    #[inline(always)]
    pub fn datast(&self) -> DATAST_R {
        DATAST_R::new(((self.bits >> 8) & 0xff) as u8)
    }
    #[doc = "Bits 4:7 - Address-hold phase duration"]
    #[inline(always)]
    pub fn addhld(&self) -> ADDHLD_R {
        ADDHLD_R::new(((self.bits >> 4) & 0x0f) as u8)
    }
    #[doc = "Bits 0:3 - Address setup phase duration"]
    #[inline(always)]
    pub fn addset(&self) -> ADDSET_R {
        ADDSET_R::new((self.bits & 0x0f) as u8)
    }
    #[doc = "Bits 16:19 - Bus turnaround phase duration"]
    #[inline(always)]
    pub fn busturn(&self) -> BUSTURN_R {
        BUSTURN_R::new(((self.bits >> 16) & 0x0f) as u8)
    }
}
impl W {
    #[doc = "Bits 28:29 - Access mode"]
    #[inline(always)]
    pub fn accmod(&mut self) -> ACCMOD_W {
        ACCMOD_W { w: self }
    }
    #[doc = "Bits 8:15 - Data-phase duration"]
    #[inline(always)]
    pub fn datast(&mut self) -> DATAST_W {
        DATAST_W { w: self }
    }
    #[doc = "Bits 4:7 - Address-hold phase duration"]
    #[inline(always)]
    pub fn addhld(&mut self) -> ADDHLD_W {
        ADDHLD_W { w: self }
    }
    #[doc = "Bits 0:3 - Address setup phase duration"]
    #[inline(always)]
    pub fn addset(&mut self) -> ADDSET_W {
        ADDSET_W { w: self }
    }
    #[doc = "Bits 16:19 - Bus turnaround phase duration"]
    #[inline(always)]
    pub fn busturn(&mut self) -> BUSTURN_W {
        BUSTURN_W { w: self }
    }
    #[doc = "Writes raw bits to the register."]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.0.bits(bits);
        self
    }
}
#[doc = "BWTR1\n\nThis register you can [`read`](crate::generic::Reg::read), [`write_with_zero`](crate::generic::Reg::write_with_zero), [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`modify`](crate::generic::Reg::modify). See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [bwtr](index.html) module"]
pub struct BWTR_SPEC;
impl crate::RegisterSpec for BWTR_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [bwtr::R](R) reader structure"]
impl crate::Readable for BWTR_SPEC {
    type Reader = R;
}
#[doc = "`write(|w| ..)` method takes [bwtr::W](W) writer structure"]
impl crate::Writable for BWTR_SPEC {
    type Writer = W;
}
#[doc = "`reset()` method sets BWTR%s to value 0"]
impl crate::Resettable for BWTR_SPEC {
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        0
    }
}
