#[doc = "Register `DCKCFGR2` reader"]
pub struct R(crate::R<DCKCFGR2_SPEC>);
impl core::ops::Deref for R {
    type Target = crate::R<DCKCFGR2_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<crate::R<DCKCFGR2_SPEC>> for R {
    #[inline(always)]
    fn from(reader: crate::R<DCKCFGR2_SPEC>) -> Self {
        R(reader)
    }
}
#[doc = "Register `DCKCFGR2` writer"]
pub struct W(crate::W<DCKCFGR2_SPEC>);
impl core::ops::Deref for W {
    type Target = crate::W<DCKCFGR2_SPEC>;
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
impl From<crate::W<DCKCFGR2_SPEC>> for W {
    #[inline(always)]
    fn from(writer: crate::W<DCKCFGR2_SPEC>) -> Self {
        W(writer)
    }
}
#[doc = "FMPI2C1 kernel clock source selection\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum FMPI2C1SEL_A {
    #[doc = "0: APB clock selected as I2C clock"]
    APB = 0,
    #[doc = "1: System clock selected as I2C clock"]
    SYSCLK = 1,
    #[doc = "2: HSI clock selected as I2C clock"]
    HSI = 2,
}
impl From<FMPI2C1SEL_A> for u8 {
    #[inline(always)]
    fn from(variant: FMPI2C1SEL_A) -> Self {
        variant as _
    }
}
#[doc = "Field `FMPI2C1SEL` reader - FMPI2C1 kernel clock source selection"]
pub struct FMPI2C1SEL_R(crate::FieldReader<u8, FMPI2C1SEL_A>);
impl FMPI2C1SEL_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        FMPI2C1SEL_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> Option<FMPI2C1SEL_A> {
        match self.bits {
            0 => Some(FMPI2C1SEL_A::APB),
            1 => Some(FMPI2C1SEL_A::SYSCLK),
            2 => Some(FMPI2C1SEL_A::HSI),
            _ => None,
        }
    }
    #[doc = "Checks if the value of the field is `APB`"]
    #[inline(always)]
    pub fn is_apb(&self) -> bool {
        **self == FMPI2C1SEL_A::APB
    }
    #[doc = "Checks if the value of the field is `SYSCLK`"]
    #[inline(always)]
    pub fn is_sysclk(&self) -> bool {
        **self == FMPI2C1SEL_A::SYSCLK
    }
    #[doc = "Checks if the value of the field is `HSI`"]
    #[inline(always)]
    pub fn is_hsi(&self) -> bool {
        **self == FMPI2C1SEL_A::HSI
    }
}
impl core::ops::Deref for FMPI2C1SEL_R {
    type Target = crate::FieldReader<u8, FMPI2C1SEL_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `FMPI2C1SEL` writer - FMPI2C1 kernel clock source selection"]
pub struct FMPI2C1SEL_W<'a> {
    w: &'a mut W,
}
impl<'a> FMPI2C1SEL_W<'a> {
    #[doc = r"Writes `variant` to the field"]
    #[inline(always)]
    pub fn variant(self, variant: FMPI2C1SEL_A) -> &'a mut W {
        unsafe { self.bits(variant.into()) }
    }
    #[doc = "APB clock selected as I2C clock"]
    #[inline(always)]
    pub fn apb(self) -> &'a mut W {
        self.variant(FMPI2C1SEL_A::APB)
    }
    #[doc = "System clock selected as I2C clock"]
    #[inline(always)]
    pub fn sysclk(self) -> &'a mut W {
        self.variant(FMPI2C1SEL_A::SYSCLK)
    }
    #[doc = "HSI clock selected as I2C clock"]
    #[inline(always)]
    pub fn hsi(self) -> &'a mut W {
        self.variant(FMPI2C1SEL_A::HSI)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x03 << 22)) | ((value as u32 & 0x03) << 22);
        self.w
    }
}
#[doc = "LPTIM1SEL\n\nValue on reset: 0"]
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum LPTIM1SEL_A {
    #[doc = "0: APB1 clock (PCLK1) selected as LPTILM1 clock"]
    APB1 = 0,
    #[doc = "1: LSI clock is selected as LPTILM1 clock"]
    LSI = 1,
    #[doc = "2: HSI clock is selected as LPTILM1 clock"]
    HSI = 2,
    #[doc = "3: LSE clock is selected as LPTILM1 clock"]
    LSE = 3,
}
impl From<LPTIM1SEL_A> for u8 {
    #[inline(always)]
    fn from(variant: LPTIM1SEL_A) -> Self {
        variant as _
    }
}
#[doc = "Field `LPTIM1SEL` reader - LPTIM1SEL"]
pub struct LPTIM1SEL_R(crate::FieldReader<u8, LPTIM1SEL_A>);
impl LPTIM1SEL_R {
    #[inline(always)]
    pub(crate) fn new(bits: u8) -> Self {
        LPTIM1SEL_R(crate::FieldReader::new(bits))
    }
    #[doc = r"Get enumerated values variant"]
    #[inline(always)]
    pub fn variant(&self) -> LPTIM1SEL_A {
        match self.bits {
            0 => LPTIM1SEL_A::APB1,
            1 => LPTIM1SEL_A::LSI,
            2 => LPTIM1SEL_A::HSI,
            3 => LPTIM1SEL_A::LSE,
            _ => unreachable!(),
        }
    }
    #[doc = "Checks if the value of the field is `APB1`"]
    #[inline(always)]
    pub fn is_apb1(&self) -> bool {
        **self == LPTIM1SEL_A::APB1
    }
    #[doc = "Checks if the value of the field is `LSI`"]
    #[inline(always)]
    pub fn is_lsi(&self) -> bool {
        **self == LPTIM1SEL_A::LSI
    }
    #[doc = "Checks if the value of the field is `HSI`"]
    #[inline(always)]
    pub fn is_hsi(&self) -> bool {
        **self == LPTIM1SEL_A::HSI
    }
    #[doc = "Checks if the value of the field is `LSE`"]
    #[inline(always)]
    pub fn is_lse(&self) -> bool {
        **self == LPTIM1SEL_A::LSE
    }
}
impl core::ops::Deref for LPTIM1SEL_R {
    type Target = crate::FieldReader<u8, LPTIM1SEL_A>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `LPTIM1SEL` writer - LPTIM1SEL"]
pub struct LPTIM1SEL_W<'a> {
    w: &'a mut W,
}
impl<'a> LPTIM1SEL_W<'a> {
    #[doc = r"Writes `variant` to the field"]
    #[inline(always)]
    pub fn variant(self, variant: LPTIM1SEL_A) -> &'a mut W {
        self.bits(variant.into())
    }
    #[doc = "APB1 clock (PCLK1) selected as LPTILM1 clock"]
    #[inline(always)]
    pub fn apb1(self) -> &'a mut W {
        self.variant(LPTIM1SEL_A::APB1)
    }
    #[doc = "LSI clock is selected as LPTILM1 clock"]
    #[inline(always)]
    pub fn lsi(self) -> &'a mut W {
        self.variant(LPTIM1SEL_A::LSI)
    }
    #[doc = "HSI clock is selected as LPTILM1 clock"]
    #[inline(always)]
    pub fn hsi(self) -> &'a mut W {
        self.variant(LPTIM1SEL_A::HSI)
    }
    #[doc = "LSE clock is selected as LPTILM1 clock"]
    #[inline(always)]
    pub fn lse(self) -> &'a mut W {
        self.variant(LPTIM1SEL_A::LSE)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x03 << 30)) | ((value as u32 & 0x03) << 30);
        self.w
    }
}
impl R {
    #[doc = "Bits 22:23 - FMPI2C1 kernel clock source selection"]
    #[inline(always)]
    pub fn fmpi2c1sel(&self) -> FMPI2C1SEL_R {
        FMPI2C1SEL_R::new(((self.bits >> 22) & 0x03) as u8)
    }
    #[doc = "Bits 30:31 - LPTIM1SEL"]
    #[inline(always)]
    pub fn lptim1sel(&self) -> LPTIM1SEL_R {
        LPTIM1SEL_R::new(((self.bits >> 30) & 0x03) as u8)
    }
}
impl W {
    #[doc = "Bits 22:23 - FMPI2C1 kernel clock source selection"]
    #[inline(always)]
    pub fn fmpi2c1sel(&mut self) -> FMPI2C1SEL_W {
        FMPI2C1SEL_W { w: self }
    }
    #[doc = "Bits 30:31 - LPTIM1SEL"]
    #[inline(always)]
    pub fn lptim1sel(&mut self) -> LPTIM1SEL_W {
        LPTIM1SEL_W { w: self }
    }
    #[doc = "Writes raw bits to the register."]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.0.bits(bits);
        self
    }
}
#[doc = "DCKCFGR2 register\n\nThis register you can [`read`](crate::generic::Reg::read), [`write_with_zero`](crate::generic::Reg::write_with_zero), [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`modify`](crate::generic::Reg::modify). See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [dckcfgr2](index.html) module"]
pub struct DCKCFGR2_SPEC;
impl crate::RegisterSpec for DCKCFGR2_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [dckcfgr2::R](R) reader structure"]
impl crate::Readable for DCKCFGR2_SPEC {
    type Reader = R;
}
#[doc = "`write(|w| ..)` method takes [dckcfgr2::W](W) writer structure"]
impl crate::Writable for DCKCFGR2_SPEC {
    type Writer = W;
}
#[doc = "`reset()` method sets DCKCFGR2 to value 0"]
impl crate::Resettable for DCKCFGR2_SPEC {
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        0
    }
}
