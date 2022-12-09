pub mod descriptors;
pub mod request;
pub mod simple_stereo_output;

#[repr(u8)]
#[derive(PartialEq, Clone, Copy)]
pub enum StreamingState {
    Enabled = 1,
    Disabled = 0,
}

#[derive(Debug)]
pub struct ClockCounter {
    pub ticks: u32,
    pub frames: u8,
    pub mck_to_fs_ratio: u8,
}

impl ClockCounter {
    pub fn new(mck_to_fs_ratio: u8) -> Self {
        Self { ticks: 0, frames: 0, mck_to_fs_ratio }
    }

    pub fn clear(&mut self) {
        self.ticks = 0;
        self.frames = 0;
    }

    pub fn add(&mut self, ticks: u32) {
        self.ticks += ticks;
        self.frames += 1;
    }

    pub fn current_rate(&self) -> u32 {
        self.ticks << (14 - self.mck_to_fs_ratio - (self.frames - 1))
    }
}
