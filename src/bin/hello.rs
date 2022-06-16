#![no_main]
#![no_std]

use crabdac_firmware as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    crabdac_firmware::exit()
}
