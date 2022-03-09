# `crabdac`

:crab: Rust-based firmware for a USB headphone amp with integrated DAC

_This project is a work-in-progress and is certainly not ready for use_

This firmware is currently configured for use on an stm32f446. The intent is to
have it drive a PCM1794a DAC IC (or two, each in mono mode :moneybag:), then out
to some headphones via I/V and buffer opamp stages.

It depends on some hacked dependencies:

* [`usb-device`](https://github.com/dgoodlad/usb-device) for isochronous transfer support
* [`synopsys-usb-otg`](https://github.com/dgoodlad/synopsys-usb-otg) for isochronous transfer support
* `stm32f4` for `ITM1_RMP` field in `TIM2_OR` register stm32-rs/stm32-rs#678
* [`stm32f4xx-hal`](https://github.com/dgoodlad/stm32f4xx-hal) for synopsys-usb-otg 0.3 (above)
* [`stm32_i2s_v12x`](https://github.com/samcrow/stm32_i2s) to fix compilation issues (memory map)

I'll be submitting upstream PRs once this all stabilises.

## License

See `LICENSE`
