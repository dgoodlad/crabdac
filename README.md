# `crabdac` firmware

> Rust-based USB to I2S firmware for STM32F411


## Setup

### 1. Dependencies

#### `flip-link`:

```console
$ cargo install flip-link
```

#### `probe-run`:

``` console
$ # make sure to install v0.2.0 or later
$ cargo install probe-run
```

### 2. Run!



``` console
$ # `rrb` is an alias for `run --release --bin`
$ DEFMT_LOG=info cargo rrb main
   Compiling crabdac-firmware v0.1.0 (/Users/dgoodlad/src/crabdac-firmware)
    Finished release [optimized + debuginfo] target(s) in 1.63s
     Running `probe-run --chip STM32F411CEUx target/thumbv7em-none-eabi/release/main`
(HOST) INFO  flashing program (55 pages / 55.00 KiB)
(HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
0.000000 INFO  init
└─ main::app::init @ src/bin/main.rs:112
```

## License

``` text
(c) 2022 David Goodlad

Permission is hereby granted, free of charge, to any
person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the
Software without restriction, including without
limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice
shall be included in all copies or substantial portions
of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
```

### Contribution

This is a personal project. If you want to contribute, that's cool! I'll do my
best to review as I can.
