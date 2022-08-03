#![no_main]
#![no_std]

use crabdac_firmware as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM3, TIM4])]
mod app {
    use byte_slice_cast::*;

    use stm32f4xx_hal::{
        prelude::*,
        timer::MonoTimerUs,
        gpio::{PA0, Alternate},
        pac::{TIM2, TIM5},
        otg_fs::{
            USB,
            UsbBus,
            UsbBusType,
        }
    };

    use usb_device::prelude::*;
    use usb_device::bus::UsbBusAllocator;

    use usbd_serial::SerialPort;

    use bbqueue::{
        BBBuffer,
        Consumer,
        Producer,
    };

    use crabdac_firmware::sof_timer::SofTimer;

    const CHANNELS: usize = 2;
    const USB_SAMPLE_SIZE: usize = 4;
    const SAMPLE_RATE: usize = 96000;
    const USB_FRAME_RATE: usize = 1000;
    const MAX_FRAME_SIZE: usize = CHANNELS * (USB_SAMPLE_SIZE * SAMPLE_RATE / USB_FRAME_RATE + 1);
    const BUFFER_SIZE: usize = MAX_FRAME_SIZE * 2;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        producer: Producer<'static, BUFFER_SIZE>,
        consumer: Consumer<'static, BUFFER_SIZE>,
        sof_timer: SofTimer<TIM2, PA0<Alternate<1>>>,

        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimerUs<TIM5>;

    #[init(local = [
        buffer: BBBuffer<BUFFER_SIZE> = BBBuffer::new(),
        usb_bus: Option<UsbBusAllocator<UsbBusType>> = None,
        usb_ep_memory: [u32; 320] = [0; 320],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr
                        .use_hse(25.MHz())
                        .sysclk(96.MHz())
                        .hclk(96.MHz())
                        .pclk1(48.MHz())
                        .pclk2(96.MHz())
                        .i2s_clk(98285714.Hz()) // Close to 98.304 MHz, -0.01% error
                        //.i2s_ckin(49152.kHz()) // 49.152 MHz = 96 kHz * 256 * 2
                        .require_pll48clk()
                        .freeze();

        let mono = cx.device.TIM5.monotonic_us(&clocks);

        let (producer, consumer) = cx.local.buffer.try_split().unwrap();

        let gpioa = cx.device.GPIOA.split();

        let tim2_etr = gpioa.pa0.into_alternate();
        let sof_timer = SofTimer::new(cx.device.TIM2, tim2_etr);

        let usb = USB {
            usb_global: cx.device.OTG_FS_GLOBAL,
            usb_device: cx.device.OTG_FS_DEVICE,
            usb_pwrclk: cx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

        *cx.local.usb_bus = Some(UsbBus::new(usb, cx.local.usb_ep_memory));

        let serial = SerialPort::new(&cx.local.usb_bus.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(cx.local.usb_bus.as_ref().unwrap(), UsbVidPid(0x1209, 0x0001))
            .manufacturer("Crabs Pty Ltd.")
            .product("CrabDAC")
            .serial_number("420.69")
            .composite_with_iads()
            .max_packet_size_0(8)
            .self_powered(true)
            .build();

        (
            Shared {
            },
            Local {
                producer,
                consumer,
                sof_timer,
                usb_dev,
                serial,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    #[inline(never)]
    #[link_section = ".data.usb_handler"]
    #[task(binds = OTG_FS, local = [producer, usb_dev, serial])]
    fn usb_handler(cx: usb_handler::Context) {
        let producer: &mut bbqueue::Producer<'static, BUFFER_SIZE> = cx.local.producer;
        let usb_dev: &mut UsbDevice<UsbBusType> = cx.local.usb_dev;
        let serial: &mut SerialPort<UsbBusType> = cx.local.serial;

        // TODO replace this with real USB data
        let bytes_received: usize = MAX_FRAME_SIZE;
        let data: [u8; MAX_FRAME_SIZE] = [0; MAX_FRAME_SIZE];
        let volume_amp: i32 = 1;

        while usb_dev.poll(&mut [serial]) {
        }

        //match producer.grant_exact(bytes_received) {
        //    Ok(mut grant) => {
        //        data[0..bytes_received].as_slice_of::<i32>().unwrap().iter()
        //            .zip(grant.as_mut_slice_of::<i32>().unwrap())
        //            .for_each(|(sample, dest)| *dest = sample * volume_amp);
        //        grant.commit(bytes_received);
        //    },
        //    Err(e) => defmt::warn!("USB :: Audio Buffer Overflow {:?}", defmt::Debug2Format(&e)),
        //}
    }
}
