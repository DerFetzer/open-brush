#![no_main]
#![no_std]

use open_brush as _; // global logger + panicking-behavior + memory layout
use stm32l0xx_hal as _;

#[rtic::app(device = stm32l0xx_hal::pac, dispatchers = [I2C1, I2C2], peripherals = true)]
mod app {

    use open_brush::monotonic::Prescaler;
    use stm32l0xx_hal::{gpio::*, pac::RCC, prelude::*, pwr::PWR, rcc::Config};
    use systick_monotonic::ExtU64;

    #[monotonic(binds = LPTIM1, default = true)]
    type Mono = open_brush::monotonic::LpTimerMono;

    // Shared resources go here
    #[shared]
    struct Shared {
        led: Pin<Output<PushPull>>,
    }

    // Local resources go here
    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::println!("init");

        let dp: stm32l0xx_hal::pac::Peripherals = cx.device;
        let cp: rtic::export::Peripherals = cx.core;

        let mut rcc = dp.RCC.freeze(Config::hsi16());
        let mut scb = cp.SCB;

        scb.set_sleepdeep();

        let _pwr = PWR::new(dp.PWR, &mut rcc); // Only initialize because StopMode struct cannot be
                                               // used due to lifetime issue
        let mono = Mono::new(dp.LPTIM, Prescaler::new(128));

        // GPIO
        let gpiob = dp.GPIOB.split(&mut rcc);

        let led = gpiob
            .pb3
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();

        blink::spawn().unwrap();

        // Setup the monotonic timer
        (Shared { led }, Local {}, init::Monotonics(mono))
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        defmt::println!("idle");

        loop {
            enter_stop_mode();
            cortex_m::asm::dsb();
            cortex_m::asm::wfi();
        }
    }

    fn enter_stop_mode() {
        let rcc = unsafe { &(*RCC::PTR) };
        let pwr = unsafe { &(*stm32l0xx_hal::pac::PWR::PTR) };

        rcc.cfgr.modify(|_, w| w.stopwuck().set_bit());
        pwr.cr.modify(|_, w| {
            w.ulp()
                .clear_bit()
                .cwuf()
                .set_bit()
                .pdds()
                .stop_mode()
                // Cat 1 device
                .lpsdsr()
                .clear_bit()
                .lpds()
                .set_bit()
        });

        while pwr.csr.read().wuf().bit_is_set() {}
    }

    #[task(shared = [led])]
    fn blink(mut cx: blink::Context) {
        cx.shared.led.lock(|led| {
            led.toggle().unwrap();
        });
        blink::spawn_after(1.secs()).unwrap();
    }
}
