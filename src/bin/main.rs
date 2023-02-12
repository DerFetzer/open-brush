#![no_main]
#![no_std]

use open_brush as _; // global logger + panicking-behavior + memory layout
use stm32l0xx_hal as _;

#[rtic::app(device = stm32l0xx_hal::pac, dispatchers = [I2C1, I2C2], peripherals = true)]
mod app {
    use cortex_m::peripheral::SCB;
    use defmt::Format;
    use heapless::spsc::{Consumer, Producer, Queue};
    use open_brush::{
        battery::{Battery, ChargingState},
        led::Leds,
        motor::Motor,
    };
    use stm32l0xx_hal::{
        exti::{Exti, ExtiLine, GpioLine, TriggerEdge},
        gpio::{Input, PullUp, *},
        pac::RCC,
        prelude::*,
        pwm,
        pwr::PWR,
        rcc::Config,
        syscfg::SYSCFG,
    };
    use systick_monotonic::{ExtU64, Systick};

    const BAT_LOW_IDLE: u16 = 3500;
    const BAT_LOW_RUNNING: u16 = 3400;

    #[derive(Debug, Clone, Copy, Format)]
    pub enum Event {
        Sleep,
        Button,
        BatteryLow,
        Charging(ChargingState),
    }

    #[monotonic(binds = SysTick, default = true)]
    type SystickMono = Systick<1_000>;

    // Shared resources go here
    #[shared]
    struct Shared {
        leds: Leds<Pin<Output<PushPull>>>,
        bat: Battery<1, 1000>,
        btn: Pin<Input<PullUp>>,
        btn_in_debounce: bool,
        motor: Motor,
        event_p: Producer<'static, Event, 8>,
    }

    // Local resources go here
    #[local]
    struct Local {
        scb: SCB,
        event_c: Consumer<'static, Event, 8>,
    }

    #[init(local = [QUEUE: Queue<Event, 8> = Queue::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::println!("init");

        let dp: stm32l0xx_hal::pac::Peripherals = cx.device;
        let cp: rtic::export::Peripherals = cx.core;

        let mut rcc = dp.RCC.freeze(Config::hsi16());
        let clocks = rcc.clocks;

        let adc = dp.ADC.constrain(&mut rcc);
        let mut exti = Exti::new(dp.EXTI);
        let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
        let _pwr = PWR::new(dp.PWR, &mut rcc); // Only initialize because StopMode struct cannot be
                                               // used due to lifetime issue
        let scb = cp.SCB;

        let mono = SystickMono::new(cp.SYST, clocks.sys_clk().0);

        // GPIO
        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);

        let led1 = gpioa
            .pa6
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led2 = gpioa
            .pa5
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led3 = gpioa
            .pa4
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led4 = gpiob
            .pb4
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led5_g = gpiob
            .pb5
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led5_r = gpiob
            .pb7
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();

        let vbat = gpiob.pb0.into_analog();

        let mota = gpioa.pa0.into_push_pull_output().set_speed(Speed::Low);
        let motb = gpioa.pa2.into_push_pull_output().set_speed(Speed::Low);

        let btn = gpioa.pa1.into_pull_up_input();
        let chg_det = gpioa.pa3.into_floating_input();

        // PWM
        let pwm_a = pwm::Timer::new(dp.TIM2, 250.Hz(), &mut rcc);

        //EXTI
        let btn_line = GpioLine::from_raw_line(btn.pin_number()).unwrap();
        exti.listen_gpio(&mut syscfg, btn.port(), btn_line, TriggerEdge::Falling);

        let chg_det_line = GpioLine::from_raw_line(chg_det.pin_number()).unwrap();
        exti.listen_gpio(&mut syscfg, chg_det.port(), chg_det_line, TriggerEdge::Both);

        let btn = btn.downgrade();
        let chg_det = chg_det.downgrade();

        let (event_p, event_c): (Producer<'static, Event, 8>, Consumer<'static, Event, 8>) =
            cx.local.QUEUE.split();

        // High level structs
        let leds = Leds {
            led1,
            led2,
            led3,
            led4,
            led5_g,
            led5_r,
        };
        let bat = Battery::new(adc, vbat, chg_det);
        let motor = Motor::new(pwm_a, mota, gpiob.pb3.into_push_pull_output(), motb);

        // Setup the monotonic timer
        (
            Shared {
                leds,
                bat,
                btn,
                btn_in_debounce: false,
                motor,
                event_p,
            },
            Local { scb, event_c },
            init::Monotonics(mono),
        )
    }

    #[idle(local = [event_c, scb])]
    fn idle(cx: idle::Context) -> ! {
        defmt::println!("idle");

        let queue = cx.local.event_c;
        let scb = cx.local.scb;

        let mut sleep_pending = false;
        let mut battery_low = false;

        loop {
            while queue.ready() {
                let event = queue.dequeue().unwrap();
                defmt::println!("Received event: {:?}", event);
                match event {
                    Event::Button | Event::Charging(_) => {
                        sleep_pending = false;
                        scb.clear_sleepdeep()
                    }
                    _ => (),
                }
                match event {
                    Event::Sleep => {
                        sleep_pending = true;
                    }
                    Event::Button => {
                        if battery_low {
                            stop_motor::spawn().unwrap();
                            signal_bat_low::spawn().unwrap();
                        } else {
                            toggle_motor::spawn().unwrap();
                        }
                    }
                    Event::BatteryLow => {
                        battery_low = true;
                        stop_motor::spawn().unwrap();
                        signal_bat_low::spawn().unwrap();
                    }
                    Event::Charging(cs) => match cs {
                        ChargingState::Charging => {
                            battery_low = false;
                            signal_charging::spawn().unwrap();
                        }
                        ChargingState::Full => {
                            signal_full::spawn().unwrap();
                        }
                        ChargingState::Off => {
                            leds_off::spawn(true).unwrap();
                        }
                        _ => {}
                    },
                }
            }
            if sleep_pending {
                scb.set_sleepdeep();
                enter_stop_mode();
                sleep_pending = false;
            }
            // cortex_m::asm::dsb();
            // cortex_m::asm::wfi();
        }
    }

    /// Reimplementation of
    /// https://github.com/stm32-rs/stm32l0xx-hal/blob/d1dc72fa5bc8715cf7c77412a5dbcb010c8c4ade/src/pwr.rs#L283
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
                .lpsdsr()
                .set_bit()
        });

        while pwr.csr.read().wuf().bit_is_set() {}
    }

    #[task(binds = EXTI0_1, shared = [btn_in_debounce])]
    fn exti0_1(mut ctx: exti0_1::Context) {
        defmt::println!("exti0_1");
        // Clear the interrupt flag.
        Exti::unpend(GpioLine::from_raw_line(1).unwrap());
        ctx.shared.btn_in_debounce.lock(|btn_in_debounce| {
            if !*btn_in_debounce {
                *btn_in_debounce = true;
                btn_debounce::spawn_after(20.millis()).unwrap();
            }
        });
    }

    #[task(shared = [bat, btn, motor, btn_in_debounce, event_p])]
    fn btn_debounce(ctx: btn_debounce::Context) {
        defmt::println!("btn_debounce");
        let btn = ctx.shared.btn;
        let btn_in_debounce = ctx.shared.btn_in_debounce;
        let event_p = ctx.shared.event_p;

        (btn, btn_in_debounce, event_p).lock(|btn, btn_in_debounce, event_p| {
            if btn.is_low().unwrap() {
                *btn_in_debounce = false;
                event_p.enqueue(Event::Button).unwrap();
            }
        });
    }

    #[task(shared = [event_p, bat, btn, motor])]
    fn toggle_motor(ctx: toggle_motor::Context) {
        defmt::println!("toggle_motor");
        let bat = ctx.shared.bat;
        let motor = ctx.shared.motor;
        let event_p = ctx.shared.event_p;

        (bat, motor, event_p).lock(|bat, motor, event_p| {
            if bat.get_battery_voltage_mv() < BAT_LOW_IDLE {
                event_p.enqueue(Event::BatteryLow).unwrap();
            } else if motor.is_running() {
                motor.stop();
                event_p.enqueue(Event::Sleep).unwrap();
            } else {
                motor.stop();
                leds_off::spawn(true).unwrap();
            }
        });
    }

    #[task(shared = [event_p, bat])]
    fn check_battery(ctx: check_battery::Context) {
        defmt::println!("check_battery");
        let event_p = ctx.shared.event_p;
        let bat = ctx.shared.bat;

        (event_p, bat).lock(|event_p, bat| {
            if bat.get_battery_voltage_mv() < BAT_LOW_RUNNING {
                event_p.enqueue(Event::BatteryLow).unwrap();
            }
        });
    }

    #[task(shared = [event_p, bat, btn, motor])]
    fn stop_motor(mut ctx: stop_motor::Context) {
        defmt::println!("stop_motor");
        ctx.shared.motor.lock(|motor| {
            motor.stop();
        });
    }

    #[task(shared=[leds], local = [leds_off_handle: Option<leds_off::SystickMono::SpawnHandle> = None])]
    fn signal_bat_low(mut ctx: signal_bat_low::Context) {
        defmt::println!("signal_bat_low");
        ctx.shared.leds.lock(|leds| leds.led5_r.set_high().unwrap());
        let handle = if let Some(Ok(handle)) = ctx
            .local
            .leds_off_handle
            .take()
            .map(|handle| handle.reschedule_after(5.secs()))
        {
            handle
        } else {
            leds_off::spawn_after(5.secs(), true).unwrap()
        };
        ctx.local.leds_off_handle.replace(handle);
    }

    #[task(shared=[leds])]
    fn signal_motor_running(mut ctx: signal_motor_running::Context) {
        defmt::println!("signal_motor_running");
        ctx.shared.leds.lock(|leds| leds.led1.set_high().unwrap());
    }

    #[task(shared=[leds])]
    fn signal_charging(mut ctx: signal_charging::Context) {
        defmt::println!("signal_charging");
        ctx.shared.leds.lock(|leds| {
            leds.led5_r.set_high().unwrap();
            leds.led5_g.set_low().unwrap();
        });
    }

    #[task(shared=[leds])]
    fn signal_full(mut ctx: signal_full::Context) {
        defmt::println!("signal_full");
        ctx.shared.leds.lock(|leds| {
            leds.led5_r.set_low().unwrap();
            leds.led5_g.set_high().unwrap();
        });
    }

    #[task(shared=[leds, event_p])]
    fn leds_off(mut ctx: leds_off::Context, sleep: bool) {
        defmt::println!("leds_off");
        ctx.shared.leds.lock(|leds| {
            leds.led1.set_low().unwrap();
            leds.led2.set_low().unwrap();
            leds.led3.set_low().unwrap();
            leds.led4.set_low().unwrap();
            leds.led5_g.set_low().unwrap();
            leds.led5_r.set_low().unwrap();
        });
        if sleep {
            ctx.shared
                .event_p
                .lock(|event_p| event_p.enqueue(Event::Sleep).unwrap());
        }
    }

    #[task(binds = EXTI2_3, shared = [event_p, bat], local = [handle_chg_det_static: Option<chg_det_static::SystickMono::SpawnHandle> = None])]
    fn exti2_3(ctx: exti2_3::Context) {
        defmt::println!("exti2_3");
        // Clear the interrupt flag.
        Exti::unpend(GpioLine::from_raw_line(3).unwrap());

        let event_p = ctx.shared.event_p;
        let bat = ctx.shared.bat;

        let now = monotonics::now();

        (event_p, bat).lock(|event_p, bat| {
            event_p
                .enqueue(Event::Charging(bat.handle_chg_det_edge(now)))
                .unwrap()
        });

        let handle = if let Some(Ok(handle)) = ctx
            .local
            .handle_chg_det_static
            .take()
            .map(|handle| handle.reschedule_after(4.secs()))
        {
            handle
        } else {
            chg_det_static::spawn_after(4.secs()).unwrap()
        };
        ctx.local.handle_chg_det_static.replace(handle);
    }

    #[task(shared = [event_p, bat])]
    fn chg_det_static(ctx: chg_det_static::Context) {
        defmt::println!("chg_det_static");
        let event_p = ctx.shared.event_p;
        let bat = ctx.shared.bat;

        (event_p, bat).lock(|event_p, bat| {
            event_p
                .enqueue(Event::Charging(bat.handle_edge_timeout()))
                .unwrap()
        });
    }
}
