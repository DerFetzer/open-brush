#![no_main]
#![no_std]

use open_brush as _; // global logger + panicking-behavior + memory layout
use stm32l0xx_hal as _;

#[rtic::app(device = stm32l0xx_hal::pac, dispatchers = [I2C1, I2C2], peripherals = true)]
mod app {
    use cortex_m::peripheral::SCB;
    use defmt::Format;
    use fugit::{Duration, TimerInstantU64};
    use heapless::spsc::{Consumer, Producer, Queue};
    use open_brush::{
        battery::{Battery, ChargingState},
        led::Leds,
        monotonic::{Prescaler, PRESC},
        motor::{Motor, MotorMode},
        UnwrapNoFmt,
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
    use systick_monotonic::{fugit::Instant, ExtU64};

    const BAT_LOW: u16 = 3600;
    const BAT_EMPTY: u16 = 3400;
    const BUTTON_HOLD_OFF_MS: u64 = 500;
    const BUTTON_MODE_CHANGE_TIMOUT_MS: u64 = 4000;
    const MOTOR_TIM_MS: u64 = 45;

    #[derive(Clone, Copy, Format)]
    pub enum Event {
        Button,
        Battery(BatteryState),
        Charging(ChargingState),
        LedsOff,
    }

    #[derive(Clone, Copy, Format)]
    pub enum BatteryState {
        Normal,
        Low,
        Empty,
    }

    #[monotonic(binds = LPTIM1, default = true)]
    type Mono = open_brush::monotonic::LpTimerMono;

    // Shared resources go here
    #[shared]
    struct Shared {
        leds: Leds,
        bat: Battery<1, 289>,
        btn: Pin<Input<PullUp>>,
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

        // dp.DBG.cr.write(|w| {
        //     w.dbg_stop()
        //         .set_bit()
        //         .dbg_sleep()
        //         .set_bit()
        //         .dbg_standby()
        //         .set_bit()
        // });
        // dp.RCC.ahbenr.modify(|_, w| w.dmaen().enabled());
        // dp.RCC.apb2enr.modify(|_, w| w.dbgen().enabled());
        // dp.RCC.apb2smenr.modify(|_, w| w.dbgsmen().enabled());

        let dbg_idcode = 0x4001_5800 as *const u32;
        defmt::println!("DBG_IDCODE: {:x}", unsafe { *dbg_idcode });

        let mut rcc = dp.RCC.freeze(Config::hsi16());

        let adc = dp.ADC.constrain(&mut rcc);
        let mut exti = Exti::new(dp.EXTI);
        let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
        let _pwr = PWR::new(dp.PWR, &mut rcc); // Only initialize because StopMode struct cannot be
                                               // used due to lifetime issue
        let scb = cp.SCB;

        let mono = Mono::new(dp.LPTIM, Prescaler::new(PRESC));

        // GPIO
        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);

        let led1 = gpioa
            .pa6
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led2 = gpioa
            .pa4
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led3 = gpioa
            .pa5
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led4 = gpiob
            .pb4
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led5_g = gpiob
            .pb7
            .into_push_pull_output()
            .set_speed(Speed::Low)
            .downgrade();
        let led5_r = gpiob
            .pb5
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
        let btn_line = GpioLine::from_raw_line(btn.pin_number()).unwrap_no_fmt();
        exti.listen_gpio(&mut syscfg, btn.port(), btn_line, TriggerEdge::Falling);

        let chg_det_line = GpioLine::from_raw_line(chg_det.pin_number()).unwrap_no_fmt();
        exti.listen_gpio(&mut syscfg, chg_det.port(), chg_det_line, TriggerEdge::Both);

        let btn = btn.downgrade();
        let chg_det = chg_det.downgrade();

        let (event_p, event_c): (Producer<'static, Event, 8>, Consumer<'static, Event, 8>) =
            cx.local.QUEUE.split();

        // High level structs
        let mut leds = Leds {
            led1,
            led2,
            led3,
            led4,
            led5_g,
            led5_r,
        };

        // Signal Reset
        leds.led2.set_high().unwrap_no_fmt();
        cortex_m::asm::delay(4_000_000);
        leds.led2.set_low().unwrap_no_fmt();

        let bat = Battery::new(adc, vbat, chg_det);
        let motor = Motor::new(pwm_a, mota, gpiob.pb3.into_push_pull_output(), motb, rcc);

        // Setup the monotonic timer
        (
            Shared {
                leds,
                bat,
                btn,
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

        let mut sleep_pending = true;

        let mut battery_empty = false;
        let mut motor_running = false;
        let mut charging = false;

        let mut last_button_press: Option<Instant<u64, 1, 289>> = None;

        let mut leds_off_handle = None;
        let mut check_battery_handle = None;

        let mut current_motor_mode = MotorMode::Normal;

        loop {
            while queue.ready() {
                let event = queue.dequeue().unwrap_no_fmt();
                defmt::println!("Received event: {:?}", event);
                match event {
                    Event::Button | Event::Charging(_) => {
                        sleep_pending = false;
                        scb.clear_sleepdeep()
                    }
                    Event::Battery(_) => {
                        if motor_running {
                            spawn_check_battery(&mut check_battery_handle);
                        }
                    }
                    _ => (),
                }
                match event {
                    Event::Button => {
                        let duration_since_last_press = last_button_press
                            .and_then(|last_button_press| {
                                monotonics::now()
                                    .checked_duration_since(last_button_press)
                                    .map(|d| d.to_millis())
                            })
                            .unwrap_or(BUTTON_HOLD_OFF_MS + 1);
                        if duration_since_last_press > BUTTON_HOLD_OFF_MS {
                            last_button_press = Some(monotonics::now());

                            if battery_empty {
                                stop_motor::spawn().unwrap_no_fmt();
                                motor_running = false;
                                signal_bat_low::spawn().unwrap_no_fmt();
                                spawn_leds_off(&mut leds_off_handle, 5.secs());
                            } else if motor_running {
                                if duration_since_last_press < BUTTON_MODE_CHANGE_TIMOUT_MS {
                                    current_motor_mode = current_motor_mode.get_next_mode();
                                    start_motor::spawn(current_motor_mode).unwrap_no_fmt();
                                    signal_motor_running::spawn(current_motor_mode).unwrap_no_fmt();
                                    if let MotorMode::Massage = current_motor_mode {
                                        let next_spawn_instant =
                                            monotonics::now() + MOTOR_TIM_MS.millis();
                                        start_motor_cycle_timer::spawn_at(
                                            next_spawn_instant,
                                            next_spawn_instant,
                                        )
                                        .unwrap_no_fmt();
                                    }
                                } else {
                                    stop_motor::spawn().unwrap_no_fmt();
                                    motor_running = false;
                                    check_battery_handle.take().map(|h| h.cancel().ok());
                                    spawn_leds_off(&mut leds_off_handle, 500.millis());
                                }
                            } else if !charging {
                                leds_off_handle.take().map(|h| h.cancel().ok());
                                start_motor::spawn(current_motor_mode).unwrap_no_fmt();
                                signal_motor_running::spawn(current_motor_mode).unwrap_no_fmt();
                                check_battery::spawn().unwrap_no_fmt();
                                spawn_check_battery(&mut check_battery_handle);
                                motor_running = true;
                            }
                        } else {
                            defmt::println!("Button event in hold off");
                        }
                    }
                    Event::Battery(BatteryState::Empty) => {
                        battery_empty = true;
                        stop_motor::spawn().unwrap_no_fmt();
                        motor_running = false;
                        signal_bat_low::spawn().unwrap_no_fmt();
                        check_battery_handle.take().map(|h| h.cancel().ok());
                        spawn_leds_off(&mut leds_off_handle, 5.secs());
                    }
                    Event::Battery(BatteryState::Low) => {
                        if motor_running {
                            signal_bat_low::spawn().unwrap_no_fmt();
                        }
                    }
                    Event::Battery(BatteryState::Normal) => {}
                    Event::Charging(cs) => {
                        charging = true;
                        stop_motor::spawn().unwrap_no_fmt();
                        motor_running = false;
                        check_battery_handle.take().map(|h| h.cancel().ok());
                        match cs {
                            ChargingState::Charging => {
                                battery_empty = false;
                                signal_charging::spawn().unwrap_no_fmt();
                            }
                            ChargingState::Full => {
                                battery_empty = false;
                                signal_full::spawn().unwrap_no_fmt();
                            }
                            ChargingState::Off => {
                                charging = false;
                                spawn_leds_off(&mut leds_off_handle, 500.millis());
                            }
                            _ => {}
                        }
                    }
                    Event::LedsOff => {
                        current_motor_mode = MotorMode::Normal;
                        leds_off_handle.take().map(|h| h.cancel().ok());
                        check_battery_handle.take().map(|h| h.cancel().ok());
                        sleep_pending = true;
                    }
                }
            }
            if sleep_pending {
                last_button_press = None;
                scb.set_sleepdeep();
                enter_stop_mode();
                cortex_m::asm::dsb();
            }
            cortex_m::asm::wfi();
        }
    }

    fn spawn_leds_off(
        spawn_handle: &mut Option<leds_off::SpawnHandle>,
        delay: Duration<u64, 1, 289>,
    ) {
        let new_handle = if let Some(Ok(handle)) = spawn_handle
            .take()
            .map(|handle| handle.reschedule_after(delay))
        {
            handle
        } else {
            leds_off::spawn_after(delay).unwrap_no_fmt()
        };
        spawn_handle.replace(new_handle);
    }

    fn spawn_check_battery(spawn_handle: &mut Option<check_battery::SpawnHandle>) {
        let delay = 5.secs();
        let new_handle = if let Some(Ok(handle)) = spawn_handle
            .take()
            .map(|handle| handle.reschedule_after(delay))
        {
            handle
        } else {
            check_battery::spawn_after(delay).unwrap_no_fmt()
        };
        spawn_handle.replace(new_handle);
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
                // Cat 1 device
                .lpsdsr()
                .clear_bit()
                .lpds()
                .set_bit()
        });

        while pwr.csr.read().wuf().bit_is_set() {}
    }

    #[task(binds = EXTI0_1, shared = [event_p])]
    fn exti0_1(mut ctx: exti0_1::Context) {
        defmt::println!("exti0_1");
        // Clear the interrupt flag.
        Exti::unpend(GpioLine::from_raw_line(1).unwrap_no_fmt());
        ctx.shared
            .event_p
            .lock(|event_p| event_p.enqueue(Event::Button).ok());
    }

    #[task(shared = [event_p, bat])]
    fn check_battery(ctx: check_battery::Context) {
        defmt::println!("check_battery");
        let event_p = ctx.shared.event_p;
        let bat = ctx.shared.bat;

        (event_p, bat).lock(|event_p, bat| {
            let voltage = bat.get_battery_voltage_mv();
            defmt::println!("Battery voltage: {}mV", voltage);
            let event = match voltage {
                v if v < BAT_EMPTY => Event::Battery(BatteryState::Empty),
                v if v < BAT_LOW => Event::Battery(BatteryState::Low),
                _ => Event::Battery(BatteryState::Normal),
            };
            event_p.enqueue(event).ok().unwrap_no_fmt();
        });
    }

    #[task(shared = [event_p, btn, motor])]
    fn start_motor(mut ctx: start_motor::Context, mode: MotorMode) {
        defmt::println!("start_motor");

        ctx.shared.motor.lock(|motor| {
            motor.start(mode);
        });
    }

    #[task(shared = [event_p, bat, btn, motor])]
    fn stop_motor(mut ctx: stop_motor::Context) {
        defmt::println!("stop_motor");
        ctx.shared.motor.lock(|motor| {
            motor.stop();
        });
    }

    #[task(shared = [motor])]
    fn start_motor_cycle_timer(
        mut ctx: start_motor_cycle_timer::Context,
        now: TimerInstantU64<289>,
    ) {
        defmt::println!("start_motor_cycle_timer");
        ctx.shared.motor.lock(|motor| {
            if let MotorMode::Massage = motor.get_mode() {
                if motor.is_running() {
                    let next_spawn_instant = now + MOTOR_TIM_MS.millis();
                    start_motor_cycle_timer::spawn_at(next_spawn_instant, next_spawn_instant)
                        .unwrap_no_fmt();
                    motor.handle_cycle_timer();
                }
            }
        });
    }

    #[task(shared=[leds])]
    fn signal_bat_low(mut ctx: signal_bat_low::Context) {
        defmt::println!("signal_bat_low");
        ctx.shared
            .leds
            .lock(|leds| leds.led5_r.set_high().unwrap_no_fmt());
    }

    #[task(shared=[leds])]
    fn signal_motor_running(mut ctx: signal_motor_running::Context, mode: MotorMode) {
        defmt::println!("signal_motor_running");
        ctx.shared.leds.lock(|leds| {
            leds.off();
            match mode {
                MotorMode::Normal => leds.led1.set_high().unwrap_no_fmt(),
                MotorMode::Sensitive => leds.led2.set_high().unwrap_no_fmt(),
                MotorMode::Polish => leds.led3.set_high().unwrap_no_fmt(),
                MotorMode::Massage => leds.led4.set_high().unwrap_no_fmt(),
            }
        });
    }

    #[task(shared=[leds])]
    fn signal_charging(mut ctx: signal_charging::Context) {
        defmt::println!("signal_charging");
        ctx.shared.leds.lock(|leds| {
            leds.led1.set_low().unwrap_no_fmt();
            leds.led5_r.set_high().unwrap_no_fmt();
            leds.led5_g.set_low().unwrap_no_fmt();
        });
    }

    #[task(shared=[leds])]
    fn signal_full(mut ctx: signal_full::Context) {
        defmt::println!("signal_full");
        ctx.shared.leds.lock(|leds| {
            leds.led1.set_low().unwrap_no_fmt();
            leds.led5_r.set_low().unwrap_no_fmt();
            leds.led5_g.set_high().unwrap_no_fmt();
        });
    }

    #[task(shared=[leds, event_p])]
    fn leds_off(mut ctx: leds_off::Context) {
        defmt::println!("leds_off");
        ctx.shared.leds.lock(|leds| {
            leds.off();
        });
        ctx.shared
            .event_p
            .lock(|event_p| event_p.enqueue(Event::LedsOff).ok().unwrap_no_fmt());
    }

    #[task(binds = EXTI2_3, shared = [event_p, bat], local = [handle_chg_det_static: Option<chg_det_static::Mono::SpawnHandle> = None])]
    fn exti2_3(ctx: exti2_3::Context) {
        defmt::println!("exti2_3");
        // Clear the interrupt flag.
        Exti::unpend(GpioLine::from_raw_line(3).unwrap_no_fmt());

        let event_p = ctx.shared.event_p;
        let bat = ctx.shared.bat;

        let now = monotonics::now();

        (event_p, bat).lock(|event_p, bat| {
            event_p
                .enqueue(Event::Charging(bat.handle_chg_det_edge(now)))
                .ok()
                .unwrap_no_fmt()
        });

        let handle = if let Some(Ok(handle)) = ctx
            .local
            .handle_chg_det_static
            .take()
            .map(|handle| handle.reschedule_after(4.secs()))
        {
            handle
        } else {
            chg_det_static::spawn_after(4.secs()).unwrap_no_fmt()
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
                .ok()
                .unwrap_no_fmt()
        });
    }
}
