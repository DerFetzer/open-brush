use cortex_m::prelude::_embedded_hal_PwmPin;
use stm32l0xx_hal::{
    gpio::{
        gpioa::{PA0, PA2},
        gpiob::PB3,
        Output, PushPull,
    },
    pac::{GPIOA, RCC, TIM2, TIM21},
    prelude::*,
    pwm::{Assigned, Pwm, Timer, C1, C2},
    rcc::Rcc,
};

#[derive(Debug, Clone, Copy)]
pub enum MotorMode {
    Normal,
    Sensitive,
    Polish,
    Massage,
}

impl MotorMode {
    fn get_freq(&self) -> (u16, u16) {
        match self {
            MotorMode::Normal => (250, 250),
            MotorMode::Sensitive => (213, 213),
            MotorMode::Polish => (250, 250),
            MotorMode::Massage => (310, 250),
        }
    }

    fn get_duty_percent(&self) -> (u8, u8) {
        match self {
            MotorMode::Normal => (70, 70),
            MotorMode::Sensitive => (70, 70),
            MotorMode::Polish => (65, 65),
            MotorMode::Massage => (55, 70),
        }
    }

    pub fn get_next_mode(&self) -> Self {
        match self {
            MotorMode::Normal => MotorMode::Sensitive,
            MotorMode::Sensitive => MotorMode::Polish,
            MotorMode::Polish => MotorMode::Massage,
            MotorMode::Massage => MotorMode::Normal,
        }
    }
}

pub struct Motor {
    is_running: bool,
    in_second_cycle: bool,
    current_mode: MotorMode,
    pwm_a_chan1: Pwm<TIM2, C1, Assigned<PA0<Output<PushPull>>>>,
    pwm_a_chan2: Pwm<TIM2, C2, Assigned<PB3<Output<PushPull>>>>,
    rcc: Rcc,
}

impl Motor {
    pub fn new(
        pwm_a: Timer<TIM2>,
        mota: PA0<Output<PushPull>>,
        dummy: PB3<Output<PushPull>>,
        /*pwm_b: Timer<TIM21>, */
        _motb: PA2<Output<PushPull>>,
        rcc: Rcc,
    ) -> Self {
        let mut pwm_a_chan1 = pwm_a.channel1.assign(mota);
        pwm_a_chan1.set_duty(pwm_a_chan1.get_max_duty() / 10 * 7);
        let mut pwm_a_chan2 = pwm_a.channel2.assign(dummy);
        pwm_a_chan2.set_duty(pwm_a_chan2.get_max_duty() / 2);

        // PWM not yet implemented for TIM21
        // let pwm_b = pwm::Timer::new(dp.TIM21, 250.Hz(), &mut rcc);

        let mut motor = Motor {
            is_running: false,
            in_second_cycle: false,
            current_mode: MotorMode::Normal,
            pwm_a_chan1,
            pwm_a_chan2,
            rcc,
        };
        motor.stop();

        unsafe {
            let rcc = &(*RCC::ptr());

            let tim2 = &(*TIM2::ptr());
            let tim21 = &(*TIM21::ptr());

            // Configure Timers

            // Enable
            rcc.apb2enr.modify(|_, w| w.tim21en().set_bit());
            cortex_m::asm::dmb();
            rcc.apb2rstr.modify(|_, w| w.tim21rst().set_bit());
            rcc.apb2rstr.modify(|_, w| w.tim21rst().clear_bit());

            motor.configure_mode(MotorMode::Normal);

            // Configure channels
            tim2.ccmr1_output().modify(|_, w| {
                w.oc1pe()
                    .set_bit()
                    .oc1m()
                    .pwm_mode1()
                    .oc2pe()
                    .set_bit()
                    .oc2m()
                    .pwm_mode2()
            });
            tim2.ccer.modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());
            tim21
                .ccmr1_output()
                .modify(|_, w| w.oc1pe().set_bit().oc1m().pwm_mode1());
            tim21.ccer.modify(|_, w| w.cc1e().set_bit());

            // Configure master/slave
            tim2.cr2.modify(|_, w| w.mms().compare_oc2());
            tim21.smcr.modify(|_, w| w.ts().itr0().sms().trigger_mode());
        }
        motor
    }
}

impl Motor {
    fn copy_pwm_registers(&mut self) {
        unsafe {
            let tim2 = &(*TIM2::ptr());
            let tim21 = &(*TIM21::ptr());

            // Copy registers from TIM2 to TIM21
            tim21
                .psc
                .write(|w| w.psc().bits(tim2.psc.read().psc().bits()));
            tim21
                .arr
                .write(|w| w.arr().bits(tim2.arr.read().arr().bits()));
            tim21
                .ccr1
                .write(|w| w.ccr().bits(tim2.ccr1.read().ccr().bits()));
        }
    }

    fn configure_mode(&mut self, mode: MotorMode) {
        self.current_mode = mode;

        let freq = mode.get_freq();
        let duty_percent = mode.get_duty_percent();

        let freq = if !self.in_second_cycle {
            freq.0
        } else {
            freq.1
        };

        let duty_percent = if !self.in_second_cycle {
            duty_percent.0
        } else {
            duty_percent.1
        };

        self.pwm_a_chan1
            .set_frequency((freq as u32).Hz(), &self.rcc);
        self.pwm_a_chan1
            .set_duty(self.pwm_a_chan1.get_max_duty() / 100 * duty_percent as u16);
        self.pwm_a_chan2
            .set_duty(self.pwm_a_chan2.get_max_duty() / 2);

        self.copy_pwm_registers();
    }

    pub fn start(&mut self, mode: MotorMode) {
        self.stop();
        self.in_second_cycle = false;
        self.configure_mode(mode);
        unsafe {
            let tim2 = &(*TIM2::ptr());
            let tim21 = &(*TIM21::ptr());

            tim21.cnt.reset();
            tim2.cnt.reset();

            tim2.cr1.modify(|_, w| w.cen().set_bit());
        }
        self.enable_pins();
        self.is_running = true;
    }

    pub fn stop(&mut self) {
        unsafe {
            let tim2 = &(*TIM2::ptr());
            let tim21 = &(*TIM21::ptr());

            tim2.cr1.modify(|_, w| w.cen().clear_bit());
            tim21.cr1.modify(|_, w| w.cen().clear_bit());
        }
        self.disable_pins();
        self.is_running = false;
    }

    pub fn disable_pins(&self) {
        unsafe {
            // GPIOs as Output
            let gpiob = &(*GPIOA::ptr());
            gpiob
                .moder
                .modify(|_, w| w.mode0().output().mode2().output());
            gpiob.bsrr.write(|w| w.br0().set_bit().br2().set_bit());
        }
    }

    pub fn enable_pins(&self) {
        unsafe {
            // GPIOs as AF
            let gpioa = &(*GPIOA::ptr());
            gpioa
                .moder
                .modify(|_, w| w.mode0().alternate().mode2().alternate());
            gpioa.afrl.modify(|_, w| w.afsel0().af2().afsel2().af0());
        }
    }

    pub fn is_running(&self) -> bool {
        self.is_running
    }

    pub fn get_mode(&self) -> MotorMode {
        self.current_mode
    }

    pub fn handle_cycle_timer(&mut self) {
        self.in_second_cycle = !self.in_second_cycle;
        self.configure_mode(self.current_mode);

        // Sync TIM21
        unsafe {
            let tim21 = &(*TIM21::ptr());

            tim21.cr1.modify(|_, w| w.cen().clear_bit());
            tim21.cnt.reset();
        }
    }
}
