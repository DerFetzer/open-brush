use cortex_m::prelude::_embedded_hal_PwmPin;
use stm32l0xx_hal::{
    gpio::{
        gpioa::{PA0, PA2},
        gpiob::PB3,
        Output, PushPull,
    },
    pac::{GPIOA, RCC, TIM2, TIM21},
    pwm::Timer,
};

pub struct Motor {
    is_running: bool,
}

impl Motor {
    pub fn new(
        pwm_a: Timer<TIM2>,
        mota: PA0<Output<PushPull>>,
        dummy: PB3<Output<PushPull>>,
        /*pwm_b: Timer<TIM21>, */
        _motb: PA2<Output<PushPull>>,
    ) -> Self {
        let mut pwm_a_chan1 = pwm_a.channel1.assign(mota);
        pwm_a_chan1.set_duty(pwm_a_chan1.get_max_duty() / 10 * 7);
        let mut pwm_a_chan2 = pwm_a.channel2.assign(dummy);
        pwm_a_chan2.set_duty(pwm_a_chan2.get_max_duty() / 2);

        // PWM not yet implemented for TIM21
        // let pwm_b = pwm::Timer::new(dp.TIM21, 250.Hz(), &mut rcc);

        let mut motor = Motor { is_running: false };
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

            // Copy registers from TIM2
            tim21
                .psc
                .write(|w| w.psc().bits(tim2.psc.read().psc().bits()));
            tim21
                .arr
                .write(|w| w.arr().bits(tim2.arr.read().arr().bits()));
            tim21
                .ccr1
                .write(|w| w.ccr().bits(tim2.ccr1.read().ccr().bits()));

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
    pub fn start(&mut self) {
        self.stop();
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
}
