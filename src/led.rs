use stm32l0xx_hal::{
    gpio::{Output, Pin, PushPull},
    prelude::OutputPin,
};

pub struct Leds {
    pub led1: Pin<Output<PushPull>>,
    pub led2: Pin<Output<PushPull>>,
    pub led3: Pin<Output<PushPull>>,
    pub led4: Pin<Output<PushPull>>,
    pub led5_g: Pin<Output<PushPull>>,
    pub led5_r: Pin<Output<PushPull>>,
}

impl Leds {
    pub fn off(&mut self) {
        self.led1.set_low().unwrap();
        self.led2.set_low().unwrap();
        self.led3.set_low().unwrap();
        self.led4.set_low().unwrap();
        self.led5_g.set_low().unwrap();
        self.led5_r.set_low().unwrap();
    }
}
