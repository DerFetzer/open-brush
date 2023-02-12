use stm32l0xx_hal::prelude::OutputPin;

pub struct Leds<L: OutputPin> {
    pub led1: L,
    pub led2: L,
    pub led3: L,
    pub led4: L,
    pub led5_g: L,
    pub led5_r: L,
}
