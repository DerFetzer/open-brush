use crate::UnwrapNoFmt;
use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::Format;
use stm32l0xx_hal::{
    adc::{Adc, Ready, SampleTime},
    gpio::{gpiob::PB0, Analog, Floating, Input, Pin},
    prelude::InputPin,
};
use systick_monotonic::fugit::Instant;

pub struct Battery<const NOM: u32, const DENOM: u32> {
    adc: Adc<Ready>,
    vbat: PB0<Analog>,
    chg_det: Pin<Input<Floating>>,
    last_falling_edge: Option<Instant<u64, NOM, DENOM>>,
    charging_state: ChargingState,
}

#[derive(Clone, Copy, Format, PartialEq, Eq)]
pub enum ChargingState {
    Unknown,
    Off,
    Charging,
    Full,
}

impl<const NOM: u32, const DENOM: u32> Battery<NOM, DENOM> {
    pub fn new(mut adc: Adc<Ready>, vbat: PB0<Analog>, chg_det: Pin<Input<Floating>>) -> Self {
        adc.set_sample_time(SampleTime::T_39_5);
        Self {
            adc,
            vbat,
            chg_det,
            last_falling_edge: None,
            charging_state: ChargingState::Off,
        }
    }

    pub fn get_battery_voltage_mv(&mut self) -> u16 {
        let reading: u16 = self.adc.read(&mut self.vbat).unwrap_no_fmt();
        (reading as u32 * 3000 / 2_u32.pow(12)) as u16 * 2
    }

    pub fn handle_chg_det_edge(&mut self, now: Instant<u64, NOM, DENOM>) -> ChargingState {
        if self.chg_det.is_high().unwrap_no_fmt() {
            if let Some(last_falling_edge) = self.last_falling_edge {
                match now.checked_duration_since(last_falling_edge) {
                    Some(duration) if duration.to_millis() < 1000 => {
                        self.charging_state = ChargingState::Charging
                    }
                    Some(_) if self.charging_state == ChargingState::Off => {
                        self.charging_state = ChargingState::Charging
                    }
                    Some(_) => self.charging_state = ChargingState::Full,
                    None => {}
                }
            }
            self.last_falling_edge = None;
        } else {
            self.last_falling_edge = Some(now);
        }
        self.charging_state
    }

    pub fn handle_edge_timeout(&mut self) -> ChargingState {
        if self.chg_det.is_high().unwrap_no_fmt() {
            self.charging_state = ChargingState::Full
        } else {
            self.charging_state = ChargingState::Off
        }
        self.charging_state
    }
}
