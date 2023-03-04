use stm32l0xx_hal::pac::{LPTIM, RCC};

const LSI_FREQ: u32 = 37_000;
pub const PRESC: u8 = 128;
const LPTIM_FREQ: u32 = LSI_FREQ / PRESC as u32;

pub struct Prescaler(u8);

impl Prescaler {
    pub const fn new(prescaler: u8) -> Self {
        match prescaler {
            1 | 2 | 4 | 8 | 16 | 32 | 64 | 128 => Self(prescaler),
            _ => panic!("Invalid prescaler value!"),
        }
    }

    const fn as_presc(&self) -> u8 {
        self.0.ilog2() as u8
    }
}

pub struct LpTimerMono {
    timer: LPTIM,
    overflow: u64,
}

impl LpTimerMono {
    pub fn new(timer: LPTIM, prescaler: Prescaler) -> Self {
        assert_eq!(prescaler.0, PRESC);

        let rcc = unsafe { &(*RCC::PTR) };

        // Enable LSI
        rcc.csr.modify(|_, w| w.lsion().set_bit());
        while rcc.csr.read().lsirdy().bit_is_clear() {}

        // Enable LPTIM clock
        rcc.ccipr.modify(|_, w| w.lptim1sel().lsi());

        // Enable and reset LPTIM
        rcc.apb1enr.modify(|_, w| w.lptim1en().set_bit());
        cortex_m::asm::dmb();
        rcc.apb1rstr.modify(|_, w| w.lptim1rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.lptim1rst().clear_bit());

        timer
            .ier
            .modify(|_, w| w.arrmie().set_bit().cmpmie().set_bit());
        timer.cfgr.write(|w| w.presc().bits(prescaler.as_presc()));

        timer.cr.write(|w| w.enable().set_bit());

        cortex_m::asm::delay(5000);

        timer.icr.write(|w| w.arrokcf().set_bit());
        timer.arr.write(|w| w.arr().bits(u16::MAX));
        while timer.isr.read().arrok().bit_is_clear() {}

        Self { timer, overflow: 0 }
    }

    #[inline(always)]
    fn handle_overflow(&mut self) -> bool {
        if self.timer.isr.read().arrm().is_set() {
            self.timer.icr.write(|w| w.arrmcf().set_bit());
            self.overflow = self.overflow.wrapping_add(1);

            true
        } else {
            false
        }
    }
}

impl rtic_monotonic::Monotonic for LpTimerMono {
    type Instant = fugit::TimerInstantU64<LPTIM_FREQ>;
    type Duration = fugit::TimerDurationU64<LPTIM_FREQ>;

    const DISABLE_INTERRUPT_ON_EMPTY_QUEUE: bool = false;

    #[inline(always)]
    fn now(&mut self) -> Self::Instant {
        let cnt = self.timer.cnt.read().cnt().bits();
        self.handle_overflow();

        Self::Instant::from_ticks((self.overflow << 16) | cnt as u64)
    }

    fn set_compare(&mut self, instant: Self::Instant) {
        let now = self.now();
        let cmp = match instant.checked_duration_since(now) {
            Some(diff) if diff.ticks() <= u16::MAX as u64 => {
                instant.duration_since_epoch().ticks() as u16
            }
            _ => 0, // Overflow
        };

        self.timer.icr.write(|w| w.cmpokcf().set_bit());
        self.timer.cmp.write(|w| w.cmp().bits(cmp));
        while self.timer.isr.read().cmpok().bit_is_clear() {}
    }

    fn clear_compare_flag(&mut self) {
        self.timer.icr.write(|w| w.cmpmcf().set_bit());
    }

    #[inline(always)]
    fn zero() -> Self::Instant {
        Self::Instant::from_ticks(0)
    }

    unsafe fn reset(&mut self) {
        self.timer
            .cr
            .write(|w| w.enable().set_bit().cntstrt().set_bit());
    }

    fn on_interrupt(&mut self) {
        self.handle_overflow();
    }
}
