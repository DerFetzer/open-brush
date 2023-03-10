#![no_main]
#![no_std]

use defmt_rtt as _; // global logger

// use panic_probe as _;
use panic_reset as _;

pub mod battery;
pub mod led;
pub mod monotonic;
pub mod motor;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub trait UnwrapNoFmt<T> {
    fn unwrap_no_fmt(self) -> T;
}

impl<T> UnwrapNoFmt<T> for Option<T> {
    fn unwrap_no_fmt(self) -> T {
        self.unwrap()
    }
}

impl<T, E> UnwrapNoFmt<T> for Result<T, E> {
    fn unwrap_no_fmt(self) -> T {
        match self {
            Ok(o) => o,
            Err(_) => panic!("called `Result::unwrap()` on an `Err` value"),
        }
    }
}

// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
