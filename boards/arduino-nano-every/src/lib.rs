//! Board Support Crate for [Arduino Nano Every]
//!
//! This crate provides abstractions for interfacing with the hardware of Arduino Nano Every.  It
//! re-exports functionality from the underlying HAL in ways that make more sense for this
//! particular board.  For example, the pins are named by what is printed on the PCB instead of the
//! MCU names.
//!
//! # Examples
//! A number of examples can be found in the [`examples/`][ex] subdirectory of this crate.
//!
//! [ex]: https://github.com/Rahix/avr-hal/tree/master/boards/arduino-nano-every/examples
//!
//! # Getting Started
//! Please follow the guide from [`avr-hal`'s README][guide] for steps on how to set up a project
//! with this board.  A rough skeleton for an application looks like this:
//!
//! ```no_run
//! #![no_std]
//! #![no_main]
//!
//! // Pull in the panic handler from panic-halt
//! extern crate panic_halt;
//!
//! // The prelude just exports all HAL traits anonymously which makes
//! // all trait methods available.  This is probably something that
//! // should always be added.
//! use arduino_nano_every::prelude::*;
//!
//! // Define the entry-point for the application.  This can only be
//! // done once in the entire dependency tree.
//! #[arduino_nano_every::entry]
//! fn main() -> ! {
//!     // Get the peripheral singletons for interacting with them.
//!     let dp = arduino_nano-every::Peripherals::take().unwrap();
//!
//!     unimplemented!()
//! }
//! ```
//!
//! [guide]: https://github.com/Rahix/avr-hal#starting-your-own-project
//!
//! # Compatible Boards
//! I'm attempting to make this work with the Arduino Nano Every.

#![no_std]

// Expose hal & pac crates
pub use atmega4809_hal as hal;
pub use crate::hal::pac;

/// See [`avr_device::entry`](https://docs.rs/avr-device/latest/avr_device/attr.entry.html).
#[cfg(feature = "rt")]
pub use crate::hal::entry;

pub use crate::pac::Peripherals;

mod pins;
pub use crate::pins::*;

pub mod prelude {
    pub use crate::hal::prelude::*;
// @todo reinstate once we have uart in avr-device/HAL
//    pub use crate::hal::usart::BaudrateArduinoExt as _;
}

/// Busy-Delay
///
/// **Note**: For just delaying, using [`arduino_nano_every::delay_ms()`][delay_ms] or
/// [`arduino_nano-every::delay_us()`][delay_us] is probably the better choice.  This type is more useful
/// when an `embedded-hal` driver needs a delay implementation.
///
/// [delay_ms]: fn.delay_ms.html
/// [delay_us]: fn.delay_us.html
pub type Delay = hal::delay::Delay<hal::clock::MHz16>;

/// Wait (busy spin) for `ms` milliseconds
pub fn delay_ms(ms: u16) {
    use prelude::*;

    Delay::new().delay_ms(ms)
}

/// Wait (busy spin) for `us` microseconds
pub fn delay_us(us: u16) {
    use prelude::*;

    Delay::new().delay_us(us)
}

/// Support for the Serial Peripheral Interface
///
/// # Example
/// For a full example, see [`examples/nano-every-spi-feedback.rs`][ex-spi].  In short:
/// ```no_run
/// let dp = arduino_nano-every::Peripherals::take().unwrap();
///
/// let mut pins = arduino_nano-every::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
///
/// // Create SPI interface.
/// let (mut spi, mut cs) = arduino_nano-every::spi::Spi::new(
///     dp.SPI,
///     pins.d13.into_output(&mut pins.ddr),
///     pins.d11.into_output(&mut pins.ddr),
///     pins.d12.into_pull_up_input(&mut pins.ddr),
///     pins.d10.into_output(&mut pins.ddr),
///     arduino_nano-every::spi::Settings::default(),
/// );
/// ```
///
/// [ex-spi]: https://github.com/Rahix/avr-hal/blob/master/boards/arduino-nano-every/examples/nano-every-spi-feedback.rs
// @todo reinstate this once done in avr-device
// pub mod spi {
//    pub use atmega4809_hal::spi::*;
//}

/// Support for the Analog to Digital Converter
///
/// # Example
/// For a full example, see [`examples/nano-every-adc.rs`][ex-adc].  In short:
/// ```no_run
/// let dp = arduino_nano-every::Peripherals::take().unwrap();
///
/// let mut pins = arduino_nano-every::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
///
/// let mut adc = arduino_nano-every::adc::Adc::new(dp.ADC, arduino_nano-every::adc::AdcSettings::default());
///
/// // Convert pin to Analog input
/// let mut a0 = pins.a0.into_analog_input(&mut adc);
///
/// let aread: u16 = nb::block!{adc.read(&mut a0)}.void_unwrap();
/// ```
///
/// [ex-adc]: https://github.com/Rahix/avr-hal/blob/master/boards/arduino-nano-every/examples/nano-every-adc.rs
// @todo reinstate this once done in avr-device
// pub mod adc {
//    pub use atmega4809_hal::adc::*;
//}

/// Support for PWM pins
///
/// The 3 timers of ATmega328P can be used for PWM on certain pins.
/// The PWM methods are from `embedded_hal::PwmPin`.
///
/// # Example
/// For a full example, see [`examples/nano-every-pwm.rs`][ex-pwm].  In short:
/// ```
/// let mut pins = arduino_nano-every::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
///
/// let mut timer1 = arduino_nano-every::pwm::Timer1Pwm::new(
///     dp.TC1,
///     arduino_nano-every::pwm::Prescaler::Prescale64,
/// );
///
/// let mut pin = pins.d9.into_output(&mut pins.ddr).into_pwm(&mut timer1);
///
/// pin.set_duty(128);
/// pin.enable();
/// ```
///
/// Here is an overview of pins and which timer they work with:
///
/// | Pin | Conversion Method |
/// | --- | --- |
/// | `pins.d3` | `.into_pwm(&mut timer2)` |
/// | `pins.d5` | `.into_pwm(&mut timer0)` |
/// | `pins.d6` | `.into_pwm(&mut timer0)` |
/// | `pins.d9` | `.into_pwm(&mut timer1)` |
/// | `pins.d10` | `.into_pwm(&mut timer1)` |
/// | `pins.d11` | `.into_pwm(&mut timer2)` |
///
/// [ex-pwm]: https://github.com/Rahix/avr-hal/blob/master/boards/arduino-nano-every/examples/nano-every-pwm.rs
// @todo reinstate this once done in avr-device
// pub mod pwm {
//    pub use atmega4809_hal::pwm::*;
//}

/// Serial (UART) interface on pins `D0` (RX) and `D1` (TX)
///
/// # Example
/// For a full example, see [`examples/leonardo-serial.rs`][ex-serial].  In short:
/// ```no_run
/// let dp = arduino_nano-every::Peripherals::take().unwrap();
///
/// let mut pins = arduino_nano-every::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
///
/// let mut serial = arduino_nano-every::Serial::new(
///     dp.USART0,
///     pins.d0,
///     pins.d1.into_output(&mut pins.ddr),
///     57600.into_baudrate(),
/// );
///
/// ufmt::uwriteln!(&mut serial, "Hello from Arduino!\r").void_unwrap();
/// ```
///
/// [ex-serial]: https://github.com/Rahix/avr-hal/blob/master/boards/arduino-nano-every/examples/nano-every-serial.rs
// @todo fix this once we have support in avr-device
//pub type Serial<IMODE> = hal::usart::Usart0<hal::clock::MHz16, IMODE>;

/// I2C Master on pins `A4` (SDA) and `A5` (SCL)
///
/// # Example
/// For a full example, see [`examples/leonardo-i2cdetect.rs`][ex-i2c].  In short:
/// ```no_run
/// let dp = arduino_nano-every::Peripherals::take().unwrap();
///
/// let mut pins = arduino_nano-every::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
///
/// let mut i2c = arduino_nano-every::I2cMaster::new(
///     dp.TWI,
///     pins.a4.into_pull_up_input(&mut pins.ddr),
///     pins.a5.into_pull_up_input(&mut pins.ddr),
///     50000,
/// );
/// ```
///
/// [ex-i2c]: https://github.com/Rahix/avr-hal/blob/master/boards/arduino-nano-every/examples/nano-every-i2cdetect.rs
// @todo reinstate this when we have support in avr-device
// pub type I2cMaster<M> = hal::i2c::I2cMaster<hal::clock::MHz16, M>;
//#[doc(hidden)]
//#[deprecated = "Please use `I2cMaster` instead of `I2c`"]
//pub type I2c<M> = I2cMaster<M>;

/// Support for the WatchDog Timer
///
/// # Note
/// Changing the watchdog configuration requires two separate writes to WDTCSR where the second
/// write must occur within 4 cycles of the first or the configuration will not change. You may need
/// to adjust optimization settings to prevent other operations from being emitted between these two
/// writes.
///
/// # Example
/// ```
/// let mut watchdog = arduino_nano-every::wdt::Wdt::new(&dp.CPU.mcusr, dp.WDT);
/// watchdog.start(arduino_nano-every::wdt::Timeout::Ms8000);
///
/// loop {
///     watchdog.feed();
/// }
/// ```
pub mod wdt {
// @todo reinstate this once we have it in avr-device
//    pub use atmega4809_hal::wdt::*;
}
