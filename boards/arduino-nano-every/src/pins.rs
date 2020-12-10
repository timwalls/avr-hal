use crate::hal::port::PortExt;

avr_hal_generic::impl_board_pins! {
    #[port_defs]
    use crate::hal::port;

    /// Generic DDR that works for all ports
    pub struct DDR {
        porta: crate::pac::PORTA,
        portb: crate::pac::PORTB,
        portc: crate::pac::PORTC,
        portd: crate::pac::PORTD,
        porte: crate::pac::PORTE,
        portf: crate::pac::PORTF,
    }

    /// Reexport of the Nano Every's pins, with the names they have on the board
    pub struct Pins {
        /// `A0`
        ///
        /// * ADC0 (ADC input channel 0)
        pub a0: portd::pd3::PD3,

        /// `A1`
        ///
        /// * ADC1 (ADC input channel 1)
        /// * PCINT9 (pin change interrupt 9)
        pub a1: portd::pd2::PD2,

        /// `A2`
        ///
        /// * ADC2 (ADC input channel 2)
        pub a2: portd::pd1::PD1,

        /// `A3`
        ///
        /// * ADC3 (ADC input channel 3)
        pub a3: portd::pd0::PD0,

        /// `A4`
        ///
        /// * ADC4 (ADC input channel 4)
        /// * SDA (2-wire serial bus data input/output line)
        pub a4: porta::pa2::PA2,

        /// `A5`
        ///
        /// ADC5 (ADC input channel 5)
        /// SCL (2-wire serial bus clock line)
        pub a5: porta::pa3::PA3,

        /// `A6`
        ///
        /// * ADC2 (ADC input channel 2)
        pub a6: portd::pd4::PD4,

        /// `A7`
        ///
        /// * ADC2 (ADC input channel 2)
        pub a7: portd::pd5::PD5,

        /// `D0` / `RX`
        ///
        /// * RXD (USART input pin)
        pub d0: portc::pc5::PC5,

        /// `D1` / `TX`
        ///
        /// * TXD (USART output pin)
        pub d1: portc::pc4::PC4,

        /// `D2`
        ///
        pub d2: porta::pa0::PA0,

        /// `D3`
        ///
        /// * **PWM**: [atmega328p_hal::timer::Timer3Pwm]
        /// * OC2B (Timer/Counter2 output compare match B output)
        pub d3: portf::pf5::PF5,

        /// `D4`
        ///
        pub d4: portc::pc6::PC6,

        /// `D5`
        ///
        /// * **PWM**: [atmega328p_hal::timer::Timer3Pwm]
        pub d5: portb::pb2::PB2,

        /// `D6`
        ///
        /// * **PWM**: [atmega328p_hal::timer::Timer3Pwm]
        pub d6: portf::pf4::PF4,

        /// `D7`
        ///
        pub d7: porta::pa1::PA1,

        /// `D8`
        ///
        pub d8: porte::pe3::PE3,

        /// `D9`
        ///
        /// * **PWM**: [atmega328p_hal::timer::Timer3Pwm]
        pub d9: portb::pb0::PB0,

        /// `D10`
        ///
        /// * **PWM**: [atmega328p_hal::timer::Timer3Pwm]
        pub d10: portb::pb1::PB1,

        /// `D11`
        ///
        /// * MOSI (SPI bus master/slave input)
        pub d11: porte::pe0::PE0,

        /// `D12`
        ///
        /// * MISO (SPI bus master input/slave output)
        pub d12: porte::pe1::PE1,

        /// `D13`
        ///
        /// * SCK (SPI bus master clock input)
        /// * L LED on Arduino Nano Every
        pub d13: porte::pe2::PE2,

        /// `led_tx`
        ///
        /// * TX0 (TX LED)
        pub led_tx: portb::pb4::PB4,

        /// `led_rx`
        ///
        /// * RX0 (RX LED)
        pub led_rx: portb::pb5::PB5,
    }
}
