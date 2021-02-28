use stm32f4xx_hal::{
    stm32::{RCC, TIM8},
    timer::{PinC1, PinC2},
};

pub struct PwmInput<TIM, PINS> {
    tim: TIM,
    pins: PINS,
}

pub trait Pins<TIM> {}

// implement the `Pins` trait wherever PC1 implements PinC1 and PC2 implements PinC2 for the given TIMer
impl<TIM, PC1, PC2> Pins<TIM> for (PC1, PC2)
where
    PC1: PinC1<TIM>,
    PC2: PinC2<TIM>,
{
}

impl<PINS> PwmInput<TIM8, PINS> {
    /// Configures a TIM peripheral as a PWM input
    pub fn tim8(tim: TIM8, pins: PINS) -> Self
    where
        PINS: Pins<TIM8>,
    {
        // NOTE(unsafe) this reference will only be used for atomic writes with no side effects.
        let rcc = unsafe { &(*RCC::ptr()) };
        // enable and reset clock.
        rcc.apb2enr.modify(|_, w| w.tim8en().set_bit());
        rcc.apb2rstr.modify(|_, w| w.tim8rst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.tim8rst().clear_bit());

        // Configure TxC1 and TxC2 as captures
        tim.ccmr1_output()
            // Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1 register (TI1 selected).
            // Select the active input for TIMx_CCR2: write the CC2S bits to 10 in the TIMx_CCMR1 register (TI1 selected).
            .write(|w| unsafe { w.cc1s().bits(0b01).cc2s().bits(0b10) });

        // enable and configure to capture on rising edge
        tim.ccer.write(|w| {
            // Select the active polarity for TI1FP1
            // (used both for capture in TIMx_CCR1 and counter clear):
            // write the CC1P and CC1NP bits to ‘0’ (active on rising edge).
            w.cc1np()
                .clear_bit()
                .cc1p()
                .clear_bit()
                // Select the active polarity for TI1FP2 (used for capture in TIMx_CCR2): write the
                // CC2P and CC2NP bits to ‘1’ (active on falling edge).
                .cc2np()
                .set_bit()
                .cc2p()
                .set_bit()
        });

        // some chip variants declare `.bits()` as unsafe, some don't
        #[allow(unused_unsafe)]
        tim.smcr.write(|w| unsafe {
            w
                // Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register
                // (TI1FP1 selected).
                .ts()
                .bits(0b101)
                // Configure the slave mode controller in reset mode: write the SMS bits to 100 in the TIMx_SMCR register.
                .sms()
                .bits(0b100)
        });
        #[allow(unused_unsafe)]
        tim.ccer
            .write(|w| unsafe { w.cc1e().set_bit().cc2e().set_bit() });

        // auto reload register
        // tim.arr.write(|w| unsafe { w.bits(core::u32::MAX) });
        // counter enable
        // tim.cr1.write(|w| w.cen().set_bit());

        PwmInput { tim, pins }
    }

    /// Releases the TIM peripheral and QEI pins
    pub fn release(self) -> (TIM8, PINS) {
        (self.tim, self.pins)
    }

    pub fn get_period(&self) -> u16 {
        self.tim.ccr1.read().bits() as u16
    }
    pub fn get_duty_cycle(&self) -> u16 {
        self.tim.ccr2.read().bits() as u16
    }
}
