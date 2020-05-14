#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_semihosting::hprintln;
use heapless;
use heapless::consts::U64;
// Halt on panic
use panic_semihosting as _;
use rtfm::app;
use stm32f4::stm32f446::Interrupt;
use stm32f4xx_hal::{gpio, pwm, prelude::*};
type PWM0_CHANNEL = gpio::gpioa::PA8<gpio::Alternate<gpio::AF1>>;

#[app(device = stm32f4::stm32f446, peripherals = true)]
const APP: () = {
    #[init]
    fn init(context: init::Context) {
        hprintln!("hello world!").unwrap();
        let mut rcc = context.device.RCC.constrain();
        let clocks = rcc.cfgr.freeze();

        let gpioa = context.device.GPIOA.split();
        let channels = (
            gpioa.pa8.into_alternate_af1(),
            gpioa.pa9.into_alternate_af1(),
        );
        // configure TIM1 for PWM
        let pwm = pwm::tim1(context.device.TIM1, channels, clocks, 501.hz());
        // each TIM has two channels
        let (mut ch1, _ch2) = pwm;
        let max_duty = ch1.get_max_duty();
        let min_duty = max_duty / 2;

        ch1.set_duty(to_scale(max_duty, min_duty, 0.0));
        ch1.enable();
    }
};

/// Convert `value` from [-1,1] to [new_min, new_max]
fn to_scale(new_max: u16, new_min: u16, value: f32) -> u16 {
    let new_max = new_max as f32;
    let new_min = new_min as f32;
    let value = value as f32;
    let old_min: f32 = -1.0;
    let old_max: f32 = 1.0;
    let old_range = old_max - old_min;  // 1- -1
    let new_range = new_max - new_min;
    let new_value = (((value - old_min) * new_range) / old_range) + new_min;
    new_value as u16
}