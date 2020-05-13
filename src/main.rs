#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m;
use cortex_m_rt::entry;
// Halt on panic
use panic_semihosting as _;
use stm32f4xx_hal::{delay as hal_delay, prelude::*, pwm, stm32, };

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        stm32::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        // Set up the system clock.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();

        let mut delay = hal_delay::Delay::new(cp.SYST, clocks);

        let gpioa = dp.GPIOA.split();
        let channels = (
            gpioa.pa8.into_alternate_af1(),
            gpioa.pa9.into_alternate_af1(),
        );
        // configure TIM1 for PWM
        let pwm = pwm::tim1(dp.TIM1, channels, clocks, 501.hz());
        // each TIM has two channels
        let (mut ch1, mut ch2) = pwm;
        let max_duty = ch1.get_max_duty();
        let min_duty = max_duty / 2;

        let center = max_duty as f32 * 0.75;
        ch1.set_duty(center as u16);
        ch2.set_duty(to_scale(max_duty, min_duty, 0.0));
        ch1.enable();
        ch2.enable();
        loop {
            ch2.set_duty(to_scale(max_duty, min_duty, 0.25));
            delay.delay_ms(1000_u32);
            ch2.set_duty(to_scale(max_duty, min_duty, -0.25));
            delay.delay_ms(1000_u32);
            ch2.set_duty(to_scale(max_duty, min_duty, 0.0));
            delay.delay_ms(1000_u32);

        }
    }

    loop {}
}

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