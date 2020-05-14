#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_semihosting::hprintln;
// Halt on panic
use panic_semihosting as _;
use rtfm::app;
use rtfm::cyccnt::Instant;
use stm32f4::stm32f446::{Interrupt, TIM1};
use stm32f4xx_hal::{prelude::*, pwm, timer};

type PWM0_CHANNEL = pwm::PwmChannels<TIM1, pwm::C1>;

#[app(device = stm32f4::stm32f446, peripherals = true)]
const APP: () = {
    struct Resources {
        pwm0: PWM0_CHANNEL
    }
    #[init]
    fn init(context: init::Context) -> init::LateResources {
        hprintln!("hello world!").unwrap();
        let rcc = context.device.RCC.constrain();
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

        let mut timer = timer::Timer::tim3(context.device.TIM3, 1.hz(), clocks);
        timer.listen(timer::Event::TimeOut);

        ch1.set_duty(to_scale(max_duty, min_duty, 0.0));
        ch1.enable();
        init::LateResources {
            pwm0: ch1
        }
    }
    #[task(binds = TIM3)]
    fn tim3_interrupt(_: tim3_interrupt::Context) {
        hprintln!("tick!").unwrap();
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