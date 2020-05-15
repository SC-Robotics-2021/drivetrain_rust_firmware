#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_semihosting::hprintln;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_spi::wrapper::Wrapper;
use ls7366::Ls7366;
// Halt on panic
use panic_semihosting as _;
use rtfm::app;
use stm32f4::stm32f446::TIM1;
use stm32f4xx_hal::{delay, prelude::*, pwm, spi, timer};
use stm32f4xx_hal::delay::Delay;

// Type declaration crap so the resources can be shared...
type Pwm0Channel1 = pwm::PwmChannels<TIM1, pwm::C1>;
// This gets ugly real quick...
type Spi1Sck = stm32f4xx_hal::gpio::gpioa::PA5<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF5>>;
type Spi1Miso = stm32f4xx_hal::gpio::gpioa::PA6<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF5>>;
type Spi1Mosi = stm32f4xx_hal::gpio::gpioa::PA7<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF5>>;
/// Encoder 1 underlying type
type Spi1Underlying = stm32f4xx_hal::spi::Spi<stm32f4::stm32f446::SPI1, (Spi1Sck, Spi1Miso, Spi1Mosi)>;
/// Type for Encoder1's Signal first Select pin
type Spi1SS1 = stm32f4xx_hal::gpio::gpioa::PA1<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>;
// Type of the first encoder
type Encoder1 = Ls7366<Encoder1Wrapper>;

pub struct Encoder1Wrapper {
    spi: Spi1Underlying,
    ss: Spi1SS1,
    delay: Delay
}

impl embedded_hal::blocking::spi::Transfer<u8> for Encoder1Wrapper {
    type Error = stm32f4xx_hal::spi::Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.ss.set_high().unwrap();
        self.ss.set_low().unwrap();
        let result = self.spi.transfer(words)?;
        self.ss.set_high().unwrap();
        Ok(result)
    }
}

impl embedded_hal::blocking::spi::Write<u8> for Encoder1Wrapper {
    type Error = stm32f4xx_hal::spi::Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.ss.set_high().unwrap();
        self.ss.set_low().unwrap();

        let result = self.spi.write(words)?;
        self.ss.set_high().unwrap();
        Ok(result)
    }
}

#[app(device = stm32f4::stm32f446, peripherals = true)]
const APP: () = {
    struct Resources {
        pwm0: Pwm0Channel1,
        spi1: Encoder1,
    }
    #[init]
    fn init(context: init::Context) -> init::LateResources {
        hprintln!("hello world!").unwrap();
        let rcc = context.device.RCC.constrain();
        let clocks = rcc.cfgr.freeze();
        // Create a delay abstraction based on SysTick
        let delay = Delay::new(context.core.SYST, clocks);

        let gpioa = context.device.GPIOA.split();
        let pwm_channels = (
            gpioa.pa8.into_alternate_af1(),
            gpioa.pa9.into_alternate_af1(),
        );
        let mut spi1_nss = gpioa.pa1.into_push_pull_output(); // SPI1_NSS
        spi1_nss.set_low().unwrap();

        let spi1_channels = (
            gpioa.pa5.into_alternate_af5(),  // SPI1_SCK,
            gpioa.pa6.into_alternate_af5(), // SPI1_MISO
            gpioa.pa7.into_alternate_af5(), // SPI1_MOSI
        );
        // configure TIM1 for PWM
        let pwm = pwm::tim1(context.device.TIM1, pwm_channels, clocks, 501.hz());
        // initialize the first of the SPI interfaces to monitor speed
        let spi1 = spi::Spi::spi1(context.device.SPI1, spi1_channels, spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        }, 14_100_000.hz(),
                                  clocks,
        );
        let wrapper = Encoder1Wrapper { spi: spi1, ss: spi1_nss , delay};
        let mut encoder1 = Ls7366::new(wrapper).unwrap();
        let initial_count = encoder1.get_count().unwrap();
        hprintln!("initial count:= {:?}", initial_count).unwrap();
        // each TIM has two channels
        let (mut ch1, _ch2) = pwm;
        let max_duty = ch1.get_max_duty();
        let min_duty = max_duty / 2;

        let mut timer = timer::Timer::tim3(context.device.TIM3, 1.hz(), clocks);
        timer.listen(timer::Event::TimeOut);

        ch1.set_duty(to_scale(max_duty, min_duty, 0.0));
        ch1.enable();
        init::LateResources {
            pwm0: ch1,
            spi1: encoder1,
        }
    }
    #[task(binds = TIM3, resources = [spi1])]
    fn tim3_interrupt(context: tim3_interrupt::Context) {
        let current_count = context.resources.spi1.get_count().unwrap();
        hprintln!("count: {:?}", current_count).unwrap();
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