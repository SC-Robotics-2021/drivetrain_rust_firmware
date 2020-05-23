#![deny(unsafe_code)]
#![no_main]
#![no_std]

use core::ops::DerefMut;

use cobs::decode_in_place;
use cortex_m_semihosting::hprintln;
use heapless::{consts, Vec};
use nb::block;
// Halt on panic
use panic_semihosting as _;
use rtfm::app;
use stm32f4::stm32f446::{TIM1, TIM2};
use stm32f4xx_hal::{gpio, prelude::*, pwm, qei, serial, timer};
use stm32f4xx_hal::delay::Delay;

// Type declaration crap so the resources can be shared...
type Pwm0Channel1 = pwm::PwmChannels<TIM1, pwm::C1>;
type EncoderNEPinA = gpio::gpioa::PA0<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF1>>;
type EncoderNEPinB = gpio::gpioa::PA1<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF1>>;
type EncoderNE = qei::Qei<TIM2, (EncoderNEPinA, EncoderNEPinB)>;

// This gets ugly real quick...
type Spi1Sck = stm32f4xx_hal::gpio::gpioa::PA5<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF5>>;
type Spi1Miso = stm32f4xx_hal::gpio::gpioa::PA6<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF5>>;
type Spi1Mosi = stm32f4xx_hal::gpio::gpioa::PA7<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF5>>;
/// Encoder 1 underlying type
type Spi1Underlying = stm32f4xx_hal::spi::Spi<stm32f4::stm32f446::SPI1, (Spi1Sck, Spi1Miso, Spi1Mosi)>;
/// Type for Encoder1's Signal first Select pin
type Spi1SS1 = stm32f4xx_hal::gpio::gpioa::PA1<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>;

type Uart4Tx = gpio::gpioc::PC10<gpio::Alternate<gpio::AF8>>;
type Uart4Rx = gpio::gpioc::PC11<gpio::Alternate<gpio::AF8>>;
type Uart4 = serial::Serial<stm32f4::stm32f446::UART4, (Uart4Tx, Uart4Rx)>;

pub struct Encoder1Wrapper {
    spi: Spi1Underlying,
    ss: Spi1SS1,
    _delay: Delay,
}

impl embedded_hal::blocking::spi::Transfer<u8> for Encoder1Wrapper {
    type Error = stm32f4xx_hal::spi::Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.ss.set_low().unwrap();
        let result = self.spi.transfer(words)?;
        self.ss.set_high().unwrap();
        Ok(result)
    }
}

impl embedded_hal::blocking::spi::Write<u8> for Encoder1Wrapper {
    type Error = stm32f4xx_hal::spi::Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
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
        encoder_ne: EncoderNE,
        uart4: Uart4,
        rx_buffer: Vec::<u8, consts::U1024>,
        count_ne: u32,
        count_se: u32,
        count_nw: u32,
        count_sw: u32,
    }

    #[init]
    fn init(context: init::Context) -> init::LateResources {
        hprintln!("hello world!").unwrap();
        let rcc = context.device.RCC.constrain();
        let clocks = rcc.cfgr.freeze();
        // Create a delay abstraction based on SysTick
        let _delay = Delay::new(context.core.SYST, clocks);

        let gpioa = context.device.GPIOA.split();
        let gpioc = context.device.GPIOC.split();

        let motor_pwm_channels = (
            gpioa.pa8.into_alternate_af1(),
            gpioa.pa9.into_alternate_af1(),
        );
        let encoder_ne_channels: (EncoderNEPinA, EncoderNEPinB) = (
            gpioa.pa0.into_alternate_af1(),
            gpioa.pa1.into_alternate_af1(),
        );


        let usart2_channels = (
            gpioc.pc10.into_alternate_af8(), // UART4_TX
            gpioc.pc11.into_alternate_af8(), // UART4_RX
        );

        // establish USART2 device
        let mut uart4 = serial::Serial::uart4(
            context.device.UART4,
            usart2_channels,
            serial::config::Config {
                baudrate: 96000.bps(),
                wordlength: serial::config::WordLength::DataBits8,
                parity: serial::config::Parity::ParityNone,
                stopbits: serial::config::StopBits::STOP1,
            },
            clocks,
        ).unwrap();
        // listen for incoming packets
        uart4.listen(serial::Event::Rxne);
        for byte in b"hello from STM32!".iter() {
            block!( uart4.write(*byte)).unwrap();
        }


        // configure TIM1 for PWM output
        let pwm = pwm::tim1(context.device.TIM1, motor_pwm_channels, clocks, 501.hz());
        // each TIM has two channels
        let (mut ch1, _ch2) = pwm;
        let max_duty = ch1.get_max_duty();
        let min_duty = max_duty / 2;

        // configure TIM2 for QEI
        let encoder_ne = qei::Qei::tim2(context.device.TIM2, encoder_ne_channels);

        // create a periodic timer to check the encoders periodically
        let mut timer = timer::Timer::tim3(context.device.TIM3, 1.hz(), clocks);
        timer.listen(timer::Event::TimeOut);

        ch1.set_duty(to_scale(max_duty, min_duty, 0.25));
        ch1.enable();

        let rx_buffer = heapless::Vec::<u8, consts::U1024>::new();

        init::LateResources {
            pwm0: ch1,
            encoder_ne,
            uart4: uart4,
            rx_buffer,
            count_ne: 0,
            count_se: 0,
            count_nw: 0,
            count_sw: 0,
        }
    }
    #[task(binds = TIM3, resources = [encoder_ne, count_ne], priority = 3)]
    fn tim3_interrupt(context: tim3_interrupt::Context) {
        // handle interrupts from TIM3, telling us to look at the encoders.
        let value = context.resources.encoder_ne.count();
        let mut count_ne = context.resources.count_ne;
        // acquire resource lock to prevent concurrent access while we write to it.
        count_ne.lock(|count_ne| {
            // critical section
            *count_ne = value;
        });
    }
    #[task(binds = UART4, resources = [uart4, rx_buffer, count_ne], priority = 10)]
    fn uart4_on_rxne(context: uart4_on_rxne::Context) {
        // these handlers need to be really quick or overruns can occur (NO SEMIHOSTING!)
        let rx_byte = context.resources.uart4.read().unwrap();
        context.resources.rx_buffer.push(rx_byte).unwrap();
        if rx_byte == 0x00 {
            // attempt to decode the buffer in-place
            let decoded_len = decode_in_place(context.resources.rx_buffer.deref_mut()).unwrap();
            // Decoded? good, drop everything less than the decoded length (-1 for sentinel)
            context.resources.rx_buffer.truncate(decoded_len - 1);
            for byte in context.resources.rx_buffer.iter() {
                block!(context.resources.uart4.write(*byte)).unwrap();
            }
            context.resources.rx_buffer.clear();
            for byte in context.resources.count_ne.to_be_bytes().iter() {
                block!(context.resources.uart4.write(*byte)).unwrap();
            }
            block!(context.resources.uart4.write(59)).unwrap();
        }
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