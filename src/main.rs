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
use stm32f4::stm32f446::{TIM1, TIM2, TIM3, TIM4, TIM5};
use stm32f4xx_hal::{gpio, prelude::*, pwm, qei, serial, timer};
use stm32f4xx_hal::delay::Delay;
use stm32f4xx_hal::gpio::{AF1, AF2, Alternate};

mod postcards;

// Type declaration crap so the resources can be shared...
// NE: tim2
type EncoderNEPinA = gpio::gpiob::PB8<Alternate<AF1>>;
type EncoderNEPinB = gpio::gpiob::PB9<Alternate<AF1>>;
// NW: tim5
type EncoderNWPinA = gpio::gpioa::PA0<Alternate<AF2>>;
type EncoderNWPinB = gpio::gpioa::PA1<Alternate<AF2>>;
// SE: tim3
type EncoderSEPinA = gpio::gpioa::PA6<Alternate<AF2>>;
type EncoderSEPinB = gpio::gpioa::PA7<Alternate<AF2>>;
// SW: tim4
type EncoderSWPinA = gpio::gpiob::PB6<Alternate<AF2>>;
type EncoderSWPinB = gpio::gpiob::PB7<Alternate<AF2>>;

type Uart4Tx = gpio::gpioc::PC10<gpio::Alternate<gpio::AF8>>;
type Uart4Rx = gpio::gpioc::PC11<gpio::Alternate<gpio::AF8>>;
type Uart4 = serial::Serial<stm32f4::stm32f446::UART4, (Uart4Tx, Uart4Rx)>;

pub struct MotorPwm {
    north_west: pwm::PwmChannels<TIM1, pwm::C1>,
    north_east: pwm::PwmChannels<TIM1, pwm::C2>,
    south_east: pwm::PwmChannels<TIM1, pwm::C3>,
    south_west: pwm::PwmChannels<TIM1, pwm::C4>,
}

pub struct MotorEncoders {
    north_west: qei::Qei<TIM5, (EncoderNWPinA, EncoderNWPinB)>,
    north_east: qei::Qei<TIM2, (EncoderNEPinA, EncoderNEPinB)>,
    south_east: qei::Qei<TIM3, (EncoderSEPinA, EncoderSEPinB)>,
    south_west: qei::Qei<TIM4, (EncoderSWPinA, EncoderSWPinB)>,
}

#[app(device = stm32f4::stm32f446, peripherals = true)]
const APP: () = {
    struct Resources {
        motors: MotorPwm,
        encoders: MotorEncoders,
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
        let gpiob = context.device.GPIOB.split();
        let gpioc = context.device.GPIOC.split();

        // TIM1/2 uses AF1 for pwm channels.
        let tim1_pwm_channels = (
            gpioa.pa8.into_alternate_af1(), // ch1 NW
            gpioa.pa9.into_alternate_af1(), // ch2 NE
            gpioa.pa10.into_alternate_af1(), // ch3 SE
            gpioa.pa11.into_alternate_af1(), // ch4 SW
        );

        // AF1 for TIM1/2
        // NE: TIM2
        let encoder_ne_channels: (EncoderNEPinA, EncoderNEPinB) = (
            gpiob.pb8.into_alternate_af1(),
            gpiob.pb9.into_alternate_af1(),
        );
        // AF2 for TIM3/4/5
        // NW: TIM5
        let encoder_nw_channels: (EncoderNWPinA, EncoderNWPinB) = (
            gpioa.pa0.into_alternate_af2(),
            gpioa.pa1.into_alternate_af2()
        );
        // TIM3
        let encoder_se_channels: (EncoderSEPinA, EncoderSEPinB) = (
            gpioa.pa6.into_alternate_af2(),
            gpioa.pa7.into_alternate_af2()
        );
        // TIM4
        let encoder_sw_channels: (EncoderSWPinA, EncoderSWPinB) = (
            gpiob.pb6.into_alternate_af2(),
            gpiob.pb7.into_alternate_af2(),
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

        // Hello world!
        for byte in b"hello from STM32!".iter() {
            block!( uart4.write(*byte)).unwrap();
        }


        // configure TIM1 for PWM output
        let pwm2 = pwm::tim1(context.device.TIM1, tim1_pwm_channels, clocks, 501.hz());
        // each TIM has two channels
        let (motor_nw, motor_ne, motor_se, motor_sw) = pwm2;
        let mut motors = MotorPwm {
            north_west: motor_nw,
            north_east: motor_ne,
            south_east: motor_se,
            south_west: motor_sw,
        };
        let max_duty = motors.north_east.get_max_duty();
        let min_duty = max_duty / 2;

        // configure QEI
        let encoders = MotorEncoders {
            north_west: qei::Qei::tim5(context.device.TIM5, encoder_nw_channels),
            north_east: qei::Qei::tim2(context.device.TIM2, encoder_ne_channels),
            south_east: qei::Qei::tim3(context.device.TIM3, encoder_se_channels),
            south_west: qei::Qei::tim4(context.device.TIM4, encoder_sw_channels),
        };

        // create a periodic timer to check the encoders periodically
        let mut timer = timer::Timer::tim6(context.device.TIM6, 1.hz(), clocks);
        // and be sure to listen for its ticks.
        timer.listen(timer::Event::TimeOut);

        // The midpoint of the motors, which translates to a stop signal.
        let stop: u16 = to_scale(max_duty, min_duty, 0.0);

        // Initialize drive motors to STOP/IDLE
        // Note: if Break/Coast = TRUE { STOP } else { IDLE }
        //  - which is set on each individual
        motors.north_east.set_duty(stop);
        motors.north_west.set_duty(stop);
        motors.south_west.set_duty(stop);
        motors.south_east.set_duty(stop);
        motors.north_west.enable();
        motors.north_east.enable();
        motors.south_west.enable();
        motors.south_east.enable();

        // allocate 1024 byte RX buffer statically
        let rx_buffer = heapless::Vec::<u8, consts::U1024>::new();

        init::LateResources {
            motors,
            encoders,
            uart4,
            rx_buffer,
            count_ne: 0,
            count_se: 0,
            count_nw: 0,
            count_sw: 0,
        }
    }
    #[task(binds = TIM6, resources = [encoders, count_ne], priority = 3)]
    fn tim6_interrupt(context: tim3_interrupt::Context) {
        // handle interrupts from TIM3, telling us to look at the encoders.
        // acquire all encoder states FIRST to ensure an accurate snapshot
        // as it may take several cycles with all the locks leading to desynchronized counts.
        let ne_current = context.resources.encoder.north_east.count();
        let se_current = context.resources.encoder.south_east.count();
        let nw_current = context.resources.encoder.north_west.count();
        let sw_current = context.resources.encoder.south_west.count();
        let mut count_ne_ptr = context.resources.count_ne;
        let mut count_se_ptr = context.resources.count_se;
        let mut count_nw_ptr = context.resources.count_nw;
        let mut count_sw_ptr = context.resources.count_sw;
        // Critcal sections, locks prevent concurrent access.
        // each shared object has its own lock...
        count_ne_ptr.lock(|count_ne_ptr| {
            // critical section
            *count_ne_ptr = ne_current
        });
        count_se_ptr.lock(|count_se_ptr| {
            // critical section
            *count_se_ptr = se_current
        });
        count_nw_ptr.lock(|count_nw_ptr| {
            // critical section
            *count_nw_ptr = nw_current
        });
        count_sw_ptr.lock(|count_sw_ptr| {
            // critical section
            *count_sw_ptr = sw_current
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
            // done with buffer, clear it out
            context.resources.rx_buffer.clear();
            // temp output is temporary.
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