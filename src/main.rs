#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use core::ops::DerefMut;

use heapless::{consts, Vec};
use nb::block;
// Halt on panic

// a panic handler is required, though for non-debug builds RTT won't be available
// as its a debugger feature.
// Thus we provide our own for non-debug builds and use RTT for debug builds.
#[cfg(debug_assertions)]
use panic_rtt_target as _;

#[cfg(not(debug_assertions))]
mod panic_handler;

use postcard::{flavors, from_bytes_cobs, serialize_with_flavor, Error};
use rtic::app;

#[cfg(debug_assertions)]
use rtt_target::{rprintln, rtt_init_print};

use stm32f4::stm32f446::{TIM1, TIM2, TIM3, TIM4, TIM5};
use stm32f4xx_hal::{
    delay::Delay,
    gpio,
    gpio::{Alternate, AF1, AF2},
    prelude::*,
    pwm, qei, serial, timer,
};

use cobs_stream::CobsDecoder;
use rover_postcards::{Request, RequestKind, Response, ResponseKind};
use stm32f4xx_hal::gpio::AlternateOD;
use jrk_g2_rs::JrkG2;
// use rover_postcards::AsCobs;

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

// can't use I2C 1 because all the pins are consumed by other peripherals
// so we use I2C 2 as it has available pins
type I2c2Sda = gpio::gpiob::PB10<AlternateOD<gpio::AF4>>;
type I2c2Scl = gpio::gpiob::PB11<AlternateOD<gpio::AF4>>;
type I2c2 = stm32f4xx_hal::i2c::I2c<stm32f4xx_hal::stm32::I2C2, (I2c2Sda, I2c2Scl)>;
type JrkI2c2 = jrk_g2_rs::JrkG2I2c<I2c2>;

pub struct MotorPwm {
    north_west: pwm::PwmChannels<TIM1, pwm::C1>,
    north_east: pwm::PwmChannels<TIM1, pwm::C2>,
    south_east: pwm::PwmChannels<TIM1, pwm::C3>,
    south_west: pwm::PwmChannels<TIM1, pwm::C4>,
}

impl MotorPwm {
    pub fn set_right_speed(&mut self, target: f32) {
        let (max_duty, min_duty) = self.get_bounds();

        self.north_east
            .set_duty(to_scale(max_duty, min_duty, target));
        self.south_east
            .set_duty(to_scale(max_duty, min_duty, target * -1.0));
    }
    pub fn set_left_speed(&mut self, target: f32) {
        let (max_duty, min_duty) = self.get_bounds();
        let setpoint = to_scale(max_duty, min_duty, target);
        self.north_west.set_duty(setpoint);
        self.south_west.set_duty(setpoint);
    }
    pub fn set_all_speed(&mut self, target: f32) {
        self.set_left_speed(target);
        self.set_right_speed(target);
    }

    fn get_bounds(&self) -> (u16, u16) {
        let max_duty = self.north_west.get_max_duty();
        let min_duty = max_duty / 2;
        (max_duty, min_duty)
    }
}

// struct holding pointers to the motor encoders
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
        motor_counts: rover_postcards::MotorCounts,
        uart4: Uart4,
        rx_buffer: CobsDecoder,
        jrk: JrkI2c2,
    }

    #[init]
    fn init(context: init::Context) -> init::LateResources {
        #[cfg(debug_assertions)]
        rtt_init_print!();

        #[cfg(debug_assertions)]
        rprintln!("hello, world!");
        /*
            This patch enables the debugger to behave correctly during a WFI
            See Errata: https://www.st.com/content/ccc/resource/technical/document/errata_sheet/c3/6b/f8/32/fc/01/48/6e/DM00155929.pdf/files/DM00155929.pdf/jcr:content/translations/en.DM00155929.pdf#%5B%7B%22num%22%3A37%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C67%2C724%2Cnull%5D
            See Also Github: https://github.com/probe-rs/probe-rs/issues/350#issuecomment-740550519
        */
        // enable the dma1 master
        context
            .device
            .RCC
            .ahb1enr
            .modify(|_, w| w.dma1en().enabled());
        // enable the debugger.
        context.device.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });

        let rcc = context.device.RCC.constrain();
        let clocks = rcc.cfgr.freeze();
        // Create a delay abstraction based on SysTick
        let _delay = Delay::new(context.core.SYST, clocks);

        let gpioa = context.device.GPIOA.split();
        let gpiob = context.device.GPIOB.split();
        let gpioc = context.device.GPIOC.split();

        // TIM1/2 uses AF1 for pwm channels.
        let tim1_pwm_channels = (
            gpioa.pa8.into_alternate_af1(),  // ch1 NW
            gpioa.pa9.into_alternate_af1(),  // ch2 NE
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
            gpioa.pa1.into_alternate_af2(),
        );
        // TIM3
        let encoder_se_channels: (EncoderSEPinA, EncoderSEPinB) = (
            gpioa.pa6.into_alternate_af2(),
            gpioa.pa7.into_alternate_af2(),
        );
        // TIM4
        let encoder_sw_channels: (EncoderSWPinA, EncoderSWPinB) = (
            gpiob.pb6.into_alternate_af2(),
            gpiob.pb7.into_alternate_af2(),
        );

        let uart4_channels = (
            gpioc.pc10.into_alternate_af8(), // UART4_TX
            gpioc.pc11.into_alternate_af8(), // UART4_RX
        );

        // establish USART2 device
        let mut uart4 = serial::Serial::uart4(
            context.device.UART4,
            uart4_channels,
            serial::config::Config {
                baudrate: 9600.bps(),
                wordlength: serial::config::WordLength::DataBits8,
                parity: serial::config::Parity::ParityNone,
                stopbits: serial::config::StopBits::STOP1,
            },
            clocks,
        )
            .unwrap();
        // listen for incoming packets
        uart4.listen(serial::Event::Rxne);
        // Hello world!
        for byte in b"hello from STM32!".iter() {
            block!(uart4.write(*byte)).unwrap();
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
        // Enable control signal output.
        motors.north_west.enable();
        motors.north_east.enable();
        motors.south_west.enable();
        motors.south_east.enable();

        // configure peripheral I2C2
        // These pins are expected to be in open drain mode.
        let sda: I2c2Sda = gpiob.pb10.into_alternate_af4_open_drain();
        let scl: I2c2Scl = gpiob.pb11.into_alternate_af4_open_drain();

        let i2c = stm32f4xx_hal::i2c::I2c::i2c2(context.device.I2C2, (sda, scl), 100.khz(), clocks);
        let jrk: JrkI2c2 = JrkI2c2::new(i2c);

        // allocate RX buffer statically
        let mut buf = cobs_stream::Buffer::new();
        // and ensure its actually that size by filling it with sentinels
        buf.resize(buf.capacity(), 0xFF)
            .expect("this resize should never fail...");
        // then pass ownership to the decoder.
        let rx_buffer = CobsDecoder::new(buf);

        let motor_counts = rover_postcards::MotorCounts {
            north_west: rover_postcards::MotorDelta { count: 0, delta: 0 },
            north_east: rover_postcards::MotorDelta { count: 0, delta: 0 },
            south_east: rover_postcards::MotorDelta { count: 0, delta: 0 },
            south_west: rover_postcards::MotorDelta { count: 0, delta: 0 },
        };
        #[cfg(debug_assertions)]
        rprintln!("done initializing!");

        init::LateResources {
            motors,
            encoders,
            motor_counts,
            uart4,
            rx_buffer,
            jrk,
        }
    }
    #[task(binds = TIM6_DAC, resources = [encoders, motor_counts], priority = 3)]
    fn tim6_interrupt(context: tim6_interrupt::Context) {
        // handle interrupts from TIM3, telling us to look at the encoders.
        // acquire all encoder states FIRST to ensure an accurate snapshot
        // as it may take several cycles with all the locks leading to desynchronized counts.
        let ne_current = context.resources.encoders.north_east.count();
        let se_current = context.resources.encoders.south_east.count();
        let nw_current = context.resources.encoders.north_west.count();
        let sw_current = context.resources.encoders.south_west.count();
        let mut counts_ptr = context.resources.motor_counts;
        // prevent concurrent access while these variables are getting updated.
        // #[cfg(debug_assertions)]
        // rprintln!("updating counts...");

        counts_ptr.lock(|counts_ptr| {
            // critical section
            counts_ptr.north_east.delta = counts_ptr.north_east.count as i64 - ne_current as i64;
            counts_ptr.south_east.delta = counts_ptr.south_east.count as i64 - se_current as i64;
            counts_ptr.north_west.delta = counts_ptr.north_west.count as i64 - nw_current as i64;
            counts_ptr.south_west.delta = counts_ptr.south_west.count as i64 - sw_current as i64;
            counts_ptr.north_east.count = ne_current;
            counts_ptr.south_east.count = se_current.into();
            counts_ptr.north_west.count = nw_current;
            counts_ptr.south_west.count = sw_current.into();
        });
    }
    #[task(binds = UART4, resources = [uart4, rx_buffer, motor_counts, motors, jrk], priority = 10)]
    fn uart4_on_rxne(mut context: uart4_on_rxne::Context) {
        let connection: &mut Uart4 = context.resources.uart4;
        // these handlers need to be really quick or overruns can occur (NO SEMIHOSTING!)
        match connection.read() {
            Err(e) => {
                #[cfg(debug_assertions)]
                rprintln!("Failed to read byte {:?}... discarding buffer.", e);

                return;
            }
            Ok(rx_byte) => {
                let decoder: &mut CobsDecoder = &mut context.resources.rx_buffer;
                #[cfg(debug_assertions)]
                rprintln!("feeding byte {:?} ", rx_byte);
                match decoder.feed(rx_byte) {
                    Ok(None) => {
                        return;
                    }
                    Err(e) => {
                        #[cfg(debug_assertions)]
                        rprintln!("Decoding failure := {:?} !", e);
                        let response = rover_postcards::Response {
                            status: rover_postcards::Status::DecodeError,
                            state: -1,
                            data: None,
                        };
                        let buf: heapless::Vec<u8, heapless::consts::U32> =
                            postcard::to_vec_cobs(&response).unwrap();
                        for byte in buf.iter() {
                            block!(context.resources.uart4.write(*byte)).unwrap()
                        }
                        return;
                    }
                    Ok(Some(_)) => {
                        let request: postcard::Result<rover_postcards::Request> =
                            postcard::from_bytes(&mut decoder.dest);

                        #[cfg(debug_assertions)]
                        rprintln!("recv'ed request {:?}", request);
                        match request {
                            Err(_) => {
                                let response = rover_postcards::Response {
                                    status: rover_postcards::Status::DecodeError,
                                    state: -1,
                                    data: None,
                                };
                                let buf: heapless::Vec<u8, heapless::consts::U32> =
                                    postcard::to_vec_cobs(&response).unwrap();
                                for byte in buf.iter() {
                                    block!(context.resources.uart4.write(*byte)).unwrap()
                                }
                            }
                            Ok(request) => {
                                let response = match request.kind {
                                    rover_postcards::RequestKind::GetMotorEncoderCounts => {
                                        rover_postcards::Response {
                                            status: rover_postcards::Status::OK,
                                            state: request.state,
                                            data: Some(ResponseKind::MotorCountResponse(
                                                *context.resources.motor_counts,
                                            )),
                                        }
                                    }
                                    rover_postcards::RequestKind::SetSpeed { target } => {
                                        context.resources.motors.set_all_speed(target);

                                        rover_postcards::Response {
                                            status: rover_postcards::Status::OK,
                                            state: request.state,
                                            data: None,
                                        }
                                    }
                                    rover_postcards::RequestKind::HaltMotors => {
                                        context.resources.motors.set_all_speed(0.0);
                                        rover_postcards::Response {
                                            status: rover_postcards::Status::OK,
                                            state: request.state,
                                            data: None,
                                        }
                                    }
                                    rover_postcards::RequestKind::SetSplitSpeed { left, right } => {
                                        context.resources.motors.set_left_speed(left);
                                        context.resources.motors.set_right_speed(right);
                                        rover_postcards::Response {
                                            status: rover_postcards::Status::OK,
                                            state: request.state,
                                            data: None,
                                        }
                                    }
                                    rover_postcards::RequestKind::SetArmPose(pose) => {
                                        let jrk: &mut JrkI2c2 = context.resources.jrk;
                                        let result =set_jrk_pose(jrk, pose);
                                        if let Ok(()) = result{
                                            rover_postcards::Response {
                                                status: rover_postcards::Status::OK,
                                                state: request.state,
                                                data: None,
                                            }
                                        } else {
                                            #[cfg(debug_assertions)]
                                            rprintln!("failed to set JRK state due to {:?}", result.err());
                                            rover_postcards::Response {
                                                status: rover_postcards::Status::ERROR,
                                                state: request.state,
                                                data: None
                                            }
                                        }

                                    }

                                    _ => Response {
                                        status: rover_postcards::Status::Unimplemented,
                                        state: request.state,
                                        data: None,
                                    },
                                };

                                #[cfg(debug_assertions)]
                                rprintln!("writing response: {:?}", response);
                                let buf: heapless::Vec<u8, heapless::consts::U1024> =
                                    postcard::to_vec_cobs(&response).unwrap();
                                for byte in buf.iter() {
                                    block!(context.resources.uart4.write(*byte)).unwrap()
                                }
                            }
                        }

                        decoder.reset().expect("failed to reset stream decoder...");
                    }
                }
            }
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
    let old_range = old_max - old_min; // 1- -1
    let new_range = new_max - new_min;
    let new_value = (((value - old_min) * new_range) / old_range) + new_min;
    new_value as u16
}


/// sets the set-point of the JRKs based on their state in the passed pose
/// if the pose for a specific axis is None, no action is performed for that axis.
fn set_jrk_pose(jrk: &mut JrkI2c2, pose: rover_postcards::KinematicArmPose) -> Result<(), stm32f4xx_hal::i2c::Error> {
    jrk.set_device(11);
    if let Some(rotation_axis) = pose.rotation_axis{
        jrk.set_target(to_scale(u16::MAX, u16::MIN, rotation_axis))?;
    }
    if let Some(upper_axis) = pose.upper_axis{
        jrk.set_device(12);
        jrk.set_target(to_scale(u16::MAX, u16::MIN, upper_axis))?;
    }
    if let Some(lower_axis) = pose.lower_axis{
        jrk.set_device(13);
        jrk.set_target(to_scale(u16::MAX, u16::MIN, lower_axis))?;
    }

    if let Some(pitch_axis) = pose.pitch_axis{
        jrk.set_device(14);
        jrk.set_target(to_scale(u16::MAX, u16::MIN, pitch_axis))?;
    }
    Ok(())

}