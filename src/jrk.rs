use jrk_g2_rs::JrkG2;
use stm32f4xx_hal::{gpio, gpio::AlternateOD};
// can't use I2C 1 because all the pins are consumed by other peripherals
// so we use I2C 2 as it has available pins
pub type I2c2Sda = gpio::gpiob::PB10<AlternateOD<gpio::AF4>>;
pub type I2c2Scl = gpio::gpiob::PB11<AlternateOD<gpio::AF4>>;
type I2c2 = stm32f4xx_hal::i2c::I2c<stm32f4xx_hal::stm32::I2C2, (I2c2Sda, I2c2Scl)>;
pub type JrkI2c2 = jrk_g2_rs::JrkG2I2c<I2c2>;

use crate::utilities::to_scale;

#[allow(dead_code)] // hopefully this stuff is used, but its not the end of the world if its not.
pub enum Jrk {
    Default,
    Pitch,
    Rotation,
    Grip,
    UpperAxis,
    LowerAxis,
}

impl Into<u8> for Jrk {
    fn into(self) -> u8 {
        match self {
            Jrk::Default => 11,
            Jrk::Pitch => 12,
            Jrk::Rotation => 13,
            Jrk::Grip => 14,
            Jrk::UpperAxis => 15,
            Jrk::LowerAxis => 16,
        }
    }
}

/// sets the set-point of the JRKs based on their state in the passed pose
/// if the pose for a specific axis is None, no action is performed for that axis.
pub fn set_jrk_pose(
    jrk: &mut JrkI2c2,
    pose: rover_postcards::KinematicArmPose,
) -> Result<(), stm32f4xx_hal::i2c::Error> {
    jrk.set_device(11);
    if let Some(rotation_axis) = pose.rotation_axis {
        jrk.set_target(to_scale(u16::MAX, u16::MIN, rotation_axis))?;
    }
    if let Some(upper_axis) = pose.upper_axis {
        jrk.set_device(12);
        jrk.set_target(to_scale(u16::MAX, u16::MIN, upper_axis))?;
    }
    if let Some(lower_axis) = pose.lower_axis {
        jrk.set_device(13);
        jrk.set_target(to_scale(u16::MAX, u16::MIN, lower_axis))?;
    }

    if let Some(pitch_axis) = pose.pitch_axis {
        jrk.set_device(14);
        jrk.set_target(to_scale(u16::MAX, u16::MIN, pitch_axis))?;
    }
    Ok(())
}
