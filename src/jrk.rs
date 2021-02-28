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
pub enum JrkMapping {
    Default,
    GripperPitch,
    GripRotation,
    Grip,
    UpperAxis,
    LowerAxis,
    Azimuth,
}

impl Into<u8> for JrkMapping {
    fn into(self) -> u8 {
        match self {
            JrkMapping::Default => 11,
            JrkMapping::GripperPitch => 12,
            JrkMapping::GripRotation => 13,
            JrkMapping::Grip => 14,
            JrkMapping::UpperAxis => 15,
            JrkMapping::LowerAxis => 16,
            JrkMapping::Azimuth => 17
        }
    }
}

/// sets the set-point of the JRKs based on their state in the passed pose
/// if the pose for a specific axis is None, no action is performed for that axis.
pub fn set_jrk_pose(
    jrk: &mut JrkI2c2,
    pose: rover_postcards::KinematicArmPose,
) -> Result<(), stm32f4xx_hal::i2c::Error> {
    if let Some(rotation_axis) = pose.grip.rotation_axis {
        set_jrk(jrk, JrkMapping::GripRotation, rotation_axis)?;
    }
    if let Some(pitch_axis) = pose.grip.pitch_axis {
        set_jrk(jrk, JrkMapping::GripperPitch, pitch_axis)?;
    }
    if let Some(grip) = pose.grip.gripper_axis {
        set_jrk(jrk, JrkMapping::Grip, grip)?;
    }
    if let Some(upper_axis) = pose.upper_axis {
        set_jrk(jrk, JrkMapping::UpperAxis, upper_axis)?;
    }
    if let Some(lower_axis) = pose.lower_axis {
        set_jrk(jrk, JrkMapping::LowerAxis, lower_axis)?;
    }
    // TODO: i don't trust binding this until the rest of the system is proven safe.
    // if let Some(azimuth) = pose.rotation_axis {
    //     set_jrk(jrk, JrkMapping::Azimuth, azimuth)?;
    // }

    Ok(())
}

/// sets an individual JRK based on its I2C address.
fn set_jrk(jrk: &mut JrkI2c2, target: JrkMapping, value: f32) -> Result<(), stm32f4xx_hal::i2c::Error> {
    jrk.set_device(target.into());
    jrk.set_target(to_scale(u16::MAX, u16::MIN, value))?;

    Ok(())
}