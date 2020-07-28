use heapless;
use heapless::consts::U32;
use heapless::Vec;
use postcard::flavors;
use postcard::serialize_with_flavor;
use serde::{Deserialize, Serialize};

type BufferType = Vec<u8, U32>;

// pub trait AsCobs {}
//
// impl dyn AsCobs
//     where Self: Serialize {
//     /// Encodes the object to a `heapless::Vec` that is COBS encoded.
//     fn encode_cobs(&self) -> BufferType {
//         // serialize to a 32 item `HVec` storage flavor
//         // using Cobs as the encoding flavor
//         // outputs a heapless:Vec
//         serialize_with_flavor::<Self, flavors::Cobs<flavors::HVec<heapless::consts::U32>>, BufferType>(
//             self, flavors::Cobs::try_new(flavors::HVec::default()).unwrap(),
//         ).unwrap()
//     }
// }


#[derive(Serialize, Deserialize, Debug, PartialEq)]
// Request kind
pub enum RequestKind {
    // Sets all Drive motors to target speed. Target must be within [-1.0, 1.0]
    SetSpeed { target: f32 },
    // Sets the left side of the drive motors to target speed.
    // Target must be within [-1.0, 1.0].
    SetLeftSpeed { target: f32 },
    // Sets the right side of the drive motors to the target speed.
    // Target must be within [-1.0, 1.0]
    SetRightSpeed { target: f32 },
    // Sets the two different drivetrain sides in the same message.
    SetSplitSpeed{ left:f32, right:f32},
    // Stop all drive actuators
    HaltMotors,
    // Halt all arm actuators
    HaltArm,
    // Halt both the arm and the drive actuators.
    Halt,
    // Set the arm to a position.
    SetArm,
    // Get the encoder counts.
    GetMotorEncoderCounts,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum Status {
    // MCU is OK with this query and completed it successfully
    OK,
    // Runtime handling error.
    ERROR,
    // Object failed to decode, no state can be returned.
    DecodeError,
    // Not implemented (yet?).
    Unimplemented,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
// MCU action request, user-provided state is returned in the Response object (requires decode)
pub struct Request {
    pub(crate) kind: RequestKind,
    pub(crate) state: i32,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
// MCU action response, can include up to 32 bytes of data and will include request-provided state
// if the object successfully decoded; -1 otherwise.
pub struct Response {
    pub(crate) status: Status,
    pub(crate) state: i32,
    pub(crate) data: Option<Vec<u8, U32>>,
}

// struct holding the current* value of the encoders
// * up to 1 second in lag as this occurs on a 1hz update timer
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct MotorCounts {
    // these two have 32 bit resolution due to their timer
    pub north_west: u32,
    pub north_east: u32,
    // different timer, which only has 16 bit resolution
    pub south_east: u16,
    pub south_west: u16,
}

// impl AsCobs for MotorCounts {}
//
// impl AsCobs for Response {}


impl MotorCounts {
    pub fn encode_cobs(&self) -> BufferType {
// serialize to a 32 item `HVec` storage flavor
// using Cobs as the encoding flavor
// outputs a heapless:Vec
        serialize_with_flavor::<Self, flavors::Cobs<flavors::HVec<heapless::consts::U32>>, BufferType>(
            self, flavors::Cobs::try_new(flavors::HVec::default()).unwrap(),
        ).unwrap()
    }
}

impl Response {
    pub fn encode_cobs(&self) -> BufferType {
// serialize to a 32 item `HVec` storage flavor
// using Cobs as the encoding flavor
// outputs a heapless:Vec
        serialize_with_flavor::<Self, flavors::Cobs<flavors::HVec<heapless::consts::U32>>, BufferType>(
            self, flavors::Cobs::try_new(flavors::HVec::default()).unwrap(),
        ).unwrap()
    }
}