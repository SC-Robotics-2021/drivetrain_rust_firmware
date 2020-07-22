use heapless;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, PartialEq)]
enum RequestKind {
    SetSpeed { target: f32 },
    SetLeftSpeed { target: f32 },
    SetRightSpeed { target: f32 },
    StopMotors,
    StopArm,
    StopAll,
    SetArm,
    GetMotorEncoderCounts,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
enum Status {
    // MCU is OK with this query and completed it successfully
    OK,
    ERROR,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
struct Request {
    kind: RequestKind,
    state: i32,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
struct Response {
    status: Status,
    state: i32,
    data: heapless::Vec<u8, heapless::consts::U32>,
}

