use crate::robomaster::outpost::consts::ROTATION_SPEED;
use bevy::prelude::Transform;

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum RotationDirection {
    Clockwise,
    CounterClockwise,
}

impl RotationDirection {
    pub const fn sign(self) -> f32 {
        match self {
            Self::Clockwise => 1.0,
            Self::CounterClockwise => -1.0,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash, Default)]
pub enum RotationMode {
    #[default]
    Forward,
    Stopped,
    Reverse,
}

impl RotationMode {
    pub const fn scale(self) -> f32 {
        match self {
            Self::Forward => 1.0,
            Self::Stopped => 0.0,
            Self::Reverse => -1.0,
        }
    }

    pub const fn next(self) -> Self {
        match self {
            Self::Forward => Self::Stopped,
            Self::Stopped => Self::Reverse,
            Self::Reverse => Self::Forward,
        }
    }
}

pub struct RotationController {
    speed: f32,
    direction: RotationDirection,
}

impl RotationController {
    pub fn new(direction: RotationDirection) -> Self {
        Self {
            speed: ROTATION_SPEED,
            direction,
        }
    }

    fn rotate(&self, transform: &mut Transform, angle: f32) {
        transform.rotate_y(angle);
    }

    pub fn step(&self, transform: &mut Transform, dt: f32, mode: RotationMode) {
        self.rotate(
            transform,
            self.direction.sign() * mode.scale() * self.speed * dt,
        );
    }
}
