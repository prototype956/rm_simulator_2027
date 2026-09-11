use crate::robomaster::power_rune::common::RuneMode;
use crate::robomaster::power_rune::consts::ROTATION_BASELINE_SMALL;
use bevy::math::Dir3;
use bevy::prelude::{Component, Transform};
use rand::{Rng, RngExt};

struct VariableRotation {
    a: f32,
    omega: f32,
    t: f32,
}

impl VariableRotation {
    pub fn random(rng: &mut impl Rng) -> Self {
        let a = rng.random_range(0.780..=1.045);
        let omega = rng.random_range(1.884..=2.0);
        Self { a, omega, t: 0.0 }
    }

    pub fn advance(&mut self, dt: f32) {
        self.t += dt;
    }

    pub fn speed(&self) -> f32 {
        let b = 2.090 - self.a;
        self.a * (self.omega * self.t).sin() + b
    }
}

pub struct RotationController {
    baseline: f32,
    direction: Dir3,
    variable: Option<VariableRotation>,
    clockwise: bool,
}

impl RotationController {
    pub fn new(clockwise: bool) -> Self {
        Self {
            baseline: ROTATION_BASELINE_SMALL,
            direction: Dir3::from_xyz(-1.0, 0.0, -1.0).unwrap(),
            variable: None,
            clockwise,
        }
    }

    pub fn variable_params(&self) -> Option<(f32, f32, f32)> {
        self.variable.as_ref().map(|v| (v.a, v.omega, v.t))
    }

    pub fn is_clockwise(&self) -> bool {
        self.clockwise
    }

    pub fn rotate(&self, transform: &mut Transform, angle: f32) {
        transform.rotate_local_axis(self.direction, angle);
    }

    pub fn set_variable(&mut self, rng: &mut impl Rng) {
        self.variable = Some(VariableRotation::random(rng));
    }

    pub fn clear_variable(&mut self) {
        self.variable = None;
    }

    pub fn begin_activation(&mut self, mode: RuneMode, rng: &mut impl Rng) {
        self.clear_variable();
        if mode == RuneMode::Large {
            self.set_variable(rng);
        }
    }

    pub fn end_activation(&mut self) {
        self.clear_variable();
    }

    pub fn sync_activation(&mut self, mode: RuneMode, activating: bool, rng: &mut impl Rng) {
        match (mode, activating, self.variable.is_some()) {
            (RuneMode::Large, true, false) => self.set_variable(rng),
            (RuneMode::Large, true, true) => {}
            _ => self.clear_variable(),
        }
    }

    pub fn current_speed(&mut self, mode: RuneMode, dt: f32) -> f32 {
        let sgn = if self.clockwise { 1.0 } else { -1.0 };
        if mode == RuneMode::Small {
            return sgn * self.baseline;
        }
        // 大机关只有在激活状态下使用变量旋转
        if let Some(variable) = &mut self.variable {
            let speed = variable.speed();
            variable.advance(dt);
            return sgn * speed;
        }
        sgn * self.baseline
    }
}

#[derive(Component)]
pub struct PowerRuneRotation {
    controller: RotationController,
}

impl PowerRuneRotation {
    pub fn new(clockwise: bool) -> Self {
        Self {
            controller: RotationController::new(clockwise),
        }
    }

    pub fn controller(&self) -> &RotationController {
        &self.controller
    }

    pub fn begin_activation(&mut self, mode: RuneMode, rng: &mut impl Rng) {
        self.controller.begin_activation(mode, rng);
    }

    pub fn end_activation(&mut self) {
        self.controller.end_activation();
    }

    pub fn sync_activation(&mut self, mode: RuneMode, activating: bool, rng: &mut impl Rng) {
        self.controller.sync_activation(mode, activating, rng);
    }

    pub fn rotate(&mut self, mode: RuneMode, transform: &mut Transform, dt: f32) {
        let speed = self.controller.current_speed(mode, dt);
        self.controller.rotate(transform, speed * dt);
    }
}
