use bevy::input::gamepad::{GamepadRumbleIntensity, GamepadRumbleRequest};
use bevy::prelude::*;
use core::time::Duration;
use std::sync::atomic::Ordering;

use crate::components::SubscribeAutoAim;

const GAMEPAD_STICK_DEADZONE: f32 = 0.12;
const GAMEPAD_TRIGGER_THRESHOLD: f32 = 0.35;
const PRECISE_GIMBAL_SCALE: f32 = 0.35;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControllerHelp {
    source: &'static str,
    manual: &'static str,
    auto_aim: &'static str,
}

impl ControllerHelp {
    const fn keyboard() -> Self {
        Self {
            source: "keyboard",
            manual: "F3 Camera | WASD Move | Arrows Aim | Space Shoot | G Dart | Q Gyro | U Remote Gyro | F5 AutoAim | Tab Slapper",
            auto_aim: "F5 AutoAim Off | WASD Move | Q Gyro | U Remote Gyro | external fire_advice shoots | Tab Slapper",
        }
    }

    const fn xbox() -> Self {
        Self {
            source: "xbox",
            manual: "View Camera | LS Move | L3 Boost | DPad Slapper Move | RS Aim | R3+RS Slapper Roll/Pitch | LB Gyro | Y Slapper Gyro | RB Shoot | X Dart | hold RT AutoAim",
            auto_aim: "release RT AutoAim Off | LS Move | L3 Boost | DPad Slapper Move | R3+RS Slapper Roll/Pitch | LB Gyro | Y Slapper Gyro | external fire_advice shoots",
        }
    }
}

impl Default for ControllerHelp {
    fn default() -> Self {
        Self::keyboard()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ChassisSpinMode {
    #[default]
    Off,
    On,
}

impl ChassisSpinMode {
    fn toggle(&mut self) {
        *self = match self {
            Self::Off => Self::On,
            Self::On => Self::Off,
        };
    }

    fn yaw_input(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::On => 1.0,
        }
    }

    fn is_on(self) -> bool {
        self == Self::On
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerInput {
    pub movement: Vec2,
    pub gimbal: Vec2,
    pub chassis_yaw: f32,
    pub chassis_roll: f32,
    pub chassis_pitch: f32,
    pub boost: bool,
    pub precise_gimbal: bool,
    pub shoot: bool,
    pub dart_just_pressed: bool,
    pub switch_slapper_just_pressed: bool,
    pub switch_camera_just_pressed: bool,
    pub auto_aim: bool,
}

impl Default for ControllerInput {
    fn default() -> Self {
        Self {
            movement: Vec2::ZERO,
            gimbal: Vec2::ZERO,
            chassis_yaw: 0.0,
            chassis_roll: 0.0,
            chassis_pitch: 0.0,
            boost: false,
            precise_gimbal: false,
            shoot: false,
            dart_just_pressed: false,
            switch_slapper_just_pressed: false,
            switch_camera_just_pressed: false,
            auto_aim: false,
        }
    }
}

impl ControllerInput {
    pub fn boost_multiplier(self) -> f32 {
        if self.boost { 2.0 } else { 1.0 }
    }

    pub fn gimbal_scale(self) -> f32 {
        if self.precise_gimbal {
            PRECISE_GIMBAL_SCALE
        } else {
            1.0
        }
    }

    fn add_movement(&mut self, movement: Vec2) {
        self.movement = clamp_axes_vec2(self.movement + movement);
    }

    fn add_gimbal(&mut self, gimbal: Vec2) {
        self.gimbal = clamp_axes_vec2(self.gimbal + gimbal);
    }

    fn add_chassis(&mut self, yaw: f32, roll: f32, pitch: f32) {
        self.chassis_yaw = (self.chassis_yaw + yaw).clamp(-1.0, 1.0);
        self.chassis_roll = (self.chassis_roll + roll).clamp(-1.0, 1.0);
        self.chassis_pitch = (self.chassis_pitch + pitch).clamp(-1.0, 1.0);
    }
}

#[derive(Resource, Debug, Default)]
pub struct ControllerState {
    pub controlled: ControllerInput,
    pub remote: ControllerInput,
    keyboard_auto_aim: bool,
    controlled_chassis_spin: ChassisSpinMode,
    remote_chassis_spin: ChassisSpinMode,
    active_gamepad: Option<Entity>,
    help: ControllerHelp,
}

impl ControllerState {
    pub fn reset_frame(&mut self) {
        self.controlled = ControllerInput::default();
        self.remote = ControllerInput::default();
    }

    pub fn auto_aim_active(&self) -> bool {
        self.keyboard_auto_aim || self.controlled.auto_aim
    }

    pub fn help_source(&self) -> &'static str {
        self.help.source
    }

    pub fn help_mode(&self) -> &'static str {
        if self.auto_aim_active() {
            "auto-aim"
        } else {
            "manual"
        }
    }

    pub fn help_controls(&self) -> &'static str {
        if self.auto_aim_active() {
            self.help.auto_aim
        } else {
            self.help.manual
        }
    }

    pub fn controlled_chassis_spin(&self) -> bool {
        self.controlled_chassis_spin.is_on()
    }

    pub fn remote_chassis_spin(&self) -> bool {
        self.remote_chassis_spin.is_on()
    }

    pub fn active_gamepad(&self) -> Option<Entity> {
        self.active_gamepad
    }

    fn toggle_keyboard_auto_aim(&mut self) {
        self.keyboard_auto_aim = !self.keyboard_auto_aim;
    }

    fn toggle_controlled_chassis_spin(&mut self) {
        self.controlled_chassis_spin.toggle();
    }

    fn toggle_remote_chassis_spin(&mut self) {
        self.remote_chassis_spin.toggle();
    }

    fn use_help(&mut self, help: ControllerHelp) {
        self.help = help;
    }

    fn use_gamepad(&mut self, gamepad: Entity) {
        self.active_gamepad = Some(gamepad);
    }

    fn clear_gamepad(&mut self) {
        self.active_gamepad = None;
    }
}

pub fn clear_controller_input(mut controller: ResMut<ControllerState>) {
    controller.reset_frame();
}

pub fn sample_keyboard_controller(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut controller: ResMut<ControllerState>,
) {
    let keyboard_used = keyboard_controller_active(&keyboard);
    if keyboard_used {
        controller.use_help(ControllerHelp::keyboard());
        controller.clear_gamepad();
    }
    if keyboard.just_pressed(KeyCode::KeyQ) {
        controller.toggle_controlled_chassis_spin();
    }
    if keyboard.just_pressed(KeyCode::KeyU) {
        controller.toggle_remote_chassis_spin();
    }

    let controlled_chassis_yaw = controller.controlled_chassis_spin.yaw_input();
    let controlled = &mut controller.controlled;
    controlled.add_movement(keyboard_vec2(
        &keyboard,
        KeyCode::KeyW,
        KeyCode::KeyA,
        KeyCode::KeyS,
        KeyCode::KeyD,
    ));
    controlled.add_chassis(controlled_chassis_yaw, 0.0, 0.0);
    controlled.add_gimbal(Vec2::new(
        keyboard_axis(&keyboard, KeyCode::ArrowLeft, KeyCode::ArrowRight),
        keyboard_axis(&keyboard, KeyCode::ArrowUp, KeyCode::ArrowDown),
    ));
    controlled.boost |= keyboard.pressed(KeyCode::ShiftLeft);
    controlled.shoot |= keyboard.pressed(KeyCode::Space);
    controlled.dart_just_pressed |= keyboard.just_pressed(KeyCode::KeyG);
    controlled.switch_slapper_just_pressed |= keyboard.just_pressed(KeyCode::Tab);
    controlled.switch_camera_just_pressed |= keyboard.just_pressed(KeyCode::F3);

    let remote_chassis_yaw = controller.remote_chassis_spin.yaw_input();
    let remote = &mut controller.remote;
    remote.add_movement(keyboard_vec2(
        &keyboard,
        KeyCode::KeyI,
        KeyCode::KeyJ,
        KeyCode::KeyK,
        KeyCode::KeyL,
    ));
    remote.add_chassis(
        remote_chassis_yaw,
        keyboard_axis(&keyboard, KeyCode::BracketLeft, KeyCode::BracketRight),
        keyboard_axis(&keyboard, KeyCode::Semicolon, KeyCode::Quote),
    );
    if !keyboard.pressed(KeyCode::ShiftLeft) {
        remote.add_gimbal(Vec2::new(
            keyboard_axis(&keyboard, KeyCode::KeyC, KeyCode::KeyB),
            0.0,
        ));
    }
    remote.add_gimbal(Vec2::new(
        0.0,
        keyboard_axis(&keyboard, KeyCode::KeyF, KeyCode::KeyV),
    ));
    remote.boost |= keyboard.pressed(KeyCode::ShiftRight);

    if keyboard.just_pressed(KeyCode::F5) {
        controller.toggle_keyboard_auto_aim();
    }
}

pub fn sample_gamepad_controller(
    gamepads: Query<(Entity, &Gamepad)>,
    mut controller: ResMut<ControllerState>,
    mut rumble_requests: MessageWriter<GamepadRumbleRequest>,
) {
    let Some((gamepad_entity, gamepad)) = gamepads.iter().next() else {
        return;
    };

    let left_stick = apply_stick_deadzone(gamepad.left_stick());
    let right_stick = apply_stick_deadzone(gamepad.right_stick());
    let dpad = gamepad.dpad();
    if gamepad_controller_active(gamepad, left_stick, right_stick, dpad) {
        controller.use_help(ControllerHelp::xbox());
        controller.use_gamepad(gamepad_entity);
    }
    if gamepad.just_pressed(GamepadButton::LeftTrigger) {
        controller.toggle_controlled_chassis_spin();
        request_gamepad_rumble(
            gamepad_entity,
            &mut rumble_requests,
            GamepadRumbleIntensity::weak_motor(0.25),
            Duration::from_millis(70),
        );
    }
    if gamepad.just_pressed(GamepadButton::North) {
        controller.toggle_remote_chassis_spin();
        request_gamepad_rumble(
            gamepad_entity,
            &mut rumble_requests,
            GamepadRumbleIntensity::weak_motor(0.25),
            Duration::from_millis(70),
        );
    }
    if gamepad.just_pressed(GamepadButton::RightTrigger2) {
        request_gamepad_rumble(
            gamepad_entity,
            &mut rumble_requests,
            GamepadRumbleIntensity::strong_motor(0.12),
            Duration::from_millis(60),
        );
    }

    let controlled_chassis_yaw = controller.controlled_chassis_spin.yaw_input();
    let remote_chassis_yaw = controller.remote_chassis_spin.yaw_input();
    let adjusting_chassis_tilt = gamepad.pressed(GamepadButton::RightThumb);
    let controlled = &mut controller.controlled;
    controlled.add_movement(left_stick);
    controlled.add_chassis(controlled_chassis_yaw, 0.0, 0.0);
    if !adjusting_chassis_tilt {
        controlled.add_gimbal(Vec2::new(-right_stick.x, right_stick.y));
    }
    controlled.precise_gimbal |=
        gamepad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0) > GAMEPAD_TRIGGER_THRESHOLD;
    controlled.boost |= gamepad.pressed(GamepadButton::LeftThumb);
    controlled.auto_aim |=
        gamepad.get(GamepadButton::RightTrigger2).unwrap_or(0.0) > GAMEPAD_TRIGGER_THRESHOLD;
    controlled.shoot |= gamepad.pressed(GamepadButton::RightTrigger);
    controlled.dart_just_pressed |= gamepad.just_pressed(GamepadButton::West);
    controlled.switch_slapper_just_pressed |= gamepad.just_pressed(GamepadButton::Start);
    controlled.switch_camera_just_pressed |= gamepad.just_pressed(GamepadButton::Select);

    let remote = &mut controller.remote;
    remote.add_movement(-dpad);
    if adjusting_chassis_tilt {
        remote.add_chassis(remote_chassis_yaw, right_stick.x, right_stick.y);
    } else {
        remote.add_chassis(remote_chassis_yaw, 0.0, 0.0);
    }
}

pub fn update_auto_aim_subscription(
    controller: Res<ControllerState>,
    enabled: Res<SubscribeAutoAim>,
) {
    let active = controller.auto_aim_active();
    if enabled.swap(active, Ordering::AcqRel) != active {
        info!(
            "Auto-aim subscription is now {}.",
            if active { "ENABLED" } else { "DISABLED" }
        );
    }
}

pub fn controller_shoot_pressed(controller: Res<ControllerState>) -> bool {
    controller.controlled.shoot
}

pub fn controller_dart_just_pressed(controller: Res<ControllerState>) -> bool {
    controller.controlled.dart_just_pressed
}

pub fn request_controller_rumble(
    controller: Option<&ControllerState>,
    rumble_requests: &mut MessageWriter<GamepadRumbleRequest>,
    intensity: GamepadRumbleIntensity,
    duration: Duration,
) {
    let Some(gamepad) = controller.and_then(ControllerState::active_gamepad) else {
        return;
    };
    request_gamepad_rumble(gamepad, rumble_requests, intensity, duration);
}

fn request_gamepad_rumble(
    gamepad: Entity,
    rumble_requests: &mut MessageWriter<GamepadRumbleRequest>,
    intensity: GamepadRumbleIntensity,
    duration: Duration,
) {
    rumble_requests.write(GamepadRumbleRequest::Add {
        gamepad,
        intensity,
        duration,
    });
}

fn keyboard_vec2(
    keyboard: &ButtonInput<KeyCode>,
    forward: KeyCode,
    left: KeyCode,
    backward: KeyCode,
    right: KeyCode,
) -> Vec2 {
    let mut input = Vec2::ZERO;
    if keyboard.pressed(forward) {
        input.y += 1.0;
    }
    if keyboard.pressed(backward) {
        input.y -= 1.0;
    }
    if keyboard.pressed(right) {
        input.x += 1.0;
    }
    if keyboard.pressed(left) {
        input.x -= 1.0;
    }
    input
}

fn keyboard_axis(keyboard: &ButtonInput<KeyCode>, positive: KeyCode, negative: KeyCode) -> f32 {
    let mut input = 0.0;
    if keyboard.pressed(positive) {
        input += 1.0;
    }
    if keyboard.pressed(negative) {
        input -= 1.0;
    }
    input
}

fn keyboard_controller_active(keyboard: &ButtonInput<KeyCode>) -> bool {
    const HELD_KEYS: [KeyCode; 22] = [
        KeyCode::KeyW,
        KeyCode::KeyA,
        KeyCode::KeyS,
        KeyCode::KeyD,
        KeyCode::ArrowLeft,
        KeyCode::ArrowRight,
        KeyCode::ArrowUp,
        KeyCode::ArrowDown,
        KeyCode::ShiftLeft,
        KeyCode::Space,
        KeyCode::KeyI,
        KeyCode::KeyJ,
        KeyCode::KeyK,
        KeyCode::KeyL,
        KeyCode::BracketLeft,
        KeyCode::BracketRight,
        KeyCode::Semicolon,
        KeyCode::Quote,
        KeyCode::KeyC,
        KeyCode::KeyB,
        KeyCode::KeyF,
        KeyCode::KeyV,
    ];
    const EDGE_KEYS: [KeyCode; 6] = [
        KeyCode::KeyG,
        KeyCode::Tab,
        KeyCode::F3,
        KeyCode::F5,
        KeyCode::KeyQ,
        KeyCode::KeyU,
    ];

    HELD_KEYS.iter().any(|&key| keyboard.pressed(key))
        || EDGE_KEYS.iter().any(|&key| keyboard.just_pressed(key))
}

fn gamepad_controller_active(
    gamepad: &Gamepad,
    left_stick: Vec2,
    right_stick: Vec2,
    dpad: Vec2,
) -> bool {
    left_stick != Vec2::ZERO
        || right_stick != Vec2::ZERO
        || dpad != Vec2::ZERO
        || gamepad.pressed(GamepadButton::LeftTrigger)
        || gamepad.pressed(GamepadButton::LeftThumb)
        || gamepad.pressed(GamepadButton::RightThumb)
        || gamepad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0) > GAMEPAD_TRIGGER_THRESHOLD
        || gamepad.get(GamepadButton::RightTrigger2).unwrap_or(0.0) > GAMEPAD_TRIGGER_THRESHOLD
        || gamepad.pressed(GamepadButton::RightTrigger)
        || gamepad.just_pressed(GamepadButton::North)
        || gamepad.just_pressed(GamepadButton::West)
        || gamepad.just_pressed(GamepadButton::Start)
        || gamepad.just_pressed(GamepadButton::Select)
}

fn apply_stick_deadzone(input: Vec2) -> Vec2 {
    let length = input.length();
    if length <= GAMEPAD_STICK_DEADZONE {
        return Vec2::ZERO;
    }
    let scaled = ((length - GAMEPAD_STICK_DEADZONE) / (1.0 - GAMEPAD_STICK_DEADZONE)).min(1.0);
    input / length * scaled
}

fn clamp_axes_vec2(input: Vec2) -> Vec2 {
    Vec2::new(input.x.clamp(-1.0, 1.0), input.y.clamp(-1.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deadzone_filters_small_stick_noise() {
        assert_eq!(apply_stick_deadzone(Vec2::new(0.05, 0.05)), Vec2::ZERO);
    }

    #[test]
    fn deadzone_preserves_full_stick_deflection() {
        assert_eq!(apply_stick_deadzone(Vec2::X), Vec2::X);
        assert_eq!(apply_stick_deadzone(Vec2::Y), Vec2::Y);
    }

    #[test]
    fn controller_input_clamps_each_axis_without_changing_diagonal_input() {
        let mut input = ControllerInput::default();
        input.add_movement(Vec2::X);
        input.add_movement(Vec2::Y);

        assert_eq!(input.movement, Vec2::ONE);

        input.add_movement(Vec2::ONE);
        assert_eq!(input.movement, Vec2::ONE);
    }

    #[test]
    fn help_provider_switches_between_manual_and_auto_aim_modes() {
        let mut controller = ControllerState::default();
        controller.use_help(ControllerHelp::xbox());

        assert_eq!(controller.help_source(), "xbox");
        assert_eq!(controller.help_mode(), "manual");
        assert!(controller.help_controls().contains("hold RT"));

        controller.controlled.auto_aim = true;
        assert_eq!(controller.help_mode(), "auto-aim");
        assert!(controller.help_controls().contains("release RT"));
    }

    #[test]
    fn reset_frame_preserves_last_help_provider() {
        let mut controller = ControllerState::default();
        controller.use_help(ControllerHelp::xbox());

        controller.reset_frame();

        assert_eq!(controller.help_source(), "xbox");
    }

    #[test]
    fn reset_frame_preserves_chassis_spin_modes() {
        let mut controller = ControllerState::default();
        controller.toggle_controlled_chassis_spin();
        controller.toggle_remote_chassis_spin();

        controller.reset_frame();

        assert!(controller.controlled_chassis_spin());
        assert!(controller.remote_chassis_spin());
    }
}
