use crate::robomaster::prelude::Team;
use bevy::prelude::*;

#[derive(Reflect, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RobotId(pub u64);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CombatRole {
    Infantry,
    Sentry,
    HeroTarget,
}

/// Stable scene identity. Armor sticker changes and ECS entity allocation do not change it.
#[derive(Component, Clone, Copy, Debug)]
pub struct RobotIdentity {
    pub id: RobotId,
    pub team: Team,
    pub role: CombatRole,
}

/// Ownership for imported chassis, gimbal, armor roots and their collider/visual descendants.
/// `root` is an ECS lookup handle; `id` is the stable simulator identity.
#[derive(Component, Reflect, Clone, Copy, Debug, PartialEq, Eq)]
#[reflect(Component)]
pub struct RobotMember {
    pub root: Entity,
    pub id: RobotId,
}

#[derive(Reflect, Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShooterKind {
    None,
    Barrel17mm,
}

/// Per-round rule snapshot. Do not replace it when the config watcher reloads TOML.
#[derive(Reflect, Clone, Copy, Debug)]
pub struct RobotRules {
    pub max_hp: u32,
    pub heat_limit: f64,
    pub cooling_per_second: f64,
    pub shooter: ShooterKind,
}

impl RobotRules {
    /// RMUL 2026 V1.2.0 table 3-2, page 17. This target has no simulated shooter.
    pub const fn hero_target() -> Self {
        Self {
            max_hp: 350,
            heat_limit: 0.0,
            cooling_per_second: 0.0,
            shooter: ShooterKind::None,
        }
    }
}

#[derive(Reflect, Clone, Copy, Debug, PartialEq, Eq)]
pub enum LifeStatus {
    Alive,
    Dead,
}

#[derive(Reflect, Clone, Debug)]
pub struct LifeState {
    pub hp: u32,
    pub status: LifeStatus,
}

/// Times are in simulation seconds; None means no shot has left the barrel this round.
#[derive(Reflect, Clone, Debug)]
pub struct ShooterState {
    pub kind: ShooterKind,
    pub last_shot_time_s: Option<f64>,
}

#[derive(Reflect, Clone, Debug, Default)]
pub struct HeatState {
    pub current: f64,
    pub cooling_locked: bool,
    pub round_locked: bool,
}

/// Referee allowance only, never the physical magazine capacity.
/// Unlimited is a training simplification, not the official initial allowance.
#[derive(Reflect, Clone, Debug, Default, PartialEq, Eq)]
pub enum FireAllowance {
    #[default]
    Unlimited,
    Limited(u32),
}

/// Owned values on each robot root: no shared mutable heat, allowance or life state.
/// Module 1 initializes these fields; later modules connect them to gameplay.
#[derive(Component, Reflect, Clone, Debug)]
#[reflect(Component)]
pub struct RobotCombatState {
    pub rules: RobotRules,
    pub shooter: ShooterState,
    pub heat: HeatState,
    pub allowance: FireAllowance,
    pub life: LifeState,
}

impl RobotCombatState {
    pub fn new(rules: RobotRules) -> Self {
        Self {
            rules,
            shooter: ShooterState {
                kind: rules.shooter,
                last_shot_time_s: None,
            },
            heat: HeatState::default(),
            allowance: FireAllowance::Unlimited,
            life: LifeState {
                hp: rules.max_hp,
                status: LifeStatus::Alive,
            },
        }
    }
}
