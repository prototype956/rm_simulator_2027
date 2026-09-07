use super::HeatState;
use super::shooting::PendingShot;
use crate::robomaster::prelude::Team;
use bevy::prelude::*;
use std::time::Duration;

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
    /// Upper limit in referee heat units; the presets specify integral values.
    pub heat_limit: u32,
    /// Referee heat units per second, applied in tenths at 10 Hz.
    pub cooling_per_second: u32,
    pub shooter: ShooterKind,
}

impl RobotRules {
    /// RMUL 2026 V1.2.0 table 3-2, page 17. This target has no simulated shooter.
    pub const fn hero_target() -> Self {
        Self {
            max_hp: 350,
            heat_limit: 0,
            cooling_per_second: 0,
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

/// Per-robot physical evaluation totals; never supplied to the vision decision pipeline.
#[derive(Reflect, Clone, Debug, Default)]
pub struct DamageStatistics {
    pub armor_contacts: u64,
    pub damaging_hits: u64,
    pub damage_dealt: u64,
    pub damage_taken: u64,
    pub kills: u64,
}

/// Durations use the Fixed simulation clock; None means no shot has left the barrel this round.
#[derive(Reflect, Clone, Debug)]
pub struct ShooterState {
    pub kind: ShooterKind,
    pub mechanics: ShooterMechanics,
    pub last_shot_at: Option<Duration>,
    pub last_manual_request_at: Option<Duration>,
    pub pending: Option<PendingShot>,
    pub actual_shots: u64,
    pub rejected_requests: u64,
}

/// Mechanical training parameters copied at spawn; these are not referee limits.
#[derive(Reflect, Clone, Copy, Debug)]
pub struct ShooterMechanics {
    pub speed_mps: f32,
    pub min_interval: Duration,
    pub launch_delay: Duration,
}

impl Default for ShooterMechanics {
    fn default() -> Self {
        Self {
            speed_mps: 25.0,
            min_interval: Duration::from_millis(50),
            launch_delay: Duration::ZERO,
        }
    }
}

impl ShooterMechanics {
    pub fn from_config(config: &crate::config::ProjectileConfig) -> Self {
        config
            .validate_shooter()
            .expect("invalid shooter configuration");
        Self {
            speed_mps: config.speed,
            min_interval: Duration::from_secs_f64(config.cooldown),
            launch_delay: Duration::from_secs_f64(config.launch_delay_s),
        }
    }
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
/// Shooting, heat, allowance and life accounting are active.
#[derive(Component, Reflect, Clone, Debug)]
#[reflect(Component)]
pub struct RobotCombatState {
    pub damage: DamageStatistics,
    pub rules: RobotRules,
    pub shooter: ShooterState,
    pub heat: HeatState,
    pub allowance: FireAllowance,
    pub life: LifeState,
}

impl RobotCombatState {
    pub fn new(rules: RobotRules) -> Self {
        Self {
            damage: DamageStatistics::default(),
            rules,
            shooter: ShooterState {
                kind: rules.shooter,
                mechanics: ShooterMechanics::default(),
                last_shot_at: None,
                last_manual_request_at: None,
                pending: None,
                actual_shots: 0,
                rejected_requests: 0,
            },
            heat: HeatState::new(Duration::ZERO),
            allowance: FireAllowance::Unlimited,
            life: LifeState {
                hp: rules.max_hp,
                status: LifeStatus::Alive,
            },
        }
    }
}
