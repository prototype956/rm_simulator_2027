use super::{CombatRole, RobotRules, ShooterKind};
use bevy::prelude::Reflect;
use serde::Deserialize;

/// Rule selections are copied to robot roots at spawn, not read every frame.
/// Hot reload stages new selections for the next scene initialization.
#[derive(Deserialize, Reflect, Clone, Debug, Default, PartialEq, Eq)]
#[serde(default, deny_unknown_fields)]
pub struct CombatConfig {
    pub controlled: TrainingPreset,
    pub target: TrainingPreset,
    /// Print the bounded current-round event buffer when resetting.
    pub event_details: bool,
    pub controlled_allowance: AllowanceConfig,
    pub target_allowance: AllowanceConfig,
}

/// RMUL 2026 V1.2.0, tables 3-3 and 3-4 (printed pages 18–20).
/// Chassis power selection affects the HP preset only; no power model is added here.
#[derive(Deserialize, Reflect, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TrainingPreset {
    #[default]
    InfantryHealthCooling,
    InfantryHealthBurst,
    InfantryPowerCooling,
    InfantryPowerBurst,
    Sentry,
}

impl TrainingPreset {
    pub const fn resolve(self) -> (CombatRole, RobotRules) {
        let (role, max_hp, heat_limit, cooling_per_second) = match self {
            Self::InfantryHealthCooling => (CombatRole::Infantry, 350, 88, 24),
            Self::InfantryHealthBurst => (CombatRole::Infantry, 350, 230, 14),
            Self::InfantryPowerCooling => (CombatRole::Infantry, 300, 88, 24),
            Self::InfantryPowerBurst => (CombatRole::Infantry, 300, 230, 14),
            Self::Sentry => (CombatRole::Sentry, 400, 260, 30),
        };
        (
            role,
            RobotRules {
                max_hp,
                heat_limit,
                cooling_per_second,
                shooter: ShooterKind::Barrel17mm,
            },
        )
    }
}

/// Training referee allowance, applied at startup/reset, never physical magazine capacity.
#[derive(Deserialize, Reflect, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[serde(tag = "mode", rename_all = "snake_case", deny_unknown_fields)]
pub enum AllowanceConfig {
    #[default]
    Unlimited,
    Limited {
        initial: u32,
    },
}
impl AllowanceConfig {
    pub fn initial_state(self) -> super::FireAllowance {
        match self {
            Self::Unlimited => super::FireAllowance::Unlimited,
            Self::Limited { initial } => super::FireAllowance::Limited(initial),
        }
    }
}
