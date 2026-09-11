//! RMUL 2026 3V3 identity and per-robot state, independent of visual armor labels.
//!
//! Identity, shooting, heat, allowance, damage and round reset share per-robot state.

mod config;
pub mod damage;
mod heat;
pub mod ledger;
pub mod reset;
pub mod shooting;
mod state;
pub mod telemetry;

pub use config::*;
pub use heat::HeatState;
pub use state::*;

use crate::components::Infantry;
use crate::robomaster::prelude::{
    ArmorRoot, ArmorSpec, HERO_ROBOT_CONFIG, INFANTRY_THREE_CONFIG, RobotConfig, SmallArmorLabel,
    Team,
};
use bevy::prelude::*;

/// Scene slot IDs survive asset loading order and future in-place round resets.
/// These are simulator IDs, not referee IDs or visual classifier labels.
pub const CONTROLLED_ROBOT_ID: RobotId = RobotId(1);
pub const TARGET_ROBOT_ID: RobotId = RobotId(2);
pub const HERO_TARGET_ID: RobotId = RobotId(3);

/// Only the robot root owns mutable combat state; descendants refer to that root.
#[derive(Bundle)]
pub struct CombatRobotBundle {
    infantry: Infantry,
    identity: RobotIdentity,
    combat: RobotCombatState,
}

impl CombatRobotBundle {
    pub fn with_allowance(mut self, config: AllowanceConfig) -> Self {
        self.combat.allowance = config.initial_state();
        info!(
            "combat allowance robot={} state={:?}",
            self.identity.id.0, self.combat.allowance
        );
        self
    }
    pub fn with_shooter_config(mut self, config: &crate::config::ProjectileConfig) -> Self {
        self.combat.shooter.mechanics = ShooterMechanics::from_config(config);
        self
    }
    pub fn training(id: RobotId, team: Team, preset: TrainingPreset) -> Self {
        let (role, rules) = preset.resolve();
        let visual = match role {
            CombatRole::Sentry => RobotConfig::new(ArmorSpec::Small(SmallArmorLabel::Sentry), 4),
            _ => INFANTRY_THREE_CONFIG,
        };
        Self::new(id, team, role, rules, visual)
    }

    /// The existing hero model is a damage target, with no 42 mm shooter in phase 1.
    pub fn hero_target(id: RobotId, team: Team) -> Self {
        Self::new(
            id,
            team,
            CombatRole::HeroTarget,
            RobotRules::hero_target(),
            HERO_ROBOT_CONFIG,
        )
    }

    fn new(
        id: RobotId,
        team: Team,
        role: CombatRole,
        rules: RobotRules,
        visual: RobotConfig,
    ) -> Self {
        let state = RobotCombatState::new(rules);
        info!(
            "combat robot={} team={:?} role={:?} hp={}/{} heat={}/{} cooling={}/s shooter={:?}",
            id.0,
            team,
            role,
            state.life.hp,
            rules.max_hp,
            state.heat.current(),
            rules.heat_limit,
            rules.cooling_per_second,
            state.shooter.kind,
        );
        Self {
            infantry: Infantry::new(team, visual),
            identity: RobotIdentity { id, team, role },
            combat: state,
        }
    }
}

pub struct CombatPlugin;

impl Plugin for CombatPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<telemetry::CombatTelemetry>()
            .add_systems(FixedLast, telemetry::sample_combat);
        app.register_type::<RobotCombatState>()
            .register_type::<RobotMember>()
            .add_plugins(shooting::ShootingPlugin)
            .add_plugins(damage::DamagePlugin)
            .add_plugins(reset::ResetPlugin)
            .add_systems(PostUpdate, report_armor_ownership);
    }
}

/// Check the actual loaded asset hierarchy once, without changing collision/visual behavior.
fn report_armor_ownership(
    armors: Query<(Entity, Option<&RobotMember>), Added<ArmorRoot>>,
    robots: Query<&RobotIdentity>,
    parents: Query<&ChildOf>,
) {
    for (armor, member) in &armors {
        let owner = parents
            .iter_ancestors(armor)
            .find_map(|root| robots.get(root).ok().map(|identity| (root, identity)));
        let Some((root, identity)) = owner else {
            // Outposts and other non-robot visual targets do not participate in combat state.
            continue;
        };
        assert_eq!(
            member.copied(),
            Some(RobotMember {
                root,
                id: identity.id
            }),
            "robot armor must refer to its identity root"
        );
        info!("combat armor={:?} owner={}", armor, identity.id.0);
    }
}
