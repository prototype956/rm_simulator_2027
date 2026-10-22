//! RMUL one-vs-one scene admission. Geometry, mechanisms and motion are shared with GUI.
use super::scenario::{Motion, PreparedScenario, ScriptedTarget};
use crate::components::InfantryLaunchOffset;
use crate::config::SimulationConfig;
use crate::robomaster::combat::*;
use crate::robomaster::prelude::ArmorRoot;
use crate::robot_motion::{self, MotionKind, MotionPlan, RobotProgram};
use crate::robots::{self, RobotProfile, RobotProfiles};
use avian3d::prelude::*;
use bevy::prelude::*;
use rand::{RngExt, SeedableRng};
use serde::{Deserialize, Serialize};
use serde_json::json;

#[derive(Deserialize, Serialize, Clone, Copy, Debug, Default)]
#[serde(rename_all = "snake_case")]
pub enum SupportLevel {
    #[default]
    Ground,
    Highland,
    Bridge,
}
impl SupportLevel {
    fn height(self) -> f32 {
        match self {
            Self::Ground => 0.,
            Self::Highland => 0.2,
            Self::Bridge => 0.27,
        }
    }
}
#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[serde(default, deny_unknown_fields)]
pub struct RobotSelection {
    pub profile: RobotProfile,
    pub preset: Option<TrainingPreset>,
    pub motion: MotionPlan,
    pub position_xz_m: Option<[f32; 2]>,
    pub yaw_rad: Option<f32>,
    pub surface: SupportLevel,
}
#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[serde(default, deny_unknown_fields)]
pub struct DuelScenario {
    pub controlled: RobotSelection,
    pub target: RobotSelection,
}
#[derive(Resource)]
pub struct PendingPrograms(pub Vec<(Entity, RobotProgram)>);

impl DuelScenario {
    pub fn from_config(c: &SimulationConfig) -> Self {
        let r = c.robots.clone().unwrap_or_default();
        Self {
            controlled: RobotSelection {
                profile: r.controlled,
                preset: Some(c.combat.controlled),
                ..default()
            },
            target: RobotSelection {
                profile: r.target,
                preset: if r.target == RobotProfile::HeavyTarget {
                    None
                } else {
                    Some(c.combat.target)
                },
                ..default()
            },
        }
    }
    pub fn configure(&self, config: &mut SimulationConfig) -> Result<(), String> {
        if !config.scene.is_league() {
            return Err("duel requires RMUL 2027 map configuration".into());
        }
        if self.controlled.profile == RobotProfile::HeavyTarget {
            return Err("heavy_target is enemy-only".into());
        }
        for robot in [&self.controlled, &self.target] {
            robot.motion.validate(robot.profile)?;
            if robot
                .position_xz_m
                .is_some_and(|p| p.iter().any(|x| !x.is_finite()))
                || robot.yaw_rad.is_some_and(|x| !x.is_finite())
            {
                return Err("duel position/yaw must be finite".into());
            }
            if robot.profile == RobotProfile::HeavyTarget && robot.preset.is_some() {
                return Err("heavy_target uses its own 450HP preset; omit preset".into());
            }
        }
        config.robots = Some(RobotProfiles {
            controlled: self.controlled.profile,
            target: self.target.profile,
        });
        config.combat.ruleset = RuleSet::Rmul2027;
        config.combat.controlled = self.controlled.preset.unwrap_or(config.combat.controlled);
        config.combat.target = self.target.preset.unwrap_or(config.combat.target);
        config.combat.controlled_allowance = allowance(config.combat.controlled);
        config.combat.target_allowance = if self.target.profile == RobotProfile::HeavyTarget {
            AllowanceConfig::Limited { initial: 0 }
        } else {
            allowance(config.combat.target)
        };
        Ok(())
    }
}
fn allowance(p: TrainingPreset) -> AllowanceConfig {
    AllowanceConfig::Limited {
        initial: if p == TrainingPreset::Sentry {
            750
        } else {
            200
        },
    }
}

fn anchor(robot: &RobotSelection, controlled: bool) -> Option<(Vec2, f32)> {
    let sign = if controlled { 1. } else { -1. };
    match robot.motion.kind {
        MotionKind::Climb | MotionKind::Jump => Some((
            Vec2::new(-2.5 * sign, 1.7 * sign),
            if controlled { 0. } else { std::f32::consts::PI },
        )),
        MotionKind::Tunnel => Some((
            Vec2::new(-3.1 * sign, -3.46 * sign),
            if controlled { 0. } else { std::f32::consts::PI },
        )),
        _ => None,
    }
}

/// Build a deterministic legal joint path. Special templates have named start/support
/// locations; ordinary scenes sample the open central floor, not arbitrary roof hits.
pub fn prepare(
    world: &mut World,
    spec: DuelScenario,
    seed: u64,
) -> Result<PreparedScenario, String> {
    let roots: Vec<_> = world
        .query::<(Entity, &RobotIdentity)>()
        .iter(world)
        .map(|(e, id)| (e, id.id))
        .collect();
    let own = roots
        .iter()
        .find(|(_, id)| *id == CONTROLLED_ROBOT_ID)
        .ok_or("controlled missing")?
        .0;
    let target = roots
        .iter()
        .find(|(_, id)| *id == TARGET_ROBOT_ID)
        .ok_or("target missing")?
        .0;
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed ^ 0x4d554c5449524f42);
    let selections = [(&spec.controlled, own, true), (&spec.target, target, false)];
    let special_own = anchor(&spec.controlled, true);
    let special_target = anchor(&spec.target, false);
    let fixed = selections.iter().any(|(s, _, _)| s.position_xz_m.is_some())
        || special_own.is_some()
        || special_target.is_some();
    let mut errors = vec![];
    for attempt in 1..=if fixed { 1 } else { 64 } {
        let mut programs = vec![];
        for (s, entity, controlled) in selections {
            let special = anchor(s, controlled);
            let other = if controlled {
                special_target
            } else {
                special_own
            };
            let sign = if controlled { 1. } else { -1. };
            let observer = other.map(|(p, _)| {
                if p.y.abs() > 3. {
                    Vec2::new(p.x.signum() * 5.2, p.y.signum() * 3.0)
                } else {
                    Vec2::new(-p.x.signum() * 0.7, p.y)
                }
            });
            let p = if let Some([x, z]) = s.position_xz_m {
                Vec2::new(x, z)
            } else if let Some((p, _)) = special {
                p
            } else if let Some(p) = observer {
                p
            } else {
                Vec2::new(
                    rng.random_range(-0.35..=0.35),
                    sign * rng.random_range(1.8..=2.2),
                )
            };
            let yaw = s.yaw_rad.unwrap_or_else(|| {
                special.map_or(
                    if controlled { 0. } else { std::f32::consts::PI },
                    |(_, yaw)| yaw,
                )
            });
            let height =
                robot_motion::support_height(world, Vec3::new(p.x, 0., p.y), s.surface.height())?;
            let pose = Transform::from_xyz(p.x, height + s.profile.root_height(), p.y)
                .with_rotation(Quat::from_rotation_y(yaw));
            world.entity_mut(entity).insert((
                pose,
                Position(pose.translation),
                Rotation(pose.rotation),
                LinearVelocity::ZERO,
                AngularVelocity::ZERO,
            ));
            programs.push((entity, RobotProgram::new(pose, s.motion.clone())));
        }
        let distance = programs[0]
            .1
            .initial
            .translation
            .distance(programs[1].1.initial.translation);
        let result = if !(2.0..=8.).contains(&distance) {
            Err("initial range must be 2..8 m".into())
        } else {
            robot_motion::validate_programs(world, &programs)
        };
        match result {
            Err(reason) => {
                errors.push(json!({"attempt":attempt,"reason":reason}));
                continue;
            }
            Ok(samples) => {
                let initial = programs[0].1.initial;
                let target_initial = programs[1].1.initial;
                let muzzle = world
                    .query_filtered::<(Entity, &RobotMember), With<InfantryLaunchOffset>>()
                    .iter(world)
                    .find(|(_, m)| m.root == own)
                    .ok_or("muzzle missing")?
                    .0;
                let center: Vec<_> = world
                    .query::<(Entity, &RobotMember)>()
                    .iter(world)
                    .filter(|(e, m)| m.root == target && world.get::<ArmorRoot>(*e).is_some())
                    .map(|(e, _)| robots::physical_matrix(world, e).transform_point3(Vec3::ZERO))
                    .collect();
                let aim = center.iter().sum::<Vec3>() / center.len() as f32;
                let direction =
                    aim - robots::physical_matrix(world, muzzle).transform_point3(Vec3::ZERO);
                let yaw = (-direction.x).atan2(-direction.z) as f64;
                let pitch = direction.y.atan2(direction.xz().length()) as f64;
                let limit = world
                    .resource::<SimulationConfig>()
                    .vehicle
                    .gimbal_pitch_limit as f64;
                if pitch.abs() > limit {
                    return Err("duel initial aim exceeds pitch limit".into());
                }
                let report = json!({"validated":true,"sampling_revision":6,"seed":seed,"attempts":attempt,"rejections":errors,
                    "duel":spec,"path_samples":samples,"initial_distance_m":distance,"enemy_fire":false,
                    "controlled_position_bevy_m":initial.translation.to_array(),"target_position_bevy_m":target_initial.translation.to_array(),
                    "initial_gimbal":{"yaw_rad":yaw,"pitch_rad":pitch},"resources":"role_hp_finite_ammo_real_heat",
                    "geometry":"existing_GLB_with_parameterized_structure_proxies","future_occlusion_allowed":true});
                world.insert_resource(PendingPrograms(programs));
                return Ok(PreparedScenario {
                    controlled_entity: own,
                    controlled_initial: initial,
                    initial_gimbal: (yaw, pitch),
                    target: ScriptedTarget {
                        entity: target,
                        initial: target_initial,
                        motion: Motion::Static {},
                        report,
                        failed: None,
                        integrated_motion: None,
                        ccd_motion_corrections: 0,
                        max_ccd_translation_correction_m: 0.,
                    },
                });
            }
        }
    }
    let prefix = if fixed {
        "invalid explicit duel".to_string()
    } else {
        format!("invalid seed {seed}")
    };
    Err(format!("{prefix}: no legal joint path: {}", json!(errors)))
}

pub fn activate(world: &mut World) {
    if let Some(programs) = world.remove_resource::<PendingPrograms>() {
        for (e, p) in programs.0 {
            world.entity_mut(e).insert((
                p.initial,
                Position(p.initial.translation),
                Rotation(p.initial.rotation),
                p,
            ));
        }
    }
}
