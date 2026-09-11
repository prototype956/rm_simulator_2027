//! In-place training round reset. Global simulation time and all IDs remain monotonic.
use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::{
    damage::{CombatDead, ExtinguishedLight},
    *,
};
use crate::components::*;
use crate::config::SimulationConfig;
use crate::robomaster::prelude::{Armor, ArmorLabel, ArmorStickerSelection, Projectile};
use crate::statistic::ProjectileStatistics;
use crate::systems::{ChassisObservationFrame, ControllerState, PreviousKinematicState};
use avian3d::physics_transform::{PreSolveDeltaPosition, PreSolveDeltaRotation};
use avian3d::prelude::*;
use bevy::prelude::*;

const EVENT_CAPACITY: usize = 1024;
const SUMMARY_CAPACITY: usize = 32;

/// A shared generation fence. Capture callbacks and reset hold the same lock, so an old
/// asynchronous frame can never publish after the new round has begun.
#[derive(Resource, Clone)]
pub struct RoundFence(pub Arc<Mutex<u64>>);
impl Default for RoundFence {
    fn default() -> Self {
        Self(Arc::new(Mutex::new(1)))
    }
}

impl RoundFence {
    /// Hold this guard through publication. An obsolete capture cannot pass this boundary.
    pub fn lock_round(&self, id: u64) -> Option<std::sync::MutexGuard<'_, u64>> {
        self.0.lock().ok().filter(|current| **current == id)
    }
}

#[derive(Clone, Debug)]
pub struct RoundEvent {
    pub id: u64,
    pub round: u64,
    pub at: Duration,
    pub detail: String,
}

#[derive(Resource)]
pub struct TrainingRound {
    pub id: u64,
    pub started_at: Duration,
    pub events: VecDeque<RoundEvent>,
    pub summaries: VecDeque<String>,
    pub dropped_events: u64,
    next_event_id: u64,
    requested: bool,
}
impl Default for TrainingRound {
    fn default() -> Self {
        Self {
            id: 1,
            started_at: Duration::ZERO,
            events: VecDeque::new(),
            summaries: VecDeque::new(),
            dropped_events: 0,
            next_event_id: 0,
            requested: false,
        }
    }
}

#[derive(Clone)]
struct InitialRobot {
    transforms: Vec<(Entity, Transform)>,
    active: Vec<Entity>,
}
#[derive(Resource, Default)]
struct InitialScene(HashMap<Entity, InitialRobot>);

pub struct ResetPlugin;
impl Plugin for ResetPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TrainingRound>()
            .init_resource::<RoundFence>()
            .init_resource::<InitialScene>()
            .add_systems(
                PreUpdate,
                reset_key
                    .after(bevy::input::InputSystems)
                    .before(crate::systems::clear_controller_input),
            )
            .add_systems(FixedFirst, apply_requested_reset);
    }
}

/// Called once from vehicle asset setup, after imported chassis and gimbal initialization.
/// Save local transforms before the first physical step, not a subsequently moving pose.
pub(crate) fn capture_initial_robot(world: &mut World, root: Entity) {
    let Some(_) = world.get::<RobotCombatState>(root) else {
        return;
    };
    let descendants: Vec<_> = world
        .query::<(Entity, &RobotMember)>()
        .iter(world)
        .filter_map(|(e, m)| (m.root == root).then_some(e))
        .collect();
    let entities: Vec<_> = std::iter::once(root).chain(descendants).collect();
    let initial = InitialRobot {
        transforms: entities
            .iter()
            .filter_map(|e| world.get::<Transform>(*e).map(|t| (*e, *t)))
            .collect(),
        active: entities
            .iter()
            .copied()
            .filter(|e| world.get::<ActiveSlapper>(*e).is_some())
            .collect(),
    };
    world
        .resource_mut::<InitialScene>()
        .0
        .entry(root)
        .or_insert(initial);
}

/// Public request entry for R and the future training API. Multiple requests coalesce.
/// Applied at FixedFirst, before admission, cooling, damage and physics.
pub fn request_scene_reset(world: &mut World) {
    world.resource_mut::<TrainingRound>().requested = true;
}

fn reset_key(world: &mut World) {
    if world
        .get_resource::<ButtonInput<KeyCode>>()
        .is_some_and(|keys| keys.just_pressed(KeyCode::KeyR))
    {
        request_scene_reset(world);
    }
}

/// Bounded evaluation channel; event IDs survive resets, while per-round buffers do not.
pub(crate) fn record_event(world: &mut World, at: Duration, detail: String) {
    let Some(mut round) = world.get_resource_mut::<TrainingRound>() else {
        return;
    };
    round.next_event_id += 1;
    let event = RoundEvent {
        id: round.next_event_id,
        round: round.id,
        at: at.saturating_sub(round.started_at),
        detail,
    };
    if round.events.len() == EVENT_CAPACITY {
        round.events.pop_front();
        round.dropped_events += 1;
    }
    round.events.push_back(event);
}

/// Reviewable summary also available to future training callers, without starting a recorder.
pub fn round_summary(world: &mut World) -> String {
    let now = world.resource::<Time<Fixed>>().elapsed();
    let round = world.resource::<TrainingRound>();
    let (id, elapsed, dropped) = (
        round.id,
        now.saturating_sub(round.started_at),
        round.dropped_events,
    );
    let mut robots: Vec<_> = world
        .query::<(&RobotIdentity, &RobotCombatState)>()
        .iter(world)
        .map(|(id, state)| {
            serde_json::json!({"robot":id.id.0, "shots":state.shooter.actual_shots,
            "rejected_requests":state.shooter.rejected_requests,"armor_contacts":state.damage.armor_contacts,"damaging_hits":state.damage.damaging_hits,
            "effective_damage":state.damage.damage_dealt,"kills":state.damage.kills,
            "hp":state.life.hp,"heat_locks":state.heat.lock_count,
            "heat_locked_s":state.heat.locked_duration(now).as_secs_f64()})
        })
        .collect();
    robots.sort_by_key(|value| value["robot"].as_u64());
    serde_json::json!({"round":id,"sim_elapsed_s":elapsed.as_secs_f64(),"dropped_events":dropped,"robots":robots}).to_string()
}

fn clear_messages<T: Message>(world: &mut World) {
    if let Some(mut messages) = world.get_resource_mut::<Messages<T>>() {
        messages.clear();
    }
}

fn apply_requested_reset(world: &mut World) {
    if !world.resource::<TrainingRound>().requested {
        return;
    }
    // A key during asset loading remains pending until every robot has an initial snapshot.
    let roots: Vec<_> = world
        .query_filtered::<Entity, With<RobotIdentity>>()
        .iter(world)
        .collect();
    if roots.is_empty()
        || roots
            .iter()
            .any(|root| !world.resource::<InitialScene>().0.contains_key(root))
    {
        return;
    }
    let config = world.resource::<SimulationConfig>().clone();
    if let Err(error) = config.projectile.validate_shooter() {
        warn!("scene reset rejected: {error}");
        world.resource_mut::<TrainingRound>().requested = false;
        return;
    }
    let summary = round_summary(world);
    info!("round summary {summary}");
    if config.combat.event_details {
        for event in &world.resource::<TrainingRound>().events {
            info!(
                "round event id={} round={} sim_s={:.9} {}",
                event.id,
                event.round,
                event.at.as_secs_f64(),
                event.detail
            );
        }
    }
    let now = world.resource::<Time<Fixed>>().elapsed();
    let fence = world.resource::<RoundFence>().clone();
    let mut generation = fence.0.lock().expect("round fence poisoned");
    let snapshots = world.resource::<InitialScene>().0.clone();
    let projectiles: Vec<_> = world
        .query_filtered::<Entity, Or<(With<Projectile>, With<DartProjectile>)>>()
        .iter(world)
        .collect();
    let cleared_projectiles = projectiles.len();
    for entity in projectiles {
        world.despawn(entity);
    }
    super::shooting::clear_round_requests(world);
    super::damage::clear_round_impacts(world);
    clear_messages::<shooting::ShotFired>(world);
    clear_messages::<shooting::FireRequestResult>(world);
    clear_messages::<damage::ArmorContact>(world);
    clear_messages::<damage::DamageApplied>(world);
    clear_messages::<damage::RobotDied>(world);
    clear_messages::<CollisionStart>(world);
    clear_messages::<CollisionEnd>(world);
    // Keep Avian's contact/island graphs coherent. Updating body transforms and physics
    // poses makes the normal broad/narrow phases retire old contacts on this same step;
    // manually clearing ContactGraph leaves dangling PhysicsIslands contact references.
    let marked: Vec<_> = world
        .query_filtered::<Entity, Or<(With<CombatDead>, With<ActiveSlapper>)>>()
        .iter(world)
        .collect();
    for entity in marked {
        world
            .entity_mut(entity)
            .remove::<(CombatDead, ActiveSlapper)>();
    }
    let lights: Vec<_> = world
        .query::<(Entity, &ExtinguishedLight)>()
        .iter(world)
        .map(|(e, light)| (e, light.original.clone()))
        .collect();
    for (entity, original) in lights {
        world
            .entity_mut(entity)
            .insert(MeshMaterial3d(original))
            .remove::<ExtinguishedLight>();
    }
    for (root, initial) in snapshots {
        if world.get::<RobotIdentity>(root).is_none() {
            continue;
        }
        for (entity, transform) in &initial.transforms {
            if world.get_entity(*entity).is_err() {
                continue;
            }
            world.entity_mut(*entity).insert(*transform);
            if world.get::<RigidBody>(*entity).is_some() {
                world.entity_mut(*entity).insert((Position(transform.translation), Rotation(transform.rotation), LinearVelocity::ZERO, AngularVelocity::ZERO,
                    PreSolveDeltaPosition::default(), PreSolveDeltaRotation::default(),
                    avian3d::dynamics::rigid_body::forces::AccumulatedLocalAcceleration::default())).remove::<Sleeping>();
            }
            if world.get::<InfantryChassis>(*entity).is_some() {
                world.entity_mut(*entity).insert(InfantryChassis::default());
            }
            if world.get::<InfantryGimbal>(*entity).is_some() {
                world.entity_mut(*entity).insert(InfantryGimbal::default());
            }
        }
        for entity in initial.active {
            if let Ok(mut entity) = world.get_entity_mut(entity) {
                entity.insert(ActiveSlapper);
            }
        }
        let identity = *world.get::<RobotIdentity>(root).unwrap();
        let bundle = if identity.role == CombatRole::HeroTarget {
            CombatRobotBundle::hero_target(identity.id, identity.team)
        } else {
            let preset = if world.get::<Controlled>(root).is_some() {
                config.combat.controlled
            } else {
                config.combat.target
            };
            let allowance = if world.get::<Controlled>(root).is_some() {
                config.combat.controlled_allowance
            } else {
                config.combat.target_allowance
            };
            CombatRobotBundle::training(identity.id, identity.team, preset)
                .with_allowance(allowance)
                .with_shooter_config(&config.projectile)
        };
        world.entity_mut(root).insert(bundle);
        let mut state = world.get_mut::<RobotCombatState>(root).unwrap();
        state.heat = HeatState::new(now);
        let label = match world.get::<RobotIdentity>(root).unwrap().role {
            CombatRole::Sentry => ArmorLabel::Sentry,
            CombatRole::HeroTarget => ArmorLabel::One,
            CombatRole::Infantry => ArmorLabel::Three,
        };
        let members: Vec<_> = world
            .query::<(Entity, &RobotMember)>()
            .iter(world)
            .filter_map(|(e, m)| (m.root == root).then_some(e))
            .collect();
        for entity in members {
            if let Some(mut armor) = world.get_mut::<Armor>(entity) {
                armor.label = label;
            }
            if world.get::<ArmorStickerSelection>(entity).is_some() {
                world
                    .entity_mut(entity)
                    .insert(ArmorStickerSelection::new(label));
            }
        }
    }
    if let Some(mut controller) = world.get_resource_mut::<ControllerState>() {
        controller.reset_round();
    }
    if let Some(enabled) = world.get_resource::<SubscribeAutoAim>() {
        enabled.store(false, std::sync::atomic::Ordering::Release);
    }
    if let Some(mut previous) = world.get_resource_mut::<PreviousKinematicState>() {
        *previous = default();
    }
    if let Some(mut frame) = world.get_resource_mut::<ChassisObservationFrame>() {
        *frame = default();
    }
    if let Some(mut stats) = world.get_resource_mut::<ProjectileStatistics>() {
        *stats = default();
    }
    crate::gimbal_actuator::reset_scene_commands(world);
    let mut round = world.resource_mut::<TrainingRound>();
    if round.summaries.len() == SUMMARY_CAPACITY {
        round.summaries.pop_front();
    }
    round.summaries.push_back(summary);
    round.id += 1;
    round.started_at = now;
    round.requested = false;
    round.events.clear();
    round.dropped_events = 0;
    *generation = round.id;
    info!(
        "scene reset round={} sim_s={:.9} cleared_projectiles={} external_control=OFF",
        round.id,
        now.as_secs_f64(),
        cleared_projectiles
    );
}
