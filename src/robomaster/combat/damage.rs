//! RMUL 2026 V1.2.0 §3.3.1.1, table 3-7 (printed page 29): 17 mm = 20 HP.
//! Training simplification: first armor contact counts, without speed/impulse/debounce gates.

use super::shooting::{ProjectileShot, cancel_robot_requests};
use super::{LifeStatus, RobotCombatState, RobotId, RobotIdentity, RobotMember};
use crate::components::{ActiveSlapper, Controlled, InfantryChassis, SubscribeAutoAim};
use crate::robomaster::prelude::{Armor, ArmorRoot, LightStrip, apply_projectile_rune_hit};
use crate::statistic::ProjectileStatistics;
use crate::systems::ControllerState;
use avian3d::prelude::*;
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use std::sync::atomic::Ordering;
use std::time::Duration;

pub const ARMOR_DAMAGE_17MM: u32 = 20;

/// CollisionStart in Avian also includes predictive contacts with positive separation.
/// Only 17 mm projectiles opt into this hook; swept CCD supplies their anti-tunneling.
#[derive(SystemParam)]
pub struct ProjectileContactHooks<'w, 's> {
    projectiles: Query<'w, 's, (), With<ProjectileShot>>,
}

impl CollisionHooks for ProjectileContactHooks<'_, '_> {
    fn modify_contacts(&self, contacts: &mut ContactPair, _commands: &mut Commands) -> bool {
        if !self.projectiles.contains(contacts.collider1)
            && !self.projectiles.contains(contacts.collider2)
        {
            return true;
        }
        // 0.1 mm tolerates float/sweep roundoff; this is geometry tolerance, not an impact
        // strength or speed threshold. Engine collision margins are already in penetration.
        contacts.manifolds.retain_mut(|manifold| {
            manifold.points.retain(|point| point.penetration >= -0.0001);
            !manifold.points.is_empty()
        });
        !contacts.manifolds.is_empty()
    }
}

/// Drive interlock on a dead robot's root and imported descendants. Reset must remove it.
/// RigidBody, colliders and physical velocities remain intact.
#[derive(Component)]
pub struct CombatDead;

/// Last physical-step position for a swept first-contact check against imported child colliders.
/// Avian 0.7's built-in SweptCcd currently sweeps body colliders, not child armor colliders.
#[derive(Component)]
pub struct ProjectileSweepStart(pub Vec3);

#[derive(Message, Clone, Copy, Debug)]
pub struct ArmorContact {
    pub projectile_id: u64,
    pub shooter: RobotId,
    pub target: Option<RobotId>,
    pub at: Duration,
}

#[derive(Message, Clone, Copy, Debug)]
pub struct DamageApplied {
    pub projectile_id: u64,
    pub shooter: RobotId,
    pub target: RobotId,
    pub at: Duration,
    pub nominal: u32,
    pub actual: u32,
    pub remaining_hp: u32,
}

#[derive(Message, Clone, Copy, Debug)]
pub struct RobotDied {
    pub robot: RobotId,
    pub killer: RobotId,
    pub projectile_id: u64,
    pub at: Duration,
}

#[derive(Resource, Default)]
struct Impacts(Vec<CollisionStart>);

/// Keep each mesh's previous handle for the future scene reset. Never alter shared GLB assets.
#[derive(Component)]
pub struct ExtinguishedLight {
    pub original: Handle<StandardMaterial>,
}

pub struct DamagePlugin;

impl Plugin for DamagePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Impacts>()
            .add_message::<ArmorContact>()
            .add_message::<DamageApplied>()
            .add_message::<RobotDied>()
            .add_systems(
                FixedPostUpdate,
                (collect_impacts, resolve_impacts)
                    .chain()
                    .after(PhysicsSystems::Last),
            )
            .add_systems(Update, extinguish_lights);
    }
}

fn collect_impacts(
    mut events: MessageReader<CollisionStart>,
    mut impacts: ResMut<Impacts>,
    spatial: SpatialQuery,
    mut projectiles: Query<
        (
            Entity,
            &Position,
            &Collider,
            &CollisionLayers,
            &mut ProjectileSweepStart,
        ),
        With<ProjectileShot>,
    >,
    colliders: Query<(&CollisionLayers, Has<Sensor>, Option<&ColliderOf>)>,
) {
    // Resolve each projectile's nearest swept hit before endpoint contact events. This covers
    // thin armor crossed during a step and prevents a farther endpoint hit winning over a wall.
    for (entity, position, shape, layers, mut start) in &mut projectiles {
        let displacement = position.0 - start.0;
        if let Ok(direction) = Dir3::new(displacement) {
            let filter =
                SpatialQueryFilter::from_mask(layers.filters).with_excluded_entities([entity]);
            if let Some(hit) = spatial.cast_shape_predicate(
                shape,
                start.0,
                Quat::IDENTITY,
                direction,
                &ShapeCastConfig::from_max_distance(displacement.length() + 0.0001),
                &filter,
                &|candidate| {
                    colliders
                        .get(candidate)
                        .is_ok_and(|(other, sensor, _)| !sensor && layers.interacts_with(*other))
                },
            ) {
                let body = colliders
                    .get(hit.entity)
                    .ok()
                    .and_then(|(_, _, body)| body.map(|b| b.body));
                impacts.0.push(CollisionStart {
                    collider1: entity,
                    collider2: hit.entity,
                    body1: Some(entity),
                    body2: body,
                });
            }
        }
        start.0 = position.0;
    }
    impacts.0.extend(events.read().copied());
}

/// Resolve after Avian finishes its contact observers. This preserves rune processing order
/// and avoids deleting bodies while the solver is traversing them. Within one physical step,
/// the first reported contact wins; no sub-step time-of-impact ordering is claimed.
fn resolve_impacts(world: &mut World) {
    let impacts = std::mem::take(&mut world.resource_mut::<Impacts>().0);
    for impact in impacts {
        resolve_impact(world, impact);
    }
}

fn ancestor_with<T: Component>(world: &World, mut entity: Entity) -> Option<Entity> {
    loop {
        if world.get::<T>(entity).is_some() {
            return Some(entity);
        }
        entity = world.get::<ChildOf>(entity)?.parent();
    }
}

fn resolve_impact(world: &mut World, impact: CollisionStart) {
    let candidate = |collider, body: Option<Entity>| {
        body.filter(|body| world.get::<ProjectileShot>(*body).is_some())
            .or_else(|| world.get::<ProjectileShot>(collider).map(|_| collider))
    };
    let (projectile, other, other_body) =
        if let Some(projectile) = candidate(impact.collider1, impact.body1) {
            (projectile, impact.collider2, impact.body2)
        } else if let Some(projectile) = candidate(impact.collider2, impact.body2) {
            (projectile, impact.collider1, impact.body1)
        } else {
            return;
        };
    let shot = *world.get::<ProjectileShot>(projectile).unwrap();
    let target = world
        .get::<RobotMember>(other)
        .map(|m| m.root)
        .or_else(|| ancestor_with::<RobotIdentity>(world, other))
        .or_else(|| other_body.filter(|body| world.get::<RobotIdentity>(*body).is_some()));
    let shooter = world
        .query::<(Entity, &RobotIdentity)>()
        .iter(world)
        .find_map(|(root, id)| (id.id == shot.request.robot).then_some((root, *id)));
    let identity = target
        .and_then(|root| world.get::<RobotIdentity>(root))
        .copied();
    // The collision layers normally exclude these pairs; also defend the damage boundary.
    if identity.is_some_and(|target| {
        target.id == shot.request.robot
            || shooter.is_some_and(|(_, shooter)| target.team == shooter.team)
    }) {
        return;
    }
    if world.get::<Sensor>(other).is_some() {
        return;
    }
    let at = world.resource::<Time<Fixed>>().elapsed();
    let armor = ancestor_with::<ArmorRoot>(world, other).is_some()
        || ancestor_with::<Armor>(world, other).is_some();
    if armor {
        world
            .resource_mut::<ProjectileStatistics>()
            .increase_armor_hit();
        if let Some((shooter, _)) = shooter {
            world
                .get_mut::<RobotCombatState>(shooter)
                .unwrap()
                .damage
                .armor_contacts += 1;
        }
        super::reset::record_event(
            world,
            at,
            format!(
                "armor_contact projectile={} shooter={} target={:?}",
                shot.id,
                shot.request.robot.0,
                identity.map(|id| id.id.0)
            ),
        );
        world.write_message(ArmorContact {
            projectile_id: shot.id,
            shooter: shot.request.robot,
            target: identity.map(|id| id.id),
            at,
        });
        info!(
            "armor contact projectile={} shooter={} target={:?} sim_s={:.9}",
            shot.id,
            shot.request.robot.0,
            identity.map(|id| id.id.0),
            at.as_secs_f64()
        );
        if let (Some(root), Some(identity)) = (target, identity) {
            apply_damage(world, root, identity.id, shooter.map(|s| s.0), shot, at);
        }
    }
    // Rune activation remains a separate mechanism, never robot HP damage.
    apply_projectile_rune_hit(world, other);
    info!(
        "projectile consumed id={} sim_s={:.9} armor={} target={:?} collider={:?} name={:?} body={:?} muzzle={:?}",
        shot.id,
        at.as_secs_f64(),
        armor,
        identity.map(|id| id.id.0),
        other,
        world.get::<Name>(other),
        other_body,
        shot.muzzle_pose.translation
    );
    // Immediate removal deduplicates all remaining contact messages in this batch. Other
    // in-flight projectiles (including those fired by a now-dead shooter) remain untouched.
    super::ledger::ended(
        world,
        projectile,
        at,
        if armor { "armor" } else { "obstacle" },
    );
    world.despawn(projectile);
}

fn apply_damage(
    world: &mut World,
    root: Entity,
    target: RobotId,
    shooter: Option<Entity>,
    shot: ProjectileShot,
    at: Duration,
) {
    let Some(mut state) = world.get_mut::<RobotCombatState>(root) else {
        return;
    };
    if state.life.status != LifeStatus::Alive || state.life.hp == 0 {
        return;
    }
    let actual = state.life.hp.min(ARMOR_DAMAGE_17MM);
    state.life.hp -= actual;
    state.damage.damage_taken += u64::from(actual);
    let remaining_hp = state.life.hp;
    if let Some(shooter) = shooter {
        let mut state = world.get_mut::<RobotCombatState>(shooter).unwrap();
        state.damage.damaging_hits += 1;
        state.damage.damage_dealt += u64::from(actual);
        if remaining_hp == 0 {
            state.damage.kills += 1;
        }
    }
    super::reset::record_event(
        world,
        at,
        format!(
            "damage projectile={} shooter={} target={} actual={} hp={}",
            shot.id, shot.request.robot.0, target.0, actual, remaining_hp
        ),
    );
    super::ledger::record(
        world,
        at,
        "damage_applied",
        serde_json::json!({"projectile_id":shot.id,"request_id":shot.request.id,"shooter":shot.request.robot.0,"target":target.0,"actual":actual,"remaining_hp":remaining_hp}),
    );
    world.write_message(DamageApplied {
        projectile_id: shot.id,
        shooter: shot.request.robot,
        target,
        at,
        nominal: ARMOR_DAMAGE_17MM,
        actual,
        remaining_hp,
    });
    info!(
        "damage projectile={} shooter={} target={} sim_s={:.9} nominal={} actual={} hp={}",
        shot.id,
        shot.request.robot.0,
        target.0,
        at.as_secs_f64(),
        ARMOR_DAMAGE_17MM,
        actual,
        remaining_hp
    );
    if remaining_hp == 0 {
        die(world, root, target, shot, at);
    }
}

fn die(world: &mut World, root: Entity, robot: RobotId, shot: ProjectileShot, at: Duration) {
    let mut state = world.get_mut::<RobotCombatState>(root).unwrap();
    state.life.status = LifeStatus::Dead;
    state.heat.on_death(at);
    cancel_robot_requests(world, root, robot, at);
    let descendants: Vec<_> = world
        .query::<(Entity, &RobotMember)>()
        .iter(world)
        .filter_map(|(entity, member)| (member.root == root).then_some(entity))
        .collect();
    for entity in std::iter::once(root).chain(descendants) {
        world.entity_mut(entity).insert(CombatDead);
        if let Some(mut chassis) = world.get_mut::<InfantryChassis>(entity) {
            chassis.yaw_velocity = 0.0;
        }
    }
    let controlled = world.get::<Controlled>(root).is_some();
    let active_target = world.get::<ActiveSlapper>(root).is_some();
    if (controlled || active_target)
        && let Some(mut controller) = world.get_resource_mut::<ControllerState>()
    {
        controller.clear_robot_input(controlled);
    }
    if controlled {
        if let Some(enabled) = world.get_resource::<SubscribeAutoAim>() {
            enabled.store(false, Ordering::Release);
        }
        crate::gimbal_actuator::clear_dead_robot_commands(world);
    }
    super::reset::record_event(
        world,
        at,
        format!(
            "death robot={} killer={} projectile={}",
            robot.0, shot.request.robot.0, shot.id
        ),
    );
    world.write_message(RobotDied {
        robot,
        killer: shot.request.robot,
        projectile_id: shot.id,
        at,
    });
    info!(
        "robot died robot={} killer={} projectile={} sim_s={:.9}",
        robot.0,
        shot.request.robot.0,
        shot.id,
        at.as_secs_f64()
    );
}

fn extinguish_lights(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    robots: Query<&RobotCombatState>,
    mut lights: Query<
        (Entity, &RobotMember, &mut MeshMaterial3d<StandardMaterial>),
        (With<LightStrip>, Without<ExtinguishedLight>),
    >,
) {
    for (entity, member, mut handle) in &mut lights {
        if !robots
            .get(member.root)
            .is_ok_and(|state| state.life.status == LifeStatus::Dead)
        {
            continue;
        }
        let Some(original) = materials.get(&handle.0) else {
            continue;
        };
        let mut dark = original.clone();
        dark.base_color = Color::srgb(0.025, 0.025, 0.025);
        dark.base_color_texture = None;
        dark.emissive = LinearRgba::BLACK;
        dark.emissive_texture = None;
        dark.unlit = false;
        commands.entity(entity).insert(ExtinguishedLight {
            original: handle.0.clone(),
        });
        handle.0 = materials.add(dark);
    }
}

/// The next round must not resolve contacts collected before its teleport boundary.
pub(super) fn clear_round_impacts(world: &mut World) {
    if let Some(mut impacts) = world.get_resource_mut::<Impacts>() {
        impacts.0.clear();
    }
}
