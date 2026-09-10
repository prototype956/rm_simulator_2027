//! Single 17 mm firing authority. Requests are consumed once at a physics boundary.
//! Talos command transport still uses its existing wall clock; firing uses Fixed simulation time.

use super::heat::{add_shot_heat, cool_robot_heat};
use super::{LifeStatus, RobotCombatState, RobotId, RobotIdentity, RobotMember, ShooterKind};
use crate::components::{
    Controlled, GameLayer, InfantryLaunchOffset, ProjectileLifetime, ProjectileSetting,
    SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::robomaster::prelude::Projectile;
use crate::statistic::ProjectileStatistics;
use crate::systems::{ControllerState, request_controller_rumble};
use avian3d::prelude::*;
use bevy::input::gamepad::{GamepadRumbleIntensity, GamepadRumbleRequest};
use bevy::prelude::*;
use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::time::Duration;

const REQUEST_CAPACITY: usize = 256;

#[derive(Reflect, Clone, Copy, Debug, PartialEq, Eq)]
pub enum FireSource {
    /// Combined keyboard/gamepad trigger, sampled by ControllerState.
    Manual,
    /// Original command timestamp, for correlation only, never used as simulation time.
    Talos { command_timestamp_ns: u64 },
    /// Compatibility adapter for optional ROS2 input; its existing limiter is retained.
    Ros2,
}

#[derive(Reflect, Clone, Copy, Debug)]
pub struct FireRequest {
    pub id: u64,
    pub robot: RobotId,
    pub source: FireSource,
    pub requested_at: Duration,
}

#[derive(Reflect, Clone, Copy, Debug)]
pub struct PendingShot {
    pub request: FireRequest,
    pub ready_at: Duration,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FireRejection {
    Dead,
    AllowanceEmpty,
    QueueFull,
    UnknownRobot,
    NoShooter,
    HeatCoolingLock,
    HeatRoundLock,
    MechanicalInterval,
    FeedBusy,
    ExternalControlDisabled,
    MissingMuzzle,
    InvalidMuzzlePose,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FireResult {
    /// An admitted request with an explicit feed delay; it is not a launched projectile.
    Feeding {
        ready_at: Duration,
    },
    Fired {
        projectile_id: u64,
    },
    Rejected(FireRejection),
}

#[derive(Message, Clone, Copy, Debug)]
pub struct FireRequestResult {
    pub request: FireRequest,
    pub resolved_at: Duration,
    pub result: FireResult,
}

/// Immutable launch truth attached only to actual 17 mm projectiles.
/// Pose and velocity use Bevy world coordinates (Y up), meters and m/s.
#[derive(Component, Clone, Copy, Debug)]
pub struct ProjectileShot {
    pub id: u64,
    pub request: FireRequest,
    pub fired_at: Duration,
    pub muzzle_pose: Transform,
    pub initial_velocity: Vec3,
}

#[derive(Message, Clone, Copy, Debug)]
pub struct ShotFired {
    pub projectile: Entity,
    pub shot: ProjectileShot,
}

#[derive(Resource, Default)]
struct FireRequests {
    next_request_id: u64,
    next_projectile_id: u64,
    queue: VecDeque<FireRequest>,
}

/// Read-only training diagnostics; does not admit, cancel or drain requests.
#[cfg(feature = "training")]
pub(crate) fn queued_requests(world: &World) -> Vec<FireRequest> {
    world
        .resource::<FireRequests>()
        .queue
        .iter()
        .copied()
        .collect()
}

/// Training-only admission fence; existing queued requests and feed remain executable.
#[cfg(feature = "training")]
#[derive(Resource)]
pub(crate) struct FireAdmissionClosed;

pub struct ShootingPlugin;

impl Plugin for ShootingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FireRequests>()
            .add_message::<FireRequestResult>()
            .add_message::<ShotFired>()
            .add_systems(
                FixedPostUpdate,
                (collect_manual_requests, execute_fire_requests)
                    .chain()
                    .before(PhysicsSystems::First),
            )
            .add_systems(Update, shot_rumble);
    }
}

/// All input adapters submit here. No projectile is created until the next firing boundary.
/// Queue capacity bounds memory even while the physics clock is paused.
pub fn request_fire(world: &mut World, robot: RobotId, source: FireSource) {
    #[cfg(feature = "training")]
    if world.contains_resource::<FireAdmissionClosed>() {
        return;
    }
    let now = world.resource::<Time<Fixed>>().elapsed();
    let mut requests = world.resource_mut::<FireRequests>();
    requests.next_request_id = requests
        .next_request_id
        .checked_add(1)
        .expect("request ID overflow");
    let request = FireRequest {
        id: requests.next_request_id,
        robot,
        source,
        requested_at: now,
    };
    super::ledger::record(
        world,
        now,
        "fire_requested",
        serde_json::json!({"request_id":request.id,"robot_id":robot.0,"source":source_data(source)}),
    );
    let mut requests = world.resource_mut::<FireRequests>();
    if requests.queue.len() < REQUEST_CAPACITY {
        requests.queue.push_back(request);
    } else {
        report(
            world,
            request,
            now,
            FireResult::Rejected(FireRejection::QueueFull),
        );
    }
}

pub(crate) fn source_data(source: FireSource) -> serde_json::Value {
    match source {
        FireSource::Talos {
            command_timestamp_ns,
        } => serde_json::json!({"kind":"command","command_timestamp_ns":command_timestamp_ns}),
        FireSource::Manual => serde_json::json!({"kind":"manual"}),
        FireSource::Ros2 => serde_json::json!({"kind":"ros2"}),
    }
}

fn result_data(result: FireResult) -> serde_json::Value {
    match result {
        FireResult::Feeding { ready_at } => {
            serde_json::json!({"kind":"feeding","ready_at_ns":ready_at.as_nanos() as u64})
        }
        FireResult::Fired { projectile_id } => {
            serde_json::json!({"kind":"fired","projectile_id":projectile_id})
        }
        FireResult::Rejected(reason) => {
            serde_json::json!({"kind":"rejected","reason":format!("{reason:?}")})
        }
    }
}

fn report(world: &mut World, request: FireRequest, now: Duration, result: FireResult) {
    if matches!(result, FireResult::Rejected(_)) {
        for (identity, mut state) in world
            .query::<(&RobotIdentity, &mut RobotCombatState)>()
            .iter_mut(world)
        {
            if identity.id == request.robot {
                state.shooter.rejected_requests += 1;
                break;
            }
        }
    }
    super::ledger::record(
        world,
        now,
        "fire_result",
        serde_json::json!({"request_id":request.id,"robot_id":request.robot.0,"source":source_data(request.source),"result":result_data(result)}),
    );
    info!(
        "fire request={} robot={} source={:?} sim_s={:.9} result={:?}",
        request.id,
        request.robot.0,
        request.source,
        now.as_secs_f64(),
        result,
    );
    super::reset::record_event(
        world,
        now,
        format!(
            "fire request={} robot={} result={:?}",
            request.id, request.robot.0, result
        ),
    );
    world.write_message(FireRequestResult {
        request,
        resolved_at: now,
        result,
    });
}

fn collect_manual_requests(world: &mut World) {
    let held = world
        .get_resource::<ControllerState>()
        .is_some_and(|s| s.controlled.shoot);
    let now = world.resource::<Time<Fixed>>().elapsed();
    let mut due = Vec::new();
    let mut robots =
        world.query_filtered::<(&RobotIdentity, &mut RobotCombatState), With<Controlled>>();
    for (identity, mut state) in robots.iter_mut(world) {
        if state.life.status == LifeStatus::Dead {
            continue;
        }
        let shooter = &mut state.shooter;
        if !held {
            shooter.last_manual_request_at = None;
        } else if shooter
            .last_manual_request_at
            .is_none_or(|last| now.saturating_sub(last) >= shooter.mechanics.min_interval)
        {
            // Repeat attempts, not successful shots. A rejected attempt is never queued for retry.
            shooter.last_manual_request_at = Some(now);
            due.push(identity.id);
        }
    }
    for robot in due {
        request_fire(world, robot, FireSource::Manual);
    }
}

fn rejection(
    world: &World,
    root: Entity,
    request: FireRequest,
    now: Duration,
) -> Option<FireRejection> {
    let state = world.get::<RobotCombatState>(root)?;
    if state.life.status == LifeStatus::Dead {
        return Some(FireRejection::Dead);
    }
    if state.shooter.kind == ShooterKind::None {
        return Some(FireRejection::NoShooter);
    }
    if matches!(request.source, FireSource::Talos { .. } | FireSource::Ros2)
        && !world
            .get_resource::<SubscribeAutoAim>()
            .is_some_and(|enabled| enabled.load(Ordering::Acquire))
    {
        return Some(FireRejection::ExternalControlDisabled);
    }
    if state.allowance == super::FireAllowance::Limited(0) {
        return Some(FireRejection::AllowanceEmpty);
    }
    // Report the permanent reason first when both independent thermal latches are set.
    if state.heat.round_locked {
        return Some(FireRejection::HeatRoundLock);
    }
    if state.heat.cooling_locked {
        return Some(FireRejection::HeatCoolingLock);
    }
    if state
        .shooter
        .last_shot_at
        .is_some_and(|last| now.saturating_sub(last) < state.shooter.mechanics.min_interval)
    {
        return Some(FireRejection::MechanicalInterval);
    }
    None
}

/// Exclusive execution makes spawn -> count -> result atomic and ordered across simultaneous
/// requests. No deferred spawn or observer can report a shot that has not yet become an entity.
pub(super) fn execute_fire_requests(world: &mut World) {
    // Deterministic convention: due cooling -> admission/feed completion -> shot heat/locks.
    cool_robot_heat(world);
    let now = world.resource::<Time<Fixed>>().elapsed();
    let robots: Vec<_> = world
        .query_filtered::<(Entity, &RobotIdentity), With<RobotCombatState>>()
        .iter(world)
        .map(|(root, identity)| (root, identity.id))
        .collect();

    // Only explicitly admitted feed delays survive between steps. Due feeds go first.
    for &(root, _) in &robots {
        let pending = world
            .get::<RobotCombatState>(root)
            .and_then(|state| state.shooter.pending);
        if let Some(pending) = pending {
            let disabled = matches!(
                pending.request.source,
                FireSource::Talos { .. } | FireSource::Ros2
            ) && !world
                .get_resource::<SubscribeAutoAim>()
                .is_some_and(|enabled| enabled.load(Ordering::Acquire));
            let thermal_locked = world
                .get::<RobotCombatState>(root)
                .unwrap()
                .heat
                .is_locked();
            if disabled || thermal_locked || pending.ready_at <= now {
                world
                    .get_mut::<RobotCombatState>(root)
                    .unwrap()
                    .shooter
                    .pending = None;
                fire_one(world, root, pending.request, now);
            }
        }
    }

    let requests: Vec<_> = world
        .resource_mut::<FireRequests>()
        .queue
        .drain(..)
        .collect();
    for request in requests {
        let Some(&(root, _)) = robots.iter().find(|(_, id)| *id == request.robot) else {
            report(
                world,
                request,
                now,
                FireResult::Rejected(FireRejection::UnknownRobot),
            );
            continue;
        };
        if let Some(reason) = rejection(world, root, request, now) {
            report(world, request, now, FireResult::Rejected(reason));
            continue;
        }
        let mut state = world.get_mut::<RobotCombatState>(root).unwrap();
        if state.shooter.pending.is_some() {
            report(
                world,
                request,
                now,
                FireResult::Rejected(FireRejection::FeedBusy),
            );
            continue;
        }
        let delay = state.shooter.mechanics.launch_delay;
        if !delay.is_zero() {
            let ready_at = now.saturating_add(delay);
            state.shooter.pending = Some(PendingShot { request, ready_at });
            report(world, request, now, FireResult::Feeding { ready_at });
        } else {
            fire_one(world, root, request, now);
        }
    }
}

/// Resolve the entire muzzle hierarchy at firing time, including base/gimbal translations.
/// Read the rigid body's physical pose, not potentially interpolated GlobalTransform data.
pub(super) fn muzzle_pose(world: &mut World, root: Entity) -> Result<Transform, FireRejection> {
    let muzzle = world
        .query_filtered::<(Entity, &RobotMember), With<InfantryLaunchOffset>>()
        .iter(world)
        .find_map(|(entity, member)| (member.root == root).then_some(entity))
        .ok_or(FireRejection::MissingMuzzle)?;
    let mut chain = Vec::new();
    let mut entity = muzzle;
    while entity != root {
        chain.push(
            *world
                .get::<Transform>(entity)
                .ok_or(FireRejection::MissingMuzzle)?,
        );
        entity = world
            .get::<ChildOf>(entity)
            .ok_or(FireRejection::MissingMuzzle)?
            .parent();
    }
    let mut root_pose = *world
        .get::<Transform>(root)
        .ok_or(FireRejection::MissingMuzzle)?;
    if let (Some(position), Some(rotation)) =
        (world.get::<Position>(root), world.get::<Rotation>(root))
    {
        root_pose.translation = position.0;
        root_pose.rotation = rotation.0;
    }
    let mut global = GlobalTransform::from(root_pose);
    for transform in chain.iter().rev() {
        global = global.mul_transform(*transform);
    }
    let pose = global.compute_transform();
    if !pose.translation.is_finite() || !pose.rotation.is_finite() || !pose.rotation.is_normalized()
    {
        return Err(FireRejection::InvalidMuzzlePose);
    }
    Ok(Transform {
        scale: Vec3::ONE,
        ..pose
    })
}

fn fire_one(world: &mut World, root: Entity, request: FireRequest, now: Duration) {
    if let Some(reason) = rejection(world, root, request, now) {
        report(world, request, now, FireResult::Rejected(reason));
        return;
    }
    let pose = match muzzle_pose(world, root) {
        Ok(pose) => pose,
        Err(reason) => {
            report(world, request, now, FireResult::Rejected(reason));
            return;
        }
    };
    let speed = world
        .get::<RobotCombatState>(root)
        .unwrap()
        .shooter
        .mechanics
        .speed_mps;
    let body_velocity = world
        .get::<LinearVelocity>(root)
        .copied()
        .unwrap_or_default();
    let angular_velocity = world
        .get::<AngularVelocity>(root)
        .copied()
        .unwrap_or_default();
    // Preserve the existing chassis translational velocity contribution; gimbal angular
    // tangential velocity and wheel/barrel dynamics are outside this mechanical model.
    let initial_velocity = body_velocity.0 + pose.rotation * Vec3::Y * speed;
    if !initial_velocity.is_finite() {
        report(
            world,
            request,
            now,
            FireResult::Rejected(FireRejection::InvalidMuzzlePose),
        );
        return;
    }
    let mut requests = world.resource_mut::<FireRequests>();
    requests.next_projectile_id = requests
        .next_projectile_id
        .checked_add(1)
        .expect("projectile ID overflow");
    let shot = ProjectileShot {
        id: requests.next_projectile_id,
        request,
        fired_at: now,
        muzzle_pose: pose,
        initial_velocity,
    };
    let config = &world.resource::<SimulationConfig>().projectile;
    let setting = world.resource::<ProjectileSetting>();
    let body = (
        RigidBody::Dynamic,
        Collider::sphere(config.diameter / 2.0),
        Mass(config.mass),
        Friction::new(config.friction),
        Restitution::ZERO,
        LinearDamping(config.linear_damping),
        GameLayer::projectile_collision_layers(world.get::<Controlled>(root).is_some()),
        Mesh3d(setting.0.clone()),
        MeshMaterial3d(setting.1.clone()),
        LinearVelocity(initial_velocity),
        angular_velocity,
        pose,
        ProjectileLifetime(Timer::from_seconds(config.lifetime, TimerMode::Once)),
        Projectile,
        shot,
    );
    // Avian CollisionStart includes speculative pairs. At 25 m/s its default prediction
    // distance can reach the ground while a bullet is still ~40 cm above it. Swept CCD
    // prevents tunneling without treating that long prediction margin as a physical hit.
    let projectile = world
        .spawn((
            body,
            SweptCcd::default(),
            SpeculativeMargin::ZERO,
            ActiveCollisionHooks::MODIFY_CONTACTS,
            super::damage::ProjectileSweepStart(pose.translation),
        ))
        .id();
    let mut state = world.get_mut::<RobotCombatState>(root).unwrap();
    state.shooter.last_shot_at = Some(now);
    state.shooter.actual_shots += 1;
    if let super::FireAllowance::Limited(remaining) = &mut state.allowance {
        *remaining -= 1;
    }
    world
        .resource_mut::<ProjectileStatistics>()
        .increase_bullet_launch();
    add_shot_heat(world, root, now);
    super::ledger::record(
        world,
        now,
        "shot_fired",
        serde_json::json!({"projectile_id":shot.id,"request_id":request.id,"robot_id":request.robot.0,"muzzle_position":pose.translation.to_array(),"initial_velocity":initial_velocity.to_array()}),
    );
    world.write_message(ShotFired { projectile, shot });
    report(
        world,
        request,
        now,
        FireResult::Fired {
            projectile_id: shot.id,
        },
    );
}

/// Resolve and discard both queued requests and admitted feeds when their robot dies.
pub(super) fn cancel_robot_requests(
    world: &mut World,
    root: Entity,
    robot: RobotId,
    now: Duration,
) {
    let mut cancelled = Vec::new();
    if let Some(mut requests) = world.get_resource_mut::<FireRequests>() {
        requests.queue.retain(|request| {
            if request.robot == robot {
                cancelled.push(*request);
                false
            } else {
                true
            }
        });
    }
    if let Some(mut state) = world.get_mut::<RobotCombatState>(root) {
        if let Some(pending) = state.shooter.pending.take() {
            cancelled.push(pending.request);
        }
        state.shooter.last_manual_request_at = None;
    }
    for request in cancelled {
        report(
            world,
            request,
            now,
            FireResult::Rejected(FireRejection::Dead),
        );
    }
}

fn shot_rumble(
    mut shots: MessageReader<ShotFired>,
    controller: Option<Res<ControllerState>>,
    controlled: Query<&RobotIdentity, With<Controlled>>,
    mut requests: MessageWriter<GamepadRumbleRequest>,
) {
    for shot in shots.read() {
        if controlled
            .iter()
            .any(|identity| identity.id == shot.shot.request.robot)
        {
            request_controller_rumble(
                controller.as_deref(),
                &mut requests,
                GamepadRumbleIntensity {
                    strong_motor: 0.45,
                    weak_motor: 0.2,
                },
                Duration::from_millis(80),
            );
        }
    }
}

/// Clear pending admission without resetting globally unique request/projectile IDs.
pub(super) fn clear_round_requests(world: &mut World) {
    if let Some(mut requests) = world.get_resource_mut::<FireRequests>() {
        requests.queue.clear();
    }
}
