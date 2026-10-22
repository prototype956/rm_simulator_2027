//! Separate sensor and privileged contracts. Neither contains a future motion program.
use crate::capture_geometry::*;
use crate::components::*;
use crate::gimbal_actuator::*;
use crate::robomaster::{
    combat::*,
    prelude::{ArmorParts, ArmorRoot, VertexData},
};
use avian3d::prelude::*;
use bevy::{ecs::system::RunSystemOnce, prelude::*};
use serde_json::{Value, json};

#[derive(Resource, Default)]
struct Sensors {
    last: Option<(u64, Quat)>,
    sequence: Option<u64>,
    imu: Value,
    thermal: Value,
}

/// Ideal installed IMU: orientation and body angular velocity, no position/velocity navigation.
/// Called after every fixed physics update; thermal data is latched only on referee samples.
pub(super) fn sample(w: &mut World) -> Result<(), String> {
    let now = w.resource::<Time<Fixed>>().elapsed();
    let ns = now.as_nanos() as u64;
    let gimbal = w
        .query_filtered::<&GlobalTransform, (With<Controlled>, With<InfantryGimbal>)>()
        .single(w)
        .map_err(|e| e.to_string())?
        .rotation();
    let muzzle_local = w
        .query_filtered::<&Transform, (With<Controlled>, With<InfantryLaunchOffset>)>()
        .single(w)
        .map_err(|e| e.to_string())?
        .rotation;
    let shot = gimbal * muzzle_local;
    let pose = transform_from_axes(
        Vec3::ZERO,
        to_ros_translation(shot * Vec3::Y),
        to_ros_translation(shot * -Vec3::X),
        to_ros_translation(shot * Vec3::Z),
    );
    let q = quat_from_wire(pose.rotation);
    let frame = w.resource::<telemetry::CombatTelemetry>().frame;
    let thermal = w.query::<(&RobotIdentity, &RobotCombatState)>().iter(w)
        .find(|(id, _)| id.id == CONTROLLED_ROBOT_ID).map(|(_, s)| {
            let r = frame.self_referee;
            json!({"valid":frame.referee_valid != 0,"sample_ns":frame.referee_sample_ns,
                "sequence":frame.referee_sample_sequence,"heat":r.heat,
                "heat_limit":s.rules.heat_limit,"base_cooling_per_second":s.rules.cooling_per_second,
                "allowance_remaining":if r.allowance_mode == 0 {-1i64} else {r.allowance_remaining as i64},
                "cooling_active":s.league.cooling_active(now),
                "cooling_remaining_s":s.league.cooling_left(now),
                "effective_cooling_per_second":r.cooling_per_second})
        }).ok_or("controlled combat state missing")?;
    let valid = w.resource::<GimbalActuatorTelemetry>().valid;
    if !w.contains_resource::<Sensors>() {
        w.insert_resource(Sensors::default());
    }
    let mut sensors = w.resource_mut::<Sensors>();
    if sensors.last.is_none_or(|(at, _)| at != ns) {
        let gyro = sensors
            .last
            .filter(|(at, _)| ns > *at)
            .map_or(Vec3::ZERO, |(at, previous)| {
                let mut delta = previous.inverse() * q;
                if delta.w < 0.0 {
                    delta = -delta;
                }
                // Rotation vector in the current installed gimbal frame, rad/s.
                let (axis, angle) = delta.to_axis_angle();
                axis * angle / ((ns - at) as f32 * 1e-9)
            });
        sensors.imu = json!({"valid":valid,"sample_ns":ns,"orientation_xyzw":q.to_array(),
            "gyro_rad_s":gyro.to_array()});
        sensors.last = Some((ns, q));
    }
    if sensors.sequence != Some(frame.referee_sample_sequence) {
        sensors.sequence = Some(frame.referee_sample_sequence);
        sensors.thermal = thermal;
    }
    Ok(())
}

fn geometry(
    robots: Query<(Entity, &GlobalTransform, &Infantry, Option<&RobotIdentity>)>,
    chassis: Query<(&GlobalTransform, &InfantryChassis)>,
    armors: Query<(Entity, &ArmorRoot, &ArmorParts)>,
    vertices: Query<(&GlobalTransform, &VertexData)>,
    parents: Query<&ChildOf>,
    children: Query<&Children>,
    names: Query<&Name>,
    globals: Query<&GlobalTransform>,
    muzzles: Query<(&RobotMember, &GlobalTransform), With<InfantryLaunchOffset>>,
    body_poses: Query<(Entity, &GlobalTransform), With<InfantryChassis>>,
) -> Value {
    let truth = capture_ground_truth(
        &robots, &chassis, &armors, &vertices, &parents, &children, &names, &globals, 0, 0,
    );
    let plates: Vec<_> = truth.armors[..truth.armor_count as usize]
        .iter()
        .filter(|p| p.owner_robot_id == 1 || p.owner_robot_id == 2)
        .map(|p| {
            json!({"owner":p.owner_robot_id,"position_ros_m":p.world_t_armor.translation,
            "rotation_ros_xyzw":quat_from_wire(p.world_t_armor.rotation).to_array(),
            "corners_ros_m":p.corners_world})
        })
        .collect();
    let bodies: Vec<_> = truth.targets[..truth.target_count as usize]
        .iter()
        .filter(|p| p.id == 1 || p.id == 2)
        .map(|p| {
            let rotation = body_poses
                .iter()
                .find(|(entity, _)| {
                    parents.iter_ancestors(*entity).any(|parent| {
                        robots
                            .get(parent)
                            .is_ok_and(|(_, _, _, id)| id.is_some_and(|id| id.id.0 == p.id))
                    })
                })
                .map(|(_, t)| t.rotation().to_array());
            json!({"owner":p.id,"rotation_bevy_xyzw":rotation,
                "yaw_rad":p.yaw,"yaw_velocity_rad_s":p.vyaw})
        })
        .collect();
    let guns: Vec<_> = muzzles
        .iter()
        .map(|(m, t)| {
            json!({"owner":m.id.0,
        "position_bevy_m":t.translation().to_array(),"rotation_bevy_xyzw":t.rotation().to_array()})
        })
        .collect();
    json!({"armors":plates,"chassis":bodies,"muzzles":guns})
}

pub(super) fn snapshot(w: &mut World) -> Result<(Value, Value), String> {
    if !w.contains_resource::<Sensors>() {
        sample(w)?;
    }
    let sensors = w.resource::<Sensors>();
    let actor = json!({"version":1,"imu":sensors.imu,"thermal":sensors.thermal});
    let now = w.resource::<Time<Fixed>>().elapsed();
    let geometry = w.run_system_once(geometry).map_err(|e| e.to_string())?;
    let mut robots: Vec<_> = w.query::<(&RobotIdentity, &Transform, &LinearVelocity,
        &AngularVelocity, &RobotCombatState, Option<&crate::robots::RobotRig>)>().iter(w)
        .map(|(id,t,v,a,s,rig)| json!({"robot_id":id.id.0,"position_bevy_m":t.translation.to_array(),
            "rotation_bevy_xyzw":t.rotation.to_array(),"velocity_bevy_m_s":v.0.to_array(),
            "angular_velocity_bevy_rad_s":a.0.to_array(),"hp":s.life.hp,"max_hp":s.rules.max_hp,
            "heat":s.heat.current(),"heat_limit":s.rules.heat_limit,"base_cooling_per_second":s.rules.cooling_per_second,
            "effective_cooling_per_second":s.effective_cooling(now),
            "allowance_remaining":match s.allowance {FireAllowance::Unlimited => -1i64, FireAllowance::Limited(n) => n as i64},
            "cooling_locked":s.heat.cooling_locked,"round_locked":s.heat.round_locked,
            "cooling_active":s.league.cooling_active(now),"cooling_remaining_s":s.league.cooling_left(now),
            "defense_active":s.league.defense_active(now),"defense_remaining_s":s.league.defense_left(now),
            "region":s.league.current_region(),"tunnel_progress":s.league.tunnel_progress(now),
            "fold":rig.map_or(0.0,|r|r.fold),"lift_m":rig.map_or(0.0,|r|r.lift_m),
            "fire_ready":rig.is_none_or(|r|r.fire_ready()),"blocked":rig.and_then(|r|r.blocked.as_deref()),
            "requested_fold":rig.is_some_and(|r|r.requested_fold),"stable_s":rig.map_or(0.0,|r|r.stable_s),
            "requested_lift_m":rig.map_or(0.0,|r|r.requested_lift_m),
            "lift_velocity_m_s":rig.map_or(0.0,|r|r.lift_velocity),
            "pending_shot_s":s.shooter.pending.map(|p|p.ready_at.saturating_sub(now).as_secs_f64()),
            "last_shot_age_s":s.shooter.last_shot_at.map(|at|now.saturating_sub(at).as_secs_f64())})).collect();
    robots.sort_by_key(|r| r["robot_id"].as_u64());
    let mut projectiles: Vec<_> = w
        .query::<(
            &shooting::ProjectileShot,
            &Position,
            &LinearVelocity,
            &ProjectileLifetime,
        )>()
        .iter(w)
        .map(|(s, p, v, l)| {
            json!({"id":s.id,"owner":s.request.robot.0,
            "position_bevy_m":p.0.to_array(),"velocity_bevy_m_s":v.0.to_array(),
            "remaining_s":l.0.remaining_secs()})
        })
        .collect();
    projectiles.sort_by_key(|p| p["id"].as_u64());
    let mut commands = w
        .resource::<GimbalCommandInbox>()
        .pending_for_reset()
        .map_err(str::to_owned)?;
    commands.extend(w.resource::<GimbalActuator>().pending_for_reset());
    for command in &mut commands {
        // These are still-live requests, not the reset/truncation audit records.
        if let Some(object) = command.as_object_mut() {
            object.remove("status");
            object.remove("source_round_id");
            object.remove("source_frame_sequence");
        }
    }
    commands.sort_by_key(|c| c["command_timestamp_ns"].as_u64());
    let pending_requests: Vec<_> = shooting::queued_requests(w)
        .iter()
        .map(super::closure::request_data)
        .collect();
    let g = w.resource::<GimbalActuatorTelemetry>();
    let critic = json!({"version":1,"sample_ns":now.as_nanos() as u64,"robots":robots,
        "geometry":geometry,"projectiles":projectiles,"pending_commands":commands,
        "pending_requests":pending_requests,
        "actuator":{"yaw_rad":g.actual_yaw_rad,"pitch_rad":g.actual_pitch_rad,
            "yaw_velocity_rad_s":g.yaw_velocity_rad_s,"pitch_velocity_rad_s":g.pitch_velocity_rad_s,
            "target_yaw_rad":g.target_yaw_rad,"target_pitch_rad":g.target_pitch_rad,
            "yaw_acceleration_rad_s2":g.yaw_acceleration_rad_s2,
            "pitch_acceleration_rad_s2":g.pitch_acceleration_rad_s2,
            "command_valid":g.command_valid,"saturation_flags":g.saturation_flags}});
    Ok((actor, critic))
}
