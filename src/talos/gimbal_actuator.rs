use crate::components::{
    Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, SubscribeAutoAim,
};
use crate::config::{GimbalActuatorConfig, GimbalActuatorMode, SimulationConfig};
use crate::systems::projectile_launch;
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
use std::collections::VecDeque;
use std::f64::consts::{FRAC_PI_2, PI};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use talos_ipc::{GimbalCmd, ShmSubscriber};

pub const GIMBAL_MODE_LEGACY: u8 = 0;
pub const GIMBAL_MODE_PHYSICAL: u8 = 1;
pub const GIMBAL_MODE_IDEAL: u8 = 2;

pub const SATURATION_YAW_VELOCITY: u8 = 1 << 0;
pub const SATURATION_PITCH_VELOCITY: u8 = 1 << 1;
pub const SATURATION_YAW_ACCELERATION: u8 = 1 << 2;
pub const SATURATION_PITCH_ACCELERATION: u8 = 1 << 3;
pub const SATURATION_PITCH_LIMIT: u8 = 1 << 4;
pub const SATURATION_COMMAND_TIMEOUT: u8 = 1 << 5;
pub const SATURATION_INTEGRATION_OVERRUN: u8 = 1 << 6;

const COMMAND_QUEUE_CAPACITY: usize = 256;
const MAX_CATCH_UP: Duration = Duration::from_millis(100);

#[derive(Clone, Copy)]
struct ReceivedCommand {
    command: GimbalCmd,
    received_at: Instant,
    received_system_ns: u64,
}

#[derive(Resource, Clone)]
pub struct GimbalCommandInbox {
    queue: Arc<Mutex<VecDeque<ReceivedCommand>>>,
    dropped: Arc<AtomicU64>,
    poll_hz_bits: Arc<AtomicU64>,
}

impl GimbalCommandInbox {
    fn drain(&self) -> Vec<ReceivedCommand> {
        let Ok(mut queue) = self.queue.lock() else {
            return Vec::new();
        };
        queue.drain(..).collect()
    }

    fn clear(&self) {
        if let Ok(mut queue) = self.queue.lock() {
            queue.clear();
        }
    }

    fn set_poll_hz(&self, value: f64) {
        self.poll_hz_bits
            .store(value.clamp(100.0, 5000.0).to_bits(), Ordering::Release);
    }

    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Acquire)
    }
}

#[derive(Resource)]
pub struct GimbalCommandReceiverWorker {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Drop for GimbalCommandReceiverWorker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

pub fn spawn_command_receiver(
    mut subscriber: ShmSubscriber,
    initial_poll_hz: f64,
) -> (GimbalCommandInbox, GimbalCommandReceiverWorker) {
    let queue = Arc::new(Mutex::new(VecDeque::with_capacity(COMMAND_QUEUE_CAPACITY)));
    let dropped = Arc::new(AtomicU64::new(0));
    let poll_hz_bits = Arc::new(AtomicU64::new(
        initial_poll_hz.clamp(100.0, 5000.0).to_bits(),
    ));
    let stop = Arc::new(AtomicBool::new(false));

    let thread_queue = queue.clone();
    let thread_dropped = dropped.clone();
    let thread_poll_hz = poll_hz_bits.clone();
    let thread_stop = stop.clone();
    let thread = std::thread::Builder::new()
        .name("talos-gimbal-rx".to_string())
        .spawn(move || {
            let mut next = Instant::now();
            while !thread_stop.load(Ordering::Acquire) {
                if let Some(command) = subscriber.recv_gimbal_cmd() {
                    let event = ReceivedCommand {
                        command,
                        received_at: Instant::now(),
                        received_system_ns: system_now_ns(),
                    };
                    if let Ok(mut queue) = thread_queue.lock() {
                        if queue.len() == COMMAND_QUEUE_CAPACITY {
                            queue.pop_front();
                            thread_dropped.fetch_add(1, Ordering::Relaxed);
                        }
                        queue.push_back(event);
                    }
                }

                let hz =
                    f64::from_bits(thread_poll_hz.load(Ordering::Acquire)).clamp(100.0, 5000.0);
                let period = Duration::from_secs_f64(1.0 / hz);
                next += period;
                let now = Instant::now();
                if next <= now {
                    next = now + period;
                }
                std::thread::sleep(next.saturating_duration_since(now));
            }
        })
        .expect("failed to spawn Talos gimbal command receiver");

    (
        GimbalCommandInbox {
            queue,
            dropped,
            poll_hz_bits,
        },
        GimbalCommandReceiverWorker {
            stop,
            thread: Some(thread),
        },
    )
}

#[derive(Clone, Copy, Default)]
struct AxisState {
    angle: f64,
    velocity: f64,
    acceleration: f64,
}

#[derive(Clone, Copy)]
struct DelayedCommand {
    command: GimbalCmd,
    activate_at: Instant,
    activate_system_ns: u64,
}

#[derive(Clone, Copy)]
struct ActiveCommand {
    timestamp_ns: u64,
    activated_at: Instant,
    activated_system_ns: u64,
    target_yaw: f64,
    target_pitch: f64,
    valid: bool,
}

#[derive(Resource)]
pub struct GimbalActuator {
    initialized: bool,
    was_enabled: bool,
    mode: GimbalActuatorMode,
    yaw: AxisState,
    pitch: AxisState,
    last_update: Instant,
    active: Option<ActiveCommand>,
    pending: VecDeque<DelayedCommand>,
    fire_level: bool,
}

impl Default for GimbalActuator {
    fn default() -> Self {
        Self {
            initialized: false,
            was_enabled: false,
            mode: GimbalActuatorMode::Physical,
            yaw: AxisState::default(),
            pitch: AxisState::default(),
            last_update: Instant::now(),
            active: None,
            pending: VecDeque::new(),
            fire_level: false,
        }
    }
}

#[derive(Resource, Clone, Copy, Default)]
pub struct GimbalActuatorTelemetry {
    pub valid: bool,
    pub state_timestamp_ns: u64,
    pub consumed_command_timestamp_ns: u64,
    pub consumed_at_timestamp_ns: u64,
    pub target_yaw_rad: f32,
    pub target_pitch_rad: f32,
    pub actual_yaw_rad: f32,
    pub actual_pitch_rad: f32,
    pub yaw_velocity_rad_s: f32,
    pub pitch_velocity_rad_s: f32,
    pub yaw_acceleration_rad_s2: f32,
    pub pitch_acceleration_rad_s2: f32,
    pub mode: u8,
    pub saturation_flags: u8,
    pub command_valid: bool,
    pub dropped_commands: u64,
}

fn system_now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|value| value.as_nanos() as u64)
        .unwrap_or(0)
}

fn unwrap_near(value: f64, reference: f64) -> f64 {
    reference + (value - reference + PI).rem_euclid(2.0 * PI) - PI
}

fn pose_angles(rotation: Quat, yaw_reference: f64) -> (f64, f64) {
    // SHOT_DIRECTION's local +Y axis is the muzzle forward axis. Extract yaw and pitch from
    // that direction instead of decomposing the complete quaternion: the old YXZ decomposition
    // is singular when the muzzle is horizontal because its internal X rotation is near -pi/2.
    let bevy_forward = rotation * Vec3::Y;
    let ros_forward =
        Vec3::new(-bevy_forward.z, -bevy_forward.x, bevy_forward.y).normalize_or_zero();
    if ros_forward == Vec3::ZERO {
        return (yaw_reference, 0.0);
    }
    let yaw = (ros_forward.y as f64).atan2(ros_forward.x as f64);
    let pitch = (ros_forward.z as f64).atan2((ros_forward.x as f64).hypot(ros_forward.y as f64));
    (unwrap_near(yaw, yaw_reference), pitch)
}

fn muzzle_world_rotation(yaw: f64, pitch: f64) -> Quat {
    Quat::from_euler(
        EulerRot::YXZ,
        yaw.rem_euclid(2.0 * PI) as f32,
        (pitch - FRAC_PI_2) as f32,
        0.0,
    )
}

fn solve_gimbal_local_rotation(
    parent_world_rotation: Quat,
    gimbal_to_muzzle_rotation: Quat,
    desired_muzzle_world_rotation: Quat,
) -> Quat {
    (parent_world_rotation.inverse()
        * desired_muzzle_world_rotation
        * gimbal_to_muzzle_rotation.inverse())
    .normalize()
}

fn command_target(command: GimbalCmd, pitch_limit: f64) -> ActiveCommand {
    let valid =
        command.distance_m >= 0.0 && command.yaw_deg.is_finite() && command.pitch_deg.is_finite();
    ActiveCommand {
        timestamp_ns: command.timestamp_ns,
        activated_at: Instant::now(),
        activated_system_ns: 0,
        target_yaw: (command.yaw_deg as f64).to_radians(),
        target_pitch: (-(command.pitch_deg as f64))
            .to_radians()
            .clamp(-pitch_limit, pitch_limit),
        valid,
    }
}

fn integrate_axis(
    axis: &mut AxisState,
    target: f64,
    dt: f64,
    natural_frequency: f64,
    damping_ratio: f64,
    max_velocity: f64,
    max_acceleration: f64,
    velocity_flag: u8,
    acceleration_flag: u8,
    flags: &mut u8,
) {
    let raw_acceleration = natural_frequency * natural_frequency * (target - axis.angle)
        - 2.0 * damping_ratio * natural_frequency * axis.velocity;
    let acceleration = raw_acceleration.clamp(-max_acceleration, max_acceleration);
    if raw_acceleration.abs() > max_acceleration {
        *flags |= acceleration_flag;
    }
    let raw_velocity = axis.velocity + acceleration * dt;
    let velocity = raw_velocity.clamp(-max_velocity, max_velocity);
    if raw_velocity.abs() > max_velocity {
        *flags |= velocity_flag;
    }
    axis.angle += 0.5 * (axis.velocity + velocity) * dt;
    axis.velocity = velocity;
    axis.acceleration = acceleration;
}

impl GimbalActuator {
    fn reset_from_pose(&mut self, yaw: f64, pitch: f64, now: Instant) {
        self.initialized = true;
        self.yaw = AxisState {
            angle: yaw,
            ..default()
        };
        self.pitch = AxisState {
            angle: pitch,
            ..default()
        };
        self.active = None;
        self.pending.clear();
        self.fire_level = false;
        self.last_update = now;
    }

    fn advance_interval(
        &mut self,
        start: Instant,
        end: Instant,
        config: &GimbalActuatorConfig,
        pitch_limit: f64,
        flags: &mut u8,
    ) {
        if end <= start {
            return;
        }
        let integration_hz = config.integration_hz.clamp(100.0, 5000.0);
        let max_dt = 1.0 / integration_hz;
        let total = end.duration_since(start).as_secs_f64();
        let steps = (total / max_dt).ceil().max(1.0) as usize;
        let dt = total / steps as f64;

        for step in 0..steps {
            let step_time = start + Duration::from_secs_f64(dt * (step + 1) as f64);
            let active_valid = self.active.is_some_and(|active| {
                active.valid
                    && step_time.duration_since(active.activated_at).as_secs_f64()
                        <= config.command_timeout_s.max(0.001)
            });
            if self.active.is_some_and(|active| active.valid) && !active_valid {
                *flags |= SATURATION_COMMAND_TIMEOUT;
            }

            let target_yaw = if active_valid {
                unwrap_near(self.active.unwrap().target_yaw, self.yaw.angle)
            } else {
                self.yaw.angle
            };
            let target_pitch = if active_valid {
                self.active.unwrap().target_pitch
            } else {
                self.pitch.angle
            };

            if self.mode == GimbalActuatorMode::Ideal {
                if active_valid {
                    self.yaw.angle = target_yaw;
                    self.pitch.angle = target_pitch;
                }
                self.yaw.velocity = 0.0;
                self.yaw.acceleration = 0.0;
                self.pitch.velocity = 0.0;
                self.pitch.acceleration = 0.0;
                continue;
            }

            integrate_axis(
                &mut self.yaw,
                target_yaw,
                dt,
                config.natural_frequency_rad_s.clamp(1.0, 500.0),
                config.damping_ratio.clamp(0.1, 5.0),
                config.yaw_max_velocity_rad_s.max(0.01),
                config.yaw_max_acceleration_rad_s2.max(0.01),
                SATURATION_YAW_VELOCITY,
                SATURATION_YAW_ACCELERATION,
                flags,
            );
            integrate_axis(
                &mut self.pitch,
                target_pitch,
                dt,
                config.natural_frequency_rad_s.clamp(1.0, 500.0),
                config.damping_ratio.clamp(0.1, 5.0),
                config.pitch_max_velocity_rad_s.max(0.01),
                config.pitch_max_acceleration_rad_s2.max(0.01),
                SATURATION_PITCH_VELOCITY,
                SATURATION_PITCH_ACCELERATION,
                flags,
            );
            let clamped_pitch = self.pitch.angle.clamp(-pitch_limit, pitch_limit);
            if clamped_pitch != self.pitch.angle {
                self.pitch.angle = clamped_pitch;
                if self.pitch.velocity.signum() == clamped_pitch.signum() {
                    self.pitch.velocity = 0.0;
                    self.pitch.acceleration = 0.0;
                }
                *flags |= SATURATION_PITCH_LIMIT;
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn update_gimbal_actuator(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    following: Res<SubscribeAutoAim>,
    inbox: Res<GimbalCommandInbox>,
    mut actuator: ResMut<GimbalActuator>,
    mut telemetry: ResMut<GimbalActuatorTelemetry>,
    gimbal_entity: Single<
        Entity,
        (
            With<Controlled>,
            With<InfantryGimbal>,
            Without<InfantryChassis>,
            Without<InfantryLaunchOffset>,
        ),
    >,
    muzzle_entity: Single<Entity, (With<InfantryLaunchOffset>, With<Controlled>)>,
    mut pose: ParamSet<(
        TransformHelper,
        Query<
            (&mut Transform, &mut InfantryGimbal),
            (
                With<Controlled>,
                Without<InfantryChassis>,
                Without<InfantryLaunchOffset>,
            ),
        >,
    )>,
) {
    let now = Instant::now();
    let enabled = following.load(Ordering::Acquire);
    let actuator_config = &config.gimbal_actuator;
    let pitch_limit = config.vehicle.gimbal_pitch_limit.max(0.01) as f64;
    inbox.set_poll_hz(actuator_config.command_poll_hz);

    let gimbal_entity = *gimbal_entity;
    let muzzle_entity = *muzzle_entity;
    let (current_gimbal_rotation, current_muzzle_rotation) = {
        let helper = pose.p0();
        let Ok(gimbal_global) = helper.compute_global_transform(gimbal_entity) else {
            return;
        };
        let Ok(muzzle_global) = helper.compute_global_transform(muzzle_entity) else {
            return;
        };
        (gimbal_global.rotation(), muzzle_global.rotation())
    };
    let yaw_reference = if actuator.initialized {
        actuator.yaw.angle
    } else {
        0.0
    };
    let (actual_yaw, actual_pitch) = pose_angles(current_muzzle_rotation, yaw_reference);

    if !enabled {
        inbox.clear();
        if actuator.was_enabled || !actuator.initialized {
            actuator.reset_from_pose(actual_yaw, actual_pitch, now);
        } else {
            actuator.yaw.angle = actual_yaw;
            actuator.pitch.angle = actual_pitch;
            actuator.last_update = now;
        }
        actuator.was_enabled = false;
        *telemetry = GimbalActuatorTelemetry {
            valid: true,
            state_timestamp_ns: system_now_ns(),
            actual_yaw_rad: actual_yaw as f32,
            actual_pitch_rad: actual_pitch as f32,
            mode: match actuator_config.mode {
                GimbalActuatorMode::Physical => GIMBAL_MODE_PHYSICAL,
                GimbalActuatorMode::Ideal => GIMBAL_MODE_IDEAL,
            },
            dropped_commands: inbox.dropped(),
            ..default()
        };
        return;
    }

    if !actuator.was_enabled || !actuator.initialized {
        actuator.reset_from_pose(actual_yaw, actual_pitch, now);
        actuator.was_enabled = true;
    }

    if actuator.mode != actuator_config.mode {
        actuator.mode = actuator_config.mode;
        actuator.yaw.velocity = 0.0;
        actuator.yaw.acceleration = 0.0;
        actuator.pitch.velocity = 0.0;
        actuator.pitch.acceleration = 0.0;
        if actuator.mode == GimbalActuatorMode::Ideal
            && let Some(active) = actuator.active
            && active.valid
        {
            actuator.yaw.angle = unwrap_near(active.target_yaw, actuator.yaw.angle);
            actuator.pitch.angle = active.target_pitch;
        }
    }

    let delay = Duration::from_secs_f64(actuator_config.command_delay_s.clamp(0.0, 1.0));
    for event in inbox.drain() {
        actuator.pending.push_back(DelayedCommand {
            command: event.command,
            activate_at: event.received_at + delay,
            activate_system_ns: event
                .received_system_ns
                .saturating_add(delay.as_nanos() as u64),
        });
    }
    actuator
        .pending
        .make_contiguous()
        .sort_by_key(|event| event.activate_at);

    let mut flags = 0_u8;
    let mut cursor = actuator.last_update;
    if now.saturating_duration_since(cursor) > MAX_CATCH_UP {
        cursor = now - MAX_CATCH_UP;
        flags |= SATURATION_INTEGRATION_OVERRUN;
    }
    let mut fire_rising_edges = 0_u32;
    while actuator
        .pending
        .front()
        .is_some_and(|event| event.activate_at <= now)
    {
        let event = actuator.pending.pop_front().unwrap();
        let activate_at = event.activate_at.max(cursor);
        actuator.advance_interval(
            cursor,
            activate_at,
            actuator_config,
            pitch_limit,
            &mut flags,
        );
        cursor = activate_at;

        let mut active = command_target(event.command, pitch_limit);
        active.activated_at = activate_at;
        active.activated_system_ns = event.activate_system_ns;
        let fire = active.valid && event.command.fire_advice == 1;
        if fire && !actuator.fire_level {
            fire_rising_edges += 1;
        }
        actuator.fire_level = fire;
        actuator.active = Some(active);
    }
    actuator.advance_interval(cursor, now, actuator_config, pitch_limit, &mut flags);
    actuator.last_update = now;

    for _ in 0..fire_rising_edges {
        commands.queue(|world: &mut World| {
            world.run_system_once(projectile_launch).unwrap();
        });
    }

    // The actuator state is absolute in the world frame. Convert the desired muzzle rotation to
    // the gimbal's local frame explicitly; a world-space delta cannot be left-multiplied onto a
    // local transform when the chassis or the fixed muzzle offset is rotated.
    let mut gimbal_query = pose.p1();
    let Ok((mut gimbal_transform, mut gimbal_data)) = gimbal_query.get_mut(gimbal_entity) else {
        return;
    };
    let parent_world_rotation = current_gimbal_rotation * gimbal_transform.rotation.inverse();
    let gimbal_to_muzzle_rotation = current_gimbal_rotation.inverse() * current_muzzle_rotation;
    let desired_muzzle_rotation = muzzle_world_rotation(actuator.yaw.angle, actuator.pitch.angle);
    gimbal_transform.rotation = solve_gimbal_local_rotation(
        parent_world_rotation,
        gimbal_to_muzzle_rotation,
        desired_muzzle_rotation,
    );
    let (local_yaw, local_pitch, _) = gimbal_transform.rotation.to_euler(EulerRot::YXZ);
    gimbal_data.local_yaw = local_yaw;
    gimbal_data.pitch = local_pitch;

    let active = actuator.active;
    let command_valid = active.is_some_and(|value| {
        value.valid
            && now.duration_since(value.activated_at).as_secs_f64()
                <= actuator_config.command_timeout_s.max(0.001)
    });
    let target_yaw = active
        .filter(|_| command_valid)
        .map_or(actuator.yaw.angle, |value| {
            unwrap_near(value.target_yaw, actuator.yaw.angle)
        });
    let target_pitch = active
        .filter(|_| command_valid)
        .map_or(actuator.pitch.angle, |value| value.target_pitch);
    *telemetry = GimbalActuatorTelemetry {
        valid: true,
        state_timestamp_ns: system_now_ns(),
        consumed_command_timestamp_ns: active.map_or(0, |value| value.timestamp_ns),
        consumed_at_timestamp_ns: active.map_or(0, |value| value.activated_system_ns),
        target_yaw_rad: target_yaw as f32,
        target_pitch_rad: target_pitch as f32,
        actual_yaw_rad: actuator.yaw.angle as f32,
        actual_pitch_rad: actuator.pitch.angle as f32,
        yaw_velocity_rad_s: actuator.yaw.velocity as f32,
        pitch_velocity_rad_s: actuator.pitch.velocity as f32,
        yaw_acceleration_rad_s2: actuator.yaw.acceleration as f32,
        pitch_acceleration_rad_s2: actuator.pitch.acceleration as f32,
        mode: match actuator.mode {
            GimbalActuatorMode::Physical => GIMBAL_MODE_PHYSICAL,
            GimbalActuatorMode::Ideal => GIMBAL_MODE_IDEAL,
        },
        saturation_flags: flags,
        command_valid,
        dropped_commands: inbox.dropped(),
    };
}
