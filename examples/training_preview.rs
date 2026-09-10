//! Optional initial-state viewer. A real headless Reset supplies every displayed robot pose.
//! The display world stays frozen; its rendering and keyboard input never step training physics.
use avian3d::prelude::*;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::ecs::query::QueryData;
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::time::TimeUpdateStrategy;
use bevy::window::PresentMode;
use clap::Parser;
use crossbeam_channel::{Receiver, Sender, unbounded};
use daedalus::components::{
    Controlled, Infantry, InfantryGimbal, InfantryLaunchOffset, InfantryViewOffset,
    SubscribeAutoAim,
};
use daedalus::config::SimulationConfig;
use daedalus::gimbal_actuator::*;
use daedalus::robomaster::combat::{CombatPlugin, RobotIdentity, RobotMember, damage};
use daedalus::robomaster::prelude::ArmorPlugins;
use daedalus::robomaster::prelude::ArmorRoot;
use daedalus::setup::{
    HeadlessScene, setup, setup_collision, setup_training_outpost, setup_training_power_rune,
    setup_vehicle,
};
use daedalus::statistic::ProjectileStatistics;
use daedalus::training::environment::PhysicalEnvironment;
use daedalus::training::protocol::Environment;
use serde_json::{Value, json};
use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t = 17)]
    seed: u64,
    #[arg(long, default_value = "config.toml")]
    config: PathBuf,
    #[arg(long, default_value = "assets")]
    assets: PathBuf,
    /// Optional Reset scenario JSON. Omitted fields are sampled by the training backend.
    #[arg(long)]
    scenario: Option<PathBuf>,
    /// Save the first loaded view once; no continuous recording.
    #[arg(long)]
    screenshot: Option<PathBuf>,
    /// Start with the same camera optics used by initial visibility validation.
    #[arg(long)]
    first_person: bool,
}

#[derive(Resource)]
struct Preview {
    requests: Sender<u64>,
    replies: Receiver<(u64, Result<Value, String>)>,
    seed: u64,
    busy: bool,
    pending: Option<Value>,
    current: Option<Value>,
    status: String,
    orbit: f32,
    elevation: f32,
    distance: f32,
    first_person: bool,
    screenshot: Option<PathBuf>,
    loaded_frames: usize,
}

#[derive(Component)]
struct PreviewCamera;
#[derive(Component)]
struct PreviewText;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let config: SimulationConfig = toml::from_str(&std::fs::read_to_string(args.config)?)?;
    config.projectile.validate_shooter()?;
    let assets = args.assets.canonicalize()?;
    let scenario = args.scenario.map_or(Ok(json!({})), |path| {
        serde_json::from_slice::<Value>(&std::fs::read(path)?).map_err(std::io::Error::other)
    })?;
    let (requests, work) = unbounded();
    let (results, replies) = unbounded();
    let worker_config = config.clone();
    let worker_assets = assets.clone();
    std::thread::spawn(move || {
        let mut environment = PhysicalEnvironment::new(worker_config, worker_assets);
        for (index, seed) in work.iter().enumerate() {
            let result = environment.reset(index as u64 + 1, seed, &scenario);
            if results.send((seed, result)).is_err() {
                break;
            }
        }
    });
    requests.send(args.seed)?;

    let resolution = (config.capture.color.width, config.capture.color.height);
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(AssetPlugin {
                    file_path: assets.to_string_lossy().into_owned(),
                    ..default()
                })
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Training initial-state preview".into(),
                        resolution: resolution.into(),
                        resizable: false,
                        present_mode: PresentMode::AutoVsync,
                        ..default()
                    }),
                    ..default()
                })
                .disable::<bevy::audio::AudioPlugin>()
                .disable::<bevy::gilrs::GilrsPlugin>(),
        )
        .add_plugins((
            PhysicsPlugins::default().with_collision_hooks::<damage::ProjectileContactHooks>(),
            ArmorPlugins,
            CombatPlugin,
        ))
        .insert_resource(config)
        .insert_resource(GlobalAmbientLight {
            brightness: 800.0,
            ..default()
        })
        .insert_resource(HeadlessScene)
        // Rendering uses host frames; zero simulation delta prevents any display-world motion.
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::ZERO))
        .insert_resource(SubscribeAutoAim(AtomicBool::new(true)))
        .insert_resource(GimbalCommandInbox::standalone())
        .init_resource::<ActuatorClock>()
        .init_resource::<GimbalActuator>()
        .init_resource::<GimbalActuatorTelemetry>()
        .init_resource::<ProjectileStatistics>()
        .insert_resource(Preview {
            requests,
            replies,
            seed: args.seed,
            busy: true,
            pending: None,
            current: None,
            status: "Loading real training Reset...".into(),
            orbit: 0.65,
            elevation: 0.65,
            distance: 10.0,
            first_person: args.first_person,
            screenshot: args.screenshot,
            loaded_frames: 0,
        })
        .add_systems(Startup, (setup, setup_view))
        .add_observer(setup_vehicle)
        .add_observer(setup_collision)
        .add_observer(setup_training_power_rune)
        .add_observer(setup_training_outpost)
        .add_systems(
            Update,
            (
                controls,
                receive_snapshot,
                apply_snapshot,
                show_valid_robots,
            )
                .chain(),
        )
        .add_systems(
            PostUpdate,
            update_gimbal_actuator.before(TransformSystems::Propagate),
        )
        .add_systems(
            PostUpdate,
            (view_camera, describe_scene, mark_robots, capture_once)
                .chain()
                .after(TransformSystems::Propagate),
        )
        .run();
    Ok(())
}

fn setup_view(mut commands: Commands, config: Res<SimulationConfig>) {
    commands.spawn((
        DirectionalLight {
            illuminance: config.render.illuminance.max(12000.0),
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_xyz(4.0, 10.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Camera3d::default(),
        PreviewCamera,
        Tonemapping::None,
        Msaa::Off,
        Projection::Perspective(PerspectiveProjection {
            fov: config.camera.fov.to_radians(),
            ..default()
        }),
        Transform::from_xyz(8.0, 9.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        PreviewText,
        Text::new("Loading..."),
        TextFont {
            font_size: FontSize::Px(19.0),
            ..default()
        },
        BackgroundColor(Color::srgba(0.02, 0.03, 0.05, 0.88)),
        Node {
            position_type: PositionType::Absolute,
            top: px(12),
            left: px(12),
            padding: UiRect::all(px(12)),
            max_width: percent(96),
            ..default()
        },
    ));
}

fn controls(keys: Res<ButtonInput<KeyCode>>, time: Res<Time<Real>>, mut preview: ResMut<Preview>) {
    let dt = time.delta_secs().min(0.1);
    for (key, delta) in [(KeyCode::ArrowLeft, -1.0), (KeyCode::ArrowRight, 1.0)] {
        if keys.pressed(key) {
            preview.orbit += delta * dt;
        }
    }
    for (key, delta) in [(KeyCode::ArrowUp, 1.0), (KeyCode::ArrowDown, -1.0)] {
        if keys.pressed(key) {
            preview.elevation = (preview.elevation + delta * dt).clamp(0.1, 1.5);
        }
    }
    for (key, delta) in [(KeyCode::PageUp, -1.0), (KeyCode::PageDown, 1.0)] {
        if keys.pressed(key) {
            preview.distance = (preview.distance + delta * dt * 5.0).clamp(2.0, 30.0);
        }
    }
    if keys.just_pressed(KeyCode::KeyC) {
        preview.first_person = !preview.first_person;
    }
    if preview.busy {
        return;
    }
    let seed = if keys.just_pressed(KeyCode::KeyN) {
        preview.seed.checked_add(1)
    } else if keys.just_pressed(KeyCode::KeyB) {
        preview.seed.checked_sub(1)
    } else if keys.just_pressed(KeyCode::KeyR) {
        Some(preview.seed)
    } else {
        None
    };
    if let Some(seed) = seed {
        match preview.requests.send(seed) {
            Ok(()) => {
                preview.seed = seed;
                preview.busy = true;
                preview.status = "Generating; previous view remains until ready...".into();
            }
            Err(error) => preview.status = error.to_string(),
        }
    }
}

fn receive_snapshot(mut preview: ResMut<Preview>) {
    if let Ok((seed, result)) = preview.replies.try_recv() {
        preview.seed = seed;
        match result {
            Ok(data) => preview.pending = Some(data),
            Err(error) => {
                preview.busy = false;
                preview.pending = None;
                preview.current = None;
                preview.status = format!("INVALID SEED: {error}");
                eprintln!("PREVIEW invalid seed={seed}: {error}");
            }
        }
    }
}

fn apply_snapshot(
    mut preview: ResMut<Preview>,
    armors: Query<&ArmorRoot>,
    mut robots: Query<(&RobotIdentity, &mut Transform, &mut Position, &mut Rotation)>,
    clock: Res<ActuatorClock>,
    mut actuator: ResMut<GimbalActuator>,
) {
    if armors.iter().count() != 14 || robots.iter().count() != 2 {
        return;
    }
    let Some(data) = preview.pending.take() else {
        return;
    };
    for (id, mut transform, mut position, mut rotation) in &mut robots {
        let state = data["evaluation"]["robots"]
            .as_array()
            .unwrap()
            .iter()
            .find(|r| r["robot_id"].as_u64() == Some(id.id.0))
            .unwrap();
        transform.translation = vector(&state["position_bevy_m"]);
        let q: [f32; 4] = serde_json::from_value(state["rotation_xyzw"].clone()).unwrap();
        transform.rotation = Quat::from_array(q);
        position.0 = transform.translation;
        rotation.0 = transform.rotation;
    }
    let initial = &data["evaluation"]["scenario"]["initial_gimbal"];
    actuator.initialize(
        initial["yaw_rad"].as_f64().unwrap(),
        initial["pitch_rad"].as_f64().unwrap(),
        &clock,
    );
    let report = &data["evaluation"]["scenario"];
    let repeated = preview.current.as_ref().is_some_and(|old| {
        old["evaluation"]["scenario"] == *report
            && old["evaluation"]["robots"] == data["evaluation"]["robots"]
    });
    println!(
        "PREVIEW seed={} same_initial_state={} scenario={report}",
        preview.seed, repeated
    );
    preview.status = if repeated {
        "Same initial state reproduced."
    } else {
        "Ready: actual training Reset, frozen at t=0."
    }
    .into();
    preview.current = Some(data);
    preview.busy = false;
    preview.loaded_frames = 0;
}

/// Never leave a prior valid pair visible under the label of an invalid seed.
#[derive(Component)]
struct HiddenRobotVisibility(Visibility);

type PreviewRobotFilter = Or<(With<RobotIdentity>, With<RobotMember>)>;

fn show_valid_robots(
    mut commands: Commands,
    preview: Res<Preview>,
    mut robots: Query<
        (Entity, &mut Visibility, Option<&HiddenRobotVisibility>),
        PreviewRobotFilter,
    >,
) {
    for (entity, mut visibility, saved) in &mut robots {
        if preview.current.is_none() {
            if saved.is_none() {
                // Armor construction can replace imported nodes in this same frame.
                commands
                    .entity(entity)
                    .try_insert(HiddenRobotVisibility(*visibility));
            }
            *visibility = Visibility::Hidden;
        } else if let Some(saved) = saved {
            *visibility = saved.0;
            commands
                .entity(entity)
                .try_remove::<HiddenRobotVisibility>();
        }
    }
}

fn vector(value: &Value) -> Vec3 {
    Vec3::from_array(serde_json::from_value(value.clone()).unwrap())
}

#[derive(QueryData)]
struct CameraPoseNode {
    transform: &'static Transform,
    root: Has<Infantry>,
    gimbal: Has<InfantryGimbal>,
    view: Has<InfantryViewOffset>,
    muzzle: Has<InfantryLaunchOffset>,
}

fn view_camera(
    preview: Res<Preview>,
    mut camera: Single<&mut Transform, With<PreviewCamera>>,
    poses: Query<CameraPoseNode, (With<Controlled>, Without<PreviewCamera>)>,
) {
    let Some(data) = &preview.current else {
        return;
    };
    if preview.first_person {
        let (mut root, mut gimbal, mut view, mut muzzle) = (None, None, None, None);
        for node in &poses {
            if node.root {
                root = Some(node.transform);
            }
            if node.gimbal {
                gimbal = Some(node.transform);
            }
            if node.view {
                view = Some(node.transform);
            }
            if node.muzzle {
                muzzle = Some(node.transform);
            }
        }
        if let (Some(root), Some(gimbal), Some(view), Some(muzzle)) = (root, gimbal, view, muzzle) {
            **camera = daedalus::systems::robot_camera_pose(root, gimbal, view, muzzle);
        }
    } else {
        let scene = &data["evaluation"]["scenario"];
        let center = (vector(&scene["controlled_position_bevy_m"])
            + vector(&scene["target_position_bevy_m"]))
            * 0.5;
        let (s, c) = preview.orbit.sin_cos();
        let (h, radius) = preview.elevation.sin_cos();
        camera.translation = center + Vec3::new(s * radius, h, c * radius) * preview.distance;
        camera.look_at(center, Vec3::Y);
    }
}

fn describe_scene(preview: Res<Preview>, mut text: Single<&mut Text, With<PreviewText>>) {
    let mut detail = String::new();
    if let Some(data) = &preview.current {
        let s = &data["evaluation"]["scenario"];
        let own = vector(&s["controlled_position_bevy_m"]);
        let target = vector(&s["target_position_bevy_m"]);
        detail = format!(
            "\nTarget: {:.2} m | bearing {:.1} deg | attempts {}\nSelf XYZ: ({:.3}, {:.3}, {:.3}) m\nTarget XYZ: ({:.3}, {:.3}, {:.3}) m\nInitial gimbal: yaw {:.1} deg / pitch {:.1} deg",
            s["target_distance_m"].as_f64().unwrap(),
            s["target_bearing_rad"].as_f64().unwrap().to_degrees(),
            s["attempts"],
            own.x,
            own.y,
            own.z,
            target.x,
            target.y,
            target.z,
            s["initial_gimbal"]["yaw_rad"]
                .as_f64()
                .unwrap()
                .to_degrees(),
            s["initial_gimbal"]["pitch_rad"]
                .as_f64()
                .unwrap()
                .to_degrees()
        );
        detail.push_str(&format!(
            "\nVisible blue plates: {} | Red sampling attempts: {}",
            s["initial_visibility"]["visible_armor_count"], s["controlled_attempts"]
        ));
    }
    text.0 = format!(
        "TRAINING INITIAL STATE | seed {}\n{}{}\nN next seed | B previous | R replay | C camera\nArrows orbit | PgUp/PgDn zoom | Close window to exit\nRed + Blue: seeded positions | No shooting",
        preview.seed, preview.status, detail
    );
}

/// Observer-only markers make the two physical spawn points visible from a distance.
fn mark_robots(preview: Res<Preview>, mut gizmos: Gizmos) {
    if preview.first_person {
        return;
    }
    let Some(data) = &preview.current else {
        return;
    };
    let scene = &data["evaluation"]["scenario"];
    for (field, color) in [
        ("controlled_position_bevy_m", Color::srgb(1.0, 0.15, 0.1)),
        ("target_position_bevy_m", Color::srgb(0.0, 0.65, 1.0)),
    ] {
        let p = vector(&scene[field]);
        gizmos.line(p + Vec3::Y * 0.4, p + Vec3::Y * 1.1, color);
        gizmos.line(p + Vec3::Y * 1.1, p + Vec3::new(0.25, 0.95, 0.0), color);
    }
}

fn capture_once(mut commands: Commands, mut preview: ResMut<Preview>) {
    if preview.current.is_none() {
        return;
    }
    preview.loaded_frames += 1;
    if preview.loaded_frames == 90
        && let Some(path) = preview.screenshot.take()
    {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    }
}
