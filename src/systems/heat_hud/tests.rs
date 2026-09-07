use super::*;
use crate::components::SubscribeAutoAim;
use crate::robomaster::combat::{CombatRobotBundle, RobotId, TrainingPreset};
use crate::robomaster::prelude::Team;
use crate::statistic::ProjectileStatistics;
use crate::systems::{ControllerState, update_help_text};
use bevy::ecs::system::RunSystemOnce;

/// Exercise the real ECS display systems: ownership, camera isolation, independent text
/// updates, and a zero-heat round latch that ordinary firing cannot reach naturally.
#[test]
fn window_hud_preserves_round_latch_and_does_not_overwrite_help() {
    let mut world = World::new();
    let mut config: SimulationConfig =
        toml::from_str(include_str!("../../../config.toml")).unwrap();
    config.preview.enabled = true;
    config.preview.heat_hud = true;
    world.insert_resource(config);
    world.insert_resource(ProjectileStatistics::default());
    world.insert_resource(ControllerState::default());
    world.insert_resource(SubscribeAutoAim(std::sync::atomic::AtomicBool::new(false)));
    let window = world
        .spawn((PreviewCamera, Camera::default(), RenderTarget::default()))
        .id();
    world.spawn((
        PreviewCamera,
        Camera {
            order: 100,
            ..default()
        },
        RenderTarget::Image(Handle::<Image>::default().into()),
    ));
    let help = world
        .spawn((HelpText, Text::new(""), Node::default(), Visibility::Hidden))
        .id();
    let robot = world
        .spawn((
            Controlled,
            CombatRobotBundle::training(
                RobotId(1),
                Team::Red,
                TrainingPreset::InfantryHealthCooling,
            ),
        ))
        .id();
    // Imported descendants also carry Controlled, but do not own combat state.
    world.spawn((Controlled, ChildOf(robot)));
    world.run_system_once(setup_window_hud).unwrap();
    let mut roots = world.query_filtered::<&UiTargetCamera, With<HeatHud>>();
    assert_eq!(roots.single(&world).unwrap().entity(), window);
    let mut health_roots =
        world.query_filtered::<&UiTargetCamera, With<super::super::health_hud::HealthOverlay>>();
    assert_eq!(health_roots.single(&world).unwrap().entity(), window);
    assert_eq!(world.get::<UiTargetCamera>(help).unwrap().entity(), window);
    world
        .get_mut::<RobotCombatState>(robot)
        .unwrap()
        .heat
        .round_locked = true;
    world.run_system_once(update_heat_hud).unwrap();
    world.run_system_once(update_help_text).unwrap();
    let mut labels = world.query::<(&HeatText, &Text, &TextColor)>();
    for (field, text, color) in labels.iter(&world) {
        match field {
            HeatText::Value => assert_eq!(text.0, "HEAT 0.0 / 88"),
            HeatText::Cooling => assert_eq!(text.0, "COOLING 24/s"),
            HeatText::Status => {
                assert_eq!(text.0, "ROUND LOCK");
                assert_eq!(color.0, LOCKED);
            }
        }
    }
    assert!(world.get::<Text>(help).unwrap().0.starts_with("auto-aim="));
    world.entity_mut(robot).remove::<Controlled>();
    world.run_system_once(update_heat_hud).unwrap();
    for (field, text, _) in labels.iter(&world) {
        if matches!(field, HeatText::Value) {
            assert_eq!(text.0, "HEAT --");
        }
    }
}
