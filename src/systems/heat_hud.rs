//! Operator-window heat display. Never attach these nodes to the offscreen capture cameras.

use bevy::camera::RenderTarget;
use bevy::prelude::*;

use super::HelpText;
use crate::capture::PreviewCamera;
use crate::components::{Controlled, MainCamera};
use crate::config::SimulationConfig;
use crate::robomaster::combat::{RobotCombatState, RobotIdentity, ShooterKind};

const NORMAL: Color = Color::srgb(0.15, 0.85, 0.80);
const LOCKED: Color = Color::srgb(1.0, 0.30, 0.30);
const MUTED: Color = Color::srgb(0.65, 0.70, 0.75);

pub struct HeatHudPlugin;

impl Plugin for HeatHudPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PostStartup, setup_window_hud)
            .add_systems(Update, update_heat_hud);
        super::health_hud::install(app);
    }
}

#[derive(Component)]
struct HeatHud;

#[derive(Component)]
enum HeatText {
    Value,
    Cooling,
    Status,
}

#[derive(Component)]
struct HeatFill;

/// Run after all Startup camera creation, including the Talos/ROS2 preview camera.
/// The explicit render target check also covers a standalone simulator without capture.
fn setup_window_hud(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    cameras: Query<(Entity, &Camera, &RenderTarget), Or<(With<PreviewCamera>, With<MainCamera>)>>,
    mut help: Query<(Entity, &mut Node, &mut Visibility), With<HelpText>>,
) {
    if !config.preview.enabled {
        return;
    }
    let Some((camera, _, _)) = cameras
        .iter()
        .filter(|(_, camera, target)| camera.is_active && matches!(target, RenderTarget::Window(_)))
        .max_by_key(|(_, camera, _)| camera.order)
    else {
        return;
    };
    for (entity, mut node, mut visibility) in &mut help {
        commands.entity(entity).insert(UiTargetCamera(camera));
        // Separate horizontal regions also keep wrapped help text away from the heat panel.
        node.width = Val::Percent(if config.preview.heat_hud { 50.0 } else { 95.0 });
        *visibility = Visibility::Visible;
    }
    if config.preview.health_hud {
        super::health_hud::spawn_health_overlay(&mut commands, camera);
    }
    if !config.preview.heat_hud {
        return;
    }
    commands
        .spawn((
            HeatHud,
            UiTargetCamera(camera),
            GlobalZIndex(1),
            Node {
                position_type: PositionType::Absolute,
                right: Val::Px(16.0),
                bottom: Val::Px(16.0),
                width: Val::Px(320.0),
                max_width: Val::Percent(45.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(12.0),
                ..default()
            },
        ))
        .with_children(|hud| {
            hud.spawn((
                Node {
                    min_height: Val::Px(96.0),
                    padding: UiRect::all(Val::Px(12.0)),
                    border: UiRect::all(Val::Px(1.0)),
                    flex_direction: FlexDirection::Column,
                    justify_content: JustifyContent::SpaceBetween,
                    row_gap: Val::Px(8.0),
                    ..default()
                },
                BackgroundColor(Color::srgba(0.025, 0.045, 0.06, 0.82)),
                BorderColor::all(Color::srgba(0.45, 0.60, 0.65, 0.65)),
            ))
            .with_children(|panel| {
                panel.spawn((
                    HeatText::Value,
                    Text::new("HEAT --"),
                    TextFont::from_font_size(20.0),
                    TextColor(MUTED),
                ));
                panel
                    .spawn((
                        Node {
                            width: Val::Percent(100.0),
                            height: Val::Px(10.0),
                            flex_shrink: 0.0,
                            ..default()
                        },
                        BackgroundColor(Color::srgb(0.14, 0.19, 0.22)),
                    ))
                    .with_children(|bar| {
                        bar.spawn((
                            HeatFill,
                            Node {
                                width: Val::Percent(0.0),
                                height: Val::Percent(100.0),
                                ..default()
                            },
                            BackgroundColor(MUTED),
                        ));
                    });
                panel
                    .spawn(Node {
                        justify_content: JustifyContent::SpaceBetween,
                        flex_wrap: FlexWrap::Wrap,
                        column_gap: Val::Px(8.0),
                        ..default()
                    })
                    .with_children(|footer| {
                        footer.spawn((
                            HeatText::Cooling,
                            Text::new("COOLING --/s"),
                            TextFont::from_font_size(14.0),
                            TextColor(MUTED),
                        ));
                        footer.spawn((
                            HeatText::Status,
                            Text::new("UNAVAILABLE"),
                            TextFont::from_font_size(14.0),
                            TextColor(MUTED),
                        ));
                    });
            });
        });
}

/// Read the root's physical truth, not a replay of shots or a locally integrated heat value.
/// Status is thermal permission only; it must not imply that ammo/mechanics allow a shot.
fn update_heat_hud(
    robots: Query<&RobotCombatState, (With<Controlled>, With<RobotIdentity>)>,
    mut texts: Query<(&HeatText, &mut Text, &mut TextColor)>,
    mut fills: Query<(&mut Node, &mut BackgroundColor), With<HeatFill>>,
) {
    if texts.is_empty() {
        return;
    }
    let state = robots
        .single()
        .ok()
        .filter(|state| state.shooter.kind != ShooterKind::None && state.rules.heat_limit > 0);
    let (value, cooling, status, fraction, color) = match state {
        Some(state) => {
            let heat = state.heat.current();
            let status = if state.heat.round_locked {
                "ROUND LOCK"
            } else if state.life.status == crate::robomaster::combat::LifeStatus::Dead {
                "DEAD"
            } else if state.heat.cooling_locked {
                "COOLING LOCK"
            } else {
                "HEAT OK"
            };
            (
                format!("HEAT {:.1} / {}", heat, state.rules.heat_limit),
                format!("COOLING {}/s", state.rules.cooling_per_second),
                status,
                (heat / f64::from(state.rules.heat_limit)).clamp(0.0, 1.0) as f32,
                if state.heat.is_locked()
                    || state.life.status == crate::robomaster::combat::LifeStatus::Dead
                {
                    LOCKED
                } else {
                    NORMAL
                },
            )
        }
        None => (
            "HEAT --".into(),
            "COOLING --/s".into(),
            "UNAVAILABLE",
            0.0,
            MUTED,
        ),
    };
    for (field, mut text, mut text_color) in &mut texts {
        let (label, color) = match field {
            HeatText::Value => (value.as_str(), color),
            HeatText::Cooling => (cooling.as_str(), MUTED),
            HeatText::Status => (status, color),
        };
        if text.0 != label {
            text.0 = label.to_owned();
        }
        text_color.set_if_neq(TextColor(color));
    }
    for (mut node, mut background) in &mut fills {
        let width = Val::Percent(fraction * 100.0);
        if node.width != width {
            node.width = width;
        }
        background.set_if_neq(BackgroundColor(color));
    }
}
