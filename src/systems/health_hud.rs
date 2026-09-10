//! Screen-facing training HP labels. Only the operator window receives this overlay.
use std::collections::HashMap;

use bevy::camera::{CameraUpdateSystems, primitives::Aabb};
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy::transform::helper::TransformHelper;
use bevy::ui::UiSystems;

use crate::capture::{CaptureCamera, PreviewCamera};
use crate::components::{CameraMode, Controlled, FollowingType};
use crate::robomaster::combat::{
    LifeStatus, RobotCombatState, RobotId, RobotIdentity, RobotMember,
};
use crate::robomaster::prelude::Team;

const WIDTH: f32 = 140.0;
const HEIGHT: f32 = 32.0;
const HEAD_GAP: f32 = 0.15;

pub(super) fn install(app: &mut App) {
    // Bevy UI layout runs BEFORE TransformSystems::Propagate. TransformHelper reads this
    // frame's local transforms without introducing a schedule cycle or a one-frame lag.
    app.add_systems(
        PostUpdate,
        update_health_hud
            .after(CameraUpdateSystems)
            .before(UiSystems::Prepare),
    );
}

#[derive(Component)]
pub(super) struct HealthOverlay;
#[derive(Component)]
pub(super) struct HealthRow {
    id: RobotId,
    max_hp: u32,
}
#[derive(Component)]
pub(super) struct HealthLabel(RobotId);
#[derive(Component)]
pub(super) struct HealthFill(RobotId);

pub(super) fn spawn_health_overlay(commands: &mut Commands, camera: Entity) {
    commands.spawn((
        HealthOverlay,
        UiTargetCamera(camera),
        GlobalZIndex(0),
        Node {
            position_type: PositionType::Absolute,
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            overflow: Overflow::clip(),
            ..default()
        },
    ));
}

fn spawn_row(parent: &mut ChildSpawnerCommands, id: RobotId, max_hp: u32) {
    parent
        .spawn((
            HealthRow { id, max_hp },
            Visibility::Hidden,
            Node {
                position_type: PositionType::Absolute,
                width: Val::Px(WIDTH),
                height: Val::Px(HEIGHT),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(3.0),
                ..default()
            },
        ))
        .with_children(|row| {
            row.spawn((
                HealthLabel(id),
                Text::new(""),
                TextFont::from_font_size(12.0),
                TextColor(Color::WHITE),
                TextShadow::default(),
                TextLayout::no_wrap(),
            ));
            row.spawn((
                Node {
                    width: Val::Percent(100.0),
                    height: Val::Px(12.0),
                    border: UiRect::all(Val::Px(1.0)),
                    ..default()
                },
                BackgroundColor(Color::srgba(0.025, 0.03, 0.04, 0.92)),
                BorderColor::all(Color::srgb(0.08, 0.08, 0.09)),
            ))
            .with_children(|bar| {
                bar.spawn((
                    HealthFill(id),
                    Node {
                        width: Val::Percent(0.0),
                        height: Val::Percent(100.0),
                        ..default()
                    },
                    BackgroundColor(Color::BLACK),
                ));
                for hp in (50..max_hp).step_by(50) {
                    bar.spawn((
                        Node {
                            position_type: PositionType::Absolute,
                            left: Val::Percent(100.0 * hp as f32 / max_hp as f32),
                            width: Val::Px(1.0),
                            height: Val::Percent(100.0),
                            ..default()
                        },
                        BackgroundColor(Color::srgba(0.02, 0.02, 0.03, 0.65)),
                    ));
                }
            });
        });
}

#[derive(SystemParam)]
pub(super) struct HealthScene<'w, 's> {
    robots: Query<
        'w,
        's,
        (
            Entity,
            &'static RobotIdentity,
            &'static RobotCombatState,
            Has<Controlled>,
        ),
    >,
    meshes: Query<'w, 's, (Entity, &'static Aabb), With<Mesh3d>>,
    members: Query<'w, 's, &'static RobotMember>,
    parents: Query<'w, 's, &'static ChildOf>,
    transforms: TransformHelper<'w, 's>,
    cameras: Query<
        'w,
        's,
        (
            Entity,
            &'static Camera,
            Has<PreviewCamera>,
            Has<CaptureCamera>,
        ),
    >,
    mode: Res<'w, CameraMode>,
    scale: Res<'w, UiScale>,
}

/// PreviewImageNode fills the viewport, but its default NodeImageMode::Auto draws
/// the texture centered with aspect preserved. Match that letterboxing on every resize.
fn preview_rect(viewport: Vec2, source: Vec2) -> Option<Rect> {
    if !source.is_finite() || source.min_element() <= 0.0 {
        return None;
    }
    let size = source * (viewport / source).min_element();
    Some(Rect::from_center_size(viewport * 0.5, size))
}

/// NDC to logical UI coordinates, including the displayed image's letterbox offset.
fn screen_anchor(ndc: Vec3, viewport: Rect, ui_scale: f32) -> Option<Vec2> {
    if !ndc.is_finite()
        || ndc.x.abs() > 1.0
        || ndc.y.abs() > 1.0
        || ndc.z <= 0.0
        || ndc.z > 1.0
        || !viewport.min.is_finite()
        || !viewport.max.is_finite()
        || viewport.size().min_element() <= 0.0
        || !ui_scale.is_finite()
        || ui_scale <= 0.0
    {
        return None;
    }
    Some(
        (viewport.min + Vec2::new((ndc.x + 1.0) * 0.5, (1.0 - ndc.y) * 0.5) * viewport.size())
            / ui_scale,
    )
}

fn top_of_mesh(aabb: &Aabb, transform: &GlobalTransform) -> f32 {
    // The Y support of a transformed AABB includes rotation and non-uniform scale.
    let affine = transform.affine();
    let center = affine.transform_point3a(aabb.center);
    let m = affine.matrix3;
    center.y
        + m.x_axis.y.abs() * aabb.half_extents.x
        + m.y_axis.y.abs() * aabb.half_extents.y
        + m.z_axis.y.abs() * aabb.half_extents.z
}

pub(super) fn update_health_hud(
    mut commands: Commands,
    overlays: Query<(Entity, &UiTargetCamera), With<HealthOverlay>>,
    scene: HealthScene,
    mut rows: Query<(Entity, &HealthRow, &mut Node, &mut Visibility)>,
    mut labels: Query<(&HealthLabel, &mut Text, &mut TextColor)>,
    mut fills: Query<(&HealthFill, &mut Node, &mut BackgroundColor), Without<HealthRow>>,
) {
    let Ok((overlay, target)) = overlays.single() else {
        return;
    };
    let viewport_camera = scene.cameras.get(target.entity()).ok();
    let projection = viewport_camera.and_then(|(entity, camera, preview, _)| {
        if !camera.is_active {
            return None;
        }
        let viewport = camera.logical_viewport_size()?;
        let (source_entity, source) = if preview {
            scene
                .cameras
                .iter()
                .find_map(|(entity, camera, _, capture)| {
                    (capture && camera.is_active).then_some((entity, camera))
                })?
        } else {
            (entity, camera)
        };
        let viewport = if preview {
            preview_rect(viewport, source.logical_viewport_size()?)?
        } else {
            Rect::from_corners(Vec2::ZERO, viewport)
        };
        Some((
            source,
            scene
                .transforms
                .compute_global_transform(source_entity)
                .ok()?,
            viewport,
        ))
    });

    let mut tops: HashMap<Entity, f32> = HashMap::new();
    for (mesh, bounds) in &scene.meshes {
        let mut owner = mesh;
        let root = loop {
            if scene.robots.contains(owner) {
                break Some(owner);
            }
            if let Ok(member) = scene.members.get(owner) {
                break Some(member.root);
            }
            let Ok(parent) = scene.parents.get(owner) else {
                break None;
            };
            owner = parent.parent();
        };
        let Some(root) = root else {
            continue;
        };
        let Ok(transform) = scene.transforms.compute_global_transform(mesh) else {
            continue;
        };
        let top = top_of_mesh(bounds, &transform);
        if top.is_finite() {
            tops.entry(root)
                .and_modify(|y| *y = y.max(top))
                .or_insert(top);
        }
    }
    for (entity, row, _, _) in &rows {
        if !scene
            .robots
            .iter()
            .any(|(_, id, state, _)| id.id == row.id && state.rules.max_hp == row.max_hp)
        {
            commands.entity(entity).despawn();
        }
    }
    for (_, id, state, _) in &scene.robots {
        if !rows
            .iter()
            .any(|(_, row, _, _)| row.id == id.id && row.max_hp == state.rules.max_hp)
        {
            commands
                .entity(overlay)
                .with_children(|parent| spawn_row(parent, id.id, state.rules.max_hp));
        }
    }
    for (_, row, mut node, mut visibility) in &mut rows {
        let position = scene
            .robots
            .iter()
            .find(|(_, id, _, _)| id.id == row.id)
            .and_then(|(root, _, _, controlled)| {
                if controlled && scene.mode.0 == FollowingType::Robot {
                    return None;
                }
                let top = tops.get(&root)?;
                let mut anchor = scene
                    .transforms
                    .compute_global_transform(root)
                    .ok()?
                    .translation();
                anchor.y = top + HEAD_GAP;
                let (camera, transform, viewport) = projection.as_ref()?;
                let ndc = camera.world_to_ndc(transform, anchor)?;
                screen_anchor(ndc, *viewport, scene.scale.0)
            });
        if let Some(position) = position {
            node.left = Val::Px(position.x - WIDTH * 0.5);
            node.top = Val::Px(position.y - HEIGHT);
            visibility.set_if_neq(Visibility::Visible);
        } else {
            visibility.set_if_neq(Visibility::Hidden);
        }
    }
    for (label, mut text, mut color) in &mut labels {
        let Some((_, id, state, controlled)) =
            scene.robots.iter().find(|(_, id, _, _)| id.id == label.0)
        else {
            continue;
        };
        let name = if controlled {
            "SELF"
        } else if id.team == Team::Red {
            "RED"
        } else {
            "BLUE"
        };
        let value = if state.life.status == LifeStatus::Dead {
            format!("{} #{}  DEAD", name, id.id.0)
        } else {
            format!(
                "{} #{}  {}/{}",
                name, id.id.0, state.life.hp, state.rules.max_hp
            )
        };
        if text.0 != value {
            text.0 = value;
        }
        color.set_if_neq(TextColor(if state.life.status == LifeStatus::Dead {
            Color::srgb(0.65, 0.65, 0.65)
        } else {
            Color::WHITE
        }));
    }
    for (fill, mut node, mut background) in &mut fills {
        let Some((_, id, state, _)) = scene.robots.iter().find(|(_, id, _, _)| id.id == fill.0)
        else {
            continue;
        };
        let fraction = if state.rules.max_hp == 0 || state.life.status == LifeStatus::Dead {
            0.0
        } else {
            (state.life.hp as f32 / state.rules.max_hp as f32).clamp(0.0, 1.0)
        };
        let width = Val::Percent(fraction * 100.0);
        if node.width != width {
            node.width = width;
        }
        let color = if state.life.status == LifeStatus::Dead {
            Color::srgb(0.45, 0.45, 0.45)
        } else if id.team == Team::Red {
            Color::srgb(1.0, 0.30, 0.30)
        } else {
            Color::srgb(0.25, 0.65, 1.0)
        };
        background.set_if_neq(BackgroundColor(color));
    }
}
