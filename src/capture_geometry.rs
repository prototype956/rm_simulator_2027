//! Shared capture geometry. Training measurements and rendered Talos truth use the same assets/axes.
use crate::components::{Infantry, InfantryChassis};
use crate::robomaster::combat::RobotIdentity;
use crate::robomaster::prelude::{ArmorParts, ArmorRoot, ArmorSpec, Side, Team, VertexData};
use bevy::prelude::*;
use talos_ipc::*;
pub(crate) const ALIGN: Mat3 = Mat3::from_cols(
    Vec3::new(0., -1., 0.),
    Vec3::new(0., 0., 1.),
    Vec3::new(-1., 0., 0.),
);
pub(crate) fn to_ros_translation(v: Vec3) -> Vec3 {
    ALIGN * v
}
fn to_ros_quat(q: Quat) -> Quat {
    let a = Quat::from_mat3(&ALIGN);
    a * q * a.inverse()
}
pub(crate) fn wire_quaternion(rotation: Quat) -> QuaternionF32 {
    let q = rotation.normalize();
    QuaternionF32 {
        x: q.x,
        y: q.y,
        z: q.z,
        w: q.w,
    }
}

pub(crate) fn quat_from_wire(rotation: QuaternionF32) -> Quat {
    Quat::from_xyzw(rotation.x, rotation.y, rotation.z, rotation.w)
}

pub(crate) fn transform_from_axes(origin: Vec3, x: Vec3, y: Vec3, z: Vec3) -> RigidTransformF32 {
    let rotation = Quat::from_mat3(&Mat3::from_cols(
        x.normalize(),
        y.normalize(),
        z.normalize(),
    ));
    RigidTransformF32 {
        translation: origin.to_array(),
        rotation: wire_quaternion(rotation),
        ..default()
    }
}

pub(crate) fn relative_transform(
    parent: RigidTransformF32,
    child: RigidTransformF32,
) -> RigidTransformF32 {
    let parent_rotation = quat_from_wire(parent.rotation);
    let child_rotation = quat_from_wire(child.rotation);
    let translation = parent_rotation.inverse()
        * (Vec3::from_array(child.translation) - Vec3::from_array(parent.translation));
    RigidTransformF32 {
        translation: translation.to_array(),
        rotation: wire_quaternion(parent_rotation.inverse() * child_rotation),
        ..default()
    }
}

pub(crate) fn compose_transform(
    parent: RigidTransformF32,
    child: RigidTransformF32,
) -> RigidTransformF32 {
    let parent_rotation = quat_from_wire(parent.rotation);
    RigidTransformF32 {
        translation: (Vec3::from_array(parent.translation)
            + parent_rotation * Vec3::from_array(child.translation))
        .to_array(),
        rotation: wire_quaternion(parent_rotation * quat_from_wire(child.rotation)),
        ..default()
    }
}

pub(crate) fn transform_near(
    left: RigidTransformF32,
    right: RigidTransformF32,
    tolerance: f32,
) -> bool {
    let translation_error =
        Vec3::from_array(left.translation).distance(Vec3::from_array(right.translation));
    let rotation_error =
        quat_from_wire(left.rotation).angle_between(quat_from_wire(right.rotation));
    translation_error <= tolerance && rotation_error <= tolerance
}

pub(crate) fn capture_ground_truth(
    robots: &Query<(Entity, &GlobalTransform, &Infantry, Option<&RobotIdentity>)>,
    chassis: &Query<(&GlobalTransform, &InfantryChassis)>,
    armor_roots: &Query<(Entity, &ArmorRoot, &ArmorParts)>,
    armor_vertices: &Query<(&GlobalTransform, &VertexData)>,
    child_of: &Query<&ChildOf>,
    children: &Query<&Children>,
    names: &Query<&Name>,
    global_transforms: &Query<&GlobalTransform>,
    frame_seq: u64,
    timestamp_ns: u64,
) -> GroundTruthBatch {
    let mut batch = GroundTruthBatch {
        frame_seq,
        timestamp_ns,
        ..default()
    };

    for (entity, transform, infantry, identity) in robots.iter() {
        if batch.target_count as usize >= GROUND_TRUTH_MAX_TARGETS {
            break;
        }
        let position = to_ros_translation(transform.translation());
        // Infantry 位于机器人根实体，但车身朝向实际施加在其 BASE/InfantryChassis
        // 子实体上。使用子实体的全局姿态，避免所有根实体的单位旋转被导出为 yaw=0。
        let (heading_transform, vyaw) = find_chassis_descendant(entity, children, chassis)
            .and_then(|chassis_entity| chassis.get(chassis_entity).ok())
            .map(|(chassis_transform, chassis_state)| {
                (chassis_transform, chassis_state.yaw_velocity)
            })
            .unwrap_or((transform, 0.0));
        let rotation = to_ros_quat(heading_transform.rotation());
        let (_, _, yaw) = rotation.to_euler(EulerRot::XYZ);
        let index = batch.target_count as usize;
        batch.targets[index] = GroundTruthTarget {
            frame_seq,
            timestamp_ns,
            id: identity.map_or(entity.to_bits() | (1 << 63), |id| id.id.0),
            team: match infantry.team {
                Team::Red => 0,
                Team::Blue => 1,
            },
            armor_label: infantry.config.armor.label() as u8,
            position: position.to_array(),
            vyaw,
            yaw,
            ..default()
        };
        batch.target_count += 1;
    }

    for (entity, armor_root, armor_parts) in armor_roots.iter() {
        if batch.armor_count as usize >= GROUND_TRUTH_MAX_ARMORS {
            break;
        }
        let Some(center) = find_named_suffix_descendant(entity, "CENTER", children, names) else {
            continue;
        };
        let Ok(center_transform) = global_transforms.get(center) else {
            continue;
        };
        let (armor_type, width_m) = match armor_root.spec {
            ArmorSpec::Small(_) => (0, 0.135),
            ArmorSpec::Large(_) => (1, 0.225),
        };
        let height_m = 0.055;
        let robot_entity = child_of
            .iter_ancestors(entity)
            .find(|ancestor| robots.get(*ancestor).is_ok());
        let (owner_center, logical_up) = if let Some(robot_entity) = robot_entity {
            let Ok((_, robot_transform, _, _)) = robots.get(robot_entity) else {
                continue;
            };
            let logical_rotation = find_chassis_descendant(robot_entity, children, chassis)
                .and_then(|chassis_entity| chassis.get(chassis_entity).ok())
                .map(|(_, state)| {
                    robot_transform.rotation()
                        * Quat::from_euler(EulerRot::YXZ, state.yaw, state.pitch, state.roll)
                })
                .unwrap_or_else(|| robot_transform.rotation());
            (robot_transform.translation(), logical_rotation * Vec3::Y)
        } else {
            // 前哨站等固定目标不带 Infantry/InfantryChassis。其旋转机构保持世界竖直，
            // 使用层级最上层实体作为外法向参考中心，并以 Bevy 世界 +Y 作为物理向上。
            let owner = child_of.iter_ancestors(entity).last().unwrap_or(entity);
            let Ok(owner_transform) = global_transforms.get(owner) else {
                continue;
            };
            (owner_transform.translation(), Vec3::Y)
        };
        let Some(left_strip) =
            armor_vertex_geometry(armor_parts, Side::Left, logical_up, armor_vertices)
        else {
            continue;
        };
        let Some(right_strip) =
            armor_vertex_geometry(armor_parts, Side::Right, logical_up, armor_vertices)
        else {
            continue;
        };

        // CENTER/BASE 节点可能带有不同的 GLB 建模轴修正。使用显式的左右灯条身份
        // 确定 x，并从灯条点云的上下端中心确定 y，从而保留装甲实际安装 roll。
        // 逻辑底盘向上只判断端点身份，不直接代替装甲 y 轴。
        let mut x_world = right_strip.center - left_strip.center;
        if x_world.length_squared() < 1.0e-8 {
            continue;
        }
        x_world = x_world.normalize();
        let mut right_axis = right_strip.axis;
        if right_axis.dot(left_strip.axis) < 0.0 {
            right_axis = -right_axis;
        }
        let mut y_world = left_strip.axis.normalize() + right_axis.normalize();
        y_world -= x_world * y_world.dot(x_world);
        if y_world.length_squared() < 1.0e-8 {
            continue;
        }
        y_world = y_world.normalize();
        if y_world.dot(logical_up) < 0.0 {
            y_world = -y_world;
        }
        let mut z_world = x_world.cross(y_world).normalize();
        let center_world = center_transform.translation();
        let outward_hint = center_world - owner_center;
        if z_world.dot(outward_hint) < 0.0 {
            // 某些资产的 VERTEX_L/R 命名以背面观察方向定义；协议统一按装甲正面观察方向。
            x_world = -x_world;
            z_world = x_world.cross(y_world).normalize();
        }
        y_world = z_world.cross(x_world).normalize();
        let world_t_armor = transform_from_axes(
            to_ros_translation(center_world),
            to_ros_translation(x_world),
            to_ros_translation(y_world),
            to_ros_translation(z_world),
        );
        let corners_world_bevy = [
            center_world - x_world * width_m * 0.5 + y_world * height_m * 0.5,
            center_world + x_world * width_m * 0.5 + y_world * height_m * 0.5,
            center_world + x_world * width_m * 0.5 - y_world * height_m * 0.5,
            center_world - x_world * width_m * 0.5 - y_world * height_m * 0.5,
        ];
        let corners_world = corners_world_bevy.map(|point| to_ros_translation(point).to_array());
        let index = batch.armor_count as usize;
        batch.armors[index] = GroundTruthArmor {
            id: (entity.to_bits() << 8) ^ armor_root.id.as_usize() as u64,
            owner_robot_id: robot_entity
                .and_then(|e| robots.get(e).ok())
                .and_then(|(_, _, _, identity)| identity)
                .map_or(0, |id| id.id.0),
            team: match armor_root.team {
                Team::Red => 0,
                Team::Blue => 1,
            },
            label: armor_root.label as u8,
            armor_type,
            width_m,
            height_m,
            world_t_armor,
            corners_world,
            ..default()
        };
        batch.armor_count += 1;
    }
    batch
}

struct ArmorVertexGeometry {
    center: Vec3,
    axis: Vec3,
}

fn armor_vertex_geometry(
    parts: &ArmorParts,
    side: Side,
    up_hint: Vec3,
    vertices: &Query<(&GlobalTransform, &VertexData)>,
) -> Option<ArmorVertexGeometry> {
    let (transform, data) = vertices.get(parts.vertex(side)).ok()?;
    if data.side != side || data.points.is_empty() {
        return None;
    }
    let world_points = data
        .points
        .iter()
        .map(|point| transform.transform_point(*point))
        .collect::<Vec<_>>();
    let up = up_hint.normalize();
    let mut min_projection = f32::INFINITY;
    let mut max_projection = f32::NEG_INFINITY;
    for point in &world_points {
        let projection = point.dot(up);
        min_projection = min_projection.min(projection);
        max_projection = max_projection.max(projection);
    }
    let span = max_projection - min_projection;
    if !span.is_finite() || span < 1.0e-4 {
        return None;
    }

    // VERTEX 网格包含灯条宽度和少量重复顶点。分别平均轴向两端 10% 的点，
    // 得到端面中心，避免直接取单个极值角点给 roll 引入灯条宽度偏差。
    let end_band = (span * 0.10).max(1.0e-4);
    let mut top_sum = Vec3::ZERO;
    let mut bottom_sum = Vec3::ZERO;
    let mut top_count = 0usize;
    let mut bottom_count = 0usize;
    for point in world_points {
        let projection = point.dot(up);
        if projection >= max_projection - end_band {
            top_sum += point;
            top_count += 1;
        }
        if projection <= min_projection + end_band {
            bottom_sum += point;
            bottom_count += 1;
        }
    }
    if top_count == 0 || bottom_count == 0 {
        return None;
    }
    let top = top_sum / top_count as f32;
    let bottom = bottom_sum / bottom_count as f32;
    let axis = top - bottom;
    if axis.length_squared() < 1.0e-8 {
        return None;
    }
    Some(ArmorVertexGeometry {
        center: (top + bottom) * 0.5,
        axis,
    })
}

fn find_chassis_descendant(
    root: Entity,
    children: &Query<&Children>,
    chassis: &Query<(&GlobalTransform, &InfantryChassis)>,
) -> Option<Entity> {
    let mut pending = vec![root];
    while let Some(entity) = pending.pop() {
        if chassis.get(entity).is_ok() {
            return Some(entity);
        }
        if let Ok(descendants) = children.get(entity) {
            pending.extend(descendants.iter());
        }
    }
    None
}

fn find_named_suffix_descendant(
    root: Entity,
    wanted: &str,
    children: &Query<&Children>,
    names: &Query<&Name>,
) -> Option<Entity> {
    let mut pending = vec![root];
    while let Some(entity) = pending.pop() {
        // GLB 节点带有装甲序号和规格前缀，例如 `1__L_ARMOR_CENTER`。
        if names
            .get(entity)
            .is_ok_and(|name| name.as_str().ends_with(wanted))
        {
            return Some(entity);
        }
        if let Ok(descendants) = children.get(entity) {
            pending.extend(descendants.iter());
        }
    }
    None
}
