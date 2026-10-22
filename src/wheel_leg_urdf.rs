//! Wheel-leg CAD import adapter. Meshes and joint origins come from the supplied URDF;
//! chassis balance and gimbal actuation retain the simulator's prescribed controllers.
use crate::components::{Controlled, GameLayer};
use crate::robomaster::combat::{RobotIdentity, RobotMember};
use crate::robots::{RigPart, RobotRig};
use avian3d::prelude::*;
use bevy::prelude::*;
use serde::Deserialize;
use std::sync::LazyLock;

#[derive(Deserialize)]
struct Joint {
    child: String,
    origin: [f32; 16],
    lower: f32,
    upper: f32,
}
#[derive(Deserialize)]
struct CollisionPart {
    name: String,
    points: Vec<[f32; 3]>,
}
#[derive(Deserialize)]
struct Link {
    #[serde(default)]
    collision_parts: Vec<CollisionPart>,
    name: String,
    hull_points: Vec<[f32; 3]>,
    zero_matrix: [f32; 16],
}
#[derive(Deserialize)]
struct Model {
    root_height_m: f32,
    auxiliary_supports: Vec<[f32; 3]>,
    joints: Vec<Joint>,
    links: Vec<Link>,
}
static MODEL: LazyLock<Model> = LazyLock::new(|| {
    serde_json::from_str(include_str!("../assets/robots/wheel_leg/rig.json"))
        .expect("validated wheel-leg metadata")
});
pub fn root_height() -> f32 {
    MODEL.root_height_m
}
#[derive(Clone, Debug)]
pub struct UrdfRig {
    pub root: Entity,
    pub base_link: Entity,
    pub chains: [[Entity; 3]; 2],
    pub yaw_link: Entity,
}
#[derive(Component, Default, Debug)]
pub struct UrdfStatus {
    pub error: Option<String>,
}
fn joint(name: &str) -> &'static Joint {
    MODEL.joints.iter().find(|j| j.child == name).unwrap()
}
fn keys(side: usize) -> [&'static str; 3] {
    if side == 0 {
        ["link_002", "link_003", "link_004"]
    } else {
        ["link_005", "link_006", "link_007"]
    }
}
fn matrix(j: &Joint, q: f32) -> Mat4 {
    Mat4::from_cols_array(&j.origin) * Mat4::from_quat(Quat::from_rotation_z(q))
}
fn forward(side: usize, q: [f32; 3]) -> Mat4 {
    keys(side)
        .iter()
        .zip(q)
        .fold(Mat4::IDENTITY, |m, (k, a)| m * matrix(joint(k), a))
}
/// Two bounded revolute joints position each wheel; the third preserves its rest orientation.
/// No link scaling or telescoping geometry is permitted.
pub fn solve_leg(side: usize, rise: f32) -> Result<[f32; 3], String> {
    let rest = forward(side, [0.; 3]);
    let target = rest.transform_point3(Vec3::ZERO) - Vec3::Z * rise;
    solve_target(side, target)
}
pub fn solve_target(side: usize, target: Vec3) -> Result<[f32; 3], String> {
    let rest = forward(side, [0.; 3]);
    let mut q = [0.; 3];
    for _ in 0..40 {
        let p = forward(side, q).transform_point3(Vec3::ZERO);
        let e = Vec2::new(target.y - p.y, target.z - p.z);
        if e.length() < 0.00001 {
            break;
        }
        let mut columns = [Vec2::ZERO; 2];
        for i in 0..2 {
            let mut v = q;
            v[i] += 0.0001;
            let d = (forward(side, v).transform_point3(Vec3::ZERO) - p) / 0.0001;
            columns[i] = Vec2::new(d.y, d.z);
        }
        let jac = Mat2::from_cols(columns[0], columns[1]);
        if jac.determinant().abs() < 1e-8 {
            return Err("singular URDF wheel-leg pose".into());
        }
        let delta = jac.inverse() * e;
        for i in 0..2 {
            let j = joint(keys(side)[i]);
            q[i] = (q[i] + delta[i].clamp(-0.25, 0.25)).clamp(j.lower, j.upper);
        }
    }
    let at = forward(side, q);
    if at.transform_point3(Vec3::ZERO).distance(target) > 0.0001 {
        return Err(format!(
            "URDF wheel-leg target unreachable: side={side}, target={target:?}"
        ));
    }
    let d = at.to_scale_rotation_translation().1.inverse() * rest.to_scale_rotation_translation().1;
    let j = joint(keys(side)[2]);
    q[2] = (2. * d.z.atan2(d.w)).clamp(j.lower, j.upper);
    Ok(q)
}
pub fn apply_legs(world: &mut World, rig: &UrdfRig, rises: [f32; 2]) -> Result<(), String> {
    let solutions = [solve_leg(0, rises[0])?, solve_leg(1, rises[1])?];
    for (side, q) in solutions.iter().enumerate() {
        for i in 0..3 {
            world
                .entity_mut(rig.chains[side][i])
                .insert(Transform::from_matrix(matrix(joint(keys(side)[i]), q[i])));
        }
    }
    world.entity_mut(rig.root).insert(UrdfStatus::default());
    Ok(())
}
pub fn set_lift(world: &mut World, rig: &UrdfRig, lift: f32) {
    if let Err(error) = apply_legs(world, rig, [lift; 2]) {
        world
            .entity_mut(rig.root)
            .insert(UrdfStatus { error: Some(error) });
    }
}
pub fn build(world: &mut World, root: Entity, nodes: &[Entity]) -> RobotRig {
    let find = |name: &str| {
        *nodes
            .iter()
            .find(|e| world.get::<Name>(**e).is_some_and(|n| n.as_str() == name))
            .unwrap_or_else(|| panic!("missing URDF adapter node {name}"))
    };
    let vehicle = find("VEHICLE");
    let base = find("BASE");
    let gimbal = find("GIMBAL");
    let chains = [
        [
            find("URDF_link_002"),
            find("URDF_link_003"),
            find("URDF_link_004"),
        ],
        [
            find("URDF_link_005"),
            find("URDF_link_006"),
            find("URDF_link_007"),
        ],
    ];
    let yaw_link = find("URDF_link_008");
    let link_entities: Vec<_> = MODEL
        .links
        .iter()
        .map(|l| (find(&format!("URDF_{}", l.name)), l))
        .collect();
    let old: Vec<_> = nodes
        .iter()
        .copied()
        .filter(|e| {
            world
                .get::<Name>(*e)
                .is_some_and(|n| n.as_str() == "CombatBodyCore")
        })
        .collect();
    let identity = *world.get::<RobotIdentity>(root).unwrap();
    let member = RobotMember {
        root,
        id: identity.id,
    };
    let layers = GameLayer::vehicle_armor_collision_layers(world.get::<Controlled>(root).is_some());
    for e in old {
        world.despawn(e);
    }
    let body_mount = world
        .spawn((
            Name::new("BodyLiftMount"),
            Transform::default(),
            ChildOf(root),
            member,
        ))
        .id();
    world.entity_mut(vehicle).insert(ChildOf(body_mount));
    let fold_mount = world
        .spawn((
            Name::new("FoldMount"),
            Transform::default(),
            ChildOf(vehicle),
            member,
        ))
        .id();
    world.entity_mut(gimbal).insert(ChildOf(fold_mount));
    for (entity, link) in link_entities {
        let shapes: Vec<_> = if link.name == "base_link" {
            link.collision_parts
                .iter()
                .map(|p| (p.name.as_str(), p.points.as_slice()))
                .collect()
        } else {
            vec![(link.name.as_str(), link.hull_points.as_slice())]
        };
        for (name, points) in shapes {
            if let Some(hull) =
                Collider::convex_hull(points.iter().map(|p| Vec3::from_array(*p)).collect())
            {
                world.spawn((
                    Name::new(format!("UrdfHull_{name}")),
                    Transform::default(),
                    ChildOf(entity),
                    member,
                    hull,
                    ColliderDensity(0.),
                    layers,
                    RigPart,
                ));
            }
        }
    }
    // Actual imported wheels support this rigid body. No stationary cylinder beneath it.
    world
        .entity_mut(root)
        .remove::<Collider>()
        .insert((UrdfStatus::default(), LockedAxes::ROTATION_LOCKED));
    let urdf = UrdfRig {
        root,
        base_link: find_native_base(world, root),
        chains,
        yaw_link,
    };
    set_lift(world, &urdf, 0.);
    RobotRig {
        folding: false,
        body_mount,
        base,
        leg_frame: None,
        fold_mount,
        gimbal,
        parked_rotation: Quat::IDENTITY,
        fold_stem: None,
        legs: vec![],
        feet: vec![],
        fold: 0.,
        requested_fold: false,
        stable_s: 0.1,
        lift_m: 0.,
        requested_lift_m: 0.,
        lift_velocity: 0.,
        blocked: None,
        urdf: Some(urdf),
    }
}
/// The exported bracket must yaw with the bore, while remaining upright when pitching.
/// The two gimbal joints retain legacy controller behavior rather than URDF torque dynamics.
pub fn sync_gimbals(world: &mut World) {
    let rigs: Vec<_> = world
        .query::<&RobotRig>()
        .iter(world)
        .filter(|r| r.urdf.is_some())
        .cloned()
        .collect();
    for rig in rigs {
        let native = rig.urdf.as_ref().unwrap();
        let local = world.get::<Transform>(rig.gimbal).unwrap().rotation;
        let base = world.get::<Transform>(rig.base).unwrap().rotation;
        let (yaw, _, _) = local.to_euler(EulerRot::YXZ);
        let zero = MODEL.links.iter().find(|l| l.name == "link_008").unwrap();
        let mut t = Transform::from_matrix(Mat4::from_cols_array(&zero.zero_matrix));
        t.rotation = base.inverse() * Quat::from_rotation_y(yaw) * t.rotation;
        world.entity_mut(native.yaw_link).insert(t);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn real_urdf_legs_keep_lengths_and_limits() {
        for side in 0..2 {
            for step in 0..=200 {
                let h = step as f32 / 1000.;
                let q = solve_leg(side, h).unwrap();
                for (k, a) in keys(side).iter().zip(q) {
                    let j = joint(k);
                    assert!((j.lower..=j.upper).contains(&a));
                }
                let rest = forward(side, [0.; 3]);
                let at = forward(side, q);
                assert!(
                    at.transform_point3(Vec3::ZERO)
                        .distance(rest.transform_point3(Vec3::ZERO) - Vec3::Z * h)
                        < 0.0001
                );
                assert!(
                    at.to_scale_rotation_translation()
                        .1
                        .angle_between(rest.to_scale_rotation_translation().1)
                        < 0.001
                );
            }
        }
    }
    #[test]
    fn unreachable_pose_is_rejected() {
        assert!(solve_leg(0, 1.).is_err());
    }
}

fn find_native_base(world: &mut World, root: Entity) -> Entity {
    world
        .query::<(Entity, &Name, &RobotMember)>()
        .iter(world)
        .find(|(_, n, m)| n.as_str() == "URDF_base_link" && m.root == root)
        .unwrap()
        .0
}
pub fn auxiliary_supports() -> Vec<Vec3> {
    MODEL
        .auxiliary_supports
        .iter()
        .map(|p| Vec3::from_array(*p))
        .collect()
}
pub fn wheel_centers(world: &World, rig: &RobotRig, pose: Transform) -> [Vec3; 2] {
    let delta = pose.to_matrix()
        * crate::robots::physical_matrix(world, rig.urdf.as_ref().unwrap().root).inverse();
    [0, 1].map(|i| {
        delta.transform_point3(
            crate::robots::physical_matrix(world, rig.urdf.as_ref().unwrap().chains[i][2])
                .transform_point3(Vec3::ZERO),
        )
    })
}
pub fn rest_wheels(world: &World, rig: &RobotRig, pose: Transform) -> [Vec3; 2] {
    let native = rig.urdf.as_ref().unwrap();
    let m = pose.to_matrix()
        * crate::robots::physical_matrix(world, native.root).inverse()
        * crate::robots::physical_matrix(world, native.base_link);
    [0, 1].map(|i| m.transform_point3(forward(i, [0.; 3]).transform_point3(Vec3::ZERO)))
}
pub fn set_world_wheels(
    world: &mut World,
    rig: &RobotRig,
    pose: Transform,
    targets: [Vec3; 2],
) -> Result<(), String> {
    let native = rig.urdf.as_ref().unwrap();
    let m = pose.to_matrix()
        * crate::robots::physical_matrix(world, native.root).inverse()
        * crate::robots::physical_matrix(world, native.base_link);
    let solutions = [
        solve_target(0, m.inverse().transform_point3(targets[0]))?,
        solve_target(1, m.inverse().transform_point3(targets[1]))?,
    ];
    for (side, q) in solutions.iter().enumerate() {
        for i in 0..3 {
            world
                .entity_mut(native.chains[side][i])
                .insert(Transform::from_matrix(matrix(joint(keys(side)[i]), q[i])));
        }
    }
    world.entity_mut(native.root).insert(UrdfStatus::default());
    Ok(())
}
pub const WHEEL_RADIUS: f32 = 0.05908;
pub fn interaction_point(world: &World, rig: &RobotRig) -> Vec3 {
    let native = rig.urdf.as_ref().unwrap();
    let feet = [0, 1].map(|i| {
        crate::robots::physical_matrix(world, native.chains[i][2]).transform_point3(Vec3::ZERO)
            - Vec3::Y * WHEEL_RADIUS
    });
    (feet[0] + feet[1]) * 0.5
}
