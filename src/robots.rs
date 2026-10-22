//! Fire-control rigs: supplied URDF wheel-leg CAD and shipped-GLB geometry proxies.
//! A rig owns visual mounts and collision mounts together. Armor dimensions never scale.
use crate::components::{Controlled, GameLayer};
use crate::robomaster::combat::{LifeStatus, RobotCombatState, RobotIdentity, RobotMember};
use avian3d::collision::collider::contact_query;
use avian3d::prelude::*;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(
    Component, Deserialize, Serialize, Reflect, Clone, Copy, Debug, Default, PartialEq, Eq,
)]
#[serde(rename_all = "snake_case")]
pub enum RobotProfile {
    #[default]
    Standard,
    WheelLeg,
    Low,
    Folding,
    HeavyTarget,
}
impl RobotProfile {
    pub fn asset(self) -> &'static str {
        if self == Self::WheelLeg {
            "robots/wheel_leg/wheel_leg.glb"
        } else if self == Self::HeavyTarget {
            "HERO.glb"
        } else {
            "vehicle.glb"
        }
    }
    pub fn root_height(self) -> f32 {
        match self {
            Self::Low | Self::Folding => 0.105,
            Self::WheelLeg => crate::wheel_leg_urdf::root_height(),
            _ => 0.236298,
        }
    }
    pub fn compact(self) -> bool {
        matches!(self, Self::Low | Self::Folding)
    }
}

#[derive(Deserialize, Serialize, Reflect, Clone, Debug, Default)]
#[serde(default, deny_unknown_fields)]
pub struct RobotProfiles {
    pub controlled: RobotProfile,
    pub target: RobotProfile,
}

#[derive(Component, Clone, Debug)]
pub struct RobotRig {
    pub urdf: Option<crate::wheel_leg_urdf::UrdfRig>,
    pub folding: bool,
    pub body_mount: Entity,
    pub base: Entity,
    pub leg_frame: Option<Entity>,
    pub fold_mount: Entity,
    pub gimbal: Entity,
    pub parked_rotation: Quat,
    pub fold_stem: Option<Entity>,
    pub legs: Vec<Entity>,
    pub feet: Vec<Entity>,
    pub fold: f32,
    pub requested_fold: bool,
    pub stable_s: f32,
    pub lift_m: f32,
    pub requested_lift_m: f32,
    pub lift_velocity: f32,
    pub blocked: Option<String>,
}
impl RobotRig {
    pub fn fire_ready(&self) -> bool {
        !self.folding || (self.fold <= 1e-6 && !self.requested_fold && self.stable_s >= 0.1)
    }
}

#[derive(Component, Clone, Copy)]
pub struct RigPart;

pub struct RobotPlugin;
impl Plugin for RobotPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedPreUpdate,
            update_rigs.before(crate::gimbal_actuator::update_gimbal_actuator),
        )
        .add_systems(
            Update,
            (operator_commands, crate::wheel_leg_urdf::sync_gimbals).chain(),
        )
        .add_systems(
            FixedPreUpdate,
            crate::wheel_leg_urdf::sync_gimbals
                .after(crate::gimbal_actuator::update_gimbal_actuator),
        );
        app.add_systems(
            FixedPreUpdate,
            crate::wheel_leg_traversal::update
                .after(update_rigs)
                .before(crate::gimbal_actuator::update_gimbal_actuator),
        )
        .add_systems(Update, crate::wheel_leg_traversal::jump_key);
    }
}

pub fn physical_matrix(world: &World, entity: Entity) -> Mat4 {
    let t = world.get::<Transform>(entity).copied().unwrap_or_default();
    if let (Some(p), Some(r)) = (world.get::<Position>(entity), world.get::<Rotation>(entity)) {
        if world.get::<RigidBody>(entity).is_some() {
            return Mat4::from_scale_rotation_translation(t.scale, r.0, p.0);
        }
    }
    world
        .get::<ChildOf>(entity)
        .map_or(Mat4::IDENTITY, |p| physical_matrix(world, p.parent()))
        * t.to_matrix()
}

fn descendants(world: &World, root: Entity) -> Vec<Entity> {
    let mut result = vec![];
    let mut pending = vec![root];
    while let Some(e) = pending.pop() {
        if let Some(c) = world.get::<Children>(e) {
            for child in c.iter() {
                result.push(child);
                pending.push(child);
            }
        }
    }
    result
}

/// Called after imported mount markers exist, before the initial scene snapshot is captured.
pub fn build_rig(world: &mut World, root: Entity, profile: RobotProfile) {
    let nodes = descendants(world, root);
    if profile == RobotProfile::WheelLeg {
        let rig = crate::wheel_leg_urdf::build(world, root, &nodes);
        world.entity_mut(root).insert((profile, rig));
        return;
    }
    let find = |name: &str| {
        nodes
            .iter()
            .copied()
            .find(|e| world.get::<Name>(*e).is_some_and(|n| n.as_str() == name))
            .expect("required GLB node")
    };
    let vehicle = find("VEHICLE");
    let gimbal = find("GIMBAL");
    let chassis = find("CHASSIS");
    let base = find("BASE");
    let main = find("GIMBAL_MAIN");
    let is_self = world.get::<Controlled>(root).is_some();
    let identity = *world.get::<RobotIdentity>(root).unwrap();
    let member = RobotMember {
        root,
        id: identity.id,
    };
    let body_mount = world
        .spawn((
            Name::new("BodyLiftMount"),
            Transform::default(),
            ChildOf(root),
            member,
        ))
        .id();
    world.entity_mut(vehicle).insert(ChildOf(body_mount));
    // HERO's baked 180-degree Z rotation is kept under its new mount, not applied twice.
    let fold_mount = world
        .spawn((
            Name::new("FoldMount"),
            Transform::default(),
            ChildOf(vehicle),
            member,
        ))
        .id();
    world.entity_mut(gimbal).insert(ChildOf(fold_mount));
    if profile.compact() {
        let wrapper = world
            .spawn((
                Name::new("LowChassisMount"),
                Transform::from_scale(Vec3::new(1., 0.45, 1.)),
                ChildOf(base),
                member,
            ))
            .id();
        world.entity_mut(chassis).insert(ChildOf(wrapper));
        for entity in &nodes {
            if world
                .get::<Name>(*entity)
                .is_some_and(|n| n.contains("ARMOR_ROOT"))
            {
                world.get_mut::<Transform>(*entity).unwrap().translation.y = 0.015;
            }
        }
        world.get_mut::<Transform>(gimbal).unwrap().scale = Vec3::splat(0.32);
        // Compact mount keeps the optical axis and bore above the unscaled armor plates.
        // Transform the structural mesh and matching attachment coordinates together.
        {
            let mut t = world.get_mut::<Transform>(main).unwrap();
            t.scale.y *= 0.8;
            t.translation.y += 0.08;
        }
        for name in ["CAM_DIRECTION", "SHOT_DIRECTION"] {
            if let Some(e) = nodes
                .iter()
                .copied()
                .find(|e| world.get::<Name>(*e).is_some_and(|n| n.as_str() == name))
            {
                let mut t = world.get_mut::<Transform>(e).unwrap();
                t.translation.y = t.translation.y * 0.8 + 0.08;
            }
        }
        world
            .get_mut::<Transform>(fold_mount)
            .unwrap()
            .translation
            .y = if profile == RobotProfile::Folding {
            0.18
        } else {
            0.025
        };
    }
    // A narrow wheel-contact proxy supports the root. It never blocks projectiles.
    let bottom = -profile.root_height() + 0.005;
    world.entity_mut(root).insert(Collider::compound(vec![(
        Vec3::new(0., bottom + 0.02, 0.),
        Quat::IDENTITY,
        Collider::cylinder(0.245, 0.04),
    )]));
    let old: Vec<_> = nodes
        .iter()
        .copied()
        .filter(|e| {
            world
                .get::<Name>(*e)
                .is_some_and(|n| n.as_str() == "CombatBodyCore")
        })
        .collect();
    for e in old {
        world.despawn(e);
    }
    // Parameterized structural proxies fit inside the imported silhouette. Keep the
    // narrow core separate from four corner wheel volumes so the broad chassis bounds
    // cannot shield armor. High-poly visual triangle meshes are not simulated at 1 kHz.
    let layers = GameLayer::vehicle_armor_collision_layers(is_self);
    let low = profile.compact();
    let heavy = profile == RobotProfile::HeavyTarget;
    let core = if heavy {
        Vec3::new(0.32, 0.317, 0.38)
    } else {
        Vec3::new(0.32, 0.231, 0.32)
    };
    let chassis_parent = if low {
        world.get::<ChildOf>(chassis).unwrap().parent()
    } else {
        base
    };
    let proxy = world
        .spawn((
            Name::new("ChassisCoreProxy"),
            Transform::from_xyz(0., if heavy { 0.0475 } else { -0.0945 }, 0.),
            ChildOf(chassis_parent),
            member,
            Collider::cuboid(core.x, core.y, core.z),
            ColliderDensity(0.),
            layers,
            RigPart,
        ))
        .id();
    // HERO's asset frame is inverted in Y; its VEHICLE parent already applies the bake.
    let _ = proxy;
    let wx = if heavy { 0.23 } else { 0.21 };
    let wz = if heavy { 0.235 } else { 0.17 };
    for x in [-wx, wx] {
        for z in [-wz, wz] {
            world.spawn((
                Name::new("WheelVolumeProxy"),
                Transform::from_xyz(x, if heavy { 0.13 } else { -0.13 }, z),
                ChildOf(chassis_parent),
                member,
                Collider::cuboid(0.09, 0.16, 0.13),
                ColliderDensity(0.),
                layers,
                RigPart,
            ));
        }
    }
    let (center, size) = if heavy {
        (Vec3::new(0.07, -0.227, 0.034), Vec3::new(0.32, 0.377, 0.39))
    } else {
        (
            Vec3::new(0.034, 0.121, 0.029),
            Vec3::new(0.33, 0.208, 0.364),
        )
    };
    let mut proxy_pose = Transform::from_translation(center);
    if low {
        proxy_pose.translation.y = proxy_pose.translation.y * 0.8 + 0.08;
        proxy_pose.scale.y = 0.8;
    }
    world.spawn((
        Name::new("UpperBodyProxy"),
        proxy_pose,
        ChildOf(gimbal),
        member,
        Collider::cuboid(size.x, size.y, size.z),
        ColliderDensity(0.),
        layers,
        RigPart,
    ));
    // Mark imported armor colliders too once they are constructed; owner lookup covers them.
    let legs = vec![];
    let feet = vec![];
    let leg_frame = None;
    let fold_stem = if profile == RobotProfile::Folding {
        let mesh = world
            .resource_mut::<Assets<Mesh>>()
            .add(Cuboid::new(0.04, 1., 0.04));
        let mat = world
            .resource_mut::<Assets<StandardMaterial>>()
            .add(Color::srgb(0.35, 0.37, 0.4));
        Some(
            world
                .spawn((
                    Name::new("FoldingSupportLink"),
                    Mesh3d(mesh),
                    MeshMaterial3d(mat),
                    Transform::from_xyz(0., 0.1, 0.).with_scale(Vec3::new(1., 0.2, 1.)),
                    ChildOf(vehicle),
                    member,
                    Collider::cuboid(0.04, 1., 0.04),
                    ColliderDensity(0.),
                    GameLayer::vehicle_armor_collision_layers(is_self),
                    RigPart,
                ))
                .id(),
        )
    } else {
        None
    };
    let parked_rotation = world.get::<Transform>(gimbal).unwrap().rotation;
    world.entity_mut(root).insert((
        profile,
        RobotRig {
            urdf: None,
            folding: profile == RobotProfile::Folding,
            body_mount,
            base,
            leg_frame,
            fold_mount,
            gimbal,
            parked_rotation,
            fold_stem,
            legs,
            feet,
            fold: 0.,
            requested_fold: false,
            stable_s: 0.1,
            lift_m: 0.,
            requested_lift_m: 0.,
            lift_velocity: 0.,
            blocked: None,
        },
    ));
}

pub fn set_mounts(world: &mut World, profile: RobotProfile, rig: &RobotRig, fold: f32, lift: f32) {
    if let Some(frame) = rig.leg_frame {
        let rotation = world.get::<Transform>(rig.base).unwrap().rotation;
        if let Some(mut t) = world.get_mut::<Transform>(frame) {
            t.rotation = rotation;
        }
    }
    if let Some(mut t) = world.get_mut::<Transform>(rig.body_mount) {
        t.translation.y = lift;
    }
    if profile == RobotProfile::Folding {
        if let Some(mut t) = world.get_mut::<Transform>(rig.fold_mount) {
            t.translation = Vec3::Y * (0.18 + (0.025 - 0.18) * fold);
            t.rotation = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2 * fold);
        }
        if let Some(stem) = rig.fold_stem {
            if let Some(mut t) = world.get_mut::<Transform>(stem) {
                t.scale.y = 0.2 - fold * 0.18;
                t.translation.y = t.scale.y * 0.5;
            }
        }
    }
    if let Some(native) = &rig.urdf {
        crate::wheel_leg_urdf::set_lift(world, native, lift);
    }
    for (leg, foot) in rig.legs.iter().zip(&rig.feet) {
        if let Some(mut t) = world.get_mut::<Transform>(*leg) {
            t.scale.y = 0.16 + lift;
            t.translation.y = -0.13 + lift * 0.5;
        }
        if let Some(mut t) = world.get_mut::<Transform>(*foot) {
            t.translation.y = -0.21;
        }
    }
}

/// Exact structural/armor colliders against environment and the other robot. Root support
/// proxy is excluded from obstruction queries; it is only a wheel contact approximation.
pub fn check_pose(world: &mut World, root: Entity, pose: Transform) -> Result<(), String> {
    if let Some(status) = world.get::<crate::wheel_leg_urdf::UrdfStatus>(root) {
        if let Some(error) = &status.error {
            return Err(error.clone());
        }
    }
    let current = physical_matrix(world, root);
    let delta = pose.to_matrix() * current.inverse();
    let env_members = GameLayer::environment_collision_layers().memberships;
    let parts: Vec<_> = world
        .query::<(
            Entity,
            &Collider,
            Option<&RobotMember>,
            Option<&RobotIdentity>,
            Option<&CollisionLayers>,
        )>()
        .iter(world)
        .filter(|(_, _, m, id, _)| id.is_none() && m.is_some_and(|m| m.root == root))
        .map(|(e, c, _, _, _)| (c.clone(), delta * physical_matrix(world, e)))
        .collect();
    let obstacles: Vec<_> = world
        .query::<(
            Entity,
            &Collider,
            Option<&RobotMember>,
            Option<&RobotIdentity>,
            Option<&CollisionLayers>,
        )>()
        .iter(world)
        .filter(|(_, _, m, id, l)| {
            id.is_none()
                && (m.is_some_and(|m| m.root != root)
                    || l.is_some_and(|l| l.memberships == env_members))
        })
        .map(|(e, c, _, _, _)| {
            let (s, q, p) = physical_matrix(world, e).to_scale_rotation_translation();
            let mut c = c.clone();
            c.set_scale(s.abs(), 10);
            let aabb = c.aabb(p, q);
            (e, c, p, q, aabb)
        })
        .collect();
    for (shape, matrix) in parts {
        let (scale, r, p) = matrix.to_scale_rotation_translation();
        let mut shape = shape;
        shape.set_scale(scale.abs(), 10);
        let bounds = shape.aabb(p, r);
        for (entity, other, x, q, aabb) in &obstacles {
            if !bounds.intersects(aabb) {
                continue;
            }
            let c = contact_query::contact(&shape, p, r, other, *x, *q, 0.)
                .map_err(|_| "unsupported geometry collision query")?;
            if c.is_some_and(|c| c.penetration > 0.001) {
                return Err(format!(
                    "blocked by {:?}",
                    world
                        .get::<Name>(*entity)
                        .map(|n| n.as_str())
                        .unwrap_or("collider")
                ));
            }
        }
    }
    Ok(())
}

pub fn request_fold(world: &mut World, root: Entity, folded: bool) -> Result<(), String> {
    if world
        .get::<RobotCombatState>(root)
        .is_some_and(|s| s.life.status != LifeStatus::Alive)
    {
        return Err("dead robot cannot actuate".into());
    }
    if world.get::<RobotProfile>(root) != Some(&RobotProfile::Folding) {
        return Err("robot has no folding mechanism".into());
    }
    let mut rig = world.get::<RobotRig>(root).unwrap().clone();
    if folded && rig.fold <= 1e-6 {
        rig.parked_rotation = world.get::<Transform>(rig.gimbal).unwrap().rotation;
    }
    let pose = physical_matrix(world, root).to_scale_rotation_translation();
    let pose = Transform::from_translation(pose.2).with_rotation(pose.1);
    let old_gimbal = *world.get::<Transform>(rig.gimbal).unwrap();
    world.get_mut::<Transform>(rig.gimbal).unwrap().rotation = rig.parked_rotation;
    // Validate the entire remaining sweep, including the fully extended pose, before starting.
    for i in 1..=40 {
        let f = rig.fold + ((if folded { 1. } else { 0. }) - rig.fold) * i as f32 / 40.;
        set_mounts(world, RobotProfile::Folding, &rig, f, rig.lift_m);
        if let Err(error) = check_pose(world, root, pose) {
            set_mounts(world, RobotProfile::Folding, &rig, rig.fold, rig.lift_m);
            world.entity_mut(rig.gimbal).insert(old_gimbal);
            world.get_mut::<RobotRig>(root).unwrap().blocked = Some(error.clone());
            return Err(error);
        }
    }
    set_mounts(world, RobotProfile::Folding, &rig, rig.fold, rig.lift_m);
    let mut r = world.get_mut::<RobotRig>(root).unwrap();
    r.parked_rotation = rig.parked_rotation;
    r.requested_fold = folded;
    r.stable_s = 0.;
    r.blocked = None;
    Ok(())
}

pub fn update_rigs(world: &mut World) {
    let dt = world.resource::<Time<Fixed>>().delta_secs();
    let roots: Vec<_> = world
        .query::<(Entity, &RobotProfile, &RobotRig, &RobotCombatState)>()
        .iter(world)
        .filter(|(_, _, _, s)| s.life.status == LifeStatus::Alive)
        .map(|(e, p, r, _)| (e, *p, r.clone()))
        .collect();
    for (root, profile, mut rig) in roots {
        if crate::wheel_leg_traversal::owns_root(world, root) {
            continue;
        }
        if let Some(frame) = rig.leg_frame {
            let rotation = world.get::<Transform>(rig.base).unwrap().rotation;
            if let Some(mut t) = world.get_mut::<Transform>(frame) {
                t.rotation = rotation;
            }
        }
        let target = if rig.requested_fold { 1. } else { 0. };
        let fold = rig.fold + (target - rig.fold).clamp(-dt / 0.8, dt / 0.8);
        let lift = rig.lift_m
            + (rig.requested_lift_m.clamp(0., 0.2) - rig.lift_m).clamp(
                -dt * crate::wheel_leg_traversal::LIFT_SPEED_M_S,
                dt * crate::wheel_leg_traversal::LIFT_SPEED_M_S,
            );
        let changed = (fold - rig.fold).abs() > 1e-7 || (lift - rig.lift_m).abs() > 1e-7;
        if changed {
            set_mounts(world, profile, &rig, fold, lift);
            let (_, r, p) = physical_matrix(world, root).to_scale_rotation_translation();
            if let Err(reason) =
                check_pose(world, root, Transform::from_translation(p).with_rotation(r))
            {
                set_mounts(world, profile, &rig, rig.fold, rig.lift_m);
                rig.blocked = Some(reason);
                rig.lift_velocity = 0.;
            } else {
                rig.lift_velocity = (lift - rig.lift_m) / dt;
                rig.fold = fold;
                rig.lift_m = lift;
                rig.blocked = None;
            }
            rig.stable_s = 0.;
        } else {
            rig.stable_s += dt;
            rig.lift_velocity = 0.;
        }
        world.entity_mut(root).insert(rig);
    }
}

fn operator_commands(world: &mut World) {
    let Some(keys) = world.get_resource::<ButtonInput<KeyCode>>() else {
        return;
    };
    let fold = keys.just_pressed(KeyCode::KeyZ);
    let lift = keys.just_pressed(KeyCode::KeyX);
    let remote = keys.pressed(KeyCode::AltLeft) || keys.pressed(KeyCode::AltRight);
    if !fold && !lift {
        return;
    }
    let roots: Vec<_> = world
        .query::<(
            Entity,
            &RobotProfile,
            Has<Controlled>,
            Has<crate::components::ActiveSlapper>,
            &RobotCombatState,
        )>()
        .iter(world)
        .filter(|(_, _, c, a, s)| {
            s.life.status == LifeStatus::Alive && if remote { *a } else { *c }
        })
        .map(|(e, p, _, _, _)| (e, *p))
        .collect();
    for (root, p) in roots {
        if fold && p == RobotProfile::Folding {
            let desired = !world.get::<RobotRig>(root).unwrap().requested_fold;
            if let Err(reason) = request_fold(world, root, desired) {
                warn!("fold request rejected: {reason}");
            }
        }
        if lift && p == RobotProfile::WheelLeg {
            let mut r = world.get_mut::<RobotRig>(root).unwrap();
            r.requested_lift_m = if r.requested_lift_m > 0.1 { 0. } else { 0.2 };
        }
    }
}

pub fn reset_rigs(world: &mut World) {
    crate::wheel_leg_traversal::reset(world);
    let rigs: Vec<_> = world
        .query::<(Entity, &RobotProfile, &RobotRig)>()
        .iter(world)
        .map(|(e, p, r)| (e, *p, r.clone()))
        .collect();
    for (e, p, mut r) in rigs {
        r.fold = 0.;
        r.requested_fold = false;
        r.stable_s = 0.1;
        r.lift_m = 0.;
        r.requested_lift_m = 0.;
        r.lift_velocity = 0.;
        r.blocked = None;
        set_mounts(world, p, &r, 0., 0.);
        world.entity_mut(e).insert(r);
    }
}

/// Extra plate velocity from a body-lift mount. Root translation/angular velocity is added
/// by the normal pre-solver sampler and must not be counted twice here.
pub fn plate_articulation_velocity(world: &World, root: Entity) -> Vec3 {
    world.get::<RobotRig>(root).map_or(Vec3::ZERO, |r| {
        let rotation = world.get::<Rotation>(root).map_or(Quat::IDENTITY, |r| r.0);
        rotation * Vec3::Y * r.lift_velocity
    })
}

/// Visible mesh bounds use every vertex and the full hierarchy, independently of colliders.
pub fn geometry_report(world: &mut World, root: Entity) -> serde_json::Value {
    let nodes = descendants(world, root);
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    for e in nodes {
        let Some(mesh) = world.get::<Mesh3d>(e) else {
            continue;
        };
        let mut parent = Some(e);
        let mut hidden = false;
        while let Some(p) = parent {
            if world.get::<Visibility>(p) == Some(&Visibility::Hidden) {
                hidden = true;
                break;
            }
            parent = world.get::<ChildOf>(p).map(|p| p.parent());
        }
        if hidden {
            continue;
        }
        let Some(mesh) = world.resource::<Assets<Mesh>>().get(&mesh.0) else {
            continue;
        };
        let Some(bevy::mesh::VertexAttributeValues::Float32x3(points)) =
            mesh.attribute(Mesh::ATTRIBUTE_POSITION)
        else {
            continue;
        };
        let matrix = physical_matrix(world, e);
        for v in points {
            let p = matrix.transform_point3(Vec3::from_array(*v));
            min = min.min(p);
            max = max.max(p);
        }
    }
    serde_json::json!({"profile":world.get::<RobotProfile>(root),"visible_min_m":min.to_array(),"visible_max_m":max.to_array(),
        "visible_height_m":max.y-min.y,"top_above_floor_m":max.y-(physical_matrix(world,root).transform_point3(Vec3::ZERO).y-world.get::<RobotProfile>(root).unwrap().root_height())})
}
