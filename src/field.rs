//! RMUL 2027 shooting field. Bevy metres: +Y up, X along the 12 m side.
//! Handbook dimensions and documented geometric approximations share rendering/physics poses.
use crate::components::GameLayer;
use crate::robomaster::prelude::Team;
use avian3d::prelude::*;
use bevy::prelude::*;
use serde::Deserialize;

#[derive(Deserialize, Reflect, Clone, Copy, Debug, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MapKind {
    #[default]
    Legacy,
    Rmul2027ThreeVsThree,
}
#[derive(Deserialize, Reflect, Clone, Debug, Default, PartialEq, Eq)]
#[serde(default, deny_unknown_fields)]
pub struct SceneConfig {
    pub map: MapKind,
}
impl SceneConfig {
    pub fn is_league(&self) -> bool {
        self.map == MapKind::Rmul2027ThreeVsThree
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegionKind {
    Highland,
    Tunnel(u8),
    Other,
}
#[derive(Component, Clone, Copy)]
pub struct FieldRegion {
    pub id: u8,
    pub kind: RegionKind,
    pub team: Option<Team>,
    pub min: Vec3,
    pub max: Vec3,
}
impl FieldRegion {
    pub fn contains(&self, p: Vec3) -> bool {
        p.cmpge(self.min).all() && p.cmple(self.max).all()
    }
}

fn mesh_body(
    world: &mut World,
    name: &str,
    mesh: Mesh,
    collider: Option<Collider>,
    transform: Transform,
    color: Color,
) {
    let mesh = world.resource_mut::<Assets<Mesh>>().add(mesh);
    let material = world
        .resource_mut::<Assets<StandardMaterial>>()
        .add(StandardMaterial {
            base_color: color,
            perceptual_roughness: 0.85,
            ..default()
        });
    let mut e = world.spawn((
        Name::new(name.to_owned()),
        Mesh3d(mesh),
        MeshMaterial3d(material),
        transform,
    ));
    if let Some(collider) = collider {
        e.insert((
            RigidBody::Static,
            collider,
            Friction::new(0.5),
            GameLayer::environment_collision_layers(),
        ));
    }
}
fn block(world: &mut World, name: &str, center: Vec3, size: Vec3, color: Color, solid: bool) {
    // Layout coordinates use positive Z towards the red wall; Bevy world -Z is map north.
    let center = Vec3::new(center.x, center.y, -center.z);
    mesh_body(
        world,
        name,
        Cuboid::from_size(size).into(),
        solid.then(|| Collider::cuboid(size.x, size.y, size.z)),
        Transform::from_translation(center),
        color,
    );
}
// All coordinates below use the red half's frame. Blue rotates exactly 180 degrees.
// Fig. 3-8: the 4.06 m deck is a flat 0.2 m platform, not a 1.2 m approach ramp.
const DECK_FRONT: f32 = -1.0;
const DECK_BACK: f32 = DECK_FRONT + 4.06;
const OPENING_BACK: f32 = DECK_BACK + 0.8;
const BRIDGE_TOP: f32 = 0.27;
const BRIDGE_THICKNESS: f32 = 0.02;
const BRIDGE_WIDTH: f32 = 0.35;

fn bridge_transition(world: &mut World, sign: f32) {
    // Unlabelled 140 mm overlap estimated from fig. 3-8. This short transition joins
    // the 200 mm deck to the 270 mm bridge. It does not cover the 800 mm clear opening.
    let z0 = DECK_BACK - 0.14;
    let z1 = DECK_BACK;
    let mut vertices = vec![
        Vec3::new(-BRIDGE_WIDTH / 2., 0.18, z0),
        Vec3::new(BRIDGE_WIDTH / 2., 0.18, z0),
        Vec3::new(-BRIDGE_WIDTH / 2., 0.25, z1),
        Vec3::new(BRIDGE_WIDTH / 2., 0.25, z1),
        Vec3::new(-BRIDGE_WIDTH / 2., 0.20, z0),
        Vec3::new(BRIDGE_WIDTH / 2., 0.20, z0),
        Vec3::new(-BRIDGE_WIDTH / 2., 0.27, z1),
        Vec3::new(BRIDGE_WIDTH / 2., 0.27, z1),
    ];
    // Outward winding on each of the six faces, with independent flat vertex normals.
    let mut indices = vec![
        4, 6, 5, 5, 6, 7, 0, 1, 2, 1, 3, 2, 0, 4, 1, 1, 4, 5, 2, 3, 6, 3, 7, 6, 0, 2, 4, 2, 6, 4,
        1, 5, 3, 3, 5, 7,
    ];
    // Mirror the layout into world north and reverse winding to retain outward faces.
    for p in &mut vertices {
        p.z = -p.z;
    }
    for triangle in indices.chunks_exact_mut(3) {
        triangle.swap(1, 2);
    }
    let mesh = Mesh::new(
        bevy::mesh::PrimitiveTopology::TriangleList,
        bevy::asset::RenderAssetUsages::default(),
    )
    .with_inserted_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vertices.iter().map(|p| p.to_array()).collect::<Vec<_>>(),
    )
    .with_inserted_indices(bevy::mesh::Indices::U32(indices))
    .with_duplicated_vertices()
    .with_computed_flat_normals();
    mesh_body(
        world,
        "BridgeTransition",
        mesh,
        Some(Collider::convex_hull(vertices).expect("valid bridge transition")),
        Transform::from_xyz(-2.5 * sign, 0., 0.).with_rotation(Quat::from_rotation_y(
            if sign > 0. { 0. } else { std::f32::consts::PI },
        )),
        Color::srgb(0.72, 0.74, 0.75),
    );
}
fn region(
    world: &mut World,
    id: u8,
    kind: RegionKind,
    team: Option<Team>,
    center: Vec3,
    size: Vec2,
) {
    let center = Vec3::new(center.x, center.y, -center.z);
    world.spawn((
        Name::new(format!("InteractionRegion{id}")),
        FieldRegion {
            id,
            kind,
            team,
            min: center - Vec3::new(size.x * 0.5, 0.05, size.y * 0.5),
            max: center + Vec3::new(size.x * 0.5, 0.05, size.y * 0.5),
        },
    ));
}
pub fn spawn(commands: &mut Commands) {
    commands.queue(|world: &mut World| {
        block(
            world,
            "LeagueFloor",
            Vec3::new(0., -0.05, 0.),
            Vec3::new(12., 0.1, 8.),
            Color::srgb(0.48, 0.48, 0.48),
            true,
        );
        for x in [-6.05, 6.05] {
            block(
                world,
                "Boundary",
                Vec3::new(x, 0.2, 0.),
                Vec3::new(0.1, 0.4, 8.2),
                Color::srgb(0.08, 0.08, 0.08),
                true,
            );
        }
        for z in [-4.05, 4.05] {
            block(
                world,
                "Boundary",
                Vec3::new(0., 0.2, z),
                Vec3::new(12., 0.4, 0.1),
                Color::srgb(0.08, 0.08, 0.08),
                true,
            );
        }
        block(
            world,
            "CentralControl",
            Vec3::new(0., 0.001, 0.),
            Vec3::new(3., 0.002, 3.),
            Color::srgb(0.32, 0.54, 0.66),
            false,
        );
        region(
            world,
            20,
            RegionKind::Other,
            None,
            Vec3::ZERO,
            Vec2::splat(3.),
        );
        for (sign, team, offset) in [(1., Team::Red, 0u8), (-1., Team::Blue, 10u8)] {
            let tint = if sign > 0. {
                Color::srgb(0.5, 0.16, 0.16)
            } else {
                Color::srgb(0.14, 0.22, 0.5)
            };
            block(
                world,
                "Supply",
                Vec3::new(-4.75 * sign, 0.002, 3. * sign),
                Vec3::new(2.5, 0.003, 2.),
                tint,
                false,
            );
            block(
                world,
                "Workbench",
                Vec3::new(-5.625 * sign, 0.2, 3.775 * sign),
                Vec3::new(0.75, 0.4, 0.45),
                Color::srgb(0.55, 0.62, 0.65),
                true,
            );
            block(
                world,
                "HighlandDeck",
                Vec3::new(-2.5 * sign, 0.1, (DECK_FRONT + DECK_BACK) * 0.5 * sign),
                Vec3::new(2., 0.2, 4.06),
                Color::srgb(0.43, 0.49, 0.53),
                true,
            );
            block(
                world,
                "HighlandCurb",
                Vec3::new(-1.575 * sign, 0.3, (DECK_FRONT + 0.9 + 0.9) * sign),
                Vec3::new(0.15, 0.2, 1.8),
                Color::srgb(0.58, 0.62, 0.65),
                true,
            );
            block(
                world,
                "HighlandControl",
                Vec3::new(-2.5 * sign, 0.202, 0.8 * sign),
                Vec3::new(1.2, 0.003, 1.2),
                Color::srgb(0.3, 0.53, 0.66),
                false,
            );
            region(
                world,
                offset + 1,
                RegionKind::Highland,
                Some(team),
                Vec3::new(-2.5 * sign, 0.2, 0.8 * sign),
                Vec2::splat(1.2),
            );
            // The tunnel is an OPEN slot. Only the 350 mm central metal bridge covers it.
            // The remaining 140 mm against the boundary is an estimated support ledge.
            block(
                world,
                "TunnelAbutment",
                Vec3::new(-2.5 * sign, 0.135, (OPENING_BACK + 4.) * 0.5 * sign),
                Vec3::new(2., 0.27, 4. - OPENING_BACK),
                Color::srgb(0.43, 0.49, 0.53),
                true,
            );
            // 70 mm overlap on the outer ledge is estimated; the labelled opening stays 800 mm.
            block(
                world,
                "BridgeStrip",
                Vec3::new(
                    -2.5 * sign,
                    BRIDGE_TOP - BRIDGE_THICKNESS * 0.5,
                    (DECK_BACK + OPENING_BACK + 0.07) * 0.5 * sign,
                ),
                Vec3::new(
                    BRIDGE_WIDTH,
                    BRIDGE_THICKNESS,
                    OPENING_BACK + 0.07 - DECK_BACK,
                ),
                Color::srgb(0.72, 0.74, 0.75),
                true,
            );
            bridge_transition(world, sign);
            region(
                world,
                offset + 2,
                RegionKind::Tunnel(0),
                Some(team),
                Vec3::new(-4.75 * sign, 0., 3. * sign),
                Vec2::new(2.5, 2.),
            );
            region(
                world,
                offset + 3,
                RegionKind::Tunnel(1),
                Some(team),
                Vec3::new(-1.9125 * sign, 0., (DECK_BACK + OPENING_BACK) * 0.5 * sign),
                Vec2::new(0.825, 0.8),
            );
        }
        info!("RMUL 2027 field loaded: 12 x 8 m; geometric approximations: docs/RMUL_2027.md");
    });
}
