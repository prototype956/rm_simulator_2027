//! RMUL-only state and physical gates; legacy training behavior is explicitly opt-out.
use super::*;
use crate::field::{FieldRegion, RegionKind};
use avian3d::prelude::*;
use bevy::prelude::*;
use std::collections::HashMap;
use std::time::Duration;

#[derive(Reflect, Clone, Debug, Default)]
pub struct LeagueState {
    pub defense_until: Option<Duration>,
    pub cooling_until: Option<Duration>,
    last_card: Option<u8>,
    reported_defense: bool,
    reported_cooling: bool,
    tunnel_start: Option<(u8, Duration)>,
}
impl LeagueState {
    pub fn defense_active(&self, now: Duration) -> bool {
        self.defense_until.is_some_and(|end| now < end)
    }
    pub fn cooling_active(&self, now: Duration) -> bool {
        self.cooling_until.is_some_and(|end| now < end)
    }
    pub fn defense_left(&self, now: Duration) -> f32 {
        self.defense_until
            .map_or(0., |t| t.saturating_sub(now).as_secs_f32())
    }
    pub fn cooling_left(&self, now: Duration) -> f32 {
        self.cooling_until
            .map_or(0., |t| t.saturating_sub(now).as_secs_f32())
    }
    /// Current physical RFID region and elapsed tunnel entry state, for privileged training only.
    pub fn current_region(&self) -> Option<u8> { self.last_card }
    pub fn tunnel_progress(&self, now: Duration) -> Option<(u8, f64)> {
        self.tunnel_start.map(|(endpoint, at)| (endpoint, now.saturating_sub(at).as_secs_f64()))
    }
    fn read_card(
        &mut self,
        card: Option<FieldRegion>,
        team: crate::robomaster::prelude::Team,
        now: Duration,
    ) {
        if card.is_some_and(|r| r.kind == RegionKind::Highland) {
            self.defense_until = Some(now + Duration::from_secs(2));
        }
        let Some(card) = card else {
            self.last_card = None;
            return;
        };
        if self.last_card == Some(card.id) {
            // Repeated RFID reads keep the departure deadline fresh, but never retrigger a buff.
            if let RegionKind::Tunnel(endpoint) = card.kind {
                if card.team == Some(team) {
                    self.tunnel_start = Some((endpoint, now));
                }
            }
            return;
        }
        self.last_card = Some(card.id);
        if let RegionKind::Tunnel(endpoint) = card.kind {
            if card.team == Some(team) {
                if self.tunnel_start.is_some_and(|(start, at)| {
                    start != endpoint && now.saturating_sub(at) <= Duration::from_secs(3)
                }) {
                    self.cooling_until = Some(now + Duration::from_secs(60));
                }
                self.tunnel_start = Some((endpoint, now));
                return;
            }
        }
        self.tunnel_start = None;
    }
}
/// Physics-step snapshot before impulses; includes children rotated by scripted chassis motion.
#[derive(Clone, Copy)]
pub struct BodySample {
    pub position: Vec3,
    pub linear: Vec3,
    pub angular: Vec3,
}
#[derive(Clone, Copy)]
pub struct ArticulationSample {
    pub at: Duration,
    pub center: Vec3,
    pub rotation: Quat,
}
#[derive(Resource, Default)]
pub struct ImpactSamples {
    pub bodies: HashMap<Entity, BodySample>,
    pub plates: HashMap<Entity, PlateSample>,
    pub last_detected: HashMap<Entity, Duration>,
    pub articulation: HashMap<Entity, ArticulationSample>,
}
#[derive(Clone, Copy)]
pub struct PlateSample {
    pub center: Vec3,
    pub normal: Vec3,
    pub velocity: Vec3,
    pub angular: Vec3,
}

pub fn update_regions(world: &mut World) {
    let now = world.resource::<Time<Fixed>>().elapsed();
    let regions: Vec<_> = world.query::<&FieldRegion>().iter(world).copied().collect();
    let native: Vec<_> = world
        .query::<(Entity, &crate::robots::RobotRig)>()
        .iter(world)
        .filter(|(_, r)| r.urdf.is_some())
        .map(|(e, r)| (e, crate::wheel_leg_urdf::interaction_point(world, r)))
        .collect();
    let mut feet = HashMap::new();
    for (e, p) in native {
        let supported = crate::wheel_leg_traversal::floor_height(world, p, p.y + 0.02)
            .is_some_and(|h| (p.y - h).abs() < 0.025);
        feet.insert(e, if supported { Some(p) } else { None });
    }
    let mut logs = Vec::new();
    for (entity, id, pos, rot, profile, mut state) in world
        .query::<(
            Entity,
            &RobotIdentity,
            &Position,
            &Rotation,
            Option<&crate::robots::RobotProfile>,
            &mut RobotCombatState,
        )>()
        .iter_mut(world)
    {
        if state.ruleset != RuleSet::Rmul2027 {
            continue;
        }
        if state.life.status != LifeStatus::Alive {
            state.league = default();
            continue;
        }
        let foot = pos.0
            + rot.0
                * Vec3::new(
                    0.,
                    profile.map_or(-0.231298, |p| -p.root_height() + 0.005),
                    0.,
                );
        let native_foot = feet.get(&entity);
        let foot = native_foot.and_then(|p| *p).unwrap_or(foot);
        let card = regions
            .iter()
            .filter(|_| !matches!(native_foot, Some(None)))
            .filter(|r| r.contains(foot))
            .min_by_key(|r| r.id)
            .copied();
        let before = (state.league.reported_defense, state.league.reported_cooling);
        let previous_cooling_until = state.league.cooling_until;
        state.league.read_card(card, id.team, now);
        let after = (
            state.league.defense_active(now),
            state.league.cooling_active(now),
        );
        state.league.reported_defense = after.0;
        state.league.reported_cooling = after.1;
        if before != after || previous_cooling_until != state.league.cooling_until {
            logs.push(format!(
                "buff robot={} defense={} cooling={} cooldown_until={:?}",
                id.id.0, after.0, after.1, state.league.cooling_until
            ));
        }
    }
    for log in logs {
        info!("{log}");
        super::reset::record_event(world, now, log);
    }
}

/// >12 m/s, signed inward. Back faces and glancing/slow contacts fail.
pub fn accepts_impact(
    normal_speed: f32,
    last: Option<Duration>,
    now: Duration,
) -> Result<(), &'static str> {
    if !normal_speed.is_finite() || normal_speed <= 12. {
        return Err("back_face_or_normal_speed");
    }
    if last.is_some_and(|last| now.saturating_sub(last) < Duration::from_millis(50)) {
        return Err("armor_detection_interval");
    }
    Ok(())
}
pub fn clear_detection_history(world: &mut World) {
    if let Some(mut samples) = world.get_resource_mut::<ImpactSamples>() {
        *samples = default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robomaster::prelude::Team;
    fn card(id: u8, kind: RegionKind, team: Team) -> FieldRegion {
        FieldRegion {
            id,
            kind,
            team: Some(team),
            min: Vec3::ZERO,
            max: Vec3::ONE,
        }
    }
    #[test]
    fn physical_detection_boundaries() {
        let t = Duration::from_secs(1);
        for speed in [-25., 0., 11.99, 12., f32::NAN] {
            assert!(accepts_impact(speed, None, t).is_err());
        }
        assert!(accepts_impact(12.01, None, t).is_ok());
        assert!(accepts_impact(25., Some(t), t + Duration::from_millis(49)).is_err());
        assert!(accepts_impact(25., Some(t), t + Duration::from_millis(50)).is_ok());
    }
    #[test]
    fn buffs_timeout_interruption_and_refresh() {
        let mut s = LeagueState::default();
        let t = Duration::from_secs(1);
        s.read_card(
            Some(card(1, RegionKind::Highland, Team::Blue)),
            Team::Red,
            t,
        );
        s.read_card(None, Team::Red, t);
        assert!(s.defense_active(t + Duration::from_millis(1999)));
        assert!(!s.defense_active(t + Duration::from_secs(2)));
        let a = Some(card(2, RegionKind::Tunnel(0), Team::Red));
        let b = Some(card(3, RegionKind::Tunnel(1), Team::Red));
        s.read_card(a, Team::Red, t);
        s.read_card(b, Team::Red, t + Duration::from_secs(3));
        assert!(s.cooling_active(t + Duration::from_secs(62)));
        assert!(!s.cooling_active(t + Duration::from_secs(63)));
        s = default();
        s.read_card(a, Team::Red, t);
        s.read_card(Some(card(20, RegionKind::Other, Team::Red)), Team::Red, t);
        s.read_card(b, Team::Red, t);
        assert!(!s.cooling_active(t));
        s = default();
        s.read_card(a, Team::Red, t);
        s.read_card(b, Team::Red, t + Duration::from_millis(3001));
        assert!(!s.cooling_active(t + Duration::from_secs(4)));
        s = default();
        s.read_card(a, Team::Blue, t);
        s.read_card(b, Team::Blue, t);
        assert!(!s.cooling_active(t));
        s = default();
        s.read_card(a, Team::Red, t);
        s.read_card(b, Team::Red, t);
        s.read_card(a, Team::Red, t + Duration::from_secs(2));
        assert_eq!(s.cooling_until, Some(t + Duration::from_secs(62)));
    }
    #[test]
    fn continuous_reads_do_not_repeat_buff_but_keep_start_fresh() {
        let mut s = LeagueState::default();
        let t = Duration::from_secs(1);
        let a = Some(card(2, RegionKind::Tunnel(0), Team::Red));
        let b = Some(card(3, RegionKind::Tunnel(1), Team::Red));
        s.read_card(a, Team::Red, t);
        s.read_card(a, Team::Red, t + Duration::from_secs(5));
        s.read_card(b, Team::Red, t + Duration::from_secs(7));
        let end = s.cooling_until;
        assert_eq!(end, Some(t + Duration::from_secs(67)));
        s.read_card(b, Team::Red, t + Duration::from_secs(10));
        assert_eq!(s.cooling_until, end);
    }
    #[test]
    fn region_height_rejects_under_highland() {
        let r = FieldRegion {
            id: 1,
            kind: RegionKind::Highland,
            team: None,
            min: Vec3::new(-1., 0.15, -1.),
            max: Vec3::new(1., 0.25, 1.),
        };
        assert!(!r.contains(Vec3::ZERO));
        assert!(r.contains(Vec3::new(0., 0.2, 0.)));
    }
}
