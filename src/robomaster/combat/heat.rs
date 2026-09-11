//! RMUL 2026 V1.2.0 §3.3.1.3, pp. 29–30 and figure 3-10.
//! Rule cooling rates are integers, so tenths represent every 100 ms deduction exactly.

use super::{RobotCombatState, RobotId, RobotIdentity, ShooterKind};
use bevy::prelude::*;
use std::time::Duration;

const COOLING_PERIOD: Duration = Duration::from_millis(100);
const SHOT_HEAT_TENTHS: u64 = 100;
const ROUND_LOCK_MARGIN: u64 = 100;

/// Per-robot thermal state. Ordinary and round locks are independent latches.
/// Clearing the ordinary latch at zero never clears the round latch.
#[derive(Reflect, Clone, Debug, Default)]
pub struct HeatState {
    current_tenths: u64,
    pub cooling_locked: bool,
    pub round_locked: bool,
    /// Last nominal cooling deadline, not the time of the most recent rendered frame.
    last_cooling_at: Duration,
    pub lock_count: u64,
    locked_since: Option<Duration>,
    locked_total: Duration,
}

impl HeatState {
    /// A new scene's state; future scene reset must use its current simulation time as epoch.
    /// New scene construction clears both latches; cooling has no in-round permanent unlock.
    pub fn new(now: Duration) -> Self {
        Self {
            last_cooling_at: now,
            ..default()
        }
    }

    /// Heat in referee units. All arithmetic and limit comparisons remain integer based.
    pub fn current(&self) -> f64 {
        self.current_tenths as f64 / 10.0
    }

    pub fn is_locked(&self) -> bool {
        self.cooling_locked || self.round_locked
    }

    /// Power-off on death clears current heat and its ordinary latch, never the round latch.
    pub(super) fn on_death(&mut self, now: Duration) {
        let before = self.snapshot();
        self.current_tenths = 0;
        self.cooling_locked = false;
        self.last_cooling_at = now;
        self.track_locks(now, before);
    }

    pub fn locked_duration(&self, now: Duration) -> Duration {
        self.locked_total
            + self
                .locked_since
                .map_or(Duration::ZERO, |at| now.saturating_sub(at))
    }

    fn track_locks(&mut self, at: Duration, before: HeatSnapshot) {
        self.lock_count += u64::from(!before.cooling_locked && self.cooling_locked)
            + u64::from(!before.round_locked && self.round_locked);
        if self.is_locked() && self.locked_since.is_none() {
            self.locked_since = Some(at);
        }
        if !self.is_locked()
            && let Some(start) = self.locked_since.take()
        {
            self.locked_total += at.saturating_sub(start);
        }
    }

    fn add_shot(&mut self, heat_limit: u32) {
        self.current_tenths = self.current_tenths.saturating_add(SHOT_HEAT_TENTHS);
        let limit = u64::from(heat_limit);
        self.cooling_locked |= self.current_tenths > limit * 10;
        // The prose says > Q2; the normative choice for this simulator follows figure 3-10's ≥.
        self.round_locked |= self.current_tenths >= (limit + ROUND_LOCK_MARGIN) * 10;
    }

    fn cool_once(&mut self, cooling_per_second: u32) {
        // Rate / 10 referee units is exactly `rate` tenths per tick.
        self.current_tenths = self
            .current_tenths
            .saturating_sub(u64::from(cooling_per_second));
        if self.current_tenths == 0 {
            self.cooling_locked = false;
        }
    }

    fn snapshot(&self) -> HeatSnapshot {
        HeatSnapshot {
            current: self.current(),
            cooling_locked: self.cooling_locked,
            round_locked: self.round_locked,
        }
    }

    fn cool_until(
        &mut self,
        now: Duration,
        rate: u32,
        mut changed: impl FnMut(Duration, HeatSnapshot, HeatSnapshot),
    ) {
        while let Some(tick) = self.last_cooling_at.checked_add(COOLING_PERIOD)
            && tick <= now
        {
            self.last_cooling_at = tick;
            let before = self.snapshot();
            self.cool_once(rate);
            self.track_locks(tick, before);
            let after = self.snapshot();
            if before != after {
                changed(tick, before, after);
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct HeatSnapshot {
    current: f64,
    cooling_locked: bool,
    round_locked: bool,
}

fn log_change(
    robot: RobotId,
    at: Duration,
    cause: &str,
    before: HeatSnapshot,
    after: HeatSnapshot,
) {
    info!(
        "heat robot={} sim_s={:.9} cause={} heat={:.1}->{:.1} cooling_locked={} round_locked={}",
        robot.0,
        at.as_secs_f64(),
        cause,
        before.current,
        after.current,
        after.cooling_locked,
        after.round_locked,
    );
    if !before.cooling_locked && after.cooling_locked {
        info!(
            "heat lock robot={} sim_s={:.9} reason=HeatCoolingLock",
            robot.0,
            at.as_secs_f64()
        );
    }
    if !before.round_locked && after.round_locked {
        info!(
            "heat lock robot={} sim_s={:.9} reason=HeatRoundLock",
            robot.0,
            at.as_secs_f64()
        );
    }
    if before.cooling_locked && !after.cooling_locked {
        info!(
            "heat unlock robot={} sim_s={:.9} reason=HeatCoolingLock heat=0.0 round_locked={}",
            robot.0,
            at.as_secs_f64(),
            after.round_locked,
        );
    }
}

/// Must finish all due cooling before admitting requests or completing a feed at this boundary.
/// Catch-up visits each nominal deadline, preserving 10 Hz behavior at any render/physics rate.
pub(super) fn cool_robot_heat(world: &mut World) {
    let now = world.resource::<Time<Fixed>>().elapsed();
    let mut changes = Vec::new();
    let mut robots = world.query::<(&RobotIdentity, &mut RobotCombatState)>();
    for (identity, mut state) in robots.iter_mut(world) {
        if state.shooter.kind != ShooterKind::Barrel17mm {
            continue;
        }
        let rate = state.rules.cooling_per_second;
        state.heat.cool_until(now, rate, |tick, before, after| {
            log_change(identity.id, tick, "cooling", before, after);
            if before.cooling_locked != after.cooling_locked
                || before.round_locked != after.round_locked
            {
                changes.push((
                    tick,
                    format!(
                        "heat robot={} cause=cooling cooling_locked={} round_locked={}",
                        identity.id.0, after.cooling_locked, after.round_locked
                    ),
                ));
            }
        });
    }
    for (at, detail) in changes {
        super::reset::record_event(world, at, detail);
    }
}

/// Called once, synchronously after actual spawn. Rejected or feeding requests never add heat.
pub(super) fn add_shot_heat(world: &mut World, root: Entity, at: Duration) {
    let id = world.get::<RobotIdentity>(root).unwrap().id;
    let mut state = world.get_mut::<RobotCombatState>(root).unwrap();
    let limit = state.rules.heat_limit;
    let before = state.heat.snapshot();
    state.heat.add_shot(limit);
    state.heat.track_locks(at, before);
    let after = state.heat.snapshot();
    log_change(id, at, "shot", before, after);
    if before.cooling_locked != after.cooling_locked || before.round_locked != after.round_locked {
        super::reset::record_event(
            world,
            at,
            format!(
                "heat robot={} cause=shot cooling_locked={} round_locked={}",
                id.0, after.cooling_locked, after.round_locked
            ),
        );
    }
}
