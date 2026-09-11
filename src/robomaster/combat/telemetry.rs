//! Talos v7 read-only snapshots. Never feed enemy truth into a control decision.
use super::reset::TrainingRound;
use super::*;
use crate::components::Controlled;
use bevy::prelude::*;
use std::time::Duration;
use talos_ipc::*;

#[derive(Resource, Default)]
pub struct CombatTelemetry {
    pub frame: CombatFrameMeta,
    next_sample: Duration,
}

/// Bits 0..7: dead, no shooter, cooling lock, round lock, allowance empty,
/// mechanical interval, feed busy, missing/invalid muzzle. Controller enable is not a weapon rule.
fn robot_meta(
    identity: &RobotIdentity,
    state: &RobotCombatState,
    now: Duration,
    muzzle_valid: bool,
) -> RobotCombatMeta {
    let mut blocks = 0;
    for (bit, blocked) in [
        state.life.status == LifeStatus::Dead,
        state.shooter.kind == ShooterKind::None,
        state.heat.cooling_locked,
        state.heat.round_locked,
        state.allowance == FireAllowance::Limited(0),
        state
            .shooter
            .last_shot_at
            .is_some_and(|last| now.saturating_sub(last) < state.shooter.mechanics.min_interval),
        state.shooter.pending.is_some(),
        !muzzle_valid,
    ]
    .into_iter()
    .enumerate()
    {
        if blocked {
            blocks |= 1 << bit;
        }
    }
    RobotCombatMeta {
        robot_id: identity.id.0,
        hp: state.life.hp,
        max_hp: state.rules.max_hp,
        heat: state.heat.current() as f32,
        heat_limit: state.rules.heat_limit as f32,
        cooling_per_second: state.rules.cooling_per_second as f32,
        allowance_remaining: match state.allowance {
            FireAllowance::Unlimited => 0,
            FireAllowance::Limited(n) => n,
        },
        allowance_mode: u8::from(matches!(state.allowance, FireAllowance::Limited(_))),
        fire_blocks: blocks,
        fire_permitted: u8::from(blocks == 0),
        team: match identity.team {
            Team::Red => 0,
            Team::Blue => 1,
        },
        role: match identity.role {
            CombatRole::Infantry => 0,
            CombatRole::Sentry => 1,
            CombatRole::HeroTarget => 2,
        },
        life: u8::from(state.life.status == LifeStatus::Dead),
        shooter: u8::from(state.shooter.kind != ShooterKind::None),
        actual_shots: state.shooter.actual_shots,
        rejected_requests: state.shooter.rejected_requests,
        damage_dealt: state.damage.damage_dealt,
        damage_taken: state.damage.damage_taken,
        kills: state.damage.kills,
        armor_contacts: state.damage.armor_contacts,
        damaging_hits: state.damage.damaging_hits,
        heat_lock_count: state.heat.lock_count,
        heat_locked_s: state.heat.locked_duration(now).as_secs_f64(),
        ..default()
    }
}

/// Sampling runs after all physical combat effects. Deadlines remain phase locked to round start;
/// a delayed physical step publishes one actual sample, never fabricated intermediate samples.
pub fn sample_combat(world: &mut World) {
    let now = world.resource::<Time<Fixed>>().elapsed();
    let (round_id, start) = {
        let round = world.resource::<TrainingRound>();
        (round.id, round.started_at)
    };
    let roots: Vec<_> = world
        .query::<(Entity, &RobotIdentity, Has<Controlled>)>()
        .iter(world)
        .map(|(e, id, controlled)| (e, *id, controlled))
        .collect();
    let mut entries = Vec::new();
    for (entity, identity, controlled) in roots {
        let muzzle_valid = super::shooting::muzzle_pose(world, entity).is_ok();
        if let Some(state) = world.get::<RobotCombatState>(entity) {
            entries.push((robot_meta(&identity, state, now, muzzle_valid), controlled));
        }
    }
    entries.sort_by_key(|(entry, _)| entry.robot_id);
    let mut telemetry = world.resource_mut::<CombatTelemetry>();
    if telemetry.frame.round_id != round_id {
        telemetry.next_sample = start;
    }
    let due = now >= telemetry.next_sample;
    if due {
        telemetry.next_sample = start
            + Duration::from_millis(
                ((now.saturating_sub(start).as_millis() / 100 + 1) * 100) as u64,
            );
        telemetry.frame.referee_sample_ns = now.as_nanos() as u64;
        telemetry.frame.referee_sample_sequence += 1;
        telemetry.frame.referee_valid = 0;
        telemetry.frame.self_referee = default();
        if let Some((state, _)) = entries.iter().find(|(_, controlled)| *controlled) {
            // Referee channel has no evaluation counters, including own attributed damage/kills.
            let mut referee = *state;
            referee.actual_shots = 0;
            referee.rejected_requests = 0;
            referee.damage_dealt = 0;
            referee.damage_taken = 0;
            referee.kills = 0;
            referee.armor_contacts = 0;
            referee.damaging_hits = 0;
            referee.heat_lock_count = 0;
            referee.heat_locked_s = 0.0;
            telemetry.frame.self_referee = referee;
            telemetry.frame.referee_valid = 1;
        }
    }
    telemetry.frame.round_id = round_id;
    telemetry.frame.round_started_ns = start.as_nanos() as u64;
    telemetry.frame.sim_time_ns = now.as_nanos() as u64;
    telemetry.frame.robot_count = entries.len().min(COMBAT_MAX_ROBOTS) as u32;
    telemetry.frame.robots.fill(default());
    for (dst, (src, _)) in telemetry.frame.robots.iter_mut().zip(entries) {
        *dst = src;
    }
    let events: Vec<_> = world
        .resource::<TrainingRound>()
        .events
        .iter()
        .rev()
        .take(COMBAT_MAX_EVENTS)
        .cloned()
        .collect();
    let dropped = world.resource::<TrainingRound>().dropped_events;
    let mut telemetry = world.resource_mut::<CombatTelemetry>();
    telemetry.frame.events.fill(default());
    telemetry.frame.events_dropped = dropped;
    telemetry.frame.event_count = events.len() as u32;
    for (dst, src) in telemetry
        .frame
        .events
        .iter_mut()
        .zip(events.into_iter().rev())
    {
        dst.id = src.id;
        dst.round_id = src.round;
        dst.round_time_ns = src.at.as_nanos() as u64;
        let mut length = src.detail.len().min(dst.detail.len() - 1);
        while !src.detail.is_char_boundary(length) {
            length -= 1;
        }
        dst.detail[..length].copy_from_slice(&src.detail.as_bytes()[..length]);
    }
}
