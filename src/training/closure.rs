//! A Reset truncates the previous world. It never invents impacts or transfers reward.
use super::protocol::MAX_MESSAGE_BYTES;
use crate::gimbal_actuator::{GimbalActuator, GimbalCommandInbox};
use crate::robomaster::combat::{ledger::CombatLedger, reset::TrainingRound, shooting::*, *};
use avian3d::prelude::*;
use bevy::prelude::*;
use serde_json::{Value, json};

const RECORD_LIMIT: usize = 4096;
const COLLECTION_BYTES: usize = 2 * 1024 * 1024;

// Keep enough envelope headroom to reject before committing a world or draining its journal.
pub(super) fn check_response_size(data: &Value) -> Result<(), String> {
    if serde_json::to_vec(data).map_err(|e| e.to_string())?.len() > MAX_MESSAGE_BYTES - 1024 {
        return Err("physical response exceeds transport budget".into());
    }
    Ok(())
}

fn bounded(records: impl IntoIterator<Item = Value>) -> Value {
    let mut kept = Vec::new();
    let mut omitted = 0;
    let mut bytes = 0;
    for record in records {
        let size = serde_json::to_vec(&record).expect("JSON value").len() + 1;
        if omitted > 0 || kept.len() >= RECORD_LIMIT || bytes + size > COLLECTION_BYTES {
            omitted += 1;
        } else {
            bytes += size;
            kept.push(record);
        }
    }
    json!({"records":kept,"omitted_count":omitted,"complete":omitted == 0})
}

pub(super) fn request_data(request: &FireRequest) -> Value {
    json!({"request_id":request.id,"robot_id":request.robot.0,
        "source":source_data(request.source),"requested_at_ns":request.requested_at.as_nanos() as u64})
}

pub(super) fn resources(state: &RobotCombatState) -> Value {
    let allowance = match state.allowance {
        FireAllowance::Unlimited => json!({"mode":"unlimited","remaining":null}),
        FireAllowance::Limited(n) => json!({"mode":"limited","remaining":n}),
    };
    let pending = state.shooter.pending.map(|p| {
        let mut data = request_data(&p.request);
        data["ready_at_ns"] = json!(p.ready_at.as_nanos() as u64);
        data
    });
    json!({"hp":state.life.hp,"life_status":format!("{:?}",state.life.status),
        "heat":state.heat.current(),"heat_limit":state.rules.heat_limit,
        "cooling_per_second":state.rules.cooling_per_second,
        "cooling_locked":state.heat.cooling_locked,"round_locked":state.heat.round_locked,
        "allowance":allowance,"pending_feed":pending,
        "actual_shots":state.shooter.actual_shots,"rejected_requests":state.shooter.rejected_requests,
        "damage_dealt":state.damage.damage_dealt,"damage_taken":state.damage.damage_taken,
        "damaging_hits":state.damage.damaging_hits,"kills":state.damage.kills})
}

/// Read-only snapshot, taken only after the replacement world has initialized successfully.
pub(super) fn snapshot(w: &mut World, committed_ns: u64, delivered_damage: u64) -> Value {
    let mut requests: Vec<_> = queued_requests(w)
        .iter()
        .map(|r| {
            let mut data = request_data(r);
            data["stage"] = json!("admission");
            data["status"] = json!("cancelled_by_reset");
            data
        })
        .collect();
    let mut damage = None;
    let mut robots = Vec::new();
    for (id, state) in w.query::<(&RobotIdentity, &RobotCombatState)>().iter(w) {
        let mut data = resources(state);
        data["robot_id"] = json!(id.id.0);
        robots.push(data);
        if id.id == CONTROLLED_ROBOT_ID {
            damage = Some(state.damage.damage_dealt);
        }
        if let Some(p) = state.shooter.pending {
            let mut data = request_data(&p.request);
            data["stage"] = json!("feeding");
            data["status"] = json!("cancelled_by_reset");
            data["ready_at_ns"] = json!(p.ready_at.as_nanos() as u64);
            requests.push(data);
        }
    }
    robots.sort_by_key(|r| r["robot_id"].as_u64());
    requests.sort_by_key(|r| r["request_id"].as_u64());
    let mut projectiles: Vec<_> = w.query::<(&ProjectileShot, Option<&Position>, Option<&LinearVelocity>)>().iter(w)
        .map(|(s,p,v)| json!({"projectile_id":s.id,"request_id":s.request.id,
            "robot_id":s.request.robot.0,"source":source_data(s.request.source),
            "fired_at_ns":s.fired_at.as_nanos() as u64,"status":"truncated_by_reset",
            "position_bevy_m":p.map(|p| p.0.to_array()),"velocity_bevy_m_s":v.map(|v| v.0.to_array())})).collect();
    projectiles.sort_by_key(|p| p["projectile_id"].as_u64());
    let inbox = w.resource::<GimbalCommandInbox>().pending_for_reset();
    let inbox_failed = inbox.is_err();
    let mut commands = inbox.unwrap_or_default();
    commands.extend(w.resource::<GimbalActuator>().pending_for_reset());
    commands.sort_by_key(|c| c["command_timestamp_ns"].as_u64());
    let ledger = w.resource::<CombatLedger>();
    let commands = bounded(commands);
    let requests = bounded(requests);
    let projectiles = bounded(projectiles);
    let events = bounded(ledger.events.iter().cloned());
    let unreported = damage.and_then(|n| n.checked_sub(delivered_damage));
    let complete = !inbox_failed
        && !ledger.failed
        && unreported.is_some()
        && [&commands, &requests, &projectiles, &events]
            .iter()
            .all(|v| v["complete"] == true);
    let last_event_id = ledger.next_id;
    let ledger_failed = ledger.failed;
    let settlement = super::settlement::data(w);
    json!({"settlement":settlement,"kind":"reset_truncation","round_id":w.resource::<TrainingRound>().id,
        "physical_time_ns":w.resource::<Time<Fixed>>().elapsed().as_nanos() as u64,
        "last_committed_time_ns":committed_ns,"complete":complete,
        "inbox_snapshot_failed":inbox_failed,"ledger_overflow":ledger_failed,
        "last_event_id":last_event_id,"unreported_damage":unreported,
        "robots":robots,"cancelled_commands":commands,"cancelled_requests":requests,
        "truncated_projectiles":projectiles,"undelivered_events":events})
}
