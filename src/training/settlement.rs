//! Post-window stepping retains physical firing/impact rules and never fabricates outcomes.
use crate::gimbal_actuator::{GimbalActuator, GimbalCommandInbox};
use crate::robomaster::combat::{ledger, shooting, *};
use bevy::prelude::*;
use serde_json::{Value, json};

#[derive(Resource)]
pub(super) struct Settlement {
    closed_at_ns: u64,
    deadline_ns: u64,
    damage_at_close: u64,
    shots_at_close: u64,
    cancelled_fire_commands: Vec<Value>,
    status: &'static str,
}

fn own_totals(w: &mut World) -> (u64, u64) {
    w.query::<(&RobotIdentity, &RobotCombatState)>()
        .iter(w)
        .find(|(id, _)| id.id == CONTROLLED_ROBOT_ID)
        .map(|(_, s)| (s.damage.damage_dealt, s.shooter.actual_shots))
        .expect("controlled robot")
}

pub(super) fn remaining(w: &mut World) -> Value {
    let queued = shooting::queued_requests(w).len();
    let feeds = w
        .query::<&RobotCombatState>()
        .iter(w)
        .filter(|s| s.shooter.pending.is_some())
        .count();
    let projectiles = w.query::<&shooting::ProjectileShot>().iter(w).count();
    json!({"queued_requests":queued,"feeding_requests":feeds,"projectiles":projectiles})
}

pub(super) fn update(w: &mut World) {
    if !w.contains_resource::<Settlement>() {
        return;
    }
    let pending = remaining(w);
    let now = w.resource::<Time<Fixed>>().elapsed().as_nanos() as u64;
    let mut state = w.resource_mut::<Settlement>();
    if state.status != "running" {
        return;
    }
    // Completion on the deadline is successful, not a timeout.
    state.status = if pending.as_object().unwrap().values().all(|n| n == 0) {
        "complete"
    } else if now >= state.deadline_ns {
        "timed_out"
    } else {
        "running"
    };
}

pub(super) fn data(w: &mut World) -> Value {
    if !w.contains_resource::<Settlement>() {
        return Value::Null;
    }
    let remaining = remaining(w);
    let (damage, shots) = own_totals(w);
    let state = w.resource::<Settlement>();
    json!({"status":state.status,"window_end_ns":state.closed_at_ns,"deadline_ns":state.deadline_ns,
        "settled_time_ns":w.resource::<Time<Fixed>>().elapsed().as_nanos() as u64 - state.closed_at_ns,
        "remaining":remaining,"cancelled_fire_commands":state.cancelled_fire_commands,
        "damage_after_window":damage - state.damage_at_close,"shots_after_window":shots - state.shots_at_close})
}

pub(super) fn finished(w: &World) -> bool {
    w.resource::<Settlement>().status != "running"
}

pub(super) fn close(w: &mut World, max_steps: u64) -> Result<(), String> {
    let mut cancelled = w
        .resource::<GimbalCommandInbox>()
        .pending_for_reset()?
        .into_iter()
        .chain(w.resource::<GimbalActuator>().pending_for_reset())
        .filter(|c| c["fire_level"] == 1)
        .collect::<Vec<_>>();
    for command in &mut cancelled {
        command["status"] = json!("fire_cancelled_by_window");
    }
    cancelled.sort_by_key(|c| c["command_timestamp_ns"].as_u64());
    let now = w.resource::<Time<Fixed>>().elapsed();
    let closed_at_ns = now.as_nanos() as u64;
    let deadline_ns = max_steps
        .checked_mul(super::protocol::CONTROL_DT_NS)
        .and_then(|n| closed_at_ns.checked_add(n))
        .ok_or("settlement deadline overflow")?;
    let (damage_at_close, shots_at_close) = own_totals(w);
    w.resource::<GimbalCommandInbox>()
        .suppress_fire_for_window()?;
    w.resource_mut::<GimbalActuator>()
        .suppress_fire_for_window();
    // Keep external control enabled so already admitted feed uses the unchanged firing rules.
    w.insert_resource(shooting::FireAdmissionClosed);
    w.insert_resource(Settlement {
        closed_at_ns,
        deadline_ns,
        damage_at_close,
        shots_at_close,
        cancelled_fire_commands: cancelled,
        status: "running",
    });
    update(w);
    ledger::record(
        w,
        now,
        "evaluation_window_closed",
        json!({"window_end_ns":closed_at_ns,"deadline_ns":deadline_ns}),
    );
    Ok(())
}
