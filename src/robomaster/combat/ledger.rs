//! Bounded per-step physical journal. The training backend drains at the step boundary;
//! the protocol caches that response before network delivery. Overflow faults the episode.
use bevy::prelude::*;
use serde_json::{Value, json};
use std::time::Duration;
#[derive(Resource, Default)]
pub struct CombatLedger {
    pub events: Vec<Value>,
    pub next_id: u64,
    pub failed: bool,
}
pub fn record(world: &mut World, at: Duration, kind: &str, data: Value) {
    let round_id = world
        .get_resource::<super::reset::TrainingRound>()
        .map_or(0, |r| r.id);
    let Some(mut ledger) = world.get_resource_mut::<CombatLedger>() else {
        return;
    };
    if ledger.events.len() >= 65536 {
        ledger.failed = true;
        return;
    }
    ledger.next_id += 1;
    let id = ledger.next_id;
    ledger.events.push(json!({"event_id":id,"round_id":round_id,"time_ns":at.as_nanos() as u64,"kind":kind,"data":data}));
}
pub fn ended(world: &mut World, entity: Entity, at: Duration, reason: &str) {
    if let Some(shot) = world
        .get::<super::shooting::ProjectileShot>(entity)
        .copied()
    {
        record(
            world,
            at,
            "projectile_ended",
            json!({"projectile_id":shot.id,"request_id":shot.request.id,"reason":reason}),
        );
    }
}
