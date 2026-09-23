//! Playback is a read-only display of sampled physical state, never a second simulation.
use bevy::prelude::*;
use serde::Deserialize;
use serde_json::Value;
use std::{collections::HashMap, path::Path};

#[derive(Deserialize)]
struct Frame {
    time_ns: u64,
    phase: String,
    metrics: Value,
    data: Value,
}

#[derive(Deserialize)]
struct Recording {
    version: u64,
    model: Value,
    #[serde(default)]
    summary: Value,
    scenario: Value,
    frames: Vec<Frame>,
}

pub(super) struct Replay {
    record: Recording,
    index: usize,
    elapsed: f64,
    speed: usize,
    paused: bool,
    label: String,
}

const SPEEDS: [f64; 4] = [0.25, 0.5, 1.0, 2.0];

fn check_vector(value: &Value, count: usize) -> bool {
    value.as_array().is_some_and(|a| {
        a.len() == count && a.iter().all(|n| n.as_f64().is_some_and(|v| (v as f32).is_finite()))
    })
}

pub(super) fn validate_data(data: &Value) -> Result<(), String> {
    for axis in ["yaw_rad", "pitch_rad"] {
        if !data["feedback"][axis].as_f64().is_some_and(f64::is_finite) {
            return Err("invalid replay gimbal feedback".into());
        }
    }
    let evaluation = &data["evaluation"];
    let robots = evaluation["robots"].as_array().ok_or("missing replay robots")?;
    if robots.len() != 2 || ![1, 2].iter().all(|id| {
        robots.iter().filter(|r| r["robot_id"].as_u64() == Some(*id)).count() == 1
    }) {
        return Err("replay requires the controlled and target robot".into());
    }
    for robot in robots {
        if !check_vector(&robot["position_bevy_m"], 3)
            || !check_vector(&robot["rotation_xyzw"], 4)
            || robot["hp"].as_u64().is_none()
        {
            return Err("invalid replay robot pose/HP".into());
        }
    }
    let projectiles = evaluation["projectiles"].as_array().ok_or("missing replay projectiles")?;
    for projectile in projectiles {
        if projectile["projectile_id"].as_u64().is_none()
            || !check_vector(&projectile["position_bevy_m"], 3)
        {
            return Err("invalid replay projectile".into());
        }
    }
    if data["events"].as_array().is_none() {
        return Err("missing replay events".into());
    }
    Ok(())
}

impl Replay {
    pub(super) fn load(path: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        let record: Recording = serde_json::from_slice(&std::fs::read(path)?)?;
        if record.version != 1 || record.frames.is_empty()
            || record.model["name"].as_str().is_none()
            || record.model["num_timesteps"].as_u64().is_none()
            || !check_vector(&record.scenario["controlled_position_bevy_m"], 3)
            || !check_vector(&record.scenario["target_position_bevy_m"], 3)
        {
            return Err("unsupported or invalid replay header".into());
        }
        let mut previous = None;
        for frame in &record.frames {
            if previous.is_some_and(|at| frame.time_ns != at + 10_000_000) {
                return Err("replay is missing a 10 ms physical frame".into());
            }
            previous = Some(frame.time_ns);
            validate_data(&frame.data).map_err(std::io::Error::other)?;
        }
        Ok(Self { record, index: 0, elapsed: 0.0, speed: 2, paused: false, label: "MODEL REPLAY".into() })
    }

    pub(super) fn set_label(&mut self, label: String) {
        self.label = label;
    }

    pub(super) fn title(&self) -> String {
        let score = self.record.summary["score"]["official_damage"].as_f64()
            .map_or_else(|| "INCOMPLETE".into(), |value| format!("{value:.0}"));
        format!("RM {} | {} | {} steps | score {}", self.label,
                self.record.model["name"].as_str().unwrap(), self.record.model["num_timesteps"], score)
    }

    pub(super) fn snapshot(&self) -> Value {
        let mut data = self.record.frames[self.index].data.clone();
        data["evaluation"]["scenario"] = self.record.scenario.clone();
        data
    }

    pub(super) fn controls(&mut self, keys: &ButtonInput<KeyCode>, dt: f64) -> bool {
        let old = self.index;
        if keys.just_pressed(KeyCode::Space) {
            self.paused = !self.paused;
        }
        if keys.just_pressed(KeyCode::Minus) {
            self.speed = self.speed.saturating_sub(1);
        }
        if keys.just_pressed(KeyCode::Equal) {
            self.speed = (self.speed + 1).min(SPEEDS.len() - 1);
        }
        if keys.just_pressed(KeyCode::KeyR) {
            self.index = 0;
            self.elapsed = 0.0;
            self.paused = false;
            return true;
        }
        if !self.paused {
            self.elapsed += dt * SPEEDS[self.speed];
            let start = self.record.frames[0].time_ns;
            let at = start as f64 + self.elapsed * 1e9;
            while self.index + 1 < self.record.frames.len()
                && self.record.frames[self.index + 1].time_ns as f64 <= at
            {
                self.index += 1;
            }
            if self.index + 1 == self.record.frames.len() {
                self.paused = true;
            }
        }
        old != self.index
    }

    pub(super) fn hud(&self) -> String {
        let frame = &self.record.frames[self.index];
        let m = &frame.metrics;
        let shots = m["shots"].as_u64().unwrap_or(0);
        let hits = m["hits"].as_u64().unwrap_or(0);
        let rate = if shots == 0 { "--".into() } else { format!("{:.1}%", 100.0 * hits as f64 / shots as f64) };
        let damage = m["official_damage"].as_f64().map_or_else(
            || if frame.phase == "settlement_timed_out" { "INCOMPLETE".into() } else { "pending".into() },
            |d| format!("{d:.0}"),
        );
        let robots = frame.data["evaluation"]["robots"].as_array().unwrap();
        let hp = robots.iter().find(|r| r["robot_id"].as_u64() == Some(2)).unwrap()["hp"].as_u64().unwrap();
        format!(
            "{}\n{} | {:.2} s evaluation + {:.2} s settlement\nRaw window damage: {:.0} | Attributed damage: {} (observed {:.0})\nWindow shots: {} | Damaging rounds: {} | Hit rate: {} | After-window shots: {}\nTarget HP: {} | End: {}\n{} | {:.2}x | Space pause | -/+ speed | R replay | C camera\nArrows orbit | PgUp/PgDn zoom | Close window to exit",
            self.title(),
            frame.phase, m["evaluation_time_ns"].as_f64().unwrap_or(0.0) / 1e9,
            m["settlement_time_ns"].as_f64().unwrap_or(0.0) / 1e9,
            m["raw_damage"].as_f64().unwrap_or(0.0), damage,
            m["eligible_damage"].as_f64().unwrap_or(0.0), shots, hits, rate,
            m["excluded_shots"].as_u64().unwrap_or(0), hp,
            m["end_reason"].as_str().unwrap_or("running"),
            if self.paused { "Paused" } else { "Playing" }, SPEEDS[self.speed],
        )
    }

    pub(super) fn draw(&self, gizmos: &mut Gizmos) {
        draw_samples(self.record.frames[self.index.saturating_sub(20)..=self.index].iter().map(|f| &f.data), gizmos);
    }
}

pub(super) fn draw_samples<'a>(samples: impl IntoIterator<Item = &'a Value>, gizmos: &mut Gizmos) {
    let samples: Vec<&Value> = samples.into_iter().collect();
    let Some(frame) = samples.last() else { return; };
    let yellow = Color::srgb(1.0, 0.85, 0.1);
    let mut previous = HashMap::new();
    // Short trails keep fast projectiles visible even when several 10 ms frames share a render.
    for sample in &samples[samples.len().saturating_sub(16)..] {
        for projectile in sample["evaluation"]["projectiles"].as_array().unwrap() {
            let id = projectile["projectile_id"].as_u64().unwrap();
            let p = super::vector(&projectile["position_bevy_m"]);
            if let Some(last) = previous.insert(id, p) {
                gizmos.line(last, p, yellow);
            }
        }
    }
    for projectile in frame["evaluation"]["projectiles"].as_array().unwrap() {
        gizmos.sphere(Isometry3d::from_translation(super::vector(&projectile["position_bevy_m"])), 0.025, yellow);
    }
    // Damage events have robot identity, not an impact position. Mark the damaged robot.
    for sample in &samples {
        for event in sample["events"].as_array().unwrap() {
            if event["kind"].as_str() != Some("damage_applied") || event["data"]["actual"].as_u64() == Some(0) {
                continue;
            }
            if let Some(robot) = frame["evaluation"]["robots"].as_array().unwrap().iter()
                .find(|r| r["robot_id"] == event["data"]["target"])
            {
                gizmos.sphere(Isometry3d::from_translation(super::vector(&robot["position_bevy_m"]) + Vec3::Y * 0.3),
                              0.45, Color::srgb(1.0, 0.2, 0.1));
            }
        }
    }
}
