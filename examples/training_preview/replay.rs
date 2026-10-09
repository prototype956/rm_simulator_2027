//! Playback is a read-only display of sampled physical state, never a second simulation.
use bevy::prelude::*;
use serde::Deserialize;
use serde_json::Value;
use std::{collections::HashMap, path::{Path, PathBuf}};

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
    #[serde(default)]
    render: Value,
    #[serde(default)]
    fingerprints: Value,
    scenario: Value,
    frames: Vec<Frame>,
}

#[derive(Deserialize)]
struct PlaylistEntry {
    index: usize,
    path: PathBuf,
    label: String,
    #[serde(default)]
    scene_seed: Option<u64>,
}

#[derive(Deserialize)]
struct PlaylistManifest {
    version: u64,
    kind: String,
    render: Value,
    fingerprints: Value,
    replays: Vec<PlaylistEntry>,
}

struct PlaylistState {
    manifest_path: PathBuf,
    manifest_render: Value,
    manifest_fingerprint: Value,
    entries: Vec<PlaylistEntry>,
    index: usize,
}

pub(super) struct Replay {
    record: Recording,
    index: usize,
    elapsed: f64,
    speed: usize,
    paused: bool,
    label: String,
    scene_label: Option<String>,
    scene_seed: Option<u64>,
    playlist: Option<PlaylistState>,
    load_error: Option<String>,
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

fn resolve_playlist_path(manifest: &Path, entry: &Path) -> PathBuf {
    if entry.is_absolute() {
        entry.to_path_buf()
    } else {
        manifest.parent().unwrap_or_else(|| Path::new(".")).join(entry)
    }
}

fn load_recording(path: &Path) -> Result<Recording, Box<dyn std::error::Error>> {
    Ok(serde_json::from_slice(&std::fs::read(path)?)?)
}

fn validate_playlist_contract(
    path: &Path,
    render: &Value,
    fingerprint: &Value,
) -> Result<(), Box<dyn std::error::Error>> {
    let value: Value = serde_json::from_slice(&std::fs::read(path)?)?;
    if value["version"].as_u64() != Some(1)
        || value["render"] != *render
        || value["fingerprints"]["simulator_config"] != *fingerprint
    {
        return Err(format!("replay does not match playlist render contract: {}", path.display()).into());
    }
    Ok(())
}

fn target_motion_hud(scenario: &Value) -> String {
    let distance = scenario["target_distance_m"]
        .as_f64()
        .map_or_else(|| "unknown".into(), |value| format!("{value:.2} m"));
    let speed = scenario["motion"]["angular_speed_rad_s"].as_f64().map_or_else(
        || {
            if scenario["motion"]["kind"].as_str() == Some("static") {
                "static".into()
            } else {
                "unknown".into()
            }
        },
        |value| format!("{value:+.2} rad/s"),
    );
    format!("Target distance: {distance} | Target speed: {speed}\n")
}

fn validate_recording(record: &Recording) -> Result<(), Box<dyn std::error::Error>> {
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
    Ok(())
}

impl Replay {
    pub(super) fn load(path: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        let value: Value = serde_json::from_slice(&std::fs::read(path)?)?;
        if value["kind"].as_str() == Some("playlist") {
            let manifest: PlaylistManifest = serde_json::from_value(value)?;
            if manifest.version != 1 || manifest.kind != "playlist" || manifest.replays.is_empty() {
                return Err("unsupported or empty replay playlist".into());
            }
            for (index, entry) in manifest.replays.iter().enumerate() {
                if entry.index != index || entry.label.is_empty() {
                    return Err("invalid replay playlist entry".into());
                }
                let entry_path = resolve_playlist_path(path, &entry.path);
                validate_playlist_contract(
                    &entry_path,
                    &manifest.render,
                    &manifest.fingerprints["simulator_config"],
                )?;
            }
            let first_path = resolve_playlist_path(path, &manifest.replays[0].path);
            let record = load_recording(&first_path)?;
            if record.render != manifest.render
                || record.fingerprints["simulator_config"]
                    != manifest.fingerprints["simulator_config"]
            {
                return Err("first replay does not match playlist render contract".into());
            }
            validate_recording(&record)?;
            let first = &manifest.replays[0];
            return Ok(Self {
                record,
                index: 0,
                elapsed: 0.0,
                speed: 2,
                paused: false,
                label: "MODEL REPLAY".into(),
                scene_label: Some(first.label.clone()),
                scene_seed: first.scene_seed,
                playlist: Some(PlaylistState {
                    manifest_path: path.to_path_buf(),
                    manifest_render: manifest.render,
                    manifest_fingerprint: manifest.fingerprints["simulator_config"].clone(),
                    entries: manifest.replays,
                    index: 0,
                }),
                load_error: None,
            });
        }
        let record: Recording = serde_json::from_value(value)?;
        validate_recording(&record)?;
        Ok(Self {
            record,
            index: 0,
            elapsed: 0.0,
            speed: 2,
            paused: false,
            label: "MODEL REPLAY".into(),
            scene_label: None,
            scene_seed: None,
            playlist: None,
            load_error: None,
        })
    }

    pub(super) fn set_label(&mut self, label: String) {
        self.label = label;
    }

    pub(super) fn title(&self) -> String {
        let score = self.record.summary["score"]["official_damage"].as_f64()
            .map_or_else(|| "INCOMPLETE".into(), |value| format!("{value:.0}"));
        let scene = self.scene_label.as_ref().map_or(String::new(), |label| format!(" | {label}"));
        format!("RM {}{} | {} | {} steps | score {}", self.label, scene,
                self.record.model["name"].as_str().unwrap(), self.record.model["num_timesteps"], score)
    }

    pub(super) fn snapshot(&self) -> Value {
        let mut data = self.record.frames[self.index].data.clone();
        data["evaluation"]["scenario"] = self.record.scenario.clone();
        data
    }

    fn switch_scene(&mut self, delta: isize) -> Result<bool, String> {
        let Some(playlist) = &self.playlist else { return Ok(false); };
        let target = (playlist.index as isize + delta).rem_euclid(playlist.entries.len() as isize) as usize;
        let entry = &playlist.entries[target];
        let entry_path = entry.path.clone();
        let entry_label = entry.label.clone();
        let entry_seed = entry.scene_seed;
        let manifest_path = playlist.manifest_path.clone();
        let manifest_render = playlist.manifest_render.clone();
        let manifest_fingerprint = playlist.manifest_fingerprint.clone();
        let path = resolve_playlist_path(&manifest_path, &entry_path);
        let record = load_recording(&path).map_err(|error| format!("scene {}: {error}", target + 1))?;
        if record.render != manifest_render
            || record.fingerprints["simulator_config"] != manifest_fingerprint
        {
            return Err(format!("scene {} does not match playlist render contract", target + 1));
        }
        validate_recording(&record).map_err(|error| format!("scene {}: {error}", target + 1))?;
        self.record = record;
        self.index = 0;
        self.elapsed = 0.0;
        self.paused = false;
        self.scene_label = Some(entry_label);
        self.scene_seed = entry_seed;
        if let Some(playlist) = &mut self.playlist {
            playlist.index = target;
        }
        self.load_error = None;
        Ok(true)
    }

    pub(super) fn controls(&mut self, keys: &ButtonInput<KeyCode>, dt: f64) -> Result<bool, String> {
        if keys.just_pressed(KeyCode::BracketLeft) {
            return self.switch_scene(-1).map_err(|error| {
                self.load_error = Some(error.clone());
                error
            });
        }
        if keys.just_pressed(KeyCode::BracketRight) {
            return self.switch_scene(1).map_err(|error| {
                self.load_error = Some(error.clone());
                error
            });
        }
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
            return Ok(true);
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
        Ok(old != self.index)
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
        let selection = if m["selected_slot"].is_null() { String::new() } else {
            format!("Selected slot: {} | Switches: {} | Action: {}\n",
                    m["selected_slot"], m["slot_switches"], m["wire_action"])
        };
        let robots = frame.data["evaluation"]["robots"].as_array().unwrap();
        let hp = robots.iter().find(|r| r["robot_id"].as_u64() == Some(2)).unwrap()["hp"].as_u64().unwrap();
        let scene = self.playlist.as_ref().map_or(String::new(), |playlist| {
            format!("Scene {}/{} | seed {}\n[ previous | ] next\n",
                    playlist.index + 1, playlist.entries.len(),
                    self.scene_seed.map_or_else(|| "unknown".into(), |seed| seed.to_string()))
        });
        let error = self.load_error.as_ref().map_or(String::new(), |error| format!("\nScene switch failed: {error}"));
        let target = target_motion_hud(&self.record.scenario);
        format!(
            "{}\n{}{}{}{} | {:.2} s evaluation + {:.2} s settlement\nRaw window damage: {:.0} | Attributed damage: {} (observed {:.0})\nWindow shots: {} | Damaging rounds: {} | Hit rate: {} | After-window shots: {}\nTarget HP: {} | End: {}\n{}{} | {:.2}x | Space pause | -/+ speed | R replay | C camera\nArrows orbit | PgUp/PgDn zoom | Close window to exit",
            self.title(),
            scene,
            target,
            error,
            frame.phase, m["evaluation_time_ns"].as_f64().unwrap_or(0.0) / 1e9,
            m["settlement_time_ns"].as_f64().unwrap_or(0.0) / 1e9,
            m["raw_damage"].as_f64().unwrap_or(0.0), damage,
            m["eligible_damage"].as_f64().unwrap_or(0.0), shots, hits, rate,
            m["excluded_shots"].as_u64().unwrap_or(0), hp,
            m["end_reason"].as_str().unwrap_or("running"),
            selection,
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

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::fs;

    fn replay_value(render: &Value, fingerprint: &str, target_x: f64, seed: u64) -> Value {
        json!({
            "version": 1,
            "model": {"name": "model.zip", "num_timesteps": 1},
            "summary": {},
            "render": render,
            "fingerprints": {"simulator_config": fingerprint},
            "scene_seed": seed,
            "scenario": {
                "controlled_position_bevy_m": [0.0, 0.0, 0.0],
                "target_position_bevy_m": [target_x, 0.0, 0.0],
                "target_distance_m": 5.0,
                "motion": {"kind": "rotation", "angular_speed_rad_s": -3.0}
            },
            "frames": [{
                "time_ns": 0,
                "phase": "evaluating",
                "metrics": {},
                "data": {
                    "feedback": {"yaw_rad": 0.0, "pitch_rad": 0.0},
                    "evaluation": {
                        "robots": [
                            {"robot_id": 1, "position_bevy_m": [0.0, 0.0, 0.0], "rotation_xyzw": [0.0, 0.0, 0.0, 1.0], "hp": 100},
                            {"robot_id": 2, "position_bevy_m": [target_x, 0.0, 0.0], "rotation_xyzw": [0.0, 0.0, 0.0, 1.0], "hp": 100}
                        ],
                        "projectiles": []
                    },
                    "events": []
                }
            }]
        })
    }

    #[test]
    fn playlist_switches_scene_and_wraps() {
        let root = std::env::temp_dir().join(format!("rm-replay-test-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        fs::create_dir_all(&root).unwrap();
        let render = json!({"simulator_root": "/sim", "config": "/sim/config.toml", "assets": "/sim/assets"});
        fs::write(root.join("one.json"), replay_value(&render, "digest", 1.0, 17).to_string()).unwrap();
        fs::write(root.join("two.json"), replay_value(&render, "digest", 2.0, 23).to_string()).unwrap();
        let manifest = json!({
            "version": 1,
            "kind": "playlist",
            "render": render,
            "fingerprints": {"simulator_config": "digest"},
            "replays": [
                {"index": 0, "path": "one.json", "label": "scene seed 17", "scene_seed": 17},
                {"index": 1, "path": "two.json", "label": "scene seed 23", "scene_seed": 23}
            ]
        });
        let playlist_path = root.join("playlist.json");
        fs::write(&playlist_path, manifest.to_string()).unwrap();
        let mut replay = Replay::load(&playlist_path).unwrap();
        assert!(replay.hud().contains("Target distance: 5.00 m | Target speed: -3.00 rad/s"));
        assert_eq!(replay.snapshot()["evaluation"]["scenario"]["target_position_bevy_m"][0], 1.0);
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::BracketRight);
        assert!(replay.controls(&keys, 0.0).unwrap());
        assert_eq!(replay.scene_seed, Some(23));
        assert_eq!(replay.snapshot()["evaluation"]["scenario"]["target_position_bevy_m"][0], 2.0);
        keys.release(KeyCode::BracketRight);
        keys.clear();
        keys.press(KeyCode::BracketRight);
        replay.controls(&keys, 0.0).unwrap();
        assert_eq!(replay.scene_seed, Some(17));
        let _ = fs::remove_dir_all(root);
    }
}
