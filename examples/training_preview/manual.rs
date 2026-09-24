//! A bounded UI controller; Python alone owns Gym and the physical world.
use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender, bounded, TryRecvError};
use serde_json::{Value, json};
use std::{collections::VecDeque, io::{self, BufRead, Write}, time::{Duration, Instant}};

#[derive(Clone, Copy, Debug, PartialEq)]
enum Operation { Reset, Step(u8) }

pub(super) struct Manual {
    requests: Sender<Operation>,
    replies: Receiver<Result<Value, String>>,
    busy: bool,
    resetting: bool,
    queued: Option<Operation>,
    running: bool,
    ended: bool,
    fault: Option<String>,
    notice: String,
    next_step: Instant,
    frame: Option<Value>,
    history: VecDeque<Value>,
    desired_slot: Option<u8>, // UI intent; the controller reports the actual selected slot separately.
}

#[cfg(test)]
mod tests {
    use super::*;

    fn controller() -> (Manual, Receiver<Operation>, Sender<Result<Value, String>>) {
        let (requests, work) = bounded(1);
        let (results, replies) = bounded(1);
        (Manual { requests, replies, busy: false, resetting: false, queued: None, running: false,
                  ended: false, fault: None, notice: String::new(), next_step: Instant::now(),
                  frame: None, history: VecDeque::new(), desired_slot: None }, work, results)
    }

    fn response(step: u64, ended: bool) -> Result<Value, String> {
        Ok(json!({"data": {}, "metrics": {"step": step, "terminated": false, "truncated": ended}}))
    }

    #[test]
    fn held_fire_repeats_and_release_stops_without_queued_shots() {
        let (mut manual, work, results) = controller();
        manual.running = true;
        let mut keys = ButtonInput::default();
        keys.press(KeyCode::KeyF);
        manual.controls(&keys);
        assert_eq!(work.try_recv().unwrap(), Operation::Step(1));
        keys.clear(); // Held across UI frames, no new press edge needed.
        manual.controls(&keys);
        assert!(manual.queued.is_none() && work.is_empty());
        results.send(response(1, false)).unwrap();
        manual.receive();
        manual.next_step = Instant::now();
        manual.controls(&keys);
        assert_eq!(work.try_recv().unwrap(), Operation::Step(1));
        keys.release(KeyCode::KeyF);
        manual.controls(&keys); // Release while an operation is in flight.
        results.send(response(2, false)).unwrap();
        manual.receive();
        manual.next_step = Instant::now();
        manual.controls(&keys);
        assert_eq!(work.try_recv().unwrap(), Operation::Step(0));
        assert!(manual.queued.is_none());
    }

    #[test]
    fn paused_fire_and_old_step_key_do_not_advance_time() {
        let (mut manual, work, _) = controller();
        let mut keys = ButtonInput::default();
        keys.press(KeyCode::KeyF);
        keys.press(KeyCode::KeyN);
        manual.controls(&keys);
        assert!(work.is_empty() && manual.queued.is_none());
        keys.press(KeyCode::Space);
        manual.controls(&keys);
        assert!(manual.running);
        assert_eq!(work.try_recv().unwrap(), Operation::Step(1));
    }

    #[test]
    fn queue_is_bounded_and_pause_discards_it_without_cancelling_inflight() {
        let (mut manual, work, results) = controller();
        manual.running = true;
        manual.controls(&ButtonInput::default());
        assert_eq!(work.try_recv().unwrap(), Operation::Step(0));
        manual.enqueue(Operation::Step(1));
        manual.enqueue(Operation::Step(0));
        assert_eq!(manual.queued, Some(Operation::Step(1)));
        assert!(manual.notice.starts_with("BUSY"));
        let mut keys = ButtonInput::default();
        keys.press(KeyCode::Space);
        manual.controls(&keys);
        assert!(!manual.running && manual.busy && manual.queued.is_none());
        results.send(response(1, false)).unwrap();
        manual.receive();
        manual.controls(&ButtonInput::default());
        assert!(work.is_empty());
    }

    #[test]
    fn reset_autoruns_after_warmup_and_terminal_state_does_not_autoreset() {
        let (mut manual, work, results) = controller();
        manual.send(Operation::Reset);
        assert_eq!(work.try_recv().unwrap(), Operation::Reset);
        let mut keys = ButtonInput::default();
        keys.press(KeyCode::KeyF);
        manual.controls(&keys);
        assert!(manual.queued.is_none());
        results.send(response(0, false)).unwrap();
        manual.receive();
        assert!(manual.running && !manual.resetting);
        manual.send(Operation::Step(0));
        work.try_recv().unwrap();
        manual.enqueue(Operation::Step(1));
        results.send(response(10, true)).unwrap();
        manual.receive();
        assert!(manual.ended && manual.queued.is_none());
        manual.controls(&keys);
        assert!(work.is_empty());
        keys.reset_all();
        keys.press(KeyCode::KeyR);
        manual.controls(&keys);
        assert_eq!(work.try_recv().unwrap(), Operation::Reset);
    }

    #[test]
    fn hud_distinguishes_clock_wait_from_physical_interlocks() {
        let (mut manual, _, _) = controller();
        manual.frame = Some(json!({"metrics": {"decision_wait_ms": 50,
            "decision_due_next": false, "clock_episode_stream": 0,
            "clock_masked": true, "action_masked": true}}));
        let hud = manual.hud();
        assert!(hud.contains("50 ms until next"));
        assert!(hud.contains("waiting for decision clock"));
        manual.frame = Some(json!({"metrics": {"action_masked": true}}));
        let hud = manual.hud();
        assert!(hud.contains("Decision clock: OFF"));
        assert!(hud.contains("fire-control mask forbids fire"));
    }

    #[test]
    fn joint_slot_keys_and_fire_use_nine_actions_without_queuing_shots() {
        let (mut manual, requests, results) = controller();
        manual.running = true;
        manual.frame = Some(json!({"metrics": {"action_mode": "joint"}}));
        let mut keys = ButtonInput::default();
        keys.press(KeyCode::Digit3);
        keys.press(KeyCode::KeyF);
        manual.controls(&keys);
        assert_eq!(requests.recv().unwrap(), Operation::Step(6));
        results.send(Ok(json!({"data": {}, "metrics": {"step": 1, "action_mode": "joint"}}))).unwrap();
        manual.receive();
        manual.next_step = Instant::now();
        keys.clear();
        keys.release(KeyCode::KeyF);
        manual.controls(&keys);
        assert_eq!(requests.recv().unwrap(), Operation::Step(5));
    }

    #[test]
    fn protocol_eof_and_bad_sequence_fail_and_preserve_last_frame() {
        assert!(read_reply(1, &mut io::Cursor::new("")).is_err());
        assert!(read_reply(1, &mut io::Cursor::new("{\"version\":1,\"id\":2,\"ok\":true}\n")).is_err());
        let (mut manual, _, results) = controller();
        manual.frame = Some(json!({"metrics": {"step": 42}}));
        results.send(Err("connection lost".into())).unwrap();
        assert!(manual.receive().is_none());
        assert_eq!(manual.frame.as_ref().unwrap()["metrics"]["step"], 42);
        assert!(manual.fault.is_some() && !manual.running);
    }
}

fn read_reply(id: u64, input: &mut impl BufRead) -> Result<Value, String> {
    let mut line = String::new();
    if input.read_line(&mut line).map_err(|e| e.to_string())? == 0 {
        return Err("Python controller disconnected".into());
    }
    let reply: Value = serde_json::from_str(&line).map_err(|e| format!("Invalid controller JSON: {e}"))?;
    if reply["version"] != 1 || reply["id"] != id {
        return Err("Controller version/request ID mismatch".into());
    }
    if reply["ok"] != true {
        return Err(reply["error"].as_str().unwrap_or("Controller failed").into());
    }
    super::replay::validate_data(&reply["data"])?;
    for field in ["controlled_position_bevy_m", "target_position_bevy_m"] {
        let value = &reply["data"]["evaluation"]["scenario"][field];
        if !value.as_array().is_some_and(|a| a.len() == 3 && a.iter().all(|v| v.as_f64().is_some_and(f64::is_finite))) {
            return Err("Invalid controller scenario".into());
        }
    }
    if reply["time_ns"].as_u64().is_none() || !reply["metrics"].is_object() {
        return Err("Missing controller time/metrics".into());
    }
    Ok(reply)
}

impl Manual {
    pub(super) fn start() -> Self {
        let (requests, work) = bounded(1);
        let (results, replies) = bounded(1);
        let write_errors = results.clone();
        std::thread::spawn(move || {
            let mut output = io::stdout().lock();
            for (index, op) in work.iter().enumerate() {
                let id = index as u64 + 1;
                let request = match op {
                    Operation::Reset => json!({"version": 1, "id": id, "op": "reset"}),
                    Operation::Step(action) => json!({"version": 1, "id": id, "op": "step", "action": action}),
                };
                if let Err(error) = writeln!(output, "{request}").and_then(|_| output.flush()) {
                    let _ = write_errors.send(Err(error.to_string()));
                    break;
                }
            }
        });
        // Read even while paused, so a controller EOF is reported without another keypress.
        std::thread::spawn(move || {
            let mut input = io::stdin().lock();
            for id in 1.. {
                let result = read_reply(id, &mut input);
                let failed = result.is_err();
                if results.send(result).is_err() || failed { break; }
            }
        });
        let mut manual = Self {
            requests, replies, busy: false, resetting: false, queued: None, running: false, ended: false,
            fault: None, notice: "Warming up with firing disabled...".into(),
            next_step: Instant::now(), frame: None, history: VecDeque::new(), desired_slot: None,
        };
        manual.send(Operation::Reset);
        manual
    }

    fn fail(&mut self, error: String) {
        self.fault = Some(error);
        self.running = false;
        self.busy = false;
        self.queued = None;
    }

    fn send(&mut self, op: Operation) {
        if let Err(error) = self.requests.try_send(op) {
            self.fail(error.to_string());
            return;
        }
        self.busy = true;
        self.next_step = Instant::now() + Duration::from_millis(10);
        if op == Operation::Reset {
            self.resetting = true;
            self.desired_slot = None;
            self.running = false;
            self.notice = "Warming up; previous frame retained until reset completes".into();
        }
    }

    fn enqueue(&mut self, op: Operation) {
        if self.fault.is_some() { return; }
        if self.ended && op != Operation::Reset {
            self.notice = "Episode ended. Press R to reset.".into();
            return;
        }
        if self.queued.is_some() {
            self.notice = "BUSY: one operation already queued; this key was not accepted".into();
        } else {
            self.queued = Some(op);
            self.notice = if self.busy { "One operation queued" } else { "" }.into();
        }
    }

    pub(super) fn controls(&mut self, keys: &ButtonInput<KeyCode>) {
        if self.resetting {
            if [KeyCode::Space, KeyCode::KeyF, KeyCode::KeyN, KeyCode::KeyR].iter().any(|k| keys.just_pressed(*k)) {
                self.notice = "Warming up: input not accepted until Reset completes".into();
            }
            return;
        }
        if self.frame.as_ref().is_some_and(|f| f["metrics"]["action_mode"] == "joint") {
            for (slot, key) in [KeyCode::Digit1, KeyCode::Digit2, KeyCode::Digit3, KeyCode::Digit4].iter().enumerate() {
                if keys.just_pressed(*key) { self.desired_slot = Some(slot as u8); }
            }
        }
        if keys.just_pressed(KeyCode::Space) && self.fault.is_none() && !self.ended {
            self.running = !self.running;
            if !self.running {
                self.queued = None;
                self.notice = "Paused (the in-flight step may finish)".into();
            } else {
                self.notice = "Running. Hold F to fire; release to track.".into();
            }
            // A pause wins over other keys pressed in the same frame.
            if !self.running { return; }
        }
        if keys.just_pressed(KeyCode::KeyR) {
            self.running = false;
            self.enqueue(Operation::Reset);
        }
        if !self.busy && self.fault.is_none() {
            if let Some(op) = self.queued.take() {
                self.send(op);
            } else if self.running && !self.ended && Instant::now() >= self.next_step {
                // Sample the current key state only when submitting the next Gym step.
                // Never queue shots while busy: a release must affect the next operation.
                let fire = u8::from(keys.pressed(KeyCode::KeyF));
                let joint = self.frame.as_ref().is_some_and(|f| f["metrics"]["action_mode"] == "joint");
                let action = if joint { self.desired_slot.map_or(0, |slot| 1 + 2 * slot + fire) } else { fire };
                self.send(Operation::Step(action));
            }
        }
    }

    pub(super) fn receive(&mut self) -> Option<Value> {
        if self.fault.is_some() { return None; }
        match self.replies.try_recv() {
            Ok(Ok(frame)) => {
                if !self.busy {
                    self.fail("Unsolicited controller response".into());
                    return None;
                }
                self.busy = false;
                self.resetting = false;
                let m = &frame["metrics"];
                self.ended = m["terminated"] == true || m["truncated"] == true;
                if m["step"] == 0 || self.ended {
                    self.running = !self.ended;
                    self.queued = None;
                    self.notice = if self.ended { "Episode ended. Press R to reset." }
                                  else { "Running. Hold F to fire; release to track." }.into();
                }
                if m["step"] == 0 { self.history.clear(); }
                self.history.push_back(frame["data"].clone());
                while self.history.len() > 21 { self.history.pop_front(); }
                let data = frame["data"].clone();
                self.frame = Some(frame);
                Some(data)
            }
            Ok(Err(error)) => { self.fail(error); None }
            Err(TryRecvError::Disconnected) => { self.fail("Controller communication stopped".into()); None }
            Err(TryRecvError::Empty) => None,
        }
    }

    pub(super) fn hud(&self) -> String {
        let status = if self.fault.is_some() { "FAULT" } else if self.ended { "ENDED" }
                     else if self.running { "RUNNING" } else { "PAUSED" };
        let detail = self.frame.as_ref().map_or_else(|| "Waiting for Gym Reset...".into(), |frame| {
            let m = &frame["metrics"];
            let last = m["last_nonzero_reward"].as_f64().map_or_else(|| "--".into(), |r|
                format!("{r:.0} at step {}", m["last_nonzero_step"]));
            let length = if m["length_overridden"] == true {
                format!("{} steps (OVERRIDE; saved {})", m["episode_steps"], m["original_episode_steps"])
            } else { format!("{} steps", m["episode_steps"]) };
            let clock = m["decision_wait_ms"].as_u64().map_or_else(
                || "Decision clock: OFF (10 ms opportunities)".into(),
                |wait| format!("Decision clock: {} ms until next | Due NOW: {} | Stream: {}",
                    wait, m["decision_due_next"], m["clock_episode_stream"]));
            let selection = if m["action_mode"] == "joint" {
                format!("\nJoint | Desired slot: {} | Selected: {} | Switches: {} | Mask: {}",
                        self.desired_slot.map_or_else(|| "--".into(), |slot| slot.to_string()), m["selected_slot"], m["slot_switches"], m["action_mask"])
            } else { String::new() };
            format!(
                "Episode {} | step {}/{} | {:.2} s\nStep reward: {:.0} | Total reward: {:.0} | Last nonzero: {}\n{}\nFire legal NOW: {} | Last input: {} | Masked: {}\nShot requested: {} | Accepted: {} | Actual launches: {}\nReject: {} | End: {} | Length: {}{}",
                m["episode"], m["step"], m["episode_steps"], m["time_s"].as_f64().unwrap_or(0.0),
                m["reward"].as_f64().unwrap_or(0.0), m["cumulative_reward"].as_f64().unwrap_or(0.0), last, clock,
                m["fire_legal_next"], m["action"],
                m["action_masked"], m["shot_requested"], m["shot_accepted"], m["actual_shots"],
                if m["clock_masked"] == true { "waiting for decision clock" }
                    else if m["action_masked"] == true { "fire-control mask forbids fire" }
                    else { m["reject_reason"].as_str().unwrap_or("--") },
                m["end_reason"].as_str().unwrap_or("--"), length, selection,
            )
        });
        format!("MANUAL GYM | {status}{}\n{detail}\n{}\nJoint: 1-4 select plate | Hold F to fire | Release F to track | Space pause/resume | R restart\nC camera | Arrows orbit | PgUp/PgDn zoom | Close to exit",
                if self.busy { " | BUSY" } else { "" }, self.fault.as_ref().unwrap_or(&self.notice))
    }

    pub(super) fn draw(&self, gizmos: &mut Gizmos) {
        super::replay::draw_samples(self.history.iter(), gizmos);
    }
}
