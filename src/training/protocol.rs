//! Versioned, length-prefixed local training transport. One server owns one environment.
//! A reconnect retries the last transaction; older IDs are deliberately not replayable.
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::io::{self, Read, Write};
use std::os::unix::fs::{MetadataExt, PermissionsExt};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};

pub const VERSION: u32 = 1;
pub const CONTROL_DT_NS: u64 = 10_000_000;
pub const MAX_MESSAGE_BYTES: usize = 16 * 1024 * 1024;

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
pub struct Request {
    pub version: u32,
    pub request_id: u64,
    #[serde(flatten)]
    pub operation: Operation,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum Operation {
    Reset {
        seed: u64,
        scenario: Value,
    },
    Advance {
        round_id: u64,
        step_id: u64,
        command: Command,
    },
    EndWindow {
        round_id: u64,
        step_id: u64,
        max_settle_steps: u64,
    },
    Settle {
        round_id: u64,
        step_id: u64,
    },
    Inspect,
    Close,
}

/// Controller output, not an RL action. Angles follow ROS gimbal convention, in radians.
/// `fire` is the pulse level produced by the shared control core, not a shot confirmation.
#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Command {
    pub valid: bool,
    pub yaw_rad: f64,
    pub pitch_rad: f64,
    pub distance_m: f64,
    pub fire: bool,
}
impl Command {
    fn validate(&self) -> Result<(), String> {
        if !self.yaw_rad.is_finite()
            || !self.pitch_rad.is_finite()
            || !self.distance_m.is_finite()
            || self.distance_m < 0.0
            || self.distance_m > f32::MAX as f64
            || self.yaw_rad.to_degrees().abs() > f32::MAX as f64
            || self.pitch_rad.to_degrees().abs() > f32::MAX as f64
        {
            return Err("command requires finite angles and nonnegative distance".into());
        }
        if !self.valid && self.fire {
            return Err("an invalid command cannot request fire".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Response {
    pub version: u32,
    pub request_id: u64,
    pub ok: bool,
    pub round_id: u64,
    pub step_id: u64,
    pub sim_time_ns: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// Backends validate before mutating. Any backend error latches the session until reset,
/// because a failed physical step may already have partly changed the world.
pub struct SettlementReply {
    pub data: Value,
    pub finished: bool,
}

pub trait Environment {
    fn reset(&mut self, round_id: u64, seed: u64, scenario: &Value) -> Result<Value, String>;
    fn advance(&mut self, command: Command) -> Result<Value, String>;
    fn inspect(&mut self) -> Result<Value, String>;
    fn end_window(&mut self, _: u64) -> Result<SettlementReply, String> {
        Err("settlement unsupported".into())
    }
    fn settle(&mut self) -> Result<SettlementReply, String> {
        Err("settlement unsupported".into())
    }
}

pub struct Session<E> {
    environment: E,
    round_id: u64,
    step_id: u64,
    failed: bool,
    closed: bool,
    window_closed: bool,
    settlement_done: bool,
    last: Option<(Request, Response)>,
}
impl<E: Environment> Session<E> {
    pub fn new(environment: E) -> Self {
        Self {
            environment,
            round_id: 0,
            step_id: 0,
            failed: false,
            closed: false,
            window_closed: false,
            settlement_done: false,
            last: None,
        }
    }

    fn response(&self, request_id: u64, result: Result<Value, String>) -> Response {
        let (data, error) = match result {
            Ok(v) => (Some(v), None),
            Err(e) => (None, Some(e)),
        };
        Response {
            version: VERSION,
            request_id,
            ok: error.is_none(),
            round_id: self.round_id,
            step_id: self.step_id,
            sim_time_ns: self.step_id * CONTROL_DT_NS,
            data,
            error,
        }
    }

    pub fn handle(&mut self, request: Request) -> Response {
        if let Some((previous, response)) = &self.last {
            if request.request_id == previous.request_id {
                return if request == *previous {
                    response.clone()
                } else {
                    self.response(
                        request.request_id,
                        Err("request_id reused with different content".into()),
                    )
                };
            }
            if request.request_id < previous.request_id {
                return self.response(request.request_id, Err("stale request_id".into()));
            }
        }
        if request.version != VERSION || request.request_id == 0 {
            return self.response(
                request.request_id,
                Err("unsupported version or zero request_id".into()),
            );
        }
        let result = self.execute(&request.operation);
        let mut response = self.response(request.request_id, result);
        if serde_json::to_vec(&response).map_or(true, |bytes| bytes.len() > MAX_MESSAGE_BYTES) {
            self.failed = true;
            response = self.response(
                request.request_id,
                Err("response exceeds limit; reset required".into()),
            );
        }
        // Cache before writing to the socket. If the reply is lost, retry does not step again.
        self.last = Some((request, response.clone()));
        response
    }

    fn check_live_round(&self, round_id: u64) -> Result<(), String> {
        if self.round_id == 0 {
            return Err("reset required".into());
        }
        if self.failed {
            return Err("backend failed; reset required".into());
        }
        if round_id != self.round_id {
            return Err("round_id mismatch".into());
        }
        Ok(())
    }

    fn execute(&mut self, op: &Operation) -> Result<Value, String> {
        if self.closed {
            return Err("session closed".into());
        }
        match op {
            Operation::Reset { seed, scenario } => {
                let round_id = self.round_id.checked_add(1).ok_or("round_id overflow")?;
                match self.environment.reset(round_id, *seed, scenario) {
                    Ok(data) => {
                        self.round_id = round_id;
                        self.step_id = 0;
                        self.failed = false;
                        self.window_closed = false;
                        self.settlement_done = false;
                        Ok(data)
                    }
                    Err(e) => {
                        self.failed = true;
                        Err(e)
                    }
                }
            }
            Operation::Advance {
                round_id,
                step_id,
                command,
            } => {
                if self.round_id == 0 {
                    return Err("reset required".into());
                }
                if self.failed {
                    return Err("backend failed; reset required".into());
                }
                if *round_id != self.round_id {
                    return Err("round_id mismatch".into());
                }
                if self.step_id.checked_add(1) != Some(*step_id)
                    || *step_id > u64::MAX / CONTROL_DT_NS
                {
                    return Err("step_id must be the next step".into());
                }
                if self.window_closed {
                    return Err("evaluation window closed; use settle or reset".into());
                }
                command.validate()?;
                match self.environment.advance(*command) {
                    Ok(data) => {
                        self.step_id = *step_id;
                        Ok(data)
                    }
                    Err(e) => {
                        self.failed = true;
                        Err(e)
                    }
                }
            }
            Operation::EndWindow {
                round_id,
                step_id,
                max_settle_steps,
            } => {
                self.check_live_round(*round_id)?;
                if self.window_closed {
                    return Err("evaluation window already closed".into());
                }
                if *step_id != self.step_id {
                    return Err("end_window requires current step_id".into());
                }
                if !(1..=6000).contains(max_settle_steps)
                    || self
                        .step_id
                        .checked_add(*max_settle_steps)
                        .is_none_or(|n| n > u64::MAX / CONTROL_DT_NS)
                {
                    return Err("max_settle_steps must be in 1..=6000 within clock range".into());
                }
                match self.environment.end_window(*max_settle_steps) {
                    Ok(reply) => {
                        self.window_closed = true;
                        self.settlement_done = reply.finished;
                        Ok(reply.data)
                    }
                    Err(e) => {
                        self.failed = true;
                        Err(e)
                    }
                }
            }
            Operation::Settle { round_id, step_id } => {
                self.check_live_round(*round_id)?;
                if !self.window_closed {
                    return Err("end_window required before settle".into());
                }
                if self.settlement_done {
                    return Err("settlement finished; inspect or reset required".into());
                }
                if self.step_id.checked_add(1) != Some(*step_id)
                    || *step_id > u64::MAX / CONTROL_DT_NS
                {
                    return Err("step_id must be the next step".into());
                }
                match self.environment.settle() {
                    Ok(reply) => {
                        self.step_id = *step_id;
                        self.settlement_done = reply.finished;
                        Ok(reply.data)
                    }
                    Err(e) => {
                        self.failed = true;
                        Err(e)
                    }
                }
            }
            Operation::Inspect => {
                if self.round_id == 0 {
                    return Err("reset required".into());
                }
                if self.failed {
                    return Err("backend failed; reset required".into());
                }
                self.environment.inspect()
            }
            Operation::Close => {
                self.closed = true;
                Ok(serde_json::json!({"closed":true}))
            }
        }
    }
}

/// EOF between frames is a disconnect; EOF inside a frame is an incomplete request.
pub fn read_frame(reader: &mut impl Read) -> io::Result<Option<Vec<u8>>> {
    let mut prefix = [0_u8; 4];
    loop {
        match reader.read(&mut prefix[..1]) {
            Ok(0) => return Ok(None),
            Ok(_) => break,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        }
    }
    reader.read_exact(&mut prefix[1..])?;
    let size = u32::from_be_bytes(prefix) as usize;
    if size == 0 || size > MAX_MESSAGE_BYTES {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid frame length",
        ));
    }
    let mut bytes = vec![0; size];
    reader.read_exact(&mut bytes)?;
    Ok(Some(bytes))
}

pub fn write_frame(writer: &mut impl Write, bytes: &[u8]) -> io::Result<()> {
    if bytes.is_empty() || bytes.len() > MAX_MESSAGE_BYTES {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid response length",
        ));
    }
    writer.write_all(&(bytes.len() as u32).to_be_bytes())?;
    writer.write_all(bytes)?;
    writer.flush()
}

/// Never unlink a pre-existing path, including another server's live socket.
pub struct SocketServer {
    listener: UnixListener,
    path: PathBuf,
    identity: (u64, u64),
}
impl SocketServer {
    pub fn bind(path: &Path) -> io::Result<Self> {
        let listener = UnixListener::bind(path)?;
        let metadata = std::fs::symlink_metadata(path)?;
        let server = Self {
            listener,
            path: path.to_owned(),
            identity: (metadata.dev(), metadata.ino()),
        };
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
        Ok(server)
    }

    pub fn serve<E: Environment>(&self, session: &mut Session<E>) -> io::Result<()> {
        loop {
            let (mut stream, _) = self.listener.accept()?;
            // A disconnected client may reconnect to retrieve the last committed result.
            if let Err(error) = serve_connection(&mut stream, session) {
                eprintln!("training connection closed: {error}");
            }
            if session.closed {
                return Ok(());
            }
        }
    }
}
impl Drop for SocketServer {
    fn drop(&mut self) {
        if let Ok(metadata) = std::fs::symlink_metadata(&self.path) {
            if (metadata.dev(), metadata.ino()) == self.identity {
                let _ = std::fs::remove_file(&self.path);
            }
        }
    }
}

pub fn parse_request(bytes: &[u8]) -> Result<Request, String> {
    let value: Value = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
    let object = value.as_object().ok_or("request must be an object")?;
    let fields: &[&str] = match object.get("op").and_then(Value::as_str) {
        Some("reset") => &["version", "request_id", "op", "seed", "scenario"],
        Some("advance") => &[
            "version",
            "request_id",
            "op",
            "round_id",
            "step_id",
            "command",
        ],
        Some("end_window") => &[
            "version",
            "request_id",
            "op",
            "round_id",
            "step_id",
            "max_settle_steps",
        ],
        Some("settle") => &["version", "request_id", "op", "round_id", "step_id"],
        Some("inspect" | "close") => &["version", "request_id", "op"],
        _ => return Err("unknown operation".into()),
    };
    if object.keys().any(|key| !fields.contains(&key.as_str())) {
        return Err("unknown request field".into());
    }
    serde_json::from_value(value).map_err(|e| e.to_string())
}

fn serve_connection<E: Environment>(
    stream: &mut UnixStream,
    session: &mut Session<E>,
) -> io::Result<()> {
    while let Some(bytes) = read_frame(stream)? {
        let response = match parse_request(&bytes) {
            Ok(request) => session.handle(request),
            Err(error) => session.response(0, Err(format!("invalid request: {error}"))),
        };
        let bytes = serde_json::to_vec(&response).map_err(io::Error::other)?;
        write_frame(stream, &bytes)?;
        if session.closed {
            break;
        }
    }
    Ok(())
}
