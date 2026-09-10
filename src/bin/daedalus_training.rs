//! Local lockstep simulator. No renderer, input device, Talos shared memory or network service.
use clap::Parser;
use daedalus::config::SimulationConfig;
use daedalus::training::{
    environment::PhysicalEnvironment,
    protocol::{Session, SocketServer},
};
use std::path::PathBuf;

#[derive(Parser)]
#[command(about = "Synchronous physical training environment (phase 2)")]
struct Args {
    #[arg(long)]
    socket: PathBuf,
    #[arg(long, default_value = "config.toml")]
    config: PathBuf,
    #[arg(long, default_value = "assets")]
    assets: PathBuf,
    /// Physics tick in microseconds; independent from the fixed 10 ms control period.
    #[arg(long, default_value_t = 1000)]
    physics_step_us: u64,
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let config: SimulationConfig = toml::from_str(&std::fs::read_to_string(args.config)?)?;
    config.projectile.validate_shooter()?;
    let assets = args.assets.canonicalize()?;
    let server = SocketServer::bind(&args.socket)?;
    eprintln!("training socket ready: {}", args.socket.display());
    let dt_ns = args
        .physics_step_us
        .checked_mul(1000)
        .ok_or("physics step overflow")?;
    let environment = PhysicalEnvironment::new(config, assets).with_physics_step_ns(dt_ns)?;
    eprintln!("physics step: {} ns; control step: 10000000 ns", dt_ns);
    server.serve(&mut Session::new(environment))?;
    Ok(())
}
