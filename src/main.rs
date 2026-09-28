//! CLI CSV du simulateur ALG_COLLISION_AVOIDANCE.

use collision_avoidance_rs::sim::{Params, simuler};
use std::{env, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() || matches!(args[0].as_str(), "-h" | "--aide" | "--help") {
        eprintln!("{USAGE}");
        return ExitCode::SUCCESS;
    }
    let seed = match args[0].parse::<u64>() {
        Ok(v) => v,
        Err(_) => {
            eprintln!("seed invalide");
            return ExitCode::from(2);
        }
    };
    let scenario = args.get(1).and_then(|v| v.parse().ok()).unwrap_or(0);
    let agents = args.get(2).and_then(|v| v.parse().ok()).unwrap_or(8);
    let steps = args.get(3).and_then(|v| v.parse().ok()).unwrap_or(80);
    let distance_min = args.get(4).and_then(|v| v.parse().ok()).unwrap_or(1.0);
    let params = Params {
        scenario,
        agents,
        steps,
        distance_min,
        ..Params::default()
    };
    let r = simuler(seed, &params);
    let reaction = r.premier_temps_reaction.map_or(-1, |v| v as i64);
    println!(
        "seed,scenario,agents,steps,distance_min,collisions,quasi_collisions,interventions,temps_reaction,distance_min_observee,surcout_commande"
    );
    println!(
        "{seed},{scenario},{agents},{steps},{distance_min:.6},{},{},{},{reaction},{:.6},{:.6}",
        r.collisions,
        r.quasi_collisions,
        r.interventions,
        r.distance_min_observee,
        r.surcout_commande
    );
    ExitCode::SUCCESS
}

const USAGE: &str = "collision_avoidance_rs <seed> [scenario] [agents] [steps] [distance_min]\nscenario: 0=nominal, 1=dense, 2=stress";
