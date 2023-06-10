use linefollower_core::geometry::closed_path::predefined_closed_path;
use linefollower_optim_cli::optimizer::RobotOptimizer;
use std::io::Write;
use std::sync::Arc;

fn main() {
    let main_path_sdf = Arc::new(predefined_closed_path());

    let ts = 1.0 / 240.0;
    let t_total = 1200.0;
    let n = (t_total / ts) as usize;
    println!("Running optimization...");
    let best_ks = RobotOptimizer::new(n, ts, main_path_sdf).find_optimal_multithreaded();
    let now = chrono::Local::now();
    let filename = format!("optimal_params_{}.txt", now.format("%Y-%m-%d_%H-%M-%S"));
    let mut file = std::fs::File::create(filename.clone()).unwrap();
    write!(
        file,
        "best Kp: {}, Ki: {}, Kd: {}, speed: {}",
        best_ks[0], best_ks[1], best_ks[2], best_ks[3]
    )
    .unwrap();
    println!("Wrote outputs to file \"{}\"", filename);
}
