use linefollower_core::geometry::closed_path::predefined_closed_path;
use linefollower_optim_cli::optimizer::RobotOptimizer;
use std::sync::Arc;

fn main() {
    let main_path_sdf = Arc::new(predefined_closed_path());

    let ts = 1.0 / 240.0;
    let t_total = 120.0;
    let n = (t_total / ts) as usize;
    let best_ks = RobotOptimizer::new(n, ts, main_path_sdf).find_optimal_multithreaded();
    println!(
        "best Kp: {}, Ki: {}, Kd: {}, speed: {}",
        best_ks[0], best_ks[1], best_ks[2], best_ks[3]
    );
}
