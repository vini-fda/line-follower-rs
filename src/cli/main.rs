use std::sync::Arc;
use line_follower_rs::simulation::optimizer::RobotOptimizer;
use line_follower_rs::geometry::sdf_paths::predefined_closed_path_sdf;

fn main() {
    let main_path_sdf = Arc::new(predefined_closed_path_sdf());

    let ts = 1.0 / 60.0;
    let t_total = 120.0;
    let n = (t_total / ts) as usize;
    let best_ks = RobotOptimizer::new(n, ts, main_path_sdf).find_optimal_multithreaded();
    println!("best Kp: {}, Ki: {}, Kd: {}, speed: {}", best_ks[0], best_ks[1], best_ks[2], best_ks[3]);
}