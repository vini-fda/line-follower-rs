use std::sync::Arc;
use crate::{ode_solver::ode_system::Vector, geometry::sdf_paths::ClosedPath};
use super::robot::RobotSimulation;
use cmaes::{ObjectiveFunction, CMAESOptions, ParallelObjectiveFunction, PlotOptions, restart::{RestartOptions, BIPOP, IPOP}, objective_function::Scale};

pub struct RobotOptimizer {
    max_iter: usize,
    path: Arc<ClosedPath<f64>>,
    dt: f64
}

impl RobotOptimizer {
    pub fn new(max_iter: usize, dt: f64, path: Arc<ClosedPath<f64>>) -> Self {
        Self {
            max_iter,
            path,
            dt
        }
    }

    fn evaluate_fitness(&self, kp: f64, ki: f64, kd: f64, speed: f64) -> f64 {
        let x0 = Vector::<7>::from_column_slice(&[0.0, -4.0, 0.1, 0.0, 0.0, 0.0, 0.0]);
        let mut robot_sim = RobotSimulation::new(x0, kp, ki, kd, speed, self.path.clone());
        let mut fitness = 0.0;
        const W: f64 = 0.1;
        for _ in 0..self.max_iter {
            let e = robot_sim.robot_error();
            let ve = robot_sim.robot_velocity_reward();
            fitness += (ve - W * e.sqrt()) * self.dt;
            robot_sim.step(self.dt);
        }
        fitness
    }

    // pub fn find_optimal_single_threaded(&mut self) -> cmaes::DVector<f64> {
    //     let x0 = vec![0.0003, 0.0006, 0.009];
    //     let mut cmaes_state = CMAESOptions::new(x0, 0.1)
    //                             .build(self)
    //                             .unwrap();
    //     let soln = cmaes_state.run();
    
    //     soln.final_mean
    // }

    pub fn find_optimal_multithreaded(&self) -> cmaes::DVector<f64> {
        // let x0 = vec![0.0003, 0.0006, 0.009, 0.7];
        // let mut cmaes_state = CMAESOptions::new(x0, 0.1)
        //                         .mode(cmaes::Mode::Maximize)
        //                         .population_size(150)
        //                         .weights(cmaes::Weights::Negative)
        //                         .enable_plot(PlotOptions::new(0, false))
        //                         .build(self)
        //                         .unwrap();
        // let soln = cmaes_state.run_parallel();
        // // get date and time to put in filename
        // let now = chrono::Local::now();
        // let filename = format!("plot_{}.png", now.format("%Y-%m-%d_%H-%M-%S"));
        // soln.reasons.iter().for_each(|r| println!("{:?}", r));
        // cmaes_state.get_plot().unwrap().save_to_file(filename, true).unwrap();
        // soln.overall_best.unwrap().point

        let restarter = RestartOptions::new(4, 0.0..=1.0, cmaes::restart::RestartStrategy::IPOP(IPOP::default()))
                        .enable_printing(true)
                        .build()
                        .unwrap();
        let f = |x: &cmaes::DVector<f64>| self.evaluate_parallel(x);
        let scale = Scale::new(f, vec![1.0, 1.0, 1.0, 2.0]);
        let f_scaled = |x: &cmaes::DVector<f64>| scale.evaluate_parallel(x);
        let results = restarter.run_parallel(|| f_scaled);
        results.best.unwrap().point
    }
}

impl ObjectiveFunction for RobotOptimizer {
    fn evaluate(&mut self, x: &cmaes::DVector<f64>) -> f64 {
        let kp = x[0];
        let ki = x[1];
        let kd = x[2];
        let speed = x[3];
        self.evaluate_fitness(kp, ki, kd, speed)
    }
}

impl<'a> ObjectiveFunction for &'a mut RobotOptimizer {
    fn evaluate(&mut self, x: &cmaes::DVector<f64>) -> f64 {
        RobotOptimizer::evaluate(*self, x)
    }
}

impl ParallelObjectiveFunction for RobotOptimizer {
    fn evaluate_parallel(&self, x: &cmaes::DVector<f64>) -> f64 {
        let kp = x[0];
        let ki = x[1];
        let kd = x[2];
        let speed = x[3];
        self.evaluate_fitness(kp, ki, kd, speed)
    }
}

impl<'a> ParallelObjectiveFunction for &'a RobotOptimizer {
    fn evaluate_parallel(&self, x: &cmaes::DVector<f64>) -> f64 {
        RobotOptimizer::evaluate_parallel(*self, x)
    }
}