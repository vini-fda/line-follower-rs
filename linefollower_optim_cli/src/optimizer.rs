use linefollower_core::simulation::robot::RobotSimulation;
use linefollower_core::{geometry::closed_path::ClosedPath, ode_solver::ode_system::Vector};
use cmaes::{
    objective_function::Scale,
    restart::{RestartOptions, BIPOP, IPOP},
    CMAESOptions, ObjectiveFunction, ParallelObjectiveFunction, PlotOptions,
};
use std::sync::Arc;

pub struct RobotOptimizer {
    max_iter: usize,
    path: Arc<ClosedPath<f64>>,
    dt: f64,
}
const KP: f64 = 2.565933287511912;
const KI: f64 = 52.33814267275805;
const KD: f64 = 10.549477731373042;
const SPEED: f64 = 1.4602563968294984;
impl RobotOptimizer {
    pub fn new(max_iter: usize, dt: f64, path: Arc<ClosedPath<f64>>) -> Self {
        Self { max_iter, path, dt }
    }

    fn evaluate_fitness(&self, kp: f64, ki: f64, kd: f64, speed: f64) -> f64 {
        let x0 = Vector::<7>::from_column_slice(&[0.0, -4.0, 0.1, 0.0, 0.0, 0.0, 0.0]);
        let mut robot_sim = RobotSimulation::new(x0, kp, ki, kd, speed, self.path.clone());
        let mut fitness = 0.0;
        const W0: f64 = 0.5;
        const W1: f64 = 0.9;
        for _ in 0..self.max_iter {
            let e = robot_sim.robot_error();
            let dist_err = robot_sim.robot_sdf_to_path();
            let dist_err = dist_err * dist_err;
            let ve = robot_sim.robot_velocity_reward();
            fitness += (ve - W0 * e - W1 * dist_err) * self.dt;
            robot_sim.step(self.dt);
        }
        fitness
    }

    pub fn find_optimal_multithreaded(&self) -> cmaes::DVector<f64> {
        let x0 = vec![KP, KI, KD, SPEED];
        let mut cmaes_state = CMAESOptions::new(x0, 0.1)
            .mode(cmaes::Mode::Maximize)
            .population_size(300)
            .weights(cmaes::Weights::Negative)
            .enable_plot(PlotOptions::new(0, false))
            .build(self)
            .unwrap();
        let soln = cmaes_state.run_parallel();
        // get date and time to put in filename
        let now = chrono::Local::now();
        let filename = format!("plot_{}.png", now.format("%Y-%m-%d_%H-%M-%S"));
        soln.reasons.iter().for_each(|r| println!("{:?}", r));
        cmaes_state
            .get_plot()
            .unwrap()
            .save_to_file(filename, true)
            .unwrap();
        soln.overall_best.unwrap().point

        // let restarter = RestartOptions::new(4, 0.0..=1.0, cmaes::restart::RestartStrategy::BIPOP(BIPOP::default()))
        //                 .mode(cmaes::Mode::Maximize)
        //                 .enable_printing(true)
        //                 .build()
        //                 .unwrap();
        // let f = |x: &cmaes::DVector<f64>| self.evaluate_parallel(x);
        // let results = restarter.run_parallel(|| f);
        // results.best.unwrap().point
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
