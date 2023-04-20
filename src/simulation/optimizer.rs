use std::sync::Arc;
use crate::{ode_solver::ode_system::Vector, geometry::sdf_paths::ClosedPath};
use super::robot::RobotSimulation;
use cmaes::{ObjectiveFunction, CMAESOptions, ParallelObjectiveFunction};

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

    fn evaluate_fitness(&self, kp: f64, ki: f64, kd: f64) -> f64 {
        let x0 = Vector::<7>::from_column_slice(&[0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0]);
        let mut robot_sim = RobotSimulation::new(x0, kp, ki, kd, &self.path);
        let mut fitness = 0.0;
        const W: f64 = 0.1;
        for _ in 0..self.max_iter {
            let e = robot_sim.robot_sdf_to_path();
            fitness += - W * e * e;
            robot_sim.step(self.dt);
        }
        fitness
    }

    pub fn find_optimal_single_thread(&mut self) -> cmaes::DVector<f64> {
        let x0 = vec![0.0003, 0.0006, 0.009];
        let mut cmaes_state = CMAESOptions::new(x0, 0.1)
                                .build(self)
                                .unwrap();
        let soln = cmaes_state.run();
    
        soln.final_mean
    }

    pub fn find_optimal_multithreaded(&self) -> cmaes::DVector<f64> {
        let x0 = vec![0.0003, 0.0006, 0.009];
        let mut cmaes_state = CMAESOptions::new(x0, 0.1)
                                .mode(cmaes::Mode::Maximize)
                                .build(self)
                                .unwrap();
        let soln = cmaes_state.run_parallel();
    
        soln.final_mean
    }
}

impl ObjectiveFunction for RobotOptimizer {
    fn evaluate(&mut self, x: &cmaes::DVector<f64>) -> f64 {
        let kp = x[0];
        let ki = x[1];
        let kd = x[2];
        self.evaluate_fitness(kp, ki, kd)
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
        self.evaluate_fitness(kp, ki, kd)
    }
}

impl<'a> ParallelObjectiveFunction for &'a RobotOptimizer {
    fn evaluate_parallel(&self, x: &cmaes::DVector<f64>) -> f64 {
        RobotOptimizer::evaluate_parallel(*self, x)
    }
}