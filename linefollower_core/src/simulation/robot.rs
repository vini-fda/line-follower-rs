use std::sync::Arc;

use nalgebra::{distance_squared, Point2, Vector2};

use crate::geometry::closed_path::ClosedPath;
use crate::geometry::track::Track;
use crate::ode_solver::integrator::Rk4;
use crate::ode_solver::ode_system::Vector;
/// The number of state variables
const NUM_STATES: usize = 7;
/// The number of control variables
const NUM_CONTROLS: usize = 2;
/// Robot geometry
const ROBOT_WHEEL_RADIUS: f64 = 0.04;
const ROBOT_SIDE_LENGTH: f64 = 0.1;

// Dynamical constants
// DC Motor constants
// const J: f64 = 0.1;
// const B: f64 = 0.1;
// const R: f64 = 0.03;
// const L: f64 = 0.1;
// const K: f64 = 0.1;
// We'll just model using the usual 2nd order system constants for now
const W0: f64 = 20.0;
const XI: f64 = 0.71;
const C0: f64 = 1.0 / (W0 * W0);
const C1: f64 = 2.0 * XI / W0;
const C2: f64 = 1.0;

//const DESIRED_SPEED: f64 = 7.5;

pub struct RobotSimulation {
    integrator: Rk4<
        fn(f64, &Vector<NUM_STATES>, &Vector<NUM_CONTROLS>) -> Vector<NUM_STATES>,
        NUM_STATES,
        NUM_CONTROLS,
    >,
    state: Vector<NUM_STATES>,
    controls: Vector<NUM_CONTROLS>,
    path: Arc<ClosedPath<f64>>,
    prev_error: f64,
    int_error: f64,
    kp: f64,
    ki: f64,
    kd: f64,
    speed: f64,
    time: f64,
}

impl RobotSimulation {
    pub fn new(
        x0: Vector<NUM_STATES>,
        kp: f64,
        ki: f64,
        kd: f64,
        speed: f64,
        path: Arc<ClosedPath<f64>>,
    ) -> Self {
        let x = x0;
        let u = Vector::<NUM_CONTROLS>::zeros();
        let integrator = Rk4::new(
            Self::robot_dynamics
                as fn(f64, &Vector<NUM_STATES>, &Vector<NUM_CONTROLS>) -> Vector<NUM_STATES>,
            0.0,
            x,
        );

        Self {
            integrator,
            state: x,
            controls: u,
            path,
            prev_error: 0.0,
            int_error: 0.0,
            time: 0.0,
            kp,
            ki,
            kd,
            speed,
        }
    }

    pub fn theta_error_estimate(&self) -> f64 {
        self.robot_sdf_to_path()
    }

    pub fn get_state(&self) -> Vector<NUM_STATES> {
        self.state
    }

    pub fn get_time(&self) -> f64 {
        self.time
    }

    pub fn robot_position(&self) -> Point2<f64> {
        Point2::<f64>::new(self.state[0], self.state[1])
    }

    pub fn robot_sdf_to_path(&self) -> f64 {
        self.path.sdf(self.robot_position())
    }

    /// Error relative to the trajectory defined by the reference position
    pub fn robot_error(&self) -> f64 {
        distance_squared(&self.reference_point(), &self.robot_position())
    }

    /// Dot product of the robot's velocity with the tangent of reference position
    pub fn robot_velocity_reward(&self) -> f64 {
        let (wl, wr) = (self.state[3], self.state[5]);
        let theta = self.state[2];
        let speed = ROBOT_WHEEL_RADIUS * (wl + wr) / 2.0;
        let vx = speed * theta.cos();
        let vy = speed * theta.sin();
        // let (tx, ty) = self.reference_tangent();
        let vt = self.robot_projection_tangent();
        let (tx, ty) = (vt[0], vt[1]);
        vx * tx + vy * ty
    }

    pub fn reference_point(&self) -> Point2<f64> {
        self.path.point_at(self.speed * self.get_time())
    }

    pub fn reference_tangent(&self) -> Vector2<f64> {
        self.path.tangent_at(self.speed * self.get_time())
    }

    pub fn robot_projection_tangent(&self) -> Vector2<f64> {
        self.path.point_projection_tangent(self.robot_position())
    }

    pub fn step(&mut self, dt: f64) {
        self.controls = self.calculate_control(dt);
        self.integrator.step(dt, &self.controls);
        self.state = self.integrator.get_state();
        self.time += dt;
    }

    fn robot_dynamics(
        _: f64,
        x: &Vector<NUM_STATES>,
        u: &Vector<NUM_CONTROLS>,
    ) -> Vector<NUM_STATES> {
        let (_, _, theta, wl, dwl, wr, dwr) = (x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
        let ul = u[0];
        let ur = u[1];

        let speed = ROBOT_WHEEL_RADIUS * (wl + wr) / 2.0;
        let d_theta = ROBOT_WHEEL_RADIUS * (wr - wl) / ROBOT_SIDE_LENGTH;
        let d_x = speed * theta.cos();
        let d_y = speed * theta.sin();
        let d_wl = dwl;
        let d_dwl = (ul - C1 * dwl - C2 * wl) / C0;
        let d_wr = dwr;
        let d_dwr = (ur - C1 * dwr - C2 * wr) / C0;

        Vector::<7>::from_column_slice(&[d_x, d_y, d_theta, d_wl, d_dwl, d_wr, d_dwr])
    }

    fn calculate_control(&mut self, dt: f64) -> Vector<NUM_CONTROLS> {
        // control system

        // estimate the robot's angle relative to the track
        // (i.e. the error in theta) by using the sensor array data
        let error_estimate = self.theta_error_estimate();
        let deriv_error = (error_estimate - self.prev_error) / dt;
        self.int_error += self.prev_error * dt;
        self.prev_error = error_estimate;

        // PID Constants
        // const KP: f64 = 0.0003;
        // const KI: f64 = 0.0006;
        // const KD: f64 = 0.009;
        // u(t) = Kp * e(t) + Ki * \int e(t) dt + Kd * \frac{de(t)}{dt}
        let desired_dtheta =
            self.kp * error_estimate + self.ki * self.int_error + self.kd * deriv_error;
        let k = ROBOT_SIDE_LENGTH * C2 / ROBOT_WHEEL_RADIUS;

        let v = k * desired_dtheta;
        let um = 2.0 * self.speed * C2 / ROBOT_WHEEL_RADIUS;

        let ul = (um - v) / 2.0;
        let ur = (um + v) / 2.0;

        Vector2::<f64>::new(ul, ur)
    }
}
