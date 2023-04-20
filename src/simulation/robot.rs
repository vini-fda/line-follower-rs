use std::sync::Arc;

use nalgebra::{Rotation2, Vector2};

use crate::geometry::sdf_paths::{predefined_closed_path_sdf, ClosedPath, SDF};
use crate::ode_solver::integrator::Rk4;
use crate::ode_solver::ode_system::Vector;
/// The number of state variables
const NUM_STATES: usize = 7;
/// The number of control variables
const NUM_CONTROLS: usize = 2;
/// Robot geometry
const ROBOT_WHEEL_RADIUS: f64 = 0.04;
const ROBOT_SIDE_LENGTH: f64 = 0.1;
const SENSOR_ARRAY_LENGTH: f64 = ROBOT_SIDE_LENGTH * 1.1;
const SENSOR_ARRAY_SEPARATION: f64 = SENSOR_ARRAY_LENGTH / 5.0;
const MAX_SENSOR_DISTANCE: f64 = 4.0 * SENSOR_ARRAY_SEPARATION / 5.0;
const SENSOR_DISTANCE_TO_ROBOT_CENTER: f64 = ROBOT_SIDE_LENGTH * 3.0 / 5.0;

// Dynamical constants
// DC Motor constants
// const J: f64 = 0.1;
// const B: f64 = 0.1;
// const R: f64 = 0.03;
// const L: f64 = 0.1;
// const K: f64 = 0.1;
// We'll just model using the usual 2nd order system constants for now
const W0: f64 = 10.0;
const XI: f64 = 0.71;
const C0: f64 = 1.0 / (W0 * W0);
const C1: f64 = 2.0 * XI / W0;
const C2: f64 = 1.0;

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
    time: f64,
}

impl RobotSimulation {
    pub fn calculate_control(&mut self, dt: f64) -> Vector<NUM_CONTROLS> {
        // control system

        // estimate the robot's angle relative to the track
        // (i.e. the error in theta) by using the sensor array data
        let error_estimate = find_theta(
            &self.sensor_distances(),
            MAX_SENSOR_DISTANCE,
            ROBOT_SIDE_LENGTH / 2.0,
        );
        let deriv_error = (error_estimate - self.prev_error) / dt;
        self.int_error += error_estimate * dt;

        // PID Constants
        // const KP: f64 = 0.0003;
        // const KI: f64 = 0.0006;
        // const KD: f64 = 0.009;
        // u(t) = Kp * e(t) + Ki * \int e(t) dt + Kd * \frac{de(t)}{dt}
        let desired_dtheta = self.kp * error_estimate + self.ki * self.int_error + self.kd * deriv_error;
        let k = ROBOT_SIDE_LENGTH * C2 / ROBOT_WHEEL_RADIUS;
        let desired_speed = 7.5;
        let v = k * desired_dtheta;

        let ul = (2.0 * desired_speed - v) / 2.0;
        let ur = (2.0 * desired_speed + v) / 2.0;

        Vector2::<f64>::new(ul, ur)
    }

    pub fn get_state(&self) -> Vector<NUM_STATES> {
        self.state
    }

    pub fn get_time(&self) -> f64 {
        self.time
    }

    pub fn new(x0: Vector<NUM_STATES>, kp: f64, ki: f64, kd: f64, path: &Arc<ClosedPath<f64>>) -> Self {
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
            path: path.clone(),
            prev_error: 0.0,
            int_error: 0.0,
            time: 0.0,
            kp,
            ki,
            kd,
        }
    }

    pub fn robot_sdf_to_path(&self) -> f64 {
        if let Some(d) = self.path.sdf(self.state[0], self.state[1]) {
            d
        } else {
            0.0
        }
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

    fn sensor_distances(&self) -> [f64; 5] {
        // we initially consider the robot pointing rightward, so the sensor array is vertical (x constant)
        let mut sensor_positions = [
            Vector2::<f64>::new(
                SENSOR_DISTANCE_TO_ROBOT_CENTER,
                2.0 * SENSOR_ARRAY_SEPARATION,
            ),
            Vector2::<f64>::new(SENSOR_DISTANCE_TO_ROBOT_CENTER, SENSOR_ARRAY_SEPARATION),
            Vector2::<f64>::new(SENSOR_DISTANCE_TO_ROBOT_CENTER, 0.0),
            Vector2::<f64>::new(SENSOR_DISTANCE_TO_ROBOT_CENTER, -SENSOR_ARRAY_SEPARATION),
            Vector2::<f64>::new(
                SENSOR_DISTANCE_TO_ROBOT_CENTER,
                -2.0 * SENSOR_ARRAY_SEPARATION,
            ),
        ];

        // now we rotate the sensor array by theta counter-clockwise
        // and translate it by (x, y)
        for p in sensor_positions.iter_mut() {
            let rotation = Rotation2::new(self.state[2]);
            let rotated = rotation * (*p);
            *p = Vector2::<f64>::new(self.state[0], self.state[1]) + rotated;
        }

        let mut sensor_distances = [0.0f64; 5];
        for i in 0..5 {
            if let Some(d) = self.path.sdf(sensor_positions[i].x, sensor_positions[i].y) {
                sensor_distances[i] = d.abs();
            } else {
                sensor_distances[i] = 1e10;
            }
        }
        sensor_distances
    }

    pub fn step(&mut self, dt: f64) {
        self.integrator.step(dt, &self.controls);
        self.state = self.integrator.get_state();
        self.controls = self.calculate_control(dt);
        self.time += dt;
    }
}

/// Attempts to estimate the angle between the robot and the track by looking at the sensor readings
/// Pretty much uses the formula y = mx + b, but with some extra checks to make sure the sensors
/// are either on one side of the track or the other (or if they cross the track)
#[inline(always)]
fn find_theta(y: &[f64; 5], l: f64, d: f64) -> f64 {
    // if sensor readings are increasing, then the sensor array is on the right side of the track
    let mut increasing = true;

    for i in 0..y.len() - 1 {
        if y[i] > y[i + 1] {
            increasing = false;
            break;
        }
    }
    if increasing {
        let m = (y[4] - y[0]) / l;
        if m.abs() <= 1.0 {
            return m.acos();
        }
    }

    // if sensor readings are decreasing, then the sensor array is on the left side of the track
    let mut decreasing = true;

    for i in 0..y.len() - 1 {
        if y[i] < y[i + 1] {
            decreasing = false;
            break;
        }
    }

    if decreasing {
        let m = (y[0] - y[4]) / l;
        if m.abs() <= 1.0 {
            return m.acos();
        }
    }

    // avoid division by zero
    if (y[4] - y[0]).abs() < 1e-6 {
        return 0.0;
    }

    // if the sensor readings cross the track, then it is V shaped
    // so we find the point where the sensor readings cross the track (0.0 <= t <= 1.0)
    let t = y[0] / (y[4] - y[0]);
    ((0.5 + t) / d).atan()
}