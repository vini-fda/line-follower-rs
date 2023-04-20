use super::ode_system::OdeSystem;

type Vector<const N: usize> = nalgebra::SVector<f64, N>;

pub trait Integrator<const N: usize> {
    fn step(&mut self, dt: f64);
    fn get_state(&self) -> Vector<N>;
}

/// Runge-Kutta 4th order integrator
pub struct Rk4<F, const N: usize> 
where F: Fn(f64, &Vector<N>) -> Vector<N> {
    system: OdeSystem<F, N>,
}

impl<F, const N: usize> Rk4<F, N> 
where F: Fn(f64, &Vector<N>) -> Vector<N> {
    pub fn new(f: F, t: f64, x: Vector<N>) -> Self {
        Self {
            system: OdeSystem { t, x, f },
        }
    }

    pub fn step(&mut self, dt: f64) {
        let f = &self.system.f;
        let t = self.system.t;
        let x = &self.system.x;

        let k1 = f(t, x);
        let k2 = f(t + dt / 2.0, &(x + dt * k1 / 2.0));
        let k3 = f(t + dt / 2.0, &(x + dt * k2 / 2.0));
        let k4 = f(t + dt, &(x + dt * k3));

        self.system.x += dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        self.system.t += dt;
    }

    pub fn get_state(&self) -> Vector<N> {
        self.system.x
    }
}

impl<const N:usize> Integrator<N> for Rk4<fn(f64, &Vector<N>) -> Vector<N>, N> {
    fn step(&mut self, dt: f64) {
        self.step(dt);
    }

    fn get_state(&self) -> Vector<N> {
        self.system.x
    }
}

/// Verlet integrator
pub struct Verlet<F, const N: usize>
 where F: Fn(f64, &Vector<N>) -> Vector<N> {
    system: OdeSystem<F, N>,
    x_prev: Vector<N>,
}

impl<F, const N: usize> Verlet<F, N> 
where F: Fn(f64, &Vector<N>) -> Vector<N> {
    pub fn new(f: F, t: f64, x: Vector<N>) -> Self {
        Self {
            system: OdeSystem { t, x, f },
            x_prev: x,
        }
    }

    pub fn step(&mut self, dt: f64) {
        let f = &self.system.f;
        let t = self.system.t;
        let x = &self.system.x;
        let x_prev = &self.x_prev;

        let x_next = 2.0 * x - x_prev + dt * dt * f(t, x);
        self.x_prev = *x;
        self.system.x = x_next;
        self.system.t += dt;
    }

    pub fn get_state(&self) -> Vector<N> {
        self.system.x
    }
}

impl<const N:usize> Integrator<N> for Verlet<fn(f64, &Vector<N>) -> Vector<N>, N> {
    fn step(&mut self, dt: f64) {
        self.step(dt);
    }

    fn get_state(&self) -> Vector<N> {
        self.system.x
    }
}