use super::ode_system::OdeSystem;

type Vector<const N: usize> = nalgebra::SVector<f64, N>;

pub trait Integrator<const N: usize, const U:usize> {
    fn step(&mut self, dt: f64, u: &Vector<U>);
    fn get_state(&self) -> Vector<N>;
}

/// Runge-Kutta 4th order integrator
pub struct Rk4<F, const N: usize, const U:usize> 
where F: FnMut(f64, &Vector<N>, &Vector<U>) -> Vector<N> {
    system: OdeSystem<F, N, U>,
}

impl<F, const N: usize, const U:usize> Rk4<F, N, U> 
where F: FnMut(f64, &Vector<N>, &Vector<U>) -> Vector<N> {
    pub fn new(f: F, t: f64, x: Vector<N>) -> Self {
        Self {
            system: OdeSystem { t, x, f },
        }
    }

    pub fn step(&mut self, dt: f64, u: &Vector<U>) {
        let f = &mut self.system.f;
        let t = self.system.t;
        let x = &self.system.x;

        let k1 = f(t, x, u);
        let k2 = f(t + dt / 2.0, &(x + dt * k1 / 2.0), u);
        let k3 = f(t + dt / 2.0, &(x + dt * k2 / 2.0), u);
        let k4 = f(t + dt, &(x + dt * k3), u);

        self.system.x += dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        self.system.t += dt;
    }

    pub fn get_state(&self) -> Vector<N> {
        self.system.x
    }
}

impl<const N:usize, const U:usize> Integrator<N, U> for Rk4<fn(f64, &Vector<N>, &Vector<U>) -> Vector<N>, N, U> {
    fn step(&mut self, dt: f64, u: &Vector<U>) {
        self.step(dt, u);
    }

    fn get_state(&self) -> Vector<N> {
        self.system.x
    }
}

/// Verlet integrator
pub struct Verlet<F, const N: usize, const U: usize>
 where F: FnMut(f64, &Vector<N>, &Vector<U>) -> Vector<N> {
    system: OdeSystem<F, N, U>,
    x_prev: Vector<N>,
}

impl<F, const N: usize, const U:usize> Verlet<F, N, U> 
where F: FnMut(f64, &Vector<N>, &Vector<U>) -> Vector<N> {
    pub fn new(f: F, t: f64, x: Vector<N>) -> Self {
        Self {
            system: OdeSystem { t, x, f },
            x_prev: x,
        }
    }

    pub fn step(&mut self, dt: f64, u: &Vector<U>) {
        let f = &mut self.system.f;
        let t = self.system.t;
        let x = &self.system.x;
        let x_prev = &self.x_prev;

        let x_next = 2.0 * x - x_prev + dt * dt * f(t, x, u);
        self.x_prev = *x;
        self.system.x = x_next;
        self.system.t += dt;
    }

    pub fn get_state(&self) -> Vector<N> {
        self.system.x
    }
}

impl<const N:usize, const U:usize> Integrator<N, U> for Verlet<fn(f64, &Vector<N>, &Vector<U>) -> Vector<N>, N, U> {
    fn step(&mut self, dt: f64, u: &Vector<U>) {
        self.step(dt, u);
    }

    fn get_state(&self) -> Vector<N> {
        self.system.x
    }
}