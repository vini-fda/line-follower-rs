use num::Float;
use std::f32::consts::PI;

pub struct Path<F: Float> {
    pub xs: Vec<F>,
    pub ys: Vec<F>,
}

pub fn create_line<F: Float>(x_0: F, y_0: F, x_1: F, y_1: F, n_points: usize) -> Path<F> {
    let mut xs = Vec::with_capacity(n_points);
    let mut ys = Vec::with_capacity(n_points);
    let dx = (x_1 - x_0) / F::from(n_points).unwrap();
    let dy = (y_1 - y_0) / F::from(n_points).unwrap();
    for i in 0..n_points {
        xs.push(x_0 + F::from(i).unwrap() * dx);
        ys.push(y_0 + F::from(i).unwrap() * dy);
    }
    Path::new(xs, ys)
}

pub fn create_arc<F: Float>(
    x_0: F,
    y_0: F,
    r: F,
    theta_0: F,
    theta_1: F,
    n_points: usize,
) -> Path<F> {
    let mut xs = Vec::with_capacity(n_points);
    let mut ys = Vec::with_capacity(n_points);
    let d_theta = (theta_1 - theta_0) / F::from(n_points).unwrap();
    for i in 0..n_points {
        let theta = theta_0 + F::from(i).unwrap() * d_theta;
        xs.push(x_0 + r * theta.cos());
        ys.push(y_0 + r * theta.sin());
    }
    Path::new(xs, ys)
}

pub fn predefined_closed_path() -> Path<f32> {
    create_line(0.0, 0.0, 8.0, 0.0, 100)
        + create_arc(8.0, -2.0, 2.0, PI / 2.0, 0.0, 100)
        + create_line(10.0, -2.0, 10.0, -10.0, 100)
        + create_arc(8.0, -10.0, 2.0, 0., -PI / 2., 100)
        + create_line(8.0, -12.0, 3.0, -12.0, 100)
        + create_arc(3.0, -11.0, 1.0, -PI / 2.0, -3.0 * PI / 2.0, 100)
        + create_line(3.0, -10.0, 7.0, -10.0, 100)
        + create_arc(7.0, -9.0, 1.0, -PI / 2., 0.0, 100)
        + create_line(8.0, -9.0, 8.0, -4.0, 100)
        + create_line(8.0, -4.0, 0.0, -4.0, 100)
        + create_arc(0.0, -2.0, 2.0, -PI / 2.0, -3.0 * PI / 2.0, 100)
}

impl<F> Path<F>
where
    F: Float,
{
    pub fn new(xs: Vec<F>, ys: Vec<F>) -> Self {
        assert!(xs.len() == ys.len(), "xs and ys must have the same length");
        Self { xs, ys }
    }

    pub fn closest_path_point(&self, x_p: F, y_p: F) -> (F, F) {
        // returns the point X on the path that is closest to the point P = (x_p, y_p)
        let mut min_dist = F::infinity();
        let mut closest_point = (F::zero(), F::zero());
        for i in 0..self.xs.len() {
            let x = self.xs[i];
            let y = self.ys[i];
            let dist = (x - x_p).powi(2) + (y - y_p).powi(2);
            if dist < min_dist {
                min_dist = dist;
                closest_point = (x, y);
            }
        }
        closest_point
    }

    pub fn points(&self) -> impl Iterator<Item = (F, F)> + '_ {
        self.xs.iter().zip(self.ys.iter()).map(|(&x, &y)| (x, y))
    }

    pub fn length(&self) -> F {
        let mut length = F::zero();
        for i in 1..self.xs.len() {
            let x_0 = self.xs[i - 1];
            let y_0 = self.ys[i - 1];
            let x_1 = self.xs[i];
            let y_1 = self.ys[i];
            length = length + ((x_1 - x_0).powi(2) + (y_1 - y_0).powi(2)).sqrt();
        }
        length
    }
}

impl<F> std::ops::Add for Path<F>
where
    F: Float,
{
    type Output = Self;
    fn add(self, other: Self) -> Self {
        let mut xs = self.xs;
        let mut ys = self.ys;
        xs.extend(other.xs);
        ys.extend(other.ys);
        Self::new(xs, ys)
    }
}
