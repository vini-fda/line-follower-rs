use num::Float;
use crate::math_utils::{distance, cross_product, dot_product};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Convex,
    Concave,
}

pub struct ArcPath<F: Float> {
    pub x_0: F,
    pub y_0: F,
    pub r: F,
    pub theta_0: F,
    pub theta_1: F,
    pub direction: Direction,
    extremal_0: (F, F),
    extremal_1: (F, F),
}

impl<F> ArcPath<F> where F: Float + std::fmt::Display {
    fn new(x_0: F, y_0: F, r: F, theta_0: F, theta_1: F, direction: Direction) -> Self {
        let delta_t = theta_1 - theta_0;
        // //let EPSILON = F::from(1e-6).unwrap();
        // println!("delta_t: {}", delta_t);
        // // sin and cos
        // println!("sin: {}", delta_t.sin());
        // println!("cos: {}", delta_t.cos());
        match direction {
            Direction::Convex => assert!(delta_t.sin() >= F::zero() && delta_t != F::zero(), "theta_0 must come before theta_1 (for a convex arc, we consider the counter-clockwise direction as positive)"),
            Direction::Concave => assert!(delta_t.sin() <= F::zero() && delta_t != F::zero(), "theta_0 must come after theta_1 (for a concave arc, we consider the clockwise direction as positive)"),
        }
        let extremal_0 = (x_0 + r * theta_0.cos(), y_0 + r * theta_0.sin());
        let extremal_1 = (x_0 + r * theta_1.cos(), y_0 + r * theta_1.sin());
        Self {
            x_0,
            y_0,
            r,
            theta_0,
            theta_1,
            direction,
            extremal_0,
            extremal_1,
        }
    }

    fn within_bounds(&self, x: F, y: F) -> bool {
        // if (self.x_0 - F::from(7.0).unwrap()).abs() <= F::from(1e-3).unwrap() && (self.y_0 + F::from(9.0).unwrap()).abs() <= F::from(1e-3).unwrap() {
        //     println!("x: {}, y: {}", x, y);
        //     //println!("extremal_0: {:?}", self.extremal_0);
        //     //println!("extremal_1: {:?}", self.extremal_1);
        // }
        let ord0 = cross_product(self.extremal_0.0 - self.x_0, self.extremal_0.1 - self.y_0, x - self.x_0, y - self.y_0);
        let ord1 = cross_product(x - self.x_0, y - self.y_0, self.extremal_1.0 - self.x_0, self.extremal_1.1 - self.y_0);
        let b = ord0 >= F::zero() && ord1 >= F::zero();
        match self.direction {
            Direction::Convex => b,
            Direction::Concave => !b,
        }
    }
}

impl<F> SDF<F> for ArcPath<F> where F: Float + std::fmt::Display {
    fn sdf(&self, x: F, y: F) -> Option<F> {
        if !self.within_bounds(x, y) {
            return None;
        }
        // we treat the arc as a circumference
        let mut signed_dist_circ = distance(x, y, self.x_0, self.y_0) - self.r;
        if self.direction == Direction::Concave {
            signed_dist_circ = -signed_dist_circ;
        }
        // println!("mouse coord (x, y) = ({:.3}, {:.3})", x, y);
        // println!("center coord (x, y) = ({:.3}, {:.3})", self.x_0, self.y_0);
        // println!("signed_dist_circ: {}", signed_dist_circ);
        Some(signed_dist_circ)
    }
}

pub struct LinePath<F: Float> {
    pub x_0: F,
    pub y_0: F,
    pub x_1: F,
    pub y_1: F,
    pub length: F,
}

impl<F> LinePath<F> where F: Float {
    fn new(x_0: F, y_0: F, x_1: F, y_1: F) -> Self {
        let length = distance(x_0, y_0, x_1, y_1);
        assert!(length != F::zero(), "the line path must have a non-zero length");
        Self {
            x_0,
            y_0,
            x_1,
            y_1,
            length,
        }
    }

    fn within_bounds(&self, x: F, y: F) -> bool {
        let t = dot_product(x - self.x_0, y - self.y_0, self.x_1 - self.x_0, self.y_1 - self.y_0) / self.length;
        t >= F::zero() && t <= self.length
    }
}

impl<F> SDF<F> for LinePath<F> where F: Float {
    fn sdf(&self, x: F, y: F) -> Option<F> {
        if !self.within_bounds(x, y) {
            return None;
        }
        // divide the entire plane into two regions: the left and the right of the line path (which goes from (x_0, y_0) to (x_1, y_1))
        // as a diagram:
        //      -      //       +
        //  -------->  //  <--------
        //      +      //      -
        // the left region is the region where the signed distance is negative
        // the right region is the region where the signed distance is positive
        // the signed distance is the distance from the point to the line path, with the sign indicating the region

        // to calculate it, we use the cross product rule for parallelograms, which gives us the signed area of the parallelogram formed by the two vectors
        // A x B = |A| |B| sin(theta)
        // then we divide by the length of the line path to get the signed distance (which is the height of the parallelogram)
        // d = A x B / |A|
        let signed_dist = cross_product(x - self.x_0, y - self.y_0, self.x_1 - self.x_0, self.y_1 - self.y_0) / self.length;
        Some(signed_dist)
    }
}

pub struct ClosedPath<F: Float> {
    circle_subpaths: Vec<ArcPath<F>>,
    line_subpaths: Vec<LinePath<F>>,
}

impl<F> ClosedPath<F> where F: Float {
    fn new(circle_subpaths: Vec<ArcPath<F>>, line_subpaths: Vec<LinePath<F>>) -> Self {
        Self {
            circle_subpaths,
            line_subpaths,
        }
    }
}

enum Path<F: Float + std::fmt::Display> {
    Arc(ArcPath<F>),
    Line(LinePath<F>),
}

impl<F> SDF<F> for ClosedPath<F> where F: Float + std::fmt::Display {
    fn sdf(&self, x: F, y: F) -> Option<F> {
        if self.circle_subpaths.is_empty() && self.line_subpaths.is_empty() {
            return None;
        }

        let ((x_best, y_best), sd_circle) = self.circle_subpaths.iter().fold(((F::infinity(), F::infinity()), F::infinity()), |((x_best, y_best), sd), circle_subpath| {
            if let Some(signed_dist) = circle_subpath.sdf(x, y) {
                if signed_dist.abs() < sd.abs() {
                    ((circle_subpath.x_0, circle_subpath.y_0), signed_dist)
                } else {
                    ((x_best, y_best), sd)
                }
            } else {
                ((x_best, y_best), sd)
            }
        });
        // println!("best circle coord (x, y) = ({:.3}, {:.3})", x_best, y_best);
        // println!("best circle sd: {}", sd_circle);

        let ((x_best, y_best), sd_line) = self.line_subpaths.iter().fold(((F::infinity(), F::infinity()), F::infinity()), |((x_best, y_best), sd), line_subpath| {
            if let Some(signed_dist) = line_subpath.sdf(x, y) {
                if signed_dist.abs() < sd.abs() {
                    ((line_subpath.x_0, line_subpath.y_0), signed_dist)
                } else {
                    ((x_best, y_best), sd)
                }
            } else {
                ((x_best, y_best), sd)
            }
        });

        // println!("best line coord (x, y) = ({:.3}, {:.3})", x_best, y_best);
        // println!("best line sd: {}", sd_line);

        if sd_circle.abs() < sd_line.abs() {
            Some(sd_circle)
        } else {
            Some(sd_line)
        }
        
    }
}

pub fn predefined_closed_path_sdf() -> ClosedPath<f64> {
    ClosedPath::new(
        vec![
            ArcPath::new(8.0, -2.0, 2.0, 0.0, PI / 2.0, Direction::Convex),
            ArcPath::new(8.0, -10.0, 2.0, -PI / 2.0, 0.0, Direction::Convex),
            ArcPath::new(3.0, -11.0, 1.0, -3.0 * PI / 2.0, -PI / 2.0, Direction::Convex),
            ArcPath::new(7.0, -9.0, 1.0, 0.0, -PI / 2.0, Direction::Concave),
            ArcPath::new(0.0, -2.0, 2.0, -3.0 * PI / 2.0, -PI / 2.0, Direction::Convex),
        ], 
        vec![
            LinePath::new(0.0, -4.0, 8.0, -4.0),
            LinePath::new(8.0, -4.0, 8.0, -9.0),
            LinePath::new(7.0, -10.0, 3.0, -10.0),
            LinePath::new(3.0, -12.0, 8.0, -12.0),
            LinePath::new(10.0, -10.0, 10.0, -2.0),
            LinePath::new(8.0, 0.0, 0.0, 0.0),
    ])
}

pub trait SDF<F: Float> {
    fn sdf(&self, x: F, y: F) -> Option<F>;
}