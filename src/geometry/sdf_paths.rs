use crate::math_utils::{cross_product, distance, dot_product};
use num::Float;
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

impl<F> ArcPath<F>
where
    F: Float + std::fmt::Display,
{
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
        let ord0 = cross_product(
            self.extremal_0.0 - self.x_0,
            self.extremal_0.1 - self.y_0,
            x - self.x_0,
            y - self.y_0,
        );
        let ord1 = cross_product(
            x - self.x_0,
            y - self.y_0,
            self.extremal_1.0 - self.x_0,
            self.extremal_1.1 - self.y_0,
        );
        let b = ord0 >= F::zero() && ord1 >= F::zero();
        match self.direction {
            Direction::Convex => b,
            Direction::Concave => !b,
        }
    }

    fn point_projection_dist(&self, x: F, y: F) -> F {
        // returns the distance of the point (x, y) on the arc path
        // assumes that (x, y) is on the arc path
        let theta = (y - self.y_0).atan2(x - self.x_0);
        let delta_theta = match self.direction {
            Direction::Convex => theta - self.theta_0,
            Direction::Concave => self.theta_0 - theta,
        };
        self.r * delta_theta
    }

    pub fn point_projection_tangent(&self, x: F, y: F) -> (F, F) {
        // returns the tangent vector of the point (x, y) on the arc path
        // assumes that (x, y) is on the arc path
        let dist = self.point_projection_dist(x, y);
        self.tangent_at(dist)
    }

    pub fn point_at(&self, d: F) -> (F, F) {
        // returns the point X on the path after traveling a distance d
        // the point X is on the arc path
        // assumes that d is within the bounds of the arc path
        let theta = match self.direction {
            Direction::Convex => d / self.r,
            Direction::Concave => -d / self.r,
        };
        let x = self.x_0 + self.r * (self.theta_0 + theta).cos();
        let y = self.y_0 + self.r * (self.theta_0 + theta).sin();
        (x, y)
    }

    pub fn tangent_at(&self, d: F) -> (F, F) {
        // returns the tangent vector at the point X on the path after traveling a distance d
        // the point X is on the arc path
        // assumes that d is within the bounds of the arc path
        let theta = match self.direction {
            Direction::Convex => d / self.r,
            Direction::Concave => -d / self.r,
        };
        let x = -(self.theta_0 + theta).sin();
        let y = (self.theta_0 + theta).cos();
        if self.direction == Direction::Concave {
            (-x, -y)
        } else {
            (x, y)
        }
    }

    pub fn length(&self) -> F {
        self.r * (self.theta_1 - self.theta_0).abs()
    }
}

impl<F> SDF<F> for ArcPath<F>
where
    F: Float + std::fmt::Display,
{
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

impl<F> LinePath<F>
where
    F: Float,
{
    fn new(x_0: F, y_0: F, x_1: F, y_1: F) -> Self {
        let length = distance(x_0, y_0, x_1, y_1);
        assert!(
            length != F::zero(),
            "the line path must have a non-zero length"
        );
        Self {
            x_0,
            y_0,
            x_1,
            y_1,
            length,
        }
    }

    pub fn point_projection_tangent(&self, _x: F, _y: F) -> (F, F) {
        // returns the tangent vector of the point (x, y) on the arc path
        // assumes that (x, y) is on the arc path
        let x = (self.x_1 - self.x_0) / self.length;
        let y = (self.y_1 - self.y_0) / self.length;
        (x, y)
    }

    fn within_bounds(&self, x: F, y: F) -> bool {
        let t = dot_product(
            x - self.x_0,
            y - self.y_0,
            self.x_1 - self.x_0,
            self.y_1 - self.y_0,
        ) / self.length;
        t >= F::zero() && t <= self.length
    }

    pub fn point_at(&self, d: F) -> (F, F) {
        // returns the point X on the path after traveling a distance d
        // the point X is on the line path (x_0, y_0) -> (x_1, y_1)
        // assumes that d is within the bounds of the line path
        let t = d / self.length;
        let x = self.x_0 + t * (self.x_1 - self.x_0);
        let y = self.y_0 + t * (self.y_1 - self.y_0);
        (x, y)
    }

    pub fn tangent_at(&self, _d: F) -> (F, F) {
        // returns the tangent vector at the point X on the path after traveling a distance d
        // the point X is on the arc path
        // assumes that d is within the bounds of the arc path
        let x = (self.x_1 - self.x_0) / self.length;
        let y = (self.y_1 - self.y_0) / self.length;
        (x, y)
    }

    pub fn length(&self) -> F {
        self.length
    }
}

impl<F> SDF<F> for LinePath<F>
where
    F: Float,
{
    fn sdf(&self, x: F, y: F) -> Option<F> {
        if !self.within_bounds(x, y) {
            let d0 = distance(x, y, self.x_0, self.y_0);
            let d1 = distance(x, y, self.x_1, self.y_1);
            let sign = cross_product(
                x - self.x_0,
                y - self.y_0,
                self.x_1 - self.x_0,
                self.y_1 - self.y_0,
            )
            .signum();
            return Some(sign * d0.min(d1));
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
        let signed_dist = cross_product(
            x - self.x_0,
            y - self.y_0,
            self.x_1 - self.x_0,
            self.y_1 - self.y_0,
        ) / self.length;
        Some(signed_dist)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SubpathIndex<F> {
    pub index: usize,
    pub subpath_type: SubpathType,
    pub start_d: F,
}

pub struct ClosedPath<F: Float> {
    circle_subpaths: Vec<ArcPath<F>>,
    line_subpaths: Vec<LinePath<F>>,
    ordering: Vec<SubpathIndex<F>>,
}

impl<F> ClosedPath<F>
where
    F: Float + std::fmt::Display,
{
    fn new(
        circle_subpaths: Vec<ArcPath<F>>,
        line_subpaths: Vec<LinePath<F>>,
        ordering: Vec<SubpathIndex<F>>,
    ) -> Self {
        Self {
            circle_subpaths,
            line_subpaths,
            ordering,
        }
    }

    pub fn point_at(&self, d: F) -> (F, F) {
        // returns the point X on the path after traveling a distance d from the start
        // the point X is on the path (x_0, y_0) -> (x_1, y_1)
        let d = d % self.length();
        // binary search for the subpath that contains the point (search by d)
        let mut i = self.ordering.partition_point(|probe| probe.start_d < d);
        i = i.saturating_sub(1);
        let subpath_index = self.ordering[i];
        let d = d - subpath_index.start_d;
        match subpath_index.subpath_type {
            SubpathType::ArcPath => self.circle_subpaths[subpath_index.index].point_at(d),
            SubpathType::LinePath => self.line_subpaths[subpath_index.index].point_at(d),
        }
    }

    pub fn tangent_at(&self, d: F) -> (F, F) {
        // returns the tangent vector at the point X on the path after traveling a distance d
        // the point X is on the path
        // assumes that d is within the bounds of the arc path
        let d = d % self.length();
        // binary search for the subpath that contains the point (search by d)
        let mut i = self.ordering.partition_point(|probe| probe.start_d < d);
        i = i.saturating_sub(1);
        let subpath_index = self.ordering[i];
        let d = d - subpath_index.start_d;
        match subpath_index.subpath_type {
            SubpathType::ArcPath => self.circle_subpaths[subpath_index.index].tangent_at(d),
            SubpathType::LinePath => self.line_subpaths[subpath_index.index].tangent_at(d),
        }
    }

    pub fn closest_subpath_index(&self, x: F, y: F) -> (usize, SubpathType) {
        // returns the index of the subpath that is closest to the point (x, y)
        let mut min_dist = F::infinity();
        let mut min_index = 0;
        let mut min_subpath_type = SubpathType::ArcPath;
        for (i, subpath) in self.circle_subpaths.iter().enumerate() {
            if let Some(dist) = subpath.sdf(x, y) {
                let dist = dist.abs();
                if dist < min_dist {
                    min_dist = dist;
                    min_index = i;
                }
            }
        }
        for (i, subpath) in self.line_subpaths.iter().enumerate() {
            if let Some(dist) = subpath.sdf(x, y) {
                let dist = dist.abs();
                if dist < min_dist {
                    min_dist = dist;
                    min_index = i;
                    min_subpath_type = SubpathType::LinePath;
                }
            }
        }
        (min_index, min_subpath_type)
    }

    pub fn point_projection_tangent(&self, x: F, y: F) -> (F, F) {
        // returns the tangent vector of the point (x, y) outside the arc path
        // assumes that (x, y) is on the arc path
        let (i, subpath_type) = self.closest_subpath_index(x, y);
        match subpath_type {
            SubpathType::ArcPath => self.circle_subpaths[i].point_projection_tangent(x, y),
            SubpathType::LinePath => self.line_subpaths[i].point_projection_tangent(x, y),
        }
    }

    pub fn length(&self) -> F {
        let mut l = F::zero();
        for subpath in self.circle_subpaths.iter() {
            l = l + subpath.length();
        }
        for subpath in self.line_subpaths.iter() {
            l = l + subpath.length();
        }
        l
    }
}

impl<F> SDF<F> for ClosedPath<F>
where
    F: Float + std::fmt::Display,
{
    fn sdf(&self, x: F, y: F) -> Option<F> {
        if self.circle_subpaths.is_empty() && self.line_subpaths.is_empty() {
            return None;
        }

        let ((x_best, y_best), sd_circle) = self.circle_subpaths.iter().fold(
            ((F::infinity(), F::infinity()), F::infinity()),
            |((x_best, y_best), sd), circle_subpath| {
                if let Some(signed_dist) = circle_subpath.sdf(x, y) {
                    if signed_dist.abs() < sd.abs() {
                        ((circle_subpath.x_0, circle_subpath.y_0), signed_dist)
                    } else {
                        ((x_best, y_best), sd)
                    }
                } else {
                    ((x_best, y_best), sd)
                }
            },
        );
        // println!("best circle coord (x, y) = ({:.3}, {:.3})", x_best, y_best);
        // println!("best circle sd: {}", sd_circle);

        let ((x_best, y_best), sd_line) = self.line_subpaths.iter().fold(
            ((F::infinity(), F::infinity()), F::infinity()),
            |((x_best, y_best), sd), line_subpath| {
                if let Some(signed_dist) = line_subpath.sdf(x, y) {
                    if signed_dist.abs() < sd.abs() {
                        ((line_subpath.x_0, line_subpath.y_0), signed_dist)
                    } else {
                        ((x_best, y_best), sd)
                    }
                } else {
                    ((x_best, y_best), sd)
                }
            },
        );

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
            ArcPath::new(7.0, -9.0, 1.0, 0.0, -PI / 2.0, Direction::Concave),
            ArcPath::new(
                3.0,
                -11.0,
                1.0,
                -3.0 * PI / 2.0,
                -PI / 2.0,
                Direction::Convex,
            ),
            ArcPath::new(8.0, -10.0, 2.0, -PI / 2.0, 0.0, Direction::Convex),
            ArcPath::new(8.0, -2.0, 2.0, 0.0, PI / 2.0, Direction::Convex),
            ArcPath::new(
                0.0,
                -2.0,
                2.0,
                -3.0 * PI / 2.0,
                -PI / 2.0,
                Direction::Convex,
            ),
        ],
        vec![
            LinePath::new(0.0, -4.0, 8.0, -4.0),
            LinePath::new(8.0, -4.0, 8.0, -9.0),
            LinePath::new(7.0, -10.0, 3.0, -10.0),
            LinePath::new(3.0, -12.0, 8.0, -12.0),
            LinePath::new(10.0, -10.0, 10.0, -2.0),
            LinePath::new(8.0, 0.0, 0.0, 0.0),
        ],
        vec![
            SubpathIndex {
                index: 0,
                start_d: 0.0,
                subpath_type: SubpathType::LinePath,
            },
            SubpathIndex {
                index: 1,
                start_d: 8.0,
                subpath_type: SubpathType::LinePath,
            },
            SubpathIndex {
                index: 0,
                start_d: 13.0,
                subpath_type: SubpathType::ArcPath,
            },
            SubpathIndex {
                index: 2,
                start_d: 13.0 + 0.5 * PI,
                subpath_type: SubpathType::LinePath,
            },
            SubpathIndex {
                index: 1,
                start_d: 17.0 + 0.5 * PI,
                subpath_type: SubpathType::ArcPath,
            },
            SubpathIndex {
                index: 3,
                start_d: 17.0 + 1.5 * PI,
                subpath_type: SubpathType::LinePath,
            },
            SubpathIndex {
                index: 2,
                start_d: 22.0 + 1.5 * PI,
                subpath_type: SubpathType::ArcPath,
            },
            SubpathIndex {
                index: 4,
                start_d: 22.0 + 2.5 * PI,
                subpath_type: SubpathType::LinePath,
            },
            SubpathIndex {
                index: 3,
                start_d: 30.0 + 2.5 * PI,
                subpath_type: SubpathType::ArcPath,
            },
            SubpathIndex {
                index: 5,
                start_d: 30.0 + 3.5 * PI,
                subpath_type: SubpathType::LinePath,
            },
            SubpathIndex {
                index: 4,
                start_d: 38.0 + 3.5 * PI,
                subpath_type: SubpathType::ArcPath,
            },
        ],
    )
}

#[derive(Debug, Clone, Copy)]
pub enum SubpathType {
    ArcPath,
    LinePath,
}

pub trait SDF<F: Float> {
    fn sdf(&self, x: F, y: F) -> Option<F>;
}
