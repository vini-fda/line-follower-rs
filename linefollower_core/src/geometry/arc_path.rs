use crate::utils::{math::cross, traits::Float};
use nalgebra::{distance, Point2, Vector2};
use serde::{Deserialize, Serialize};

use super::track::Track;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArcPath<F: Float> {
    pub center: Point2<F>,
    pub r: F,
    pub theta0: F,
    pub theta1: F,
    p0: Point2<F>,
    v0: Vector2<F>,
    v1: Vector2<F>,
    counterclockwise: bool,
    length: F,
}

impl<F> ArcPath<F>
where
    F: Float,
{
    pub fn new(center: Point2<F>, r: F, theta0: F, theta1: F) -> Self {
        let delta_t = theta1 - theta0;
        assert!(
            delta_t != F::zero(),
            "the arc path must have a non-zero length"
        );
        let length = r * num::Float::abs(delta_t);
        let counterclockwise = delta_t > F::zero();
        let v0 = Vector2::new(num::Float::cos(theta0), num::Float::sin(theta0));
        let v1 = Vector2::new(num::Float::cos(theta1), num::Float::sin(theta1));
        let p0 = center + v0 * r;

        Self {
            center,
            r,
            theta0,
            theta1,
            p0,
            v0,
            v1,
            counterclockwise,
            length,
        }
    }

    fn within_bounds(&self, p: Point2<F>) -> bool {
        let v = p - self.center;
        let ord0 = cross(&self.v0, &v);
        let ord1 = cross(&v, &self.v1);
        match self.counterclockwise {
            true => ord0 >= F::zero() && ord1 >= F::zero(),
            false => ord0 <= F::zero() && ord1 <= F::zero(),
        }
    }
}

impl<F> Track<F> for ArcPath<F>
where
    F: Float,
{
    fn sdf(&self, p: Point2<F>) -> F {
        // we treat the arc as a circumference
        if !self.within_bounds(p) {
            return F::infinity();
        }
        let mut signed_dist = distance(&p, &self.center) - self.r;
        if !self.counterclockwise {
            signed_dist = -signed_dist;
        }
        signed_dist
    }

    fn length(&self) -> F {
        self.length
    }

    fn first_point(&self) -> Point2<F> {
        self.p0
    }

    fn point_at(&self, d: F) -> Point2<F> {
        // returns the point X on the path after traveling a distance d
        // the point X is on the arc path
        // assumes that d is within the bounds of the arc path
        let theta = match self.counterclockwise {
            true => d / self.r,
            false => -d / self.r,
        };
        let v = Vector2::new(
            num::Float::cos(self.theta0 + theta),
            num::Float::sin(self.theta0 + theta),
        );
        self.center + v * self.r
    }

    fn tangent_at(&self, d: F) -> Vector2<F> {
        // returns the tangent vector at the point X on the path after traveling a distance d
        // the point X is on the arc path
        // assumes that d is within the bounds of the arc path
        let theta = match self.counterclockwise {
            true => d / self.r,
            false => -d / self.r,
        };
        let v = Vector2::new(
            -num::Float::sin(self.theta0 + theta),
            num::Float::cos(self.theta0 + theta),
        );
        if self.counterclockwise {
            v
        } else {
            -v
        }
    }

    fn point_projection_distance(&self, p: Point2<F>) -> F {
        // returns the distance of the point (x, y) when projected along the arc path
        // assumes that (x, y) is on the arc path
        let v = p - self.center;
        self.r * v.angle(&self.v0)
    }

    fn point_projection_tangent(&self, p: Point2<F>) -> Vector2<F> {
        let d = self.point_projection_distance(p);
        self.tangent_at(d)
    }
}

// macro for creating a new arc path
// usage: arc_path!(center_x, center_y, r, theta0, theta1)
// where center_x, center_y are the x and y coordinates of the center of the arc
// r is the radius of the arc
// theta0 is the angle of the arc at the start point
// theta1 is the angle of the arc at the end point
#[macro_export]
macro_rules! new_arc_path {
    ($center_x:expr, $center_y:expr, $r:expr, $theta0:expr, $theta1:expr) => {
        ArcPath::new(Point2::new($center_x, $center_y), $r, $theta0, $theta1)
    };
}
