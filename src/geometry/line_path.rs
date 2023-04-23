use crate::utils::{math::cross, traits::Float};
use nalgebra::{distance, Point2, Vector2};

use super::track::Track;

#[derive(Debug, Clone)]
pub struct LinePath<F: Float> {
    pub p0: Point2<F>,
    pub p1: Point2<F>,
    pub length: F,
    v: Vector2<F>,
}

impl<F> LinePath<F>
where
    F: Float,
{
    pub fn new(p0: Point2<F>, p1: Point2<F>) -> Self {
        let length = distance(&p0, &p1);
        let v = (p1 - p0) / length;
        assert!(
            length != F::zero(),
            "the line path must have a non-zero length"
        );
        Self { p0, p1, length, v }
    }
}

impl<F> Track<F> for LinePath<F>
where
    F: Float,
{
    fn sdf(&self, p: Point2<F>) -> F {
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
        let u = p - self.p0;
        let v = self.p1 - self.p0;
        let signed_dist = cross(&u, &v) / self.length;
        let dot = u.dot(&self.v);
        if !(dot >= F::zero() && dot <= self.length) {
            let d0 = distance(&p, &self.p0);
            let d1 = distance(&p, &self.p1);
            let sign = num::Float::signum(signed_dist);
            return sign * num::Float::min(d0, d1);
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
        // the point X is on the line path (x_0, y_0) -> (x_1, y_1)
        // assumes that d is within the bounds of the line path
        self.p0 + self.v * d
    }

    fn tangent_at(&self, _d: F) -> Vector2<F> {
        // returns the tangent vector of the point at distance d on the line path
        (self.p1 - self.p0) / self.length
    }

    fn point_projection_distance(&self, p: Point2<F>) -> F {
        let u = p - self.p0;
        u.dot(&self.v)
    }

    fn point_projection_tangent(&self, _p: Point2<F>) -> Vector2<F> {
        // returns the tangent vector of the point p on the line path
        self.v
    }
}

// macro for creating a new line path
// the line path is defined by two points
// usage: line_path![x0, y0, x1, y1]
#[macro_export]
macro_rules! new_line_path {
    ($x0:expr, $y0:expr, $x1:expr, $y1:expr) => {
        LinePath::new(Point2::new($x0, $y0), Point2::new($x1, $y1))
    };
}
