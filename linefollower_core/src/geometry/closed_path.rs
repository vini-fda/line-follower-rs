use super::arc_path::ArcPath;
use super::line_path::LinePath;
use crate::new_arc_path;
use crate::new_line_path;
use crate::utils::traits::Float;
use nalgebra::{Point2, Vector2};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

use super::track::Track;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SubPath<F: Float> {
    Arc(ArcPath<F>),
    Line(LinePath<F>),
}

impl<F> Track<F> for SubPath<F>
where
    F: Float,
{
    fn sdf(&self, p: Point2<F>) -> F {
        match self {
            SubPath::Arc(arc) => arc.sdf(p),
            SubPath::Line(line) => line.sdf(p),
        }
    }

    fn length(&self) -> F {
        match self {
            SubPath::Arc(arc) => arc.length(),
            SubPath::Line(line) => line.length(),
        }
    }

    fn point_at(&self, d: F) -> Point2<F> {
        match self {
            SubPath::Arc(arc) => arc.point_at(d),
            SubPath::Line(line) => line.point_at(d),
        }
    }

    fn tangent_at(&self, d: F) -> Vector2<F> {
        match self {
            SubPath::Arc(arc) => arc.tangent_at(d),
            SubPath::Line(line) => line.tangent_at(d),
        }
    }

    fn point_projection_distance(&self, p: Point2<F>) -> F {
        match self {
            SubPath::Arc(arc) => arc.point_projection_distance(p),
            SubPath::Line(line) => line.point_projection_distance(p),
        }
    }
    // SAME implementation as the default
    // just did this to fix the error:
    // error[E0599]: no method named `sample_tangents_num` found for enum `SubPath` in the current scope
    // BUG REPORT??
    fn sample_points_num(&self, n: usize) -> Box<dyn Iterator<Item = Point2<F>> + '_> {
        let nf = F::from_usize(n).unwrap();
        let delta = self.length() / nf;
        Box::new(
            (0..=n)
                .map(move |i| F::from_usize(i).unwrap() * delta)
                .map(|d| self.point_at(d)),
        )
    }
    fn sample_tangents_num(&self, n: usize) -> Box<dyn Iterator<Item = Vector2<F>> + '_> {
        let nf = F::from_usize(n).unwrap();
        let delta = self.length() / nf;
        Box::new(
            (0..=n)
                .map(move |i| F::from_usize(i).unwrap() * delta)
                .map(|d| self.tangent_at(d)),
        )
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct ClosedPath<F: Float> {
    p0: Point2<F>,
    subpaths: Vec<SubPath<F>>,
    starts: Vec<F>,
    length: F,
}

impl<F> ClosedPath<F>
where
    F: Float,
{
    pub fn new(subpaths: Vec<SubPath<F>>) -> Self {
        debug_assert!(is_valid_closed_path(&subpaths), "invalid closed path");
        let starts = subpaths
            .iter()
            .scan(F::zero(), |state, subpath| {
                let start = *state;
                *state += subpath.length();
                Some(start)
            })
            .collect::<Vec<_>>();
        let length = *starts.last().unwrap() + subpaths.last().unwrap().length();
        let p0 = subpaths.first().unwrap().point_at(F::zero());
        Self {
            p0,
            subpaths,
            starts,
            length,
        }
    }

    fn first_subpath_dist(&self, d: F) -> (F, &SubPath<F>) {
        // returns the subpath that contains the point X on the path after traveling a distance d from the start
        // the point X is on the path (x_0, y_0) -> (x_1, y_1)
        let d = d % self.length();
        // binary search for the subpath that contains the point (search by d)
        let i = self.starts.partition_point(|&x| x < d).saturating_sub(1);
        (d - self.starts[i], &self.subpaths[i])
    }

    fn closest_subpath(&self, p: Point2<F>) -> &SubPath<F> {
        // returns the subpath that is closest to the point P
        let f = |sd| num::Float::abs(sd);
        self.subpaths
            .iter()
            .min_by(|a, b| {
                let sd_a = a.sdf(p);
                let sd_b = b.sdf(p);
                f(sd_a).partial_cmp(&f(sd_b)).unwrap()
            })
            .unwrap()
    }
}

impl<F> Track<F> for ClosedPath<F>
where
    F: Float,
{
    fn sdf(&self, p: Point2<F>) -> F {
        // returns the sdf of the path which is closest to the point P
        let f = |sd| num::Float::abs(sd);
        self.subpaths
            .iter()
            .map(|subpath| subpath.sdf(p))
            .min_by(|a, b| f(*a).partial_cmp(&f(*b)).unwrap())
            .unwrap()
    }

    fn length(&self) -> F {
        self.length
    }

    fn first_point(&self) -> Point2<F> {
        self.p0
    }

    fn point_at(&self, d: F) -> Point2<F> {
        // returns the point X on the path after traveling a distance d from the start
        // the point X is on the path (x_0, y_0) -> (x_1, y_1)
        let (x, subpath) = self.first_subpath_dist(d);
        subpath.point_at(x)
    }

    fn tangent_at(&self, d: F) -> Vector2<F> {
        // returns the tangent vector at the point X on the path after traveling a distance d
        // the point X is on the path
        let (x, subpath) = self.first_subpath_dist(d);
        subpath.tangent_at(x)
    }

    fn point_projection_distance(&self, _p: Point2<F>) -> F {
        todo!()
    }

    fn point_projection_tangent(&self, p: Point2<F>) -> Vector2<F> {
        let subpath = self.closest_subpath(p);
        subpath.point_projection_tangent(p)
    }
}

pub fn predefined_closed_path() -> ClosedPath<f64> {
    ClosedPath::new(vec![
        SubPath::Line(new_line_path![0.0, -4.0, 8.0, -4.0]),
        SubPath::Line(new_line_path![8.0, -4.0, 8.0, -9.0]),
        SubPath::Arc(new_arc_path![7.0, -9.0, 1.0, 0.0, -PI / 2.0]),
        SubPath::Line(new_line_path![7.0, -10.0, 3.0, -10.0]),
        SubPath::Arc(new_arc_path![3.0, -11.0, 1.0, PI / 2.0, 3.0 * PI / 2.0]),
        SubPath::Line(new_line_path![3.0, -12.0, 8.0, -12.0]),
        SubPath::Arc(new_arc_path![8.0, -10.0, 2.0, -PI / 2.0, 0.0]),
        SubPath::Line(new_line_path![10.0, -10.0, 10.0, -2.0]),
        SubPath::Arc(new_arc_path![8.0, -2.0, 2.0, 0.0, PI / 2.0]),
        SubPath::Line(new_line_path![8.0, 0.0, 0.0, 0.0]),
        SubPath::Arc(new_arc_path![0.0, -2.0, 2.0, PI / 2.0, 3.0 * PI / 2.0]),
    ])
}

pub fn is_valid_closed_path<F>(subpaths: &[SubPath<F>]) -> bool
where
    F: Float,
{
    // checks if the subpaths form a valid closed path
    // a valid closed path is a path that starts and ends at the same point
    // and the subpaths are connected to each other
    if subpaths.len() < 2 {
        return false;
    }
    let mut it = subpaths.iter();
    let mut prev = it.next().unwrap().last_point();
    // TODO: remove magic number
    let epsilon = F::epsilon() * F::from(100.0).unwrap();
    for subpath in it {
        let p1 = subpath.first_point();
        let p2 = prev;
        // we can't check if p1 == p2 because of floating point precision
        // so we check if distance <= epsilon
        if (p1 - p2).norm() > epsilon {
            return false;
        }
        prev = subpath.last_point();
    }
    // check if the last point is the same as the first point
    let p1 = subpaths.first().unwrap().first_point();
    let p2 = prev;
    if (p1 - p2).norm() > epsilon {
        return false;
    }
    true
}

#[cfg(test)]
pub mod tests {
    use super::*;

    #[test]
    fn test_predefined_path_validity() {
        let path = predefined_closed_path();
        assert!(is_valid_closed_path(&path.subpaths));
    }
}
