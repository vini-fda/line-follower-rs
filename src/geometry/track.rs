use crate::utils::traits::Float;
use nalgebra::{Point2, Vector2};
pub trait Track<F>
where
    F: Float,
{
    fn sdf(&self, p: Point2<F>) -> F;
    fn length(&self) -> F;
    fn first_point(&self) -> Point2<F> {
        self.point_at(F::zero())
    }
    fn point_at(&self, d: F) -> Point2<F>;
    fn tangent_at(&self, d: F) -> Vector2<F>;
    fn point_projection_distance(&self, p: Point2<F>) -> F;
    fn point_projection_tangent(&self, p: Point2<F>) -> Vector2<F> {
        let d = self.point_projection_distance(p);
        self.tangent_at(d)
    }
}

pub fn sample_points<F, T>(track: &T, dx: F) -> impl Iterator<Item = Point2<F>> + '_
where
    F: Float,
    T: Track<F>,
{
    let mut d = F::zero();
    std::iter::from_fn(move || {
        d += dx;
        if d > track.length() {
            None
        } else {
            Some(track.point_at(d))
        }
    })
}
