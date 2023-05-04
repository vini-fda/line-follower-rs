use egui::Pos2;
use linefollower_core::utils::traits::Float;
use nalgebra::Point2;

pub trait IntoPoint2<F>
where
    F: Float,
{
    fn into_point2(self) -> Point2<F>;
}

impl<F> IntoPoint2<F> for Pos2
where
    F: Float,
{
    fn into_point2(self) -> Point2<F> {
        Point2::new(F::from_f32(self.x).unwrap(), F::from_f32(self.y).unwrap())
    }
}

pub trait IntoPos2 {
    fn into_pos2(self) -> Pos2;
}

impl<F> IntoPos2 for Point2<F>
where
    F: Float,
{
    fn into_pos2(self) -> Pos2 {
        Pos2::new(self.x.to_f32().unwrap(), self.y.to_f32().unwrap())
    }
}
