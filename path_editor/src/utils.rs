use egui::Pos2;
use nalgebra::Point2;

pub trait IntoPoint2 {
    fn into_point2(self) -> Point2<f32>;
}

impl IntoPoint2 for Pos2 {
    fn into_point2(self) -> Point2<f32> {
        Point2::new(self.x, self.y)
    }
}