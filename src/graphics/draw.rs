use macroquad::{prelude::Color, shapes::draw_line};
use nalgebra::Point2;

use crate::utils::traits::Float;

pub fn draw_closed_curve<F>(points: &[Point2<F>], color: Color, stroke_width: f32)
where
    F: Float,
{
    for i in 1..points.len() {
        draw_line(
            points[i - 1].x.to_f32().unwrap(),
            points[i - 1].y.to_f32().unwrap(),
            points[i].x.to_f32().unwrap(),
            points[i].y.to_f32().unwrap(),
            stroke_width,
            color,
        );
    }
    draw_line(
        points[points.len() - 1].x.to_f32().unwrap(),
        points[points.len() - 1].y.to_f32().unwrap(),
        points[0].x.to_f32().unwrap(),
        points[0].y.to_f32().unwrap(),
        stroke_width,
        color,
    );
}
