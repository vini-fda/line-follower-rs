use std::f32::consts::PI;
use macroquad::prelude::*;

use macroquad::{prelude::Color, shapes::draw_line};
use nalgebra::Point2;

use crate::utils::math::lattice_points;
use crate::utils::traits::Float;
pub const ROBOT_SIDE_LENGTH: f32 = 0.1;
pub const SENSOR_ARRAY_LENGTH: f32 = ROBOT_SIDE_LENGTH * 1.1;

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

pub fn draw_vector(x: f32, y: f32, dx: f32, dy: f32, color: Color) {
    draw_line(x, y, x + dx, y + dy, 0.01, color);
}

pub fn draw_robot(x: f32, y: f32, angle: f32, color: Color) {
    let angle = angle - 90.0;
    let w = ROBOT_SIDE_LENGTH;
    let r = w / 2f32.sqrt();

    let (cos_t, sin_t) = ((angle * PI / 180.0).cos(), (angle * PI / 180.0).sin());
    let l = SENSOR_ARRAY_LENGTH;
    draw_line(
        x + l * 0.5 * (cos_t - sin_t),
        y + l * 0.5 * (cos_t + sin_t),
        x - l * 0.5 * (cos_t + sin_t),
        y + l * 0.5 * (cos_t - sin_t),
        0.02,
        BLUE,
    );
    draw_poly(x, y, 4, r, angle + 45.0, color);
}

pub fn draw_grid(origin: Vec2, camera: &Camera2D, dx: f32, dy: f32) {
    // draw an "infinite" grid which is zoomable and pannable
    // uses draw_grid_from_bounds
    // the bounds depend on the camera's zoom and position
    // calculate min_bounds and max_bounds in world space
    let (w, h) = (screen_width(), screen_height());
    let min_bounds = camera.screen_to_world(vec2(0., h));
    let max_bounds = camera.screen_to_world(vec2(w, 0.));
    draw_grid_from_bounds(origin, min_bounds, max_bounds, dx, dy);
}

pub fn draw_grid_from_bounds(origin: Vec2, min_bounds: Vec2, max_bounds: Vec2, dx: f32, dy: f32) {
    // draw an "infinite" grid which is zoomable and pannable
    let (x_0, y_0) = (origin[0], origin[1]);
    let (x_min, y_min) = (min_bounds[0], min_bounds[1]);
    let (x_max, y_max) = (max_bounds[0], max_bounds[1]);
    let range_x = lattice_points(x_0, x_min, x_max, dx);
    let range_y = lattice_points(y_0, y_min, y_max, dy);
    const THICKNESS: f32 = 0.005;
    const ALPHA: f32 = 0.3;
    for x in range_x {
        draw_line(
            x,
            y_min,
            x,
            y_max,
            THICKNESS,
            Color::new(0.5, 0.5, 0.5, ALPHA),
        );
    }
    for y in range_y {
        draw_line(
            x_min,
            y,
            x_max,
            y,
            THICKNESS,
            Color::new(0.5, 0.5, 0.5, ALPHA),
        );
    }
    draw_line(
        0.0,
        y_min,
        0.0,
        y_max,
        THICKNESS * 1.1,
        Color::new(0., 0., 0., ALPHA),
    );
    draw_line(
        x_min,
        0.0,
        x_max,
        0.0,
        THICKNESS * 1.1,
        Color::new(0., 0., 0., ALPHA),
    );
}