use std::f32::consts::PI;

// based on https://github.com/emilk/egui/blob/master/crates/egui_demo_app/src/wrap_app.rs lines 43-52
use egui::{containers::*, widgets::*, *};
use linefollower_core::{
    geometry::{closed_path::SubPath, track::Track},
    utils::math::sigmoid,
};
use mint::{Point2, Vector2};
use nalgebra as na;

use crate::tools::tool::Tool;

pub struct Canvas {
    pub zoom: f32,
    pub focus_center: Pos2,
}

impl Canvas {
    pub fn new() -> Self {
        Self {
            zoom: 1.0,
            focus_center: Pos2::ZERO,
        }
    }
}

impl Default for Canvas {
    fn default() -> Self {
        Self::new()
    }
}

impl Canvas {
    pub fn to_screen(&self, painter: &Painter, p: Pos2) -> Pos2 {
        let transform = self.world_to_screen_transform(painter);

        transform * (p - self.focus_center).to_pos2()
    }

    pub fn to_world(&self, painter: &Painter, p: Pos2) -> Pos2 {
        let transform = self.screen_to_world_transform(painter);

        (transform * p) + self.focus_center.to_vec2()
    }

    pub fn draw_path(&self, painter: &Painter, stroke: Stroke, path: &[Pos2]) {
        let points: Vec<Pos2> = path.iter().map(|p| self.to_screen(painter, *p)).collect();
        let shape = egui::Shape::line(points, stroke);
        painter.extend(std::iter::once(shape));
    }

    pub fn draw_direction_arrow(
        &self,
        painter: &Painter,
        stroke: Stroke,
        center: Point2<f32>,
        dir: Vector2<f32>,
    ) {
        const LENGTH: f32 = 0.005;
        let shift = 5.0 * PI / 6.0;
        let rotation = na::Rotation2::new(shift);
        let center_na: na::Point2<f32> = center.into();
        let dir_na: na::Vector2<f32> = dir.into();
        let p_right: Point2<f32> = (center_na + rotation * dir_na * LENGTH).into();
        let p_left: Point2<f32> = (center_na + rotation.inverse() * dir_na * LENGTH).into();
        let points = [p_left, center, p_right]
            .into_iter()
            .map(|p| self.to_screen(painter, p.into()))
            .collect();
        let shape = egui::Shape::line(points, stroke);
        painter.extend(std::iter::once(shape));
    }

    pub fn draw_circle(&self, painter: &Painter, stroke: Stroke, center: Pos2, radius: f32) {
        let center = self.to_screen(painter, center);
        let shape = egui::Shape::circle_stroke(center, radius, stroke);
        painter.extend(std::iter::once(shape));
    }

    pub fn draw_displayable_subpaths(&self, painter: &Painter, displayable_subpaths: &[Vec<Pos2>]) {
        let green_stroke = Stroke::new(1.0, Color32::from_rgb(25, 200, 100));

        let shapes = displayable_subpaths
            .iter()
            .filter(|line| line.len() >= 2)
            .map(|line| {
                let points: Vec<Pos2> = line.iter().map(|p| self.to_screen(painter, *p)).collect();
                egui::Shape::line(points, green_stroke)
            });

        painter.extend(shapes);
    }

    fn world_to_screen_transform(&self, painter: &Painter) -> emath::RectTransform {
        let rect = painter.clip_rect();
        let mut sqr_prop = rect.square_proportions() / self.zoom;
        sqr_prop.y *= -1.0;
        emath::RectTransform::from_to(Rect::from_center_size(Pos2::ZERO, sqr_prop), rect)
    }

    fn screen_to_world_transform(&self, painter: &Painter) -> emath::RectTransform {
        self.world_to_screen_transform(painter).inverse()
    }

    pub fn draw_line_from_screen_coords(
        &self,
        painter: &Painter,
        p0: Pos2,
        p1: Pos2,
        color: Color32,
    ) {
        let stroke = Stroke::new(1.0, color);
        // let p0 = self.to_screen(painter, p0);
        // let p1 = self.to_screen(painter, p1);
        let shape = egui::Shape::line_segment([p0, p1], stroke);
        painter.extend(std::iter::once(shape));
    }
}
