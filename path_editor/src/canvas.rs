// based on https://github.com/emilk/egui/blob/master/crates/egui_demo_app/src/wrap_app.rs lines 43-52
use egui::{containers::*, widgets::*, *};
use linefollower_core::{geometry::closed_path::SubPath, utils::math::sigmoid};

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
    pub fn draw<T>(&self, ui: &Ui, painter: &Painter, tool: &T, curves: &[Vec<Pos2>]) where T: Tool + ?Sized {
        self.draw_displayable_subpaths(painter, curves);
        tool.draw(ui, self, painter);
    }

    pub fn to_screen(&self, painter: &Painter, p: Pos2) -> Pos2 {
        let transform = self.world_to_screen_transform(painter);
        
        transform * (p - self.focus_center).to_pos2()
    }

    pub fn to_world(&self, painter: &Painter, p: Pos2) -> Pos2 {
        let transform = self.screen_to_world_transform(painter);
        
        (transform * p) + self.focus_center.to_vec2()
    }

    fn draw_displayable_subpaths(&self, painter: &Painter, displayable_subpaths: &[Vec<Pos2>]) {
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

    pub fn screen_to_world(&self, pos: Pos2, painter: &Painter) -> Pos2 {
        let rect = painter.clip_rect();
        let mut sqr_prop = rect.square_proportions() / self.zoom;
        sqr_prop.y *= -1.0;
        let to_world = emath::RectTransform::from_to(
            rect,
            Rect::from_center_size(Pos2::ZERO, sqr_prop),
        );
        to_world * pos
    }

    fn world_to_screen_transform(&self, painter: &Painter) -> emath::RectTransform {
        let rect = painter.clip_rect();
        let mut sqr_prop = rect.square_proportions() / self.zoom;
        sqr_prop.y *= -1.0;
        emath::RectTransform::from_to(
            Rect::from_center_size(Pos2::ZERO, sqr_prop),
            rect,
        )
    }

    fn screen_to_world_transform(&self, painter: &Painter) -> emath::RectTransform {
        self.world_to_screen_transform(painter).inverse()
    }

    pub fn draw_line_from_screen_coords(&self, painter: &Painter, p0: Pos2, p1: Pos2, color: Color32) {
        let stroke = Stroke::new(1.0, color);
        // let p0 = self.to_screen(painter, p0);
        // let p1 = self.to_screen(painter, p1);
        let shape = egui::Shape::line_segment([p0, p1], stroke);
        painter.extend(std::iter::once(shape));
    }
}
