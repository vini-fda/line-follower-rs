use egui::*;
use linefollower_core::geometry::{closed_path::SubPath, line_path::LinePath};
use crate::canvas::Canvas;

use super::{super::utils::IntoPoint2, paintable::Paintable};

pub struct LineStart {}
pub struct OnePoint {
    pub p: Pos2,
}

pub struct LinePathTool<S: LineToolState> {
    state: S,
}

pub trait LineToolState {}
impl LineToolState for LineStart {}
impl LineToolState for OnePoint {}

impl LinePathTool<LineStart> {
    pub fn new() -> Self {
        Self {
            state: LineStart {},
        }
    }
    fn add_point(&mut self, p: Pos2) -> LinePathTool<OnePoint> {
        LinePathTool {
            state: OnePoint { p },
        }
    }
}

impl Default for LinePathTool<LineStart> {
    fn default() -> Self {
        Self::new()
    }
}

impl LinePathTool<OnePoint> {
    fn create_line(&self, p: Pos2) -> SubPath<f32> {
        let linepath = LinePath::new(self.state.p.into_point2(), p.into_point2());
        SubPath::Line(linepath)
    }
    fn screen_to_world(&self, pos: Pos2, canvas: &Canvas, painter: &Painter) -> Pos2 {
        let rect = painter.clip_rect();
        let mut sqr_prop = rect.square_proportions() / canvas.zoom;
        sqr_prop.y *= -1.0;
        let to_world = emath::RectTransform::from_to(
            rect,
            Rect::from_center_size(Pos2::ZERO, sqr_prop),
        );
        to_world * pos
    }
}

impl Paintable for LinePathTool<LineStart> {
    fn paint(&self, _ui: &Ui, _canvas: &Canvas, _painter: &Painter) {
    }
}

impl Paintable for LinePathTool<OnePoint> {
    fn paint(&self, ui: &Ui, canvas: &Canvas, painter: &Painter) {
        let mut shapes: Vec<Shape> = Vec::new();
        let rect = painter.clip_rect();
        let mut sqr_prop = rect.square_proportions() / canvas.zoom;
        sqr_prop.y *= -1.0;
        let to_screen = emath::RectTransform::from_to(
            Rect::from_center_size(Pos2::ZERO, sqr_prop),
            rect,
        );
        let mut paint_line = |points: [Pos2; 2], color: Color32, width: f32| {
            let line = [to_screen * (points[0] - canvas.focus_center).to_pos2(), to_screen * (points[1] - canvas.focus_center).to_pos2()];

            // culling
            if rect.intersects(Rect::from_two_pos(line[0], line[1])) {
                shapes.push(Shape::line_segment(line, (width, color)));
            }
        };
        let start = self.state.p;
        let mouse_pos = ui.input(|i| i.pointer.hover_pos());
        if let Some(mouse_pos) = mouse_pos {
            let end = self.screen_to_world(mouse_pos, canvas, painter);
            paint_line([start, end], Color32::from_rgb(255, 0, 0), 1.0);
            painter.extend(shapes);
        }
    }
}