use crate::{canvas::Canvas, utils::IntoPos2};
use egui::*;
use linefollower_core::geometry::{closed_path::SubPath, line_path::LinePath};
use nalgebra::Point2;

use super::{super::utils::IntoPoint2, tool::Tool};

pub struct LineStart {}
pub struct OnePoint {
    pub p: Pos2,
}

#[derive(Clone, Copy, Debug)]
pub enum LinePathToolState {
    Start,
    OnePoint,
}

pub struct LinePathTool {
    state: LinePathToolState,
    p0: Point2<f64>,
}

impl LinePathTool {
    pub fn new() -> Self {
        Self {
            state: LinePathToolState::Start,
            p0: Point2::new(0.0, 0.0),
        }
    }
}

impl Default for LinePathTool {
    fn default() -> Self {
        Self::new()
    }
}

impl Tool for LinePathTool {
    fn on_input(&mut self, _response: &Response, _input: &InputState) {}
    fn on_click(&mut self, p: Pos2) -> Option<SubPath<f64>> {
        match self.state {
            LinePathToolState::Start => {
                self.state = LinePathToolState::OnePoint;
                self.p0 = p.into_point2();
                None
            }
            LinePathToolState::OnePoint => {
                self.state = LinePathToolState::Start;
                let p1 = p.into_point2();
                Some(SubPath::Line(LinePath::new(self.p0, p1)))
            }
        }
    }
    fn reset_state(&mut self) {
        self.state = LinePathToolState::Start;
    }
    fn draw(&self, ui: &Ui, canvas: &Canvas, painter: &Painter) {
        if let LinePathToolState::OnePoint = self.state {
            if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
                let red = Color32::from_rgb(255, 0, 0);
                let p0 = canvas.to_screen(painter, self.p0.into_pos2());
                canvas.draw_line_from_screen_coords(painter, p0, mouse_pos, red);
            }
        }
    }
}
