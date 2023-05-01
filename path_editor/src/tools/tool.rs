use egui::{Painter, Pos2, Ui};
use linefollower_core::geometry::closed_path::SubPath;

use crate::canvas::Canvas;

use super::{free_tool::FreeTool, line_tool::LinePathTool};
pub trait Tool {
    fn on_click(&mut self, p: Pos2) -> Option<SubPath<f64>>;
    fn draw(&self, ui: &Ui, canvas: &Canvas, painter: &Painter);
    fn reset_state(&mut self);
}
