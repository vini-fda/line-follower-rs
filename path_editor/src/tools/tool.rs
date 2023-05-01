use egui::{Pos2, Painter, Ui};
use linefollower_core::geometry::closed_path::SubPath;

use crate::canvas::Canvas;

use super::{line_tool::LinePathTool, free_tool::FreeTool};
pub trait Tool {
    fn on_click(&mut self, p: Pos2) -> Option<SubPath<f64>>;
    fn draw(&self, ui: &Ui, canvas: &Canvas, painter: &Painter);
}
