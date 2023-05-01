use egui::{Painter, Ui};

use crate::canvas::Canvas;

use super::tool::Tool;

pub struct FreeTool {}

impl Tool for FreeTool {
    fn on_click(
        &mut self,
        _p: egui::Pos2,
    ) -> Option<linefollower_core::geometry::closed_path::SubPath<f64>> {
        None
    }
    fn draw(&self, _ui: &Ui, _canvas: &Canvas, _painter: &Painter) {}
}
