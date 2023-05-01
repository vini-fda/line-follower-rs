use egui::{InputState, Painter, Response, Ui};

use crate::canvas::Canvas;

use super::tool::Tool;

pub struct FreeTool {}

impl Tool for FreeTool {
    fn on_input(&mut self, _response: &Response, _input: &InputState) {}
    fn on_click(
        &mut self,
        _p: egui::Pos2,
    ) -> Option<linefollower_core::geometry::closed_path::SubPath<f64>> {
        None
    }
    fn draw(&self, _ui: &Ui, _canvas: &Canvas, _painter: &Painter) {}
    fn reset_state(&mut self) {}
}
