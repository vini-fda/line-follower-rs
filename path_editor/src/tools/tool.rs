use egui::{InputState, Painter, Pos2, Response, Ui};
use linefollower_core::geometry::closed_path::SubPath;

use crate::canvas::Canvas;

pub trait Tool {
    fn on_input(&mut self, response: &Response, input: &InputState);
    fn on_click(&mut self, p: Pos2) -> Option<SubPath<f64>>;
    fn draw(&self, ui: &Ui, canvas: &Canvas, painter: &Painter);
    fn reset_state(&mut self);
}
