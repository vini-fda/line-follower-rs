use super::paintable::Paintable;

pub struct FreeTool {}

impl Paintable for FreeTool {
    fn paint(&self, _ui: &egui::Ui, _canvas: &crate::canvas::Canvas, _painter: &egui::Painter) {}
}