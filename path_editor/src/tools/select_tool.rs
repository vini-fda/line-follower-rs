use crate::canvas::Canvas;
use egui::{Color32, InputState, Painter, Pos2, Response, Ui};
use linefollower_core::geometry::closed_path::SubPath;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SelectToolState {
    Start,
    OnceClicked,
}

#[derive(PartialEq)]
pub struct SelectTool {
    state: SelectToolState,
    p0: Pos2,
}

impl SelectTool {
    pub fn new() -> Self {
        Self {
            state: SelectToolState::Start,
            p0: Pos2::ZERO,
        }
    }
    pub fn on_input(&mut self, response: &Response, input: &InputState) {
        match self.state {
            SelectToolState::Start => {
                if response.hovered() && input.pointer.primary_clicked() {
                    self.state = SelectToolState::OnceClicked;
                    self.p0 = input.pointer.interact_pos().unwrap();
                }
            }
            SelectToolState::OnceClicked => {
                if response.hovered() && input.pointer.primary_clicked() {
                    self.state = SelectToolState::Start;
                }
            }
        }
    }
    pub fn on_click(&mut self, _p: egui::Pos2) -> Option<SubPath<f64>> {
        None
    }
    pub fn draw(&self, ui: &Ui, _canvas: &Canvas, painter: &Painter) {
        match self.state {
            SelectToolState::Start => {}
            SelectToolState::OnceClicked => {
                if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
                    let color = Color32::from_rgba_unmultiplied(200, 255, 255, 50);
                    let shape = egui::Shape::rect_filled(
                        egui::Rect::from_two_pos(self.p0, mouse_pos),
                        egui::Rounding::none(),
                        color,
                    );
                    painter.extend(std::iter::once(shape));
                }
            }
        }
    }
}

impl Default for SelectTool {
    fn default() -> Self {
        Self::new()
    }
}
