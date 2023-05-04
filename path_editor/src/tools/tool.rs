use egui::{Color32, InputState, Painter, Pos2, Response, Ui};
use linefollower_core::geometry::closed_path::SubPath;

use crate::{canvas::Canvas, tools::select_tool::SelectToolState};

use super::{
    arc_tool::ArcPathTool, free_tool::FreeTool, line_tool::LinePathTool, select_tool::SelectTool,
};

#[derive(PartialEq)]
pub enum Tool {
    Free(FreeTool),
    ArcPath(ArcPathTool),
    LinePath(LinePathTool),
    Select(SelectTool),
}

impl Tool {
    pub fn new() -> Self {
        Self::Free(FreeTool {})
    }
    pub fn on_input(&mut self, response: &Response, input: &InputState) {
        match self {
            Tool::Free(_) => {}
            Tool::ArcPath(tool) => tool.on_input(response, input),
            Tool::LinePath(tool) => tool.on_input(response, input),
            Tool::Select(tool) => tool.on_input(response, input),
        }
    }
    pub fn on_click(&mut self, p: Pos2) -> Option<SubPath<f64>> {
        match self {
            Tool::Free(_) => None,
            Tool::ArcPath(tool) => tool.on_click(p),
            Tool::LinePath(tool) => tool.on_click(p),
            Tool::Select(_) => None,
        }
    }
    pub fn draw(&self, ui: &Ui, canvas: &Canvas, painter: &Painter) {
        match self {
            Tool::Free(_) => {}
            Tool::ArcPath(tool) => tool.draw(ui, canvas, painter),
            Tool::LinePath(tool) => tool.draw(ui, canvas, painter),
            Tool::Select(tool) => tool.draw(ui, canvas, painter),
        }
    }
}

impl Default for Tool {
    fn default() -> Self {
        Self::new()
    }
}
