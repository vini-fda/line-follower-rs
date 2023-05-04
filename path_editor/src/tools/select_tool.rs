use crate::{
    canvas::Canvas,
    curve_graph::{CurveGraph, ValidTrack},
};
use egui::{Color32, InputState, Painter, Pos2, Response, Ui};
use linefollower_core::geometry::closed_path::{ClosedPath, SubPath};
use mint::Point2;
use petgraph::{stable_graph::NodeIndex, visit::NodeRef};
use std::io::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SelectToolState {
    Start,
    OnceClicked,
}

pub struct SelectTool {
    state: SelectToolState,
    p0: Pos2,
    closed_path: Option<ClosedPath<f64>>,
    closed_path_json: Option<String>,
    save_file_name: String,
}

impl SelectTool {
    pub fn new() -> Self {
        Self {
            state: SelectToolState::Start,
            p0: Pos2::ZERO,
            closed_path: None,
            closed_path_json: None,
            save_file_name: String::new(),
        }
    }
    pub fn ui(&mut self, ui: &mut Ui) {
        ui.label("Selected Track");
        ui.separator();
        match self.closed_path_json {
            Some(ref closed_path_json) => {
                ui.text_edit_singleline(&mut self.save_file_name);
                if ui.button("Save track").clicked() {
                    // save the json into a file
                    // let the user choose the file name

                    let mut file = std::fs::File::create(&self.save_file_name).unwrap();
                    file.write_all(closed_path_json.as_bytes()).unwrap();
                }
                ui.label(closed_path_json);
            }
            None => {
                ui.label("No valid selection");
            }
        }
    }
    pub fn on_input(
        &mut self,
        response: &Response,
        input: &InputState,
        ui: &Ui,
        canvas: &Canvas,
        painter: &Painter,
        graph: &CurveGraph,
    ) {
        match self.state {
            SelectToolState::Start => {
                if response.hovered() && input.pointer.primary_clicked() {
                    self.state = SelectToolState::OnceClicked;
                    self.p0 = input.pointer.interact_pos().unwrap();
                }
            }
            SelectToolState::OnceClicked => {
                if response.hovered() && input.pointer.primary_clicked() {
                    self.closed_path = self.selected_track(ui, canvas, painter, graph);
                    if let Some(ref closed_path) = self.closed_path {
                        let json = serde_json::to_string_pretty(closed_path).unwrap();
                        self.closed_path_json = Some(json);
                    }
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
    /// Returns the selection rectangle if the tool is in the OnceClicked state.
    fn selection_rectangle(
        &self,
        ui: &Ui,
        canvas: &Canvas,
        painter: &Painter,
    ) -> Option<SelectionRectangle> {
        match self.state {
            SelectToolState::Start => None,
            SelectToolState::OnceClicked => {
                if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
                    let p0 = canvas.to_world(painter, self.p0).into();
                    let p1 = canvas.to_world(painter, mouse_pos).into();
                    Some(SelectionRectangle::new(p0, p1))
                } else {
                    None
                }
            }
        }
    }
    pub fn selected_points(
        &self,
        ui: &Ui,
        canvas: &Canvas,
        painter: &Painter,
        graph: &CurveGraph,
    ) -> Option<Vec<NodeIndex>> {
        match self.state {
            SelectToolState::Start => None,
            SelectToolState::OnceClicked => {
                let selection_rectangle = self.selection_rectangle(ui, canvas, painter)?;
                let mut selected_points = Vec::new();
                for i in graph.node_indices() {
                    let point = graph[i];
                    if selection_rectangle.contains(point) {
                        selected_points.push(i);
                    }
                }

                Some(selected_points)
            }
        }
    }
    pub fn selected_track(
        &self,
        ui: &Ui,
        canvas: &Canvas,
        painter: &Painter,
        graph: &CurveGraph,
    ) -> Option<ClosedPath<f64>> {
        match self.state {
            SelectToolState::Start => None,
            SelectToolState::OnceClicked => {
                let selected_points = self.selected_points(ui, canvas, painter, graph)?;
                graph.valid_track(&selected_points)
            }
        }
    }
}

impl Default for SelectTool {
    fn default() -> Self {
        Self::new()
    }
}

pub struct SelectionRectangle {
    min: Point2<f32>,
    max: Point2<f32>,
}

impl SelectionRectangle {
    pub fn new(p0: Point2<f32>, p1: Point2<f32>) -> Self {
        let xmin = p0.x.min(p1.x);
        let xmax = p0.x.max(p1.x);
        let ymin = p0.y.min(p1.y);
        let ymax = p0.y.max(p1.y);
        Self {
            min: Point2::from_slice(&[xmin, ymin]),
            max: Point2::from_slice(&[xmax, ymax]),
        }
    }
    pub fn contains(&self, p: Point2<f32>) -> bool {
        self.min.x <= p.x && p.x <= self.max.x && self.min.y <= p.y && p.y <= self.max.y
    }
}
