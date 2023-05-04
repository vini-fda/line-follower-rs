use crate::{
    canvas::Canvas,
    tools::{arc_tool::ArcPathTool, line_tool::LinePathTool, select_tool::SelectTool, tool::Tool},
};
use egui::*;
use linefollower_core::{
    geometry::{closed_path::SubPath, line_path::LinePath, track::Track},
    utils::{math::sigmoid, traits::Float},
};
use mint::Point2;
use petgraph::{prelude::DiGraph, stable_graph::NodeIndex};

type CurveGraph = DiGraph<Point2<f32>, SubPath<f64>>;

pub struct PathEditorApp {
    canvas: Canvas,
    tool: Tool,
    curve_graph: CurveGraph,
}

trait AddSubPath<F>
where
    F: Float,
{
    fn add_subpath(&mut self, subpath: SubPath<F>);
}

impl AddSubPath<f64> for CurveGraph {
    fn add_subpath(&mut self, subpath: SubPath<f64>) {
        let p0: Point2<f32> = subpath.first_point().cast::<f32>().into();
        let p1: Point2<f32> = subpath.last_point().cast::<f32>().into();
        const MIN_DISTANCE_SQR: f32 = 1e-10;
        let mut i0 = None;
        let mut i1 = None;
        // before adding the nodes, check if they are close enough to existing nodes
        for node in self.node_indices() {
            let p = self[node];
            fn distance_sqr(p0: Point2<f32>, p1: Point2<f32>) -> f32 {
                (p0.x - p1.x).powi(2) + (p0.y - p1.y).powi(2)
            }
            if distance_sqr(p, p0) <= MIN_DISTANCE_SQR {
                i0 = Some(node);
            }
            if distance_sqr(p, p1) <= MIN_DISTANCE_SQR {
                i1 = Some(node);
            }
        }
        // add nodes if they don't exist
        let i0 = i0.unwrap_or_else(|| self.add_node(p0));
        let i1 = i1.unwrap_or_else(|| self.add_node(p1));
        // add the edge
        self.add_edge(i0, i1, subpath);
    }
}

impl PathEditorApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            tool: Tool::new(),
            canvas: Canvas::default(),
            curve_graph: DiGraph::new(),
        }
    }
}

impl eframe::App for PathEditorApp {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        egui::CentralPanel::default()
            .frame(egui::Frame::dark_canvas(&ctx.style()))
            .show(ctx, |ui| {
                // get input
                const MIN_ZOOM: f32 = 0.1;
                const MAX_ZOOM: f32 = 10.0;
                // get mouse scroll to adjust zoom
                let scroll = ui.input(|i| i.scroll_delta);
                // calculate zoom from mouse scroll
                let mw = sigmoid(scroll.y) - 0.5;
                let new_zoom = self.canvas.zoom * (mw * 0.1).exp();
                if new_zoom <= MIN_ZOOM {
                    self.canvas.zoom = MIN_ZOOM;
                } else if new_zoom >= MAX_ZOOM {
                    self.canvas.zoom = MAX_ZOOM;
                } else {
                    self.canvas.zoom = new_zoom;
                }
                // use WASD to move the camera center of focus
                let mut move_center = |mut dir: Vec2| {
                    dir.x /= self.canvas.zoom;
                    dir.y /= self.canvas.zoom;
                    self.canvas.focus_center += dir;
                };
                const SPEED: f32 = 0.01;
                let mut v = Vec2::ZERO;
                // ATTENTION: currently this has been fixed
                // by putting the UI in continuous mode
                if ui.input(|i| i.key_down(Key::W)) {
                    v += vec2(0.0, 1.0);
                }
                if ui.input(|i| i.key_down(Key::A)) {
                    v += vec2(-1.0, 0.0);
                }
                if ui.input(|i| i.key_down(Key::S)) {
                    v += vec2(0.0, -1.0);
                }
                if ui.input(|i| i.key_down(Key::D)) {
                    v += vec2(1.0, 0.0);
                }
                // normalize diagonal movement
                if v != Vec2::ZERO {
                    v = v.normalized();
                    v.x *= SPEED;
                    v.y *= SPEED;
                    move_center(v);
                }

                let (mut response, painter) =
                    ui.allocate_painter(ui.available_size(), Sense::click().union(Sense::hover()));
                // Make sure we allocate what we used (everything)
                ui.expand_to_include_rect(painter.clip_rect());
                // check for mouse click
                if response.hovered() {
                    const SNAP_RADIUS: f32 = 30.0;
                    if ui.input(|i| i.pointer.primary_clicked()) {
                        let pos = ui.input(|i| i.pointer.interact_pos());
                        if let Some(mut pos) = pos {
                            // snapping
                            for node in self.curve_graph.raw_nodes().iter() {
                                let snap_point =
                                    self.canvas.to_screen(&painter, node.weight.into());
                                let distance = snap_point.distance(pos);
                                if distance <= SNAP_RADIUS {
                                    pos = snap_point;
                                    break;
                                }
                            }
                            let pos = self.canvas.to_world(&painter, pos);
                            let subpath = self.tool.on_click(pos);
                            if let Some(subpath) = subpath {
                                self.curve_graph.add_subpath(subpath);
                                response.mark_changed();
                            }
                        }
                    }
                    // show snap radius
                    let blue_stroke = Stroke::new(0.5, Color32::BLUE);
                    for snap_point in self.curve_graph.raw_nodes().iter().map(|node| node.weight) {
                        self.canvas.draw_circle(
                            &painter,
                            blue_stroke,
                            snap_point.into(),
                            SNAP_RADIUS,
                        );
                    }
                }
                ui.input(|i| self.tool.on_input(&response, i));
                self.canvas.draw_subpaths(
                    &painter,
                    self.curve_graph.raw_edges().iter().map(|edge| &edge.weight),
                );
                let green_stroke = Stroke::new(1.0, Color32::from_rgb(25, 200, 100));
                for subpath in self.curve_graph.raw_edges().iter().map(|edge| &edge.weight) {
                    const NUM_SAMPLES: usize = 10;
                    let points = subpath.sample_points_num(NUM_SAMPLES);
                    let tangents = subpath.sample_tangents_num(NUM_SAMPLES);
                    let subpath_iter = points.zip(tangents);
                    for (point, dir) in subpath_iter {
                        self.canvas.draw_direction_arrow(
                            &painter,
                            green_stroke,
                            point.cast::<f32>().into(),
                            dir.cast::<f32>().into(),
                        );
                    }
                }
                self.tool.draw(ui, &self.canvas, &painter);
            });
        egui::Window::new("Tools").show(ctx, |ui| {
            // Tool selector: either ArcPath or LinePath creators
            // ui.selectable_value(&mut self.tooltype, ToolType::Free, "Free");
            // ui.selectable_value(&mut self.tooltype, ToolType::ArcPath, "Arc Path");
            // ui.selectable_value(&mut self.tooltype, ToolType::LinePath, "Line Path");
            if ui
                .add(SelectableLabel::new(
                    matches!(self.tool, Tool::Free(_)),
                    "Free",
                ))
                .clicked()
            {
                self.tool = Tool::new();
            }
            if ui
                .add(SelectableLabel::new(
                    matches!(self.tool, Tool::ArcPath(_)),
                    "Arc Path",
                ))
                .clicked()
            {
                self.tool = Tool::ArcPath(ArcPathTool::default());
            }
            if ui
                .add(SelectableLabel::new(
                    matches!(self.tool, Tool::LinePath(_)),
                    "Line Path",
                ))
                .clicked()
            {
                self.tool = Tool::LinePath(LinePathTool::default());
            }
            if ui
                .add(SelectableLabel::new(
                    matches!(self.tool, Tool::Select(_)),
                    "Selection",
                ))
                .clicked()
            {
                self.tool = Tool::Select(SelectTool::default());
            }
        });
        egui::Window::new("Subpaths").show(ctx, |ui| {
            for subpath in self.curve_graph.raw_edges().iter().map(|edge| &edge.weight) {
                ui.label(format!("{:?}", subpath));
            }
        });
        // if the user presses ESC, the tool will switch to Free
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.tool = Tool::new();
        }
        // Taken from the egui demo (crates/egui_demo_app/src/backend_panel.rs)
        // "To ensure the UI is up to date you need to call `egui::Context::request_repaint()` each
        // time such an event happens. You can also chose to call `request_repaint()` once every second
        // or after every single frame - this is called [`Continuous`](RunMode::Continuous) mode,
        // and for games and interactive tools that need repainting every frame anyway, this should be the default."
        ctx.request_repaint();
    }
}
