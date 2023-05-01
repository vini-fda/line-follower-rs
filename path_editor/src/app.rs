use crate::{
    canvas::Canvas,
    tools::{
        line_tool::{LinePathTool, LineStart},
        tool::Tool,
    },
    utils::IntoPos2,
};
use egui::*;
use linefollower_core::{
    geometry::{closed_path::SubPath, line_path::LinePath, track::sample_points},
    utils::math::sigmoid,
};
use nalgebra::Point2;

pub struct Curves {
    // these are the subpaths used for exporting
    // and making calculations
    pub subpaths: Vec<SubPath<f64>>,
    // this holds equivalent data, but is used
    // for display purposes
    pub displayable_subpaths: Vec<Vec<Pos2>>,
}

impl Curves {
    pub fn new() -> Self {
        Self {
            subpaths: Vec::new(),
            displayable_subpaths: Vec::new(),
        }
    }

    pub fn add_subpath(&mut self, subpath: SubPath<f64>) {
        // sample points from subpath
        let samples = generate_displayable_points(&subpath);
        self.displayable_subpaths.push(samples);
        self.subpaths.push(subpath)
    }
}

pub fn generate_displayable_points(subpath: &SubPath<f64>) -> Vec<Pos2> {
    match subpath {
        SubPath::Arc(arc) => sample_points(arc, 0.01)
            .map(|p| p.into_pos2())
            .collect::<Vec<_>>(),
        SubPath::Line(line) => {
            vec![line.p0.into_pos2(), line.p1.into_pos2()]
        }
    }
}

impl Default for Curves {
    fn default() -> Self {
        Self::new()
    }
}

pub struct PathEditorApp {
    tooltype: ToolType,
    canvas: Canvas,
    tool: Box<dyn Tool>,
    curves: Curves,
}

#[derive(Default, PartialEq)]
enum ToolType {
    #[default]
    Free,
    ArcPath,
    LinePath,
}

// pub fn draw_closed_curve<F>(points: &[Point2<F>], color: Color, stroke_width: f32)
// where
//     F: Float,
// {
//     for i in 1..points.len() {
//         draw_line(
//             points[i - 1].x.to_f32().unwrap(),
//             points[i - 1].y.to_f32().unwrap(),
//             points[i].x.to_f32().unwrap(),
//             points[i].y.to_f32().unwrap(),
//             stroke_width,
//             color,
//         );
//     }
//     draw_line(
//         points[points.len() - 1].x.to_f32().unwrap(),
//         points[points.len() - 1].y.to_f32().unwrap(),
//         points[0].x.to_f32().unwrap(),
//         points[0].y.to_f32().unwrap(),
//         stroke_width,
//         color,
//     );
// }

// impl Paintable for SubPath<f64> {
//     fn paint(&self, ui: &Ui, canvas: &Canvas, painter: &Painter) {

//     }
// }

impl PathEditorApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            tool: Box::new(super::tools::free_tool::FreeTool {}),
            tooltype: ToolType::Free,
            canvas: Canvas::default(),
            curves: Curves::new(),
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
                if response.hovered() && ui.input(|i| i.pointer.primary_clicked()) {
                    let pos = ui.input(|i| i.pointer.interact_pos());
                    if let Some(pos) = pos {
                        println!("pos (screen) = [{:.4}, {:.4}]", pos.x, pos.y);
                        let pos = self.canvas.to_world(&painter, pos);
                        println!("pos (world) = [{:.4}, {:.4}]", pos.x, pos.y);
                        let subpath = self.tool.on_click(pos);
                        if let Some(subpath) = subpath {
                            self.curves.add_subpath(subpath);
                            response.mark_changed();
                        }
                    }
                }

                self.canvas.draw(
                    ui,
                    &painter,
                    self.tool.as_ref(),
                    &self.curves.displayable_subpaths,
                );
            });
        egui::Window::new("Tools").show(ctx, |ui| {
            // Tool selector: either ArcPath or LinePath creators
            // ui.selectable_value(&mut self.tooltype, ToolType::Free, "Free");
            // ui.selectable_value(&mut self.tooltype, ToolType::ArcPath, "Arc Path");
            // ui.selectable_value(&mut self.tooltype, ToolType::LinePath, "Line Path");
            if ui
                .add(SelectableLabel::new(
                    self.tooltype == ToolType::Free,
                    "Free",
                ))
                .clicked()
            {
                self.tooltype = ToolType::Free;
                self.tool = Box::new(super::tools::free_tool::FreeTool {});
            }
            // if ui.add(SelectableLabel::new(self.tooltype == ToolType::ArcPath, "Arc Path")).clicked() {
            //     self.tooltype = ToolType::ArcPath;
            //     self.tool = Box::new(super::tools::arc_tool::ArcTool::default());
            // }
            if ui
                .add(SelectableLabel::new(
                    self.tooltype == ToolType::LinePath,
                    "Line Path",
                ))
                .clicked()
            {
                self.tooltype = ToolType::LinePath;
                self.tool = Box::<LinePathTool>::default();
            }
        });
        egui::Window::new("Subpaths").show(ctx, |ui| {
            for subpath in &self.curves.subpaths {
                ui.label(format!("{:?}", subpath));
            }
        });
        // if the user presses ESC, the tool will switch to Free
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.tooltype = ToolType::Free;
        }
        // Taken from the egui demo (crates/egui_demo_app/src/backend_panel.rs)
        // "To ensure the UI is up to date you need to call `egui::Context::request_repaint()` each
        // time such an event happens. You can also chose to call `request_repaint()` once every second
        // or after every single frame - this is called [`Continuous`](RunMode::Continuous) mode,
        // and for games and interactive tools that need repainting every frame anyway, this should be the default."
        ctx.request_repaint();
    }
}
