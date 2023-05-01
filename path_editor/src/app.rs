use crate::{
    canvas::Canvas,
    tools::{
        arc_tool::ArcPathTool,
        free_tool::FreeTool,
        line_tool::{LinePathTool, LineStart},
        tool::Tool,
    },
    utils::{IntoPoint2, IntoPos2},
};
use egui::*;
use linefollower_core::{
    geometry::{closed_path::SubPath, line_path::LinePath, track::Track},
    utils::math::sigmoid,
};

pub struct Curves {
    // these are the subpaths used for exporting
    // and making calculations
    pub subpaths: Vec<SubPath<f64>>,
    // this holds equivalent data, but is used
    // for display purposes
    pub displayable_subpaths: Vec<Vec<Pos2>>,
    // extremal points (used for snapping)
    pub extremal_points: Vec<[Pos2; 2]>,
}

impl Curves {
    pub fn new() -> Self {
        Self {
            subpaths: Vec::new(),
            displayable_subpaths: Vec::new(),
            extremal_points: Vec::new(),
        }
    }

    pub fn add_subpath(&mut self, subpath: SubPath<f64>) {
        // sample points from subpath
        let samples = generate_displayable_points(&subpath);
        self.displayable_subpaths.push(samples);
        let extremities = [
            subpath.first_point().into_pos2(),
            subpath.last_point().into_pos2(),
        ];
        self.extremal_points.push(extremities);
        self.subpaths.push(subpath);
    }
}

pub fn generate_displayable_points(subpath: &SubPath<f64>) -> Vec<Pos2> {
    match subpath {
        SubPath::Arc(arc) => arc
            .sample_points_num(100)
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

                ui.input(|i| self.tool.on_input(i));

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
                            println!("pos (screen) = [{:.4}, {:.4}]", pos.x, pos.y);
                            // snapping
                            for snap_point in self.curves.extremal_points.iter().flatten() {
                                let snap_point = self.canvas.to_screen(&painter, *snap_point);
                                let distance = snap_point.distance(pos);
                                if distance <= SNAP_RADIUS {
                                    pos = snap_point;
                                    break;
                                }
                            }
                            println!("pos (snap) = [{:.4}, {:.4}]", pos.x, pos.y);
                            let pos = self.canvas.to_world(&painter, pos);
                            println!("pos (world) = [{:.4}, {:.4}]", pos.x, pos.y);
                            let subpath = self.tool.on_click(pos);
                            if let Some(subpath) = subpath {
                                self.curves.add_subpath(subpath);
                                response.mark_changed();
                            }
                        }
                    }
                    // show snap radius
                    let blue_stroke = Stroke::new(0.5, Color32::BLUE);
                    for snap_point in self.curves.extremal_points.iter().flatten() {
                        self.canvas
                            .draw_circle(&painter, blue_stroke, *snap_point, SNAP_RADIUS);
                    }
                }
                self.canvas
                    .draw_displayable_subpaths(&painter, &self.curves.displayable_subpaths);
                let green_stroke = Stroke::new(1.0, Color32::from_rgb(25, 200, 100));
                for subpath in &self.curves.subpaths {
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
                    self.tooltype == ToolType::Free,
                    "Free",
                ))
                .clicked()
            {
                self.tooltype = ToolType::Free;
                self.tool = Box::new(super::tools::free_tool::FreeTool {});
            }
            if ui
                .add(SelectableLabel::new(
                    self.tooltype == ToolType::ArcPath,
                    "Arc Path",
                ))
                .clicked()
            {
                self.tooltype = ToolType::ArcPath;
                self.tool = Box::<ArcPathTool>::default();
            }
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
            self.tool = Box::new(FreeTool {});
            //self.tool.reset_state();
        }
        // Taken from the egui demo (crates/egui_demo_app/src/backend_panel.rs)
        // "To ensure the UI is up to date you need to call `egui::Context::request_repaint()` each
        // time such an event happens. You can also chose to call `request_repaint()` once every second
        // or after every single frame - this is called [`Continuous`](RunMode::Continuous) mode,
        // and for games and interactive tools that need repainting every frame anyway, this should be the default."
        ctx.request_repaint();
    }
}
