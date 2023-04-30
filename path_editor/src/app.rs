use crate::{canvas::Canvas, tools::{tool::Tool, paintable::Paintable, line_tool::{LinePathTool, LineStart}}};
use egui::*;
use linefollower_core::geometry::{closed_path::SubPath, line_path::LinePath};
use nalgebra::Point2;


pub struct PathEditorApp {
    tool: Box<dyn Tool>,
    tooltype: ToolType,
    canvas: Canvas,
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
            tool: Box::new(super::tools::free_tool::FreeTool{}),
            tooltype: ToolType::Free,
            canvas: Canvas::default(),
        }
    }
}

impl eframe::App for PathEditorApp {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        egui::CentralPanel::default()
            .frame(egui::Frame::dark_canvas(&ctx.style()))
            .show(ctx, |ui| {
                self.canvas.ui(ui);
                let painter = Painter::new(
                    ui.ctx().clone(),
                    ui.layer_id(),
                    ui.available_rect_before_wrap(),
                );
                self.tool.paint(ui, &self.canvas, &painter);
            });
        egui::Window::new("Tools").show(ctx, |ui| {
            // Tool selector: either ArcPath or LinePath creators
            // ui.selectable_value(&mut self.tooltype, ToolType::Free, "Free");
            // ui.selectable_value(&mut self.tooltype, ToolType::ArcPath, "Arc Path");
            // ui.selectable_value(&mut self.tooltype, ToolType::LinePath, "Line Path");
            if ui.add(SelectableLabel::new(self.tooltype == ToolType::Free, "Free")).clicked() {
                self.tooltype = ToolType::Free;
                self.tool = Box::new(super::tools::free_tool::FreeTool{});
            }
            // if ui.add(SelectableLabel::new(self.tooltype == ToolType::ArcPath, "Arc Path")).clicked() {
            //     self.tooltype = ToolType::ArcPath;
            //     self.tool = Box::new(super::tools::arc_tool::ArcTool::default());
            // }
            if ui.add(SelectableLabel::new(self.tooltype == ToolType::LinePath, "Line Path")).clicked() {
                self.tooltype = ToolType::LinePath;
                self.tool = Box::<LinePathTool<LineStart>>::default();
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
