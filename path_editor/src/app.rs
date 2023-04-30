use crate::canvas::Canvas;
use egui::*;
use linefollower_core::geometry::{closed_path::SubPath, line_path::LinePath};
use nalgebra::Point2;

#[derive(Default)]
pub struct PathEditorApp {
    tool: Tool,
    canvas: Canvas,
}

#[derive(Default, PartialEq)]
enum Tool {
    #[default]
    Free,
    ArcPath,
    LinePath,
}



impl PathEditorApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        Self::default()
    }
}

impl eframe::App for PathEditorApp {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        egui::CentralPanel::default()
            .frame(egui::Frame::dark_canvas(&ctx.style()))
            .show(ctx, |ui| {
                self.canvas.ui(ui);
            });
        egui::Window::new("Tools").show(ctx, |ui| {
            // Tool selector: either ArcPath or LinePath creators
            ui.selectable_value(&mut self.tool, Tool::Free, "Free");
            ui.selectable_value(&mut self.tool, Tool::ArcPath, "Arc Path");
            ui.selectable_value(&mut self.tool, Tool::LinePath, "Line Path");
        });
        // if the user presses ESC, the tool will switch to Free
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.tool = Tool::Free;
        }
        // Taken from the egui demo (crates/egui_demo_app/src/backend_panel.rs)
        // "To ensure the UI is up to date you need to call `egui::Context::request_repaint()` each
        // time such an event happens. You can also chose to call `request_repaint()` once every second
        // or after every single frame - this is called [`Continuous`](RunMode::Continuous) mode,
        // and for games and interactive tools that need repainting every frame anyway, this should be the default."
        ctx.request_repaint();
    }
}
