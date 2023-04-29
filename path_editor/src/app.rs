use crate::canvas::Canvas;

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
    }
}
