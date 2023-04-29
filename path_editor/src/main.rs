use path_editor::app::PathEditorApp;

fn main() {
    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "Path Editor",
        native_options,
        Box::new(|cc| Box::new(PathEditorApp::new(cc))),
    )
    .unwrap();
}
