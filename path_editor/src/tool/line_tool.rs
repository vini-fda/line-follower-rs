use egui::Pos2;
use linefollower_core::geometry::{closed_path::SubPath, line_path::LinePath};
use super::super::utils::IntoPoint2;

pub struct Start {}
pub struct OnePoint {
    pub p: Pos2,
}

pub struct LinePathTool<S: LineToolState> {
    state: S,
}

pub trait LineToolState {}
impl LineToolState for Start {}
impl LineToolState for OnePoint {}

impl LinePathTool<Start> {
    pub fn new() -> Self {
        Self {
            state: Start {},
        }
    }
    fn add_point(&mut self, p: Pos2) -> LinePathTool<OnePoint> {
        LinePathTool {
            state: OnePoint { p },
        }
    }
}

impl Default for LinePathTool<Start> {
    fn default() -> Self {
        Self::new()
    }
}

impl LinePathTool<OnePoint> {
    fn create_line(&self, p: Pos2) -> SubPath<f32> {
        let linepath = LinePath::new(self.state.p.into_point2(), p.into_point2());
        SubPath::Line(linepath)
    }
}