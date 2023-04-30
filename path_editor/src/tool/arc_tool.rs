use egui::Pos2;
use linefollower_core::geometry::{closed_path::SubPath, arc_path::ArcPath};
use super::super::utils::IntoPoint2;

pub struct Start {}
pub struct CenterPoint {
    pub center: Pos2,
}
// state after putting the center point and one point for the arc
pub struct FirstArcPoint {
    pub center: Pos2,
    pub p0: Pos2,
}

pub struct ArcPathTool<S: ArcToolState> {
    state: S,
}

pub trait ArcToolState {}
impl ArcToolState for Start {}
impl ArcToolState for CenterPoint {}
impl ArcToolState for FirstArcPoint {}

impl ArcPathTool<Start> {
    pub fn new() -> Self {
        Self {
            state: Start {},
        }
    }
    fn add_point(&mut self, p: Pos2) -> ArcPathTool<CenterPoint> {
        ArcPathTool {
            state: CenterPoint { center: p },
        }
    }
}

impl Default for ArcPathTool<Start> {
    fn default() -> Self {
        Self::new()
    }
}

impl ArcPathTool<CenterPoint> {
    fn add_point(&self, p: Pos2) -> ArcPathTool<FirstArcPoint> {
        ArcPathTool {
            state: FirstArcPoint {
                center: self.state.center,
                p0: p,
            },
        }
    }
}

impl ArcPathTool<FirstArcPoint> {
    fn create_arc(&self, p: Pos2) -> SubPath<f32> {
        let p0 = self.state.p0.into_point2();
        let p1 = p.into_point2();
        let center = self.state.center.into_point2();
        let v0 = p0 - center;
        let v1 = p1 - center;
        let r = v0.norm();
        let theta0 = v0.y.atan2(v0.x);
        let theta1 = v1.y.atan2(v1.x);
        let arcpath = ArcPath::new(center, r, theta0, theta1);
        SubPath::Arc(arcpath)
    }
}