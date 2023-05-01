use std::f64::consts::PI;

use crate::{canvas::Canvas, utils::IntoPos2};

use super::{super::utils::IntoPoint2, tool::Tool};
use egui::{Color32, Painter, Pos2, Stroke, Ui};
use linefollower_core::geometry::{arc_path::ArcPath, closed_path::SubPath};
use nalgebra::{ComplexField, Point2, Vector2};

pub enum ArcPathToolState {
    Start,
    CenterPoint,
    FirstArcPoint,
}

pub struct ArcPathTool {
    state: ArcPathToolState,
    center: Point2<f64>,
    p0: Point2<f64>,
    theta0: f64,
    r: f64,
}

impl ArcPathTool {
    pub fn new() -> Self {
        Self {
            state: ArcPathToolState::Start,
            center: Point2::new(0.0, 0.0),
            p0: Point2::new(0.0, 0.0),
            theta0: 0.0,
            r: 0.0,
        }
    }
    fn arc_point(&self) -> Point2<f64> {
        self.center + self.r * Vector2::new(self.theta0.cos(), self.theta0.sin())
    }
    fn vector_angle(&self, v: Vector2<f64>) -> f64 {
        let t = v.y.atan2(v.x);
        if t < 0.0 {
            2.0 * PI + t
        } else {
            t
        }
    }
}

impl Default for ArcPathTool {
    fn default() -> Self {
        Self::new()
    }
}

impl Tool for ArcPathTool {
    fn on_click(&mut self, p: Pos2) -> Option<SubPath<f64>> {
        match self.state {
            ArcPathToolState::Start => {
                self.state = ArcPathToolState::CenterPoint;
                self.center = p.into_point2();
                None
            }
            ArcPathToolState::CenterPoint => {
                self.state = ArcPathToolState::FirstArcPoint;
                self.p0 = p.into_point2();
                let v0 = self.p0 - self.center;
                self.theta0 = self.vector_angle(v0);
                self.r = v0.norm();
                None
            }
            ArcPathToolState::FirstArcPoint => {
                self.state = ArcPathToolState::Start;
                let v1 = p.into_point2() - self.center;
                let mut theta1 = self.vector_angle(v1);
                if theta1 < self.theta0 {
                    theta1 += 2.0 * PI;
                }
                Some(SubPath::Arc(ArcPath::new(
                    self.center,
                    self.r,
                    self.theta0,
                    theta1,
                )))
            }
        }
    }
    fn reset_state(&mut self) {
        self.state = ArcPathToolState::Start;
    }
    fn draw(&self, ui: &Ui, canvas: &Canvas, painter: &Painter) {
        match self.state {
            ArcPathToolState::Start => {}
            ArcPathToolState::CenterPoint => {
                if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
                    let red = Color32::from_rgb(255, 0, 0);
                    let p0 = canvas.to_screen(painter, self.center.into_pos2());
                    canvas.draw_line_from_screen_coords(painter, p0, mouse_pos, red);
                }
            }
            ArcPathToolState::FirstArcPoint => {
                if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
                    // same as before...
                    let red = Color32::from_rgb(255, 0, 0);
                    let p0 = canvas.to_screen(painter, self.center.into_pos2());
                    let p1 = canvas.to_screen(painter, self.arc_point().into_pos2());
                    canvas.draw_line_from_screen_coords(painter, p0, p1, red);
                    //...but now we also an arc
                    let p = canvas.to_world(painter, mouse_pos);
                    let v1 = p.into_point2() - self.center;
                    let mut theta1 = self.vector_angle(v1);
                    if theta1 < self.theta0 {
                        theta1 += 2.0 * PI;
                    }
                    let dtheta = theta1 - self.theta0;
                    const NUM_POINTS: u32 = 100;
                    let path: Vec<Pos2> = (0..=NUM_POINTS)
                        .map(|n| n as f64 * dtheta / (NUM_POINTS as f64) + self.theta0)
                        .map(|theta| {
                            (self.center + self.r * Vector2::new(theta.cos(), theta.sin()))
                                .into_pos2()
                        })
                        .collect();
                    let stroke = Stroke::new(1.0, red);
                    canvas.draw_path(painter, stroke, &path);
                }
            }
        }
    }
}
