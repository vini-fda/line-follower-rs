use crate::{canvas::Canvas, utils::IntoPos2};
use egui::*;
use linefollower_core::geometry::{closed_path::SubPath, line_path::LinePath};
use nalgebra::Point2;

use super::{super::utils::IntoPoint2, tool::Tool};

pub struct LineStart {}
pub struct OnePoint {
    pub p: Pos2,
}

#[derive(Clone, Copy, Debug)]
pub enum LinePathToolState {
    Start,
    OnePoint,
}

pub struct LinePathTool {
    state: LinePathToolState,
    p0: Point2<f64>,
}

impl LinePathTool {
    pub fn new() -> Self {
        Self {
            state: LinePathToolState::Start,
            p0: Point2::new(0.0, 0.0),
        }
    }

    fn screen_to_world(&self, pos: Pos2, canvas: &Canvas, painter: &Painter) -> Pos2 {
        let rect = painter.clip_rect();
        let mut sqr_prop = rect.square_proportions() / canvas.zoom;
        sqr_prop.y *= -1.0;
        let to_world =
            emath::RectTransform::from_to(rect, Rect::from_center_size(Pos2::ZERO, sqr_prop));
        to_world * pos
    }
}

impl Default for LinePathTool {
    fn default() -> Self {
        Self::new()
    }
}

// impl Paintable for LinePathTool {
//     fn paint(&self, ui: &Ui, canvas: &Canvas, painter: &Painter) {
//         if let LinePathToolState::OnePoint = self.state {
//             // let mut shapes: Vec<Shape> = Vec::new();
//             // let rect = painter.clip_rect();
//             // let mut sqr_prop = rect.square_proportions() / canvas.zoom;
//             // sqr_prop.y *= -1.0;
//             // let to_screen = emath::RectTransform::from_to(
//             //     Rect::from_center_size(Pos2::ZERO, sqr_prop),
//             //     rect,
//             // );
//             // let mut paint_line = |points: [Pos2; 2], color: Color32, width: f32| {
//             //     let line = [to_screen * (points[0] - canvas.focus_center).to_pos2(), to_screen * (points[1] - canvas.focus_center).to_pos2()];

//             //     // culling
//             //     if rect.intersects(Rect::from_two_pos(line[0], line[1])) {
//             //         shapes.push(Shape::line_segment(line, (width, color)));
//             //     }
//             // };
//             // let start = self.p0.into_pos2();
//             // let mouse_pos = ui.input(|i| i.pointer.hover_pos());
//             // if let Some(mouse_pos) = mouse_pos {
//             //     let end = self.screen_to_world(mouse_pos, canvas, painter);
//             //     paint_line([start, end], Color32::from_rgb(255, 0, 0), 1.0);
//             //     painter.extend(shapes);
//             // }
//             let mouse_pos = ui.input(|i| i.pointer.hover_pos());
//             if let Some(p1) = mouse_pos {
//                 let red = Color32::from_rgb(255, 0, 0);
//                 let p0 = canvas.to_screen(painter, self.p0.into_pos2());
//                 canvas.draw_line_from_screen_coords(painter, p0, p1, red);
//             }
//         }
//     }
// }

impl Tool for LinePathTool {
    fn on_click(&mut self, p: Pos2) -> Option<SubPath<f64>> {
        match self.state {
            LinePathToolState::Start => {
                self.state = LinePathToolState::OnePoint;
                self.p0 = p.into_point2();
                None
            }
            LinePathToolState::OnePoint => {
                self.state = LinePathToolState::Start;
                let p1 = p.into_point2();
                Some(SubPath::Line(LinePath::new(self.p0, p1)))
            }
        }
    }
    fn draw(&self, ui: &Ui, canvas: &Canvas, painter: &Painter) {
        if let LinePathToolState::OnePoint = self.state {
            if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
                let red = Color32::from_rgb(255, 0, 0);
                let p0 = canvas.to_screen(painter, self.p0.into_pos2());
                canvas.draw_line_from_screen_coords(painter, p0, mouse_pos, red);
            }
        }
    }
}
