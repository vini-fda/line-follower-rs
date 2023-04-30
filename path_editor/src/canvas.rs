// based on https://github.com/emilk/egui/blob/master/crates/egui_demo_app/src/wrap_app.rs lines 43-52
use egui::{containers::*, widgets::*, *};
use linefollower_core::{geometry::closed_path::SubPath, utils::math::sigmoid};

pub struct Canvas {
    pub subpaths: Vec<SubPath<f64>>,
    pub zoom: f32,
    pub focus_center: Pos2,
}

impl Default for Canvas {
    fn default() -> Self {
        Self {
            subpaths: Vec::new(),
            zoom: 1.0,
            focus_center: Pos2::ZERO,
        }
    }
}

impl Canvas {
    pub fn ui(&mut self, ui: &mut Ui) {
        //paint
        let painter = Painter::new(
            ui.ctx().clone(),
            ui.layer_id(),
            ui.available_rect_before_wrap(),
        );
        self.paint(&painter);
        // Make sure we allocate what we used (everything)
        ui.expand_to_include_rect(painter.clip_rect());
        // get input
        const MIN_ZOOM: f32 = 0.1;
        const MAX_ZOOM: f32 = 10.0;
        // get mouse scroll to adjust zoom
        let scroll = ui.input(|i| i.scroll_delta);
        // calculate zoom from mouse scroll
        let mw = sigmoid(scroll.y) - 0.5;
        let new_zoom = self.zoom * (mw * 0.1).exp();
        if new_zoom <= MIN_ZOOM {
            self.zoom = MIN_ZOOM;
        } else if new_zoom >= MAX_ZOOM {
            self.zoom = MAX_ZOOM;
        } else {
            self.zoom = new_zoom;
        }
        // use WASD to move the camera center of focus
        let mut move_center = |mut dir: Vec2| {
            dir.x /= self.zoom;
            dir.y /= self.zoom;
            self.focus_center += dir;
        };
        const SPEED: f32 = 0.01;
        let mut v = Vec2::ZERO;
        // ATTENTION: currently this has been fixed
        // by putting the UI in continuous mode
        if ui.input(|i| i.key_down(Key::W)) {
            ui.label("W is down!");
            v += vec2(0.0, 1.0);
        }
        if ui.input(|i| i.key_down(Key::A)) {
            ui.label("A is down!");
            v += vec2(-1.0, 0.0);
        }
        if ui.input(|i| i.key_down(Key::S)) {
            ui.label("S is down!");
            v += vec2(0.0, -1.0);
        }
        if ui.input(|i| i.key_down(Key::D)) {
            ui.label("D is down!");
            v += vec2(1.0, 0.0);
        }
        // normalize diagonal movement
        if v != Vec2::ZERO {
            v = v.normalized();
            v.x *= SPEED;
            v.y *= SPEED;
        }
        move_center(v);


        // get mouse position
        let pos = ui.input(|i| i.pointer.hover_pos());
        // show mouse position
        if let Some(pos) = pos {
            ui.label(format!("Mouse Position (screen coordinates) = ({:.1}, {:.1})", pos.x, pos.y));
            // convert mouse position to canvas/world coordinates
            let pos = self.screen_to_world(pos, &painter);
            ui.label(format!("Mouse Position (canvas coordinates) = ({:.1}, {:.1})", pos.x, pos.y));
        }
    }
    fn screen_to_world(&self, pos: Pos2, painter: &Painter) -> Pos2 {
        let rect = painter.clip_rect();
        let mut sqr_prop = rect.square_proportions() / self.zoom;
        sqr_prop.y *= -1.0;
        let to_world = emath::RectTransform::from_to(
            rect,
            Rect::from_center_size(Pos2::ZERO, sqr_prop),
        );
        to_world * pos
    }
    fn paint(&mut self, painter: &Painter) {
        let mut shapes: Vec<Shape> = Vec::new();
        let rect = painter.clip_rect();
        let mut sqr_prop = rect.square_proportions() / self.zoom;
        sqr_prop.y *= -1.0;
        let to_screen = emath::RectTransform::from_to(
            Rect::from_center_size(Pos2::ZERO, sqr_prop),
            rect,
        );
        let mut paint_line = |points: [Pos2; 2], color: Color32, width: f32| {
            let line = [to_screen * (points[0] - self.focus_center).to_pos2(), to_screen * (points[1] - self.focus_center).to_pos2()];

            // culling
            if rect.intersects(Rect::from_two_pos(line[0], line[1])) {
                shapes.push(Shape::line_segment(line, (width, color)));
            }
        };
        let center = pos2(0.0, 0.0);
        let end = pos2(0.4, 0.4);
        paint_line([center, end], Color32::from_additive_luminance(255), 1.0);
        painter.extend(shapes);
        //todo!()
    }
}
