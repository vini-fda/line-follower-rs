use egui::{Color32, plot::{Line, PlotPoints}};
use line_follower_rs::math_utils::lattice_points;
use macroquad::prelude::*;
use std::f32::consts::PI;

fn window_conf() -> Conf {
    Conf {
        window_title: "Line Follower Simulation".to_owned(),
        high_dpi: true,
        ..Default::default()
    }
}

fn draw_robot(x: f32, y: f32, angle: f32, color: Color) {
    let w = 0.1;
    let r = w / 2f32.sqrt();
    
    let (cos_t, sin_t) = ((angle * PI / 180.0).cos(), (angle * PI / 180.0).sin());
    let l = 1.1*w;
    draw_line(l*0.5*(cos_t - sin_t), l*0.5*(cos_t + sin_t), -l*0.5*(cos_t + sin_t), l*0.5*(cos_t - sin_t), 0.02, BLUE);
    draw_poly(x, y, 4, r, angle + 45.0, color);
}

fn draw_grid(origin: Vec2, camera: &Camera2D, dx: f32, dy: f32) {
    // draw an "infinite" grid which is zoomable and pannable
    // uses draw_grid_from_bounds
    // the bounds depend on the camera's zoom and position
    // calculate min_bounds and max_bounds in world space
    let (w, h) = (screen_width(), screen_height());
    let min_bounds = camera.screen_to_world(vec2(0., h));
    let max_bounds = camera.screen_to_world(vec2(w, 0.));
    draw_grid_from_bounds(origin, min_bounds, max_bounds, dx, dy);
}

fn draw_grid_from_bounds(origin: Vec2, min_bounds: Vec2, max_bounds: Vec2, dx: f32, dy: f32) {
    // draw an "infinite" grid which is zoomable and pannable
    let (x_0, y_0) = (origin[0], origin[1]);
    let (x_min, y_min) = (min_bounds[0], min_bounds[1]);
    let (x_max, y_max) = (max_bounds[0], max_bounds[1]);
    let range_x = lattice_points(x_0, x_min, x_max, dx);
    let range_y = lattice_points(y_0, y_min, y_max, dy);
    const THICKNESS: f32 = 0.005;
    const ALPHA: f32 = 0.3;
    for x in range_x {
        draw_line(x, y_min, x, y_max, THICKNESS, Color::new(0.5, 0.5, 0.5, ALPHA));
    }
    for y in range_y {
        draw_line(x_min, y, x_max, y, THICKNESS, Color::new(0.5, 0.5, 0.5, ALPHA));
    }
    draw_line(0.0, y_min, 0.0, y_max, THICKNESS * 1.1, Color::new(0., 0., 0., ALPHA));
    draw_line(x_min, 0.0, x_max, 0.0, THICKNESS * 1.1, Color::new(0., 0., 0., ALPHA));
}

fn draw_primitives() {
    //draw_line(-0.1, 0.1, 0.1, 0.8, 0.05, BLUE);
    //draw_rectangle(-0.1, 0.1, 0.2, 0.2, GREEN);
    //draw_circle(0., 0., 0.1, YELLOW);
    //draw_poly(0.0, 0.0, 4, 0.1 * 2.0f32.sqrt(), 45.0, RED);
    draw_robot(0.0, 0.0, 15.0, RED);
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut show_egui_demo_windows = false;
    let mut egui_demo_windows = egui_demo_lib::DemoWindows::default();
    let mut draw_primitives_after_egui = false;

    let mut should_draw_grid = true;

    let mut pixels_per_point: Option<f32> = None;

    let mut zoom: f32 = 1.0;

    const CAMERA_SPEED: f32 = 3.0e-2;
    
    let mut camera_center: Vec2 = Vec2::ZERO;

    loop {
        clear_background(WHITE);

        egui_macroquad::ui(|egui_ctx| {
            if pixels_per_point.is_none() {
                pixels_per_point = Some(egui_ctx.pixels_per_point());
            }

            if show_egui_demo_windows {
                egui_demo_windows.ui(egui_ctx);
            }

            egui::Window::new("Options").show(egui_ctx, |ui| {
                ui.checkbox(&mut show_egui_demo_windows, "Show egui demo windows");
                ui.checkbox(
                    &mut draw_primitives_after_egui,
                    "Draw macroquad primitives after egui",
                );

                ui.checkbox(&mut should_draw_grid, "Draw grid");

                let response = ui.add(
                    egui::Slider::new(pixels_per_point.as_mut().unwrap(), 0.75..=3.0)
                        .logarithmic(true),
                );

                ui.add(egui::Slider::new(&mut zoom, 0.1..=10.0).logarithmic(true));

                // Don't change scale while dragging the slider
                if response.drag_released() {
                    egui_ctx.set_pixels_per_point(pixels_per_point.unwrap());
                }
            });

            // add a transparent overlay on top of macroquad primitives
            // let my_frame = egui::containers::Frame::none().fill(egui::Color32::TRANSPARENT);
            // egui::CentralPanel::default().frame(my_frame).show(egui_ctx, |ui| {
            //     // make plot with grid
            //     // center plot (x,y) at camera's (0,0)
            //     let plot = egui::plot::Plot::new("debug_view_coordinates")
            //                 .view_aspect(1.0)
            //                 .allow_zoom(false)
            //                 .allow_drag(false)
            //                 .allow_scroll(false)
            //                 .show_background(false);
                
            //     plot.show(ui, |plot_ui| {
            //         // plot_ui.line(
            //         //     Line::new(PlotPoints::from_explicit_callback(move |x| 1.0 * x,..,100,))
            //         // )
            //     });
            // });
        });

        // WASD camera movement
        let mut camera_velocity: Vec2 = Vec2::ZERO;

        if is_key_down(KeyCode::W) {
            camera_velocity.y += 1.0;
        }
        if is_key_down(KeyCode::S) {
            camera_velocity.y -= 1.0;
        }
        if is_key_down(KeyCode::A) {
            camera_velocity.x -= 1.0;
        }
        if is_key_down(KeyCode::D) {
            camera_velocity.x += 1.0;
        }
        // normalize camera velocity
        if camera_velocity != Vec2::ZERO {
            camera_velocity = camera_velocity.normalize() * CAMERA_SPEED / zoom;
        }

        camera_center += camera_velocity;

        let camera = Camera2D {
            zoom: vec2(zoom, zoom * screen_width() / screen_height()),
            target: camera_center,
            ..Default::default()
        };

        set_camera(&camera);

        if should_draw_grid {
            draw_grid(Vec2::ZERO, &camera, 0.1, 0.1);
        }
        

        if !draw_primitives_after_egui {
            draw_primitives();
        }
        egui_macroquad::draw();
        if draw_primitives_after_egui {
            draw_primitives();
        }

        next_frame().await
    }
}
