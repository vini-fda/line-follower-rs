use egui::{Color32, plot::{Line, PlotPoints}};
use line_follower_rs::math_utils::lattice_points;
use macroquad::prelude::*;
use nalgebra::Vector2;
use std::f32::consts::PI;
use line_follower_rs::geometry::interpolated_paths::{Path, predefined_closed_path};
use line_follower_rs::geometry::sdf_paths::{SDF, predefined_closed_path_sdf};
use line_follower_rs::simulation::integrator::{Integrator, Rk4, Verlet};
use line_follower_rs::simulation::ode_system::{OdeSystem, Vector};
use itertools::Itertools;
use nalgebra::Rotation2;

fn window_conf() -> Conf {
    Conf {
        window_title: "Line Follower Simulation".to_owned(),
        high_dpi: true,
        ..Default::default()
    }
}

fn draw_robot(x: f32, y: f32, angle: f32, color: Color) {
    let angle = angle - 90.0;
    let w = 0.1;
    let r = w / 2f32.sqrt();
    
    let (cos_t, sin_t) = ((angle * PI / 180.0).cos(), (angle * PI / 180.0).sin());
    let l = 1.1*w;
    draw_line(x + l*0.5*(cos_t - sin_t), y + l*0.5*(cos_t + sin_t), x -l*0.5*(cos_t + sin_t), y + l*0.5*(cos_t - sin_t), 0.02, BLUE);
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

fn draw_path(path: &Path<f32>, color: Color) {
    for ((x0, y0), (x1, y1)) in path.points().tuple_windows() {
        draw_line(x0, y0, x1, y1, 0.04, color)
    }
}

#[inline(always)]
fn find_theta(y: &[f64; 5], l: f64, d: f64) -> f64 {
    // check if ys is strictly increasing or decreasing
    let mut increasing = true;

    for i in 0..y.len() - 1 {
        if y[i] > y[i + 1] {
            increasing = false;
            break;
        }
    }
    if increasing {
        let m = (y[4] - y[0]) / l;
        if m.abs() <= 1.0 {
            return m.acos();
        }
    }
    
    let mut decreasing = true;

    for i in 0..y.len() - 1 {
        if y[i] < y[i + 1] {
            decreasing = false;
            break;
        }
    }

    if decreasing {
        let m = (y[0] - y[4]) / l;
        if m.abs() <= 1.0 {
            return m.acos();
        }
    }

    if (y[4] - y[0]).abs() < 1e-6 {
        return 0.0;
    }

    let t = y[0] / (y[4] - y[0]);
    ((0.5 + t) / d).atan()
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

    // sample once per frame
    let mut mouse_sdf_history = [0.0f32; 400];
    let mut i = 0;

    let mut wl_history = [0.0f32; 400];
    let mut wl_i = 0;

    let mut wr_history = [0.0f32; 400];
    let mut wr_i = 0;

    let mut prev_t = 0.0;
    let mut prev_error = 0.0;
    let main_path_sdf = predefined_closed_path_sdf();

    let robot_dynamics = |t: f64, x: &Vector<8>| {
        let (x, y, theta, wl, dwl, wr, dwr, int_error) = (x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]);
        const ROBOT_WHEEL_RADIUS: f64 = 0.04;
        const ROBOT_SIDE_LENGTH: f64 = 0.1;

        const SENSOR_ARRAY_LENGTH: f64 = ROBOT_SIDE_LENGTH * 1.1;
        const SENSOR_ARRAY_SEPARATION: f64 = SENSOR_ARRAY_LENGTH / 5.0;
        const MAX_SENSOR_DISTANCE: f64 = 4.0 * SENSOR_ARRAY_SEPARATION / 5.0;
        const SENSOR_DISTANCE_TO_ROBOT_CENTER: f64 = ROBOT_SIDE_LENGTH * 3.0 / 5.0;

        // we initially consider the robot pointing rightward, so the sensor array is vertical (x constant)
        let mut sensor_positions = [
            Vector2::<f64>::new(SENSOR_DISTANCE_TO_ROBOT_CENTER, 2.0 * SENSOR_ARRAY_SEPARATION),
            Vector2::<f64>::new(SENSOR_DISTANCE_TO_ROBOT_CENTER, SENSOR_ARRAY_SEPARATION),
            Vector2::<f64>::new(SENSOR_DISTANCE_TO_ROBOT_CENTER, 0.0),
            Vector2::<f64>::new(SENSOR_DISTANCE_TO_ROBOT_CENTER, -SENSOR_ARRAY_SEPARATION),
            Vector2::<f64>::new(SENSOR_DISTANCE_TO_ROBOT_CENTER, -2.0 * SENSOR_ARRAY_SEPARATION),
        ];

        // now we rotate the sensor array by theta counter-clockwise
        // and translate it by (x, y)
        for p in sensor_positions.iter_mut() {
            let rotation = Rotation2::new(theta);
            let rotated = rotation * (*p);
            *p = Vector2::<f64>::new(x, y) + rotated;
        }

        let mut sensor_distances = [0.0f64; 5];
        for i in 0..5 {
            if let Some(d) = main_path_sdf.sdf(sensor_positions[i].x, sensor_positions[i].y) {
                sensor_distances[i] = d.abs();
            } else {
                sensor_distances[i] = 1e10;
            }
        }

        const KP: f64 = 0.8;
        const KI: f64 = 0.2;
        const KD: f64 = 1.5;
        
        // estimate the robot's angle relative to the track 
        // (i.e. the error in theta) by using the sensor array data
        let error_estimate = find_theta(&sensor_distances, MAX_SENSOR_DISTANCE, ROBOT_SIDE_LENGTH / 2.0);
        let deriv_error = if t > prev_t {
            (error_estimate - prev_error) / (t - prev_t)
        } else {
            0.0
        };
         
        prev_error = error_estimate;
        prev_t = t;
        let desired_dtheta = KP * error_estimate + KI * int_error + KD * deriv_error;
        let k = ROBOT_SIDE_LENGTH * (B*R + K*K) / (K * ROBOT_WHEEL_RADIUS);
        let u = 5.0;
        let v = k * desired_dtheta;
        let ul = (u - v) / 2.0;
        let ur = (u + v) / 2.0;

        if ur.is_nan() || ul.is_nan() {
            panic!("nan");
        }

        const J: f64 = 0.1;
        const B: f64 = 0.1;
        const R: f64 = 0.03;
        const L: f64 = 0.1;
        const K: f64 = 0.1;

        let c0: f64 = 1.0;
        let c1: f64 = 1.414;
        let c2: f64 = 1.0;

        let speed = ROBOT_WHEEL_RADIUS * (wl + wr) / 2.0;
        let d_theta = ROBOT_WHEEL_RADIUS * (wr - wl) / ROBOT_SIDE_LENGTH;
        let d_x = speed * theta.cos();
        let d_y = speed * theta.sin();
        let d_wl = dwl;
        let d_dwl = (ul - c1 * dwl - c2 * wl) / c0;
        let d_wr = dwr;
        let d_dwr = (ur - c1 * dwr - c2 * wr) / c0;

        //(x, y, theta, wl, dwl, wr, dwr, int_error)
        Vector::<8>::from_column_slice(&[d_x, d_y, d_theta, d_wl, d_dwl, d_wr, d_dwr, error_estimate])
    };
    let initial_condition = Vector::<8>::from_column_slice(&[0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let mut robot_integrator = Rk4::new(robot_dynamics, 0.0, initial_condition);

    

    let main_path = predefined_closed_path();
    

    loop {
        clear_background(WHITE);

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

        let mouse_world_pos = camera.screen_to_world(macroquad::input::mouse_position().into());

        set_camera(&camera);

        // draw egui
        zoom *= (mouse_wheel().1 * 0.1).exp();

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

                // show mouse position in world coordinates
                let (mouse_x, mouse_y) = (mouse_world_pos.x, mouse_world_pos.y);
                ui.label(format!("Mouse position: ({:.3}, {:.3})", mouse_x, mouse_y));

                // show distance to path
                let distance = main_path_sdf.sdf(mouse_x as f64, mouse_y as f64);
                if let Some(d) = distance {
                    mouse_sdf_history[i] = d as f32;
                    ui.label(format!("Distance to path: {:.3}", d));
                } else {
                    mouse_sdf_history[i] = 0.0;
                    ui.label("Distance to path: N/A");
                }
                i = (i + 1) % mouse_sdf_history.len();
                let (mouse_wheel_x, mouse_wheel_y) = mouse_wheel();
                ui.label(format!("Mouse wheel: ({:.3}, {:.3})", mouse_wheel_x, mouse_wheel_y));

                // Don't change scale while dragging the slider
                if response.drag_released() {
                    egui_ctx.set_pixels_per_point(pixels_per_point.unwrap());
                }
            });

            egui::Window::new("Plot").show(egui_ctx, |ui| {
                let plot = egui::plot::Plot::new("debug_view_coordinates")
                            .view_aspect(1.0)
                            .allow_zoom(false)
                            .allow_drag(false)
                            .allow_scroll(false)
                            .show_background(false)
                            .include_y(2.0)
                            .include_y(-2.0);
                plot.show(ui, |plot_ui| {
                    plot_ui.line(
                        Line::new(PlotPoints::from_ys_f32(&mouse_sdf_history))
                            .color(egui::Color32::from_rgb(0, 0, 255))
                            .name("Distance to path")
                    )
                });
            });

            egui::Window::new("Omega_l plot").show(egui_ctx, |ui| {
                let plot = egui::plot::Plot::new("debug_view_coordinates")
                            .view_aspect(1.0)
                            .allow_zoom(false)
                            .allow_drag(false)
                            .allow_scroll(false)
                            .show_background(false)
                            .include_y(10.0)
                            .include_y(-10.0);
                plot.show(ui, |plot_ui| {
                    plot_ui.line(
                        Line::new(PlotPoints::from_ys_f32(&wl_history))
                            .color(egui::Color32::from_rgb(20, 200, 255))
                            .name("wl")
                    );
                    plot_ui.line(
                        Line::new(PlotPoints::from_ys_f32(&wr_history))
                            .color(egui::Color32::from_rgb(200, 20, 255))
                            .name("wr")
                    );
                    // plot_ui.line(
                    //     Line::new(PlotPoints::from_ys_f32(&[0.5; 400]))
                    //         .color(egui::Color32::from_rgb(255, 255, 0))
                    //         .name("ref")
                    // );
                });
            });
        });

        if should_draw_grid {
            draw_grid(Vec2::ZERO, &camera, 0.1, 0.1);
        }
        
        draw_path(&main_path, BLACK);
        robot_integrator.step(1.0/60.0);
        let sim_result = robot_integrator.get_state();
        wl_history[wl_i] = sim_result[3] as f32;
        wl_i = (wl_i + 1) % wl_history.len();

        wr_history[wr_i] = sim_result[5] as f32;
        wr_i = (wr_i + 1) % wl_history.len();
        draw_robot(sim_result[0] as f32, sim_result[1] as f32, sim_result[2] as f32 * 180.0 / PI, RED);
        egui_macroquad::draw();

        next_frame().await
    }
}
