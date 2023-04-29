use egui::plot::{Legend, Line, PlotPoints, Points};
use egui::{RichText, TextStyle};
use itertools::Itertools;
use linefollower_core::geometry::closed_path::predefined_closed_path;
use linefollower_core::geometry::track::sample_points;
use linefollower_core::ode_solver::ode_system::Vector;
use linefollower_core::simulation::robot::RobotSimulation;
use linefollower_core::utils::math::sigmoid;
use linefollower_gui::graphics::draw::{draw_closed_curve, ROBOT_SIDE_LENGTH, SENSOR_ARRAY_LENGTH};
use macroquad::color::Color;
use macroquad::miniquad::conf::Icon;
use macroquad::prelude::{
    is_key_down, mouse_wheel, vec2, Camera2D, KeyCode, Vec2, GREEN, PURPLE, RED, SKYBLUE, YELLOW,
};
use macroquad::shapes::draw_circle;
use macroquad::window::{next_frame, screen_height, screen_width, Conf};
use std::f32::consts::PI;
use std::sync::Arc;

const MAX_ZOOM: f32 = 15.0;
const MIN_ZOOM: f32 = 0.01;

// PID Constants
const KP: f64 = 3.130480505558367; //2.565933287511912; //3.49;
const KI: f64 = 73.01770822094774; //52.33814267275805; //37.46;
const KD: f64 = 11.273635752474997; //10.549477731373042; //13.79;
const SPEED: f64 = 1.6710281486754923; //1.4602563968294984; //1.04;

// Kp: , Ki: , Kd:

struct ColorScheme {
    pub darkmode: bool,
}

impl ColorScheme {
    fn new(darkmode: bool) -> Self {
        Self { darkmode }
    }

    fn background(&self) -> Color {
        if self.darkmode {
            Color::new(0.1, 0.1, 0.1, 1.0)
        } else {
            Color::new(1.0, 1.0, 1.0, 1.0)
        }
    }

    fn path(&self) -> Color {
        if self.darkmode {
            Color::new(1.0, 1.0, 1.0, 1.0)
        } else {
            Color::new(0.1, 0.1, 0.1, 1.0)
        }
    }
}

fn window_conf() -> Conf {
    let file_bytes = include_bytes!("../assets/logo.ico");
    let icon_dir = ico::IconDir::read(std::io::Cursor::new(file_bytes.as_slice())).unwrap();
    const EXPECTED_NUM_ICONS: usize = 3;
    assert_eq!(EXPECTED_NUM_ICONS, icon_dir.entries().len());
    // Print the size of each image in the ICO file:
    let entries = icon_dir.entries();
    let small = entries[0]
        .decode()
        .unwrap()
        .rgba_data()
        .try_into()
        .expect("slice with incorrect length");
    let medium = entries[1]
        .decode()
        .unwrap()
        .rgba_data()
        .try_into()
        .expect("slice with incorrect length");
    let big = entries[2]
        .decode()
        .unwrap()
        .rgba_data()
        .try_into()
        .expect("slice with incorrect length");

    Conf {
        window_title: "Line Follower Simulation".to_owned(),
        high_dpi: true,
        icon: Some(Icon { small, medium, big }),
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    const DT: f64 = 1.0 / 60.0;
    let mut should_draw_grid = false;
    let mut pixels_per_point: Option<f32> = Some(1.5);
    let mut zoom: f32 = 0.3;
    const CAMERA_SPEED: f32 = 3.0e-2;
    let mut camera_center: Vec2 = [0.0, -4.0].into();
    let mut follow_robot = true;
    let mut color_scheme = ColorScheme::new(true);

    let mut show_omega_plot = false;
    let mut show_robot_distance_plot = false;

    // control simulation speed
    let mut speed_multiplier = 1;

    // pause simulation
    let mut paused = false;

    // sample once per frame
    let mut robot_sdf_history = [0.0f32; 600];
    let mut i = 0;

    let mut wl_history = [0.0f32; 600];
    let mut wl_i = 0;

    let mut wr_history = [0.0f32; 600];
    let mut wr_i = 0;
    let main_path = predefined_closed_path();
    let path_points = sample_points(&main_path, 0.1).collect_vec();

    let initial_condition = Vector::<7>::from_column_slice(&[0.0, -4.0, 0.1, 0.0, 0.0, 0.0, 0.0]);
    let mut robot_sim = RobotSimulation::new(
        initial_condition,
        KP,
        KI,
        KD,
        SPEED,
        Arc::new(main_path.clone()),
    );

    // initial config of egui context
    egui_macroquad::ui(|egui_ctx| {
        if let Some(pixels_per_point) = pixels_per_point {
            egui_ctx.set_pixels_per_point(pixels_per_point);
        }
    });
    egui_macroquad::draw();

    loop {
        macroquad::window::clear_background(color_scheme.background());

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

        if follow_robot {
            let robot_state = robot_sim.get_state();
            let robot_pos = vec2(robot_state[0] as f32, robot_state[1] as f32);
            camera_center = robot_pos;
        } else {
            camera_center += camera_velocity;
        }

        let camera = Camera2D {
            zoom: vec2(zoom, zoom * screen_width() / screen_height()),
            target: camera_center,
            ..Default::default()
        };

        let mouse_world_pos = camera.screen_to_world(macroquad::input::mouse_position().into());

        macroquad::prelude::set_camera(&camera);

        if !paused {
            const STEPS: usize = 4;
            const STEP_SIZE: f64 = DT / STEPS as f64;
            for _ in 0..speed_multiplier {
                for _ in 0..STEPS {
                    robot_sim.step(STEP_SIZE);
                }
                wl_history[wl_i] = robot_sim.get_state()[3] as f32;
                wl_i = (wl_i + 1) % wl_history.len();

                wr_history[wr_i] = robot_sim.get_state()[5] as f32;
                wr_i = (wr_i + 1) % wl_history.len();

                robot_sdf_history[i] = robot_sim.robot_sdf_to_path() as f32;
                i = (i + 1) % robot_sdf_history.len();
            }
        }
        // calculate zoom from mouse scroll
        let mw = sigmoid(mouse_wheel().1) - 0.5;
        let new_zoom = zoom * (mw * 0.1).exp();
        if new_zoom <= MIN_ZOOM {
            zoom = MIN_ZOOM;
        } else if new_zoom >= MAX_ZOOM {
            zoom = MAX_ZOOM;
        } else {
            zoom = new_zoom;
        }

        egui_macroquad::ui(|egui_ctx| {
            if pixels_per_point.is_none() {
                pixels_per_point = Some(egui_ctx.pixels_per_point());
            }

            egui::SidePanel::left("left_panel")
                .resizable(false)
                .show(egui_ctx, |ui| {
                    ui.label(RichText::new("⛭ Options").heading());
                    ui.separator();
                    ui.checkbox(&mut color_scheme.darkmode, "Dark mode");
                    ui.checkbox(&mut should_draw_grid, "Draw grid");
                    ui.checkbox(&mut follow_robot, "Follow robot with camera");
                    ui.checkbox(&mut paused, "Pause simulation");
                    // simulation speed label
                    let sim_speed_label = ui.label("Simulation speed: ");
                    ui.add(egui::Slider::new(&mut speed_multiplier, 1..=3).clamp_to_range(true))
                        .labelled_by(sim_speed_label.id);
                    // edit egui's pixels per point
                    let ppp_label = ui.label("Pixels per point: ");
                    let response = ui
                        .add(
                            egui::Slider::new(pixels_per_point.as_mut().unwrap(), 0.75..=3.0)
                                .logarithmic(true),
                        )
                        .labelled_by(ppp_label.id);
                    // edit zoom
                    let zoom_label = ui.label("Zoom: ");
                    ui.add(egui::Slider::new(&mut zoom, 0.1..=10.0).logarithmic(true))
                        .labelled_by(zoom_label.id);

                    ui.label(RichText::new("ℹ Info").heading());
                    ui.separator();
                    // show mouse position in world coordinates
                    let (mouse_x, mouse_y) = (mouse_world_pos.x, mouse_world_pos.y);
                    ui.label(format!("Mouse position: ({:.3}, {:.3})", mouse_x, mouse_y));

                    // show distance to path
                    ui.label(format!("Distance to path: {:.3}", robot_sdf_history[i]));

                    let (mouse_wheel_x, mouse_wheel_y) = mouse_wheel();
                    ui.label(format!(
                        "Mouse wheel: ({:.3}, {:.3})",
                        mouse_wheel_x, mouse_wheel_y
                    ));

                    ui.label(format!("Total time: {:.3} s", robot_sim.get_time()));

                    // Don't change scale while dragging the slider
                    if response.drag_released() {
                        egui_ctx.set_pixels_per_point(pixels_per_point.unwrap());
                    }

                    use egui::special_emojis::GITHUB;
                    ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                        let heart_color = egui::Color32::from_rgb(229, 75, 75);
                        ui.hyperlink_to(
                            format!("{} GitHub repo", GITHUB),
                            "https://github.com/vini-fda/line-follower-rs",
                        );
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 0.0;
                            ui.label("Made with ");
                            ui.colored_label(heart_color, "❤");
                            ui.label(" by vini-fda");
                        });
                    });
                });

            egui::SidePanel::right("right_panel")
                .resizable(false)
                .show(egui_ctx, |ui| {
                    ui.vertical(|ui| {
                        ui.label(RichText::new("🗠 Plots").heading());
                        ui.separator();
                        ui.toggle_value(&mut show_omega_plot, "Plot omegas (ωl and ωr)")
                            .on_hover_text(
                                "Plot the left and right wheel angular velocities over time",
                            );
                        ui.toggle_value(&mut show_robot_distance_plot, "Plot robot distance")
                            .on_hover_text("Plot the distance of the robot to the path over time");

                        ui.label(RichText::new("🔧 Parameters").heading());
                        ui.separator();
                        ui.label(format!("Robot side length: {:.3}", ROBOT_SIDE_LENGTH));
                        ui.label(format!("Sensor array length: {:.3}", SENSOR_ARRAY_LENGTH));
                        // KP, KI, KD
                        ui.label(format!("KP = {:.3}", KP));
                        ui.label(format!("KI = {:.3}", KI));
                        ui.label(format!("KD = {:.3}", KD));
                    });
                });

            if show_omega_plot {
                egui::Window::new("Angular velocities").show(egui_ctx, |ui| {
                    let wl_color = egui::Color32::from_rgb(20, 200, 255);
                    let wr_color = egui::Color32::from_rgb(200, 20, 255);

                    ui.horizontal_wrapped(|ui| {
                        // Trick so we don't have to add spaces in the text below:
                        let width =
                            ui.fonts(|f| f.glyph_width(&TextStyle::Body.resolve(ui.style()), ' '));
                        ui.spacing_mut().item_spacing.x = width;
                        ui.label("This plot shows the angular velocities of the ");
                        ui.colored_label(wl_color, "left (ωl)");
                        ui.label(" and ");
                        ui.colored_label(wr_color, "right (ωr)");
                        ui.label(" wheels over time, in rad/s.");
                    });
                    let plot = egui::plot::Plot::new("debug_view_omegas")
                        .label_formatter(|name, value| {
                            if !name.is_empty() {
                                format!("{}: {:.*} rad/s", name, 1, value.y)
                            } else {
                                "".to_owned()
                            }
                        })
                        .view_aspect(2.0)
                        .allow_zoom(false)
                        .allow_drag(false)
                        .allow_scroll(false)
                        .legend(Legend::default())
                        .show_background(false);

                    plot.show(ui, |plot_ui| {
                        plot_ui.line(
                            Line::new(PlotPoints::from_ys_f32(&wl_history))
                                .color(wl_color)
                                .name("ωl(t)"),
                        );
                        plot_ui.line(
                            Line::new(PlotPoints::from_ys_f32(&wr_history))
                                .color(wr_color)
                                .name("ωr(t)"),
                        );
                    });
                });
            }

            if show_robot_distance_plot {
                egui::Window::new("Distance to track").show(egui_ctx, |ui| {
                    let positive_color = egui::Color32::from_rgb(229, 75, 75);
                    let negative_color = egui::Color32::from_rgb(92, 200, 255);
                    ui.horizontal_wrapped(|ui| {
                        // Trick so we don't have to add spaces in the text below:
                        let width = ui.fonts(|f|f.glyph_width(&TextStyle::Body.resolve(ui.style()), ' '));
                        ui.spacing_mut().item_spacing.x = width;
                        ui.label("This plot shows the distance of the robot to the path over time, in meters.");
                        ui.label("The distance can be either ");
                        ui.colored_label(positive_color, "positive");
                        ui.label(" which means it is outside the track, or ");
                        ui.colored_label(negative_color, "negative");
                        ui.label(" which means it is inside the track.");
                    });
                    let plot = egui::plot::Plot::new("debug_view_robot_distance")
                        .label_formatter(|name, value| {
                            if !name.is_empty() {
                                format!("{}: {:.3} m", name, value.y)
                            } else {
                                "".to_owned()
                            }
                        })
                        .view_aspect(2.0)
                        .allow_zoom(false)
                        .allow_drag(false)
                        .allow_scroll(false)
                        .show_background(false)
                        .include_y(0.0);
                    // .include_y(1.0)
                    // .include_y(-1.0);
                    plot.show(ui, |plot_ui| {
                        let positive_points = robot_sdf_history
                        .iter()
                        .enumerate()
                        .filter(|(_, &d)| d >= 0.0)
                        .map(|(i, &d)| [i as f64, d as f64])
                        .collect::<Vec<_>>();

                        plot_ui.points(
                            Points::new(PlotPoints::new(positive_points))
                                .color(positive_color)
                                .stems(0.0)
                                .name("d(t)"),
                        );

                        let negative_points = robot_sdf_history
                        .iter()
                        .enumerate()
                        .filter(|(_, &d)| d < 0.0)
                        .map(|(i, &d)| [i as f64, d as f64])
                        .collect::<Vec<_>>();

                        plot_ui.points(
                            Points::new(PlotPoints::new(negative_points))
                                .color(negative_color)
                                .stems(0.0)
                                .name("d(t)"),
                        );
                    });
                });
            }
        });

        if should_draw_grid {
            linefollower_gui::graphics::draw::draw_grid(Vec2::ZERO, &camera, 0.1, 0.1);
        }

        draw_closed_curve(&path_points, color_scheme.path(), 0.03);

        linefollower_gui::graphics::draw::draw_robot(
            robot_sim.get_state()[0] as f32,
            robot_sim.get_state()[1] as f32,
            robot_sim.get_state()[2] as f32 * 180.0 / PI,
            RED,
        );
        let pr = robot_sim.reference_point();
        draw_circle(pr.x as f32, pr.y as f32, 0.05, PURPLE);
        let tr = robot_sim.reference_tangent();
        // draw tangent vector to reference point
        linefollower_gui::graphics::draw::draw_vector(
            pr.x as f32,
            pr.y as f32,
            tr.x as f32 * 0.1,
            tr.y as f32 * 0.1,
            YELLOW,
        );
        // draw robot projection tangent vector
        let projection_tangent = robot_sim.robot_projection_tangent();
        linefollower_gui::graphics::draw::draw_vector(
            robot_sim.get_state()[0] as f32,
            robot_sim.get_state()[1] as f32,
            projection_tangent[0] as f32 * 0.1,
            projection_tangent[1] as f32 * 0.1,
            GREEN,
        );

        // draw robot direction vector
        let theta = robot_sim.get_state()[2] as f32;
        linefollower_gui::graphics::draw::draw_vector(
            robot_sim.get_state()[0] as f32,
            robot_sim.get_state()[1] as f32,
            theta.cos() * 0.1,
            theta.sin() * 0.1,
            SKYBLUE,
        );

        egui_macroquad::draw();

        next_frame().await
    }
}
