// main.rs

// hides console on release
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::collections::LinkedList;

use macroquad::prelude::*;
// use macroquad::input::{is_key_down, is_key_pressed};
use colorsys::{Rgb, Hsl};


// a module to deal with the fps
pub mod fps {
    use std::time::{SystemTime, UNIX_EPOCH};
    use std::collections::LinkedList;

    // FPS counter struct
    pub struct FPS {
        frame_times: LinkedList<u128>,
        max_frames: usize,
        frame: usize,
    }

    impl FPS {
        pub fn new(max_frames: usize) -> FPS {
            FPS { frame_times: LinkedList::new(), max_frames, frame: 0 }
        }

        // add the current frame time and remove the oldest frame if there are
        // too many
        // should be called every frame as well as before fps
        pub fn update(&mut self) {
            self.frame_times.push_back(FPS::now());
            if self.frame_times.len() > self.max_frames {
                self.frame_times.pop_front();
            }
            self.frame += 1;
        }

        // returns the current fps, calculated as the amount of frames divided 
        // by the difference between the newest and oldest 
        pub fn fps(&self) -> usize {
            if self.frame_times.len() < 2 {
                0
            } else {
                let diff = 
                    self.frame_times.back().unwrap() - 
                    self.frame_times.front().unwrap();
                if diff == 0 { 
                    0 
                } else {
                    ((self.frame_times.len() as f64) / (diff as f64) * 1000.0) as usize
                }
            }
        }

        // returns the current frame
        pub fn frame(&self) -> usize {
            self.frame
        }

        // gets the current time
        fn now() -> u128 {
            let start = SystemTime::now();
            start
                .duration_since(UNIX_EPOCH)
                .expect("Time went backwards")
                .as_millis()
        }
    }
}


// double pendulum state
// theta1 and theta2 are the inner angles of the pendulum
struct DoublePendulum {
    origin_x: f64,
    origin_y: f64,
    length: f64,
    theta1: f64,
    theta2: f64,
    mass: f64,
    angular1: f64,
    angular2: f64,
    color: macroquad::color::Color,
    prev_angles: LinkedList<(f64, f64)>,
}

impl DoublePendulum {
    fn new(origin_x: f64, origin_y: f64, length: f64, theta1: f64, theta2: f64, color: macroquad::color::Color) -> DoublePendulum {
        DoublePendulum { origin_x, origin_y, length, theta1, theta2, mass: 1.0, angular1: 0.0, angular2: 0.0, color, prev_angles: LinkedList::new() }
    }

    fn draw(&self) {
        let radius = 8.0;
        let thickness = 7.0;

        // let line_color = Color::new(0.7, 0.7, 0.7, 0.5);
        // let node_color = Color::new(0.0, 0.0, 0.0, 1.0);

        // let line_color = Color::new(0.08, 0.05, 0.05, 1.0);
        // let node_color = Color::new(0.08, 0.05, 0.05, 1.0);

        let line_color = self.color;
        let node_color = self.color;

        let ox = self.origin_x as f32;
        let oy = self.origin_y as f32;
        let dx1 = self.dx1() as f32;
        let dy1 = self.dy1() as f32;
        let dx2 = self.dx2() as f32;
        let dy2 = self.dy2() as f32;

        draw_line(ox, oy, ox + dx1, oy + dy1, thickness, line_color);
        draw_line(ox + dx1, oy + dy1, ox + dx2, oy + dy2, thickness, line_color);
        draw_circle(ox, oy, radius, node_color);
        draw_circle(ox + dx1, oy + dy1, radius, node_color);
        draw_circle(ox + dx2, oy + dy2, radius, node_color);
        // draw_circle(ox + dx2, oy + dy2, radius, self.color);
    }

    fn draw_trace(&self) {
        // let trace_color = Color::new(0.3, 0.3, 1.0, 1.0);
        // let trace_color = Color::new(0.545, 0.867, 0.945, 0.25);
        let trace_color = self.color;

        let ox = self.origin_x as f32;
        let oy = self.origin_y as f32;

        let mut thickness = 10.0;
        let angle_zip = self.prev_angles.iter().zip(self.prev_angles.iter().skip(1));
        for (i, j) in angle_zip {
            let (a1, a2) = i;
            let (b1, b2) = j;

            let t1x = self.length * 100.0 * (a1.sin() + a2.sin());
            let t1y = self.length * 100.0 * (a1.cos() + a2.cos());

            let t2x = self.length * 100.0 * (b1.sin() + b2.sin());
            let t2y = self.length * 100.0 * (b1.cos() + b2.cos());

            draw_line(ox + t1x as f32, oy + t1y as f32, ox + t2x as f32, oy + t2y as f32, thickness, trace_color);

            thickness *= 0.99;
        }

    }
}


const g: f64 = 9.81;


// the motion of the pendulum is calculated using the Runge-Kutta method (RK4)
// https://en.wikipedia.org/wiki/Double_pendulum
// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
// https://www.diego.assencio.com/?index=1500c66ae7ab27bb0106467c68feebc6
impl DoublePendulum {
    fn update(&mut self, timestep: f64) {
        // if the timestep is too large, the solver becomes inaccurate 
        // this can happen if the program lags, ex, the user moves the window
        if timestep > 0.02 {
            return;
        }

        // runge-kutta implementation
        let current = DVec4::new(self.theta1, self.theta2, self.angular1, self.angular2);

        self.prev_angles.push_front((self.theta1, self.theta2));
        if self.prev_angles.len() > 144 {
            self.prev_angles.pop_back();
        }

        let k1 = self.runge_kutta_func(current);
        let k2 = self.runge_kutta_func(current + (timestep / 2.0) * k1);
        let k3 = self.runge_kutta_func(current + (timestep / 2.0) * k2);
        let k4 = self.runge_kutta_func(current + timestep * k3);

        let updated = current + (timestep / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

        self.theta1 = updated.x;
        self.theta2 = updated.y;
        self.angular1 = updated.z;
        self.angular2 = updated.w;
    }

    // lots of stuff cancels since l1 = l2 and m1 = m2
    fn a1(&self, theta1: f64, theta2: f64) -> f64 {
        (theta1 - theta2).cos() / 2.0
    }

    // lots of stuff cancels since l1 = l2 and m1 = m2
    fn a2(&self, theta1: f64, theta2: f64) -> f64 {
        (theta1 - theta2).cos()
    }

    fn f1(&self, theta1: f64, theta2: f64, angular1: f64, angular2: f64) -> f64 {
        -(angular2 * angular2) * (theta1 - theta2).sin() / 2.0
        - g / self.length * theta1.sin()
    }

    fn f2(&self, theta1: f64, theta2: f64, angular1: f64, angular2: f64) -> f64 {
        (angular1 * angular1) * (theta1 - theta2).sin()
        - g / self.length * theta2.sin()
    }

    fn g1(&self, t1: f64, t2: f64, ang1: f64, ang2: f64) -> f64 {
        (self.f1(t1, t2, ang1, ang2) - self.a1(t1, t2) * self.f2(t1, t2, ang1, ang2)) /
        (1.0 - self.a1(t1, t2)*self.a2(t1, t2))
    }

    fn g2(&self, t1: f64, t2: f64, ang1: f64, ang2: f64) -> f64 {
        (-self.a2(t1, t2) * self.f1(t1, t2, ang1, ang2) + self.f2(t1, t2, ang1, ang2)) /
        (1.0 - self.a1(t1, t2)*self.a2(t1, t2))
    }

    fn runge_kutta_func(&self, params: DVec4) -> DVec4 {
        DVec4::new(
            params.z,
            params.w,
            self.g1(params.x, params.y, params.z, params.w),
            self.g2(params.x, params.y, params.z, params.w),
        )
    }
}

// these functions get the endpoints of each rod
// useful for drawing the double pendulum
impl DoublePendulum {
    fn dx1(&self) -> f64 {
        self.length * 100.0 * self.theta1.sin()
    }

    fn dy1(&self) -> f64 {
        self.length * 100.0 * self.theta1.cos()
    }

    fn dx2(&self) -> f64 {
        self.length * 100.0 * (self.theta1.sin() + self.theta2.sin())
    }

    fn dy2(&self) -> f64 {
        self.length * 100.0 * (self.theta1.cos() + self.theta2.cos())
    }
}


async fn run() {
    let mut fps_counter = fps::FPS::new(64);
    let dp_count = 300;

    let origin = (300.0, 300.0);
    let length = 1.0;

    let angle1: f64 = 180.0;
    let angle2: f64 = 180.0 + macroquad::rand::gen_range(-1.0 as f64, 1.0 as f64);

    let offset = 0.0001;

    // each pendulum is created with a slight offset in the second angle
    // to show the chaotic behavior
    let mut dp_vec: Vec<DoublePendulum> = Vec::new();
    for i in 0..dp_count {
        let c = Rgb::from(Hsl::new(255.0 * (i as f64) / ((dp_count - 1) as f64), 100.0, 80.0, None));

        dp_vec.push(
            DoublePendulum::new(
                origin.0, origin.1, length,
                angle1.to_radians(),
                (angle2 + offset * (i as f64)).to_radians(),
                Color::new(c.red() as f32 / 255.0, c.green() as f32 / 255.0, c.blue() as f32 / 255.0, 0.25),
            )
        );
    }

    macroquad::window::request_new_screen_size(600.0, 600.0);
    for i in 0..dp_vec.len() {
        dp_vec[i].draw_trace();
        if is_key_down(KeyCode::LeftShift) {
            dp_vec[i].draw();
        }
    }
    next_frame().await;

    // update loop
    let bg_color = Color::new(0.95, 0.95, 0.95, 1.0);
    loop {
        clear_background(bg_color);

        // restart if the R key was pressed
        if is_key_pressed(KeyCode::R) {
            return;
        }

        fps_counter.update();
        draw_text(&format!("R to restart"), 10.0, 20.0, 20.0, BLACK);
        draw_text(&format!("SHIFT to show pendulum"), 10.0, 40.0, 20.0, BLACK);
        draw_text(&format!("FPS: {}", fps_counter.fps()), 10.0, 60.0, 20.0, BLACK);
        draw_text(&format!("Frame: {}", fps_counter.frame()), 10.0, 80.0, 20.0, BLACK);
        
        for i in 0..dp_vec.len() {
            
            dp_vec[i].draw_trace();
        }
        if is_key_down(KeyCode::LeftShift) {
            for i in 0..dp_vec.len() {
                dp_vec[i].draw();
            }  
        }
        for i in 0..dp_vec.len() {
            dp_vec[i].update(macroquad::time::get_frame_time() as f64);
        }

        next_frame().await
    }
}


#[macroquad::main("Double Pendulum")]
async fn main() {
    loop {
        run().await;
    }
}
