extern crate asi_core0 as asi;
extern crate piston;
extern crate sdl2_window;
extern crate opengl_graphics;
extern crate graphics;
extern crate vecmath;
extern crate rand;
extern crate bee_challenge;

use opengl_graphics::*;
use sdl2_window::Sdl2Window;
use piston::window::*;
use piston::input::*;
use graphics::*;
use bee_challenge::{run, WingController};

struct Backend {
    gl: GlGraphics,
}

impl bee_challenge::Backend for Backend {
    type Graphics = GlGraphics;
    type Texture = Texture;

    fn load_bee_texture(&self) -> Texture {
        Texture::from_path(
            "assets/bee.png",
            &TextureSettings::new()
        ).unwrap()
    }

    fn draw<F: FnMut(Context, &mut Self::Graphics)>(
        &mut self,
        render_args: RenderArgs,
        mut f: F
    ) {
        self.gl.draw(render_args.viewport(), |c, g| f(c, g));
    }
}

fn main() {
    let opengl = OpenGL::V3_2;
    let settings = WindowSettings::new("asi_core0: bee", [640, 480])
        .exit_on_esc(true)
        .opengl(opengl);
    let mut window: Sdl2Window = settings.build().unwrap();

    let backend = Backend {
        gl: GlGraphics::new(opengl)
    };
    let tune = 0.1;
    let left = HeuristicController {
        started: false,
        value: 0.0,
        old_vector: [0.0; 2],
        prediction: [1.0; 2],
        error: 20.0,
        tune,
        children: vec![],
    };
    let right = HeuristicController {
        started: false,
        value: 0.0,
        old_vector: [0.0; 2],
        prediction: [1.0; 2],
        error: 20.0,
        tune,
        children: vec![],
    };
    run(&mut window, left, right, backend);
}

pub struct HeuristicController {
    /// Whether the controller has started.
    pub started: bool,
    /// Control value.
    pub value: f64,
    /// The old vector.
    pub old_vector: [f64; 2],
    /// The predicted change in distance per unit of time.
    pub prediction: [f64; 2],
    /// Predicted error in predicted change.
    pub error: f64,
    /// A factor describing how much to tune predictions.
    pub tune: f64,
    /// A list of sub-controlelrs which predicts how the controller would behave
    /// if the control value was changed.
    /// These sub-controllers advice the prediction.
    pub children: Vec<HeuristicController>,
}

impl WingController for HeuristicController {
    fn get_started(&self) -> bool {self.started}
    fn set_started(&mut self, value: bool) {self.started = value}
    fn get_value(&self) -> f64 {self.value}
    fn get_prediction(&self) -> [f64; 2] {self.prediction}
    fn set_prediction(&mut self, value: [f64; 2]) {self.prediction = value}
    fn get_error(&self) -> f64 {self.error}
    fn set_error(&mut self, value: f64) {self.error = value}
    fn get_old_vector(&self) -> [f64; 2] {self.old_vector}
    fn set_old_vector(&mut self, value: [f64; 2]) {self.old_vector = value}
    fn get_tune(&self) -> f64 {self.tune}

    fn set_value(&mut self, value: f64) {
        if value == self.value {return};

        let mut min_value_dist: Option<(usize, f64)> = None;
        for i in 0..self.children.len() {
            let ref mut child = self.children[i];
            let value_dist = (child.value - value) * (child.value - value);
            if min_value_dist.is_none() || min_value_dist.unwrap().1 > value_dist {
                min_value_dist = Some((i, value_dist));
            }
        }

        if let Some((i, child_value)) = min_value_dist {
            if child_value < (self.value - value) * (self.value - value) {
                use std::mem::swap;

                // Swap values with child since it is more likely to be
                // a better starting point than the current one.
                let ref mut child = self.children[i];
                swap(&mut self.prediction, &mut child.prediction);
                swap(&mut self.value, &mut child.value);
                swap(&mut self.old_vector, &mut child.old_vector);
                swap(&mut self.error, &mut child.error);
                self.value = value;
                return;
            }
        }
        if self.children.len() < 1 {
            // Remember what is learned so far.
            let new_child = HeuristicController {children: vec![], ..*self};
            self.children.push(new_child);
        }

        self.value = value;
    }

    fn predict(&self, new_value: f64, dt: f64) -> [f64; 2] {
        use vecmath::vec2_add as add;
        use vecmath::vec2_scale as scale;
        use vecmath::vec2_len as len;

        // Use weighted average of predictions of itself and children.
        // Compute weight by taking into account how close they are to the new value,
        // how close their last distance is to extrapolated half distance,
        // and how far into the future and the predicted error.
        // Estimate the predicted distance given no change.
        let dist = len(add(self.old_vector, scale(self.prediction, 0.5 * dt)));
        let mut weight = 1.0 / ((self.value - new_value) * (self.value - new_value) + dt + self.error);
        let mut prediction = scale(self.prediction, weight);
        for i in 0..self.children.len() {
            let ref child = self.children[i];
            // Compute the predicted distance of child.
            let child_dist = len(add(child.old_vector, scale(child.prediction, 0.5 * dt)));
            let w = 1.0 /
                    ((child.value - new_value) * (child.value - new_value) +
                     (child_dist - dist) * (child_dist - dist) + dt + child.error);
            prediction = add(prediction, scale(child.prediction, w));
            weight += w;
        }
        scale(prediction, dt / weight)
    }
}
