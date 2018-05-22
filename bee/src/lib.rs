extern crate asi_core0 as asi;
extern crate vecmath;
extern crate interpolation;
extern crate graphics;
extern crate piston;
extern crate rand;

use asi::{ActuatorId, MemoryId, SensorId, Runtime};
use piston::input::*;
use interpolation::EaseFunction;
use graphics::*;
use std::error::Error;
use std::fmt;

const BEE_TEXTURE: usize = 0;

pub struct AppRenderSettings {
    pub draw_bee_pos: bool,
    pub draw_target_pos: bool,
}

pub struct AppPhysicsSettings {
    pub gravity: [f64; 2],
}

pub struct App<Texture> {
    pub hive: Hive,
    pub flower: Flower,
    pub bee: Bee,
    pub textures: Vec<Texture>,
    pub render_settings: AppRenderSettings,
    pub physics_settings: AppPhysicsSettings,
    pub runtime: asi::StandardRuntime<Sensor, Actuator, Memory, DecisionMaker>,
}

impl<Texture: ImageSize> App<Texture> {
    pub fn event(&mut self, e: &impl GenericEvent) {
        use vecmath::vec2_add as add;

        if let Some(args) = e.update_args() {
            // Move bee.
            {
                let gravity = match self.bee.state {
                    BeeState::Landed(_) => [0.0; 2],
                    BeeState::InAir => self.physics_settings.gravity,
                };
                let agent = self.runtime.agent.as_mut().unwrap();

                // Update sensors.
                for sensor in &mut agent.sensors {
                    match *sensor {
                        Sensor::Position(ref mut pos) => {
                            *pos = self.bee.pos;
                        }
                    }
                }

                // Update actuators.
                let mut force = gravity;
                for actuator in &mut agent.actuators {
                    match *actuator {
                        Actuator::FlapLeft(ref mut wing) |
                        Actuator::FlapRight(ref mut wing) => {
                            force = add(force, wing.update(args.dt));
                        }
                    }
                }

                math::particle(&mut self.bee.pos, &mut self.bee.vel, force, args.dt);
            }

            self.runtime.update(args.dt);
        }
    }

    pub fn draw(&self, c: &Context, g: &mut impl Graphics<Texture = Texture>) {
        image(&self.textures[BEE_TEXTURE], c.transform
            .trans(self.bee.pos[0] - 20.0, self.bee.pos[1] - 20.0).zoom(0.13), g);
        if self.render_settings.draw_bee_pos {
            rectangle(color::hex("ff0000"), [self.bee.pos[0], self.bee.pos[1], 2.0, 2.0], c.transform, g);
        }
        if self.render_settings.draw_target_pos {
            let target = self.runtime.agent.as_ref().unwrap().decision_maker.target;
            rectangle(color::hex("0000ff"), [target[0], target[1], 2.0, 2.0], c.transform, g);
        }
    }
}

pub struct Hive {
    pub pos: [f64; 2],
}

pub struct Flower {
    pub pos: [f64; 2],
}

pub enum Object {
    Hive,
    Flower,
}

pub enum BeeState {
    Landed(Object),
    InAir,
}

pub struct Bee {
    pub pos: [f64; 2],
    pub vel: [f64; 2],
    pub state: BeeState,
}

pub enum Sensor {
    /// The position of the bee.
    Position([f64; 2]),
}

impl asi::Sensor<Memory> for Sensor {
    fn next(&mut self) {}
    fn try_receive(&mut self) -> Option<Result<Memory, Box<Error>>> {
        match *self {
            Sensor::Position(pos) => Some(Ok(Memory::Position(pos))),
        }
    }
}

/// Used to store the states of left and right wing actuators.
pub struct FlapWing {
    /// Wether instruction to flap the wing was received.
    pub received: bool,
    /// The time it takes to flap wing before a new flap can be done.
    pub repeat_delay: f64,
    /// The impulse delivered at maximum wing velocity for thole cycle.
    pub impulse: [f64; 2],
    /// The characteristic ease function of the force dynamics.
    pub force_function: EaseFunction,
    /// The remaining time of flapping.
    pub remaining: f64,
}

impl FlapWing {
    /// Updates the flapping actuator, returning the force during this interval.
    pub fn update(&mut self, dt: f64) -> [f64; 2] {
        use interpolation::Ease;
        use vecmath::vec2_scale as scale;

        if self.remaining > 0.0 {self.remaining -= dt};
        if self.remaining > 0.0 {
            let ease = ((self.repeat_delay - self.remaining) / self.repeat_delay)
                .calc(self.force_function);
            scale(self.impulse, ease / self.repeat_delay)
        } else {
            [0.0; 2]
        }
    }
}

#[derive(Debug)]
pub enum FlapError {
    IsAlreadyFlapping,
}

impl fmt::Display for FlapError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.description())
    }
}

impl Error for FlapError {
    fn description(&self) -> &str {
        "Wing is already flapping"
    }
}

const FLAP_LEFT: ActuatorId = ActuatorId(0);
const FLAP_RIGHT: ActuatorId = ActuatorId(1);
const DUMMY: MemoryId = MemoryId(0);
const MEMORY_POSITION: MemoryId = MemoryId(1);
const POSITION: SensorId = SensorId(0);

pub enum Actuator {
    /// Flap left wing.
    FlapLeft(FlapWing),
    /// Flap right wing.
    FlapRight(FlapWing),
}

impl asi::Actuator<Memory> for Actuator {
    fn send(&mut self, _data: &Memory) {
        match *self {
            Actuator::FlapLeft(ref mut wing) |
            Actuator::FlapRight(ref mut wing) => {
                if wing.remaining > 0.0 {return};
                if wing.received {
                    wing.received = false;
                    wing.remaining = wing.repeat_delay;
                }
            }
        }
    }
    fn try_confirm(&mut self) -> Option<Result<(), Box<Error>>> {
        match *self {
            Actuator::FlapLeft(ref mut wing) |
            Actuator::FlapRight(ref mut wing) => {
                return if !wing.received {
                    wing.received = true;
                    Some(Ok(()))
                } else {
                    Some(Err(Box::new(FlapError::IsAlreadyFlapping) as Box<Error>))
                };
            }
        }
    }
}

pub enum Memory {
    /// Used to feed actuators that require no input.
    Dummy,
    /// The position of the bee.
    Position([f64; 2]),
}

impl Default for Memory {
    fn default() -> Self {Memory::Dummy}
}

pub enum SubGoal {
    GoToFlower,
    GoToHive,
    CollectHoney,
}

pub struct FlapState {
    pub wait: f64,
}

impl FlapState {
    pub fn update(&mut self, dt: f64) {
        if self.wait > 0.0 {self.wait -= dt};
    }
}

/// Used to control actuators with unknown and noisy behavior.
/// Based on a universal heuristic of distance to target,
/// which makes the controller dimensional independent.
///
/// This controller is designed for rapid learning and adapting.
/// It learns continuously and rapidly in response to the environment.
/// The idea is that the controller must be adaptable to a wide
/// range of environments where being a fast learner is better
/// than deep analysis.
///
/// The design logic of this controller might seem absurdly recursive.
/// This is because how it learns depends on how it is used.
///
/// The controller predicts how distance to target will change
/// from being told a new control value,
/// relative to the distance if the control value was not changed.
///
/// The controller assumes that its prediction might or not might
/// be followed, such that it tries to predict what the change
/// will actually be, instead of trying to predict conditioned on the change.
/// The predictor takes into account the effect of response to the prediction,
/// which is reflected in the predicted change.
///
/// The user of the controller decides what to do about the prediction,
/// which is natural since you only want to try a new value if it
/// brings the distance to target closer to the goal.
///
/// Since the predictor takes into account what the user will do,
/// the user knows that the decision acts on the best guess
/// that the predictor has instead of an alternative future.
///
/// Not only does the controller predict changes in distance,
/// but it also predicts its own error, which is used to improve learning.
pub struct DistanceHeuristicController {
    /// Whether the controller has started.
    pub started: bool,
    /// Control value.
    pub value: f64,
    /// The old distance.
    pub old_distance: [f64; 2],
    /// The predicted change in distance per unit of time.
    pub prediction: [f64; 2],
    /// Predicted error in predicted change.
    pub error: f64,
    /// A factor describing how much to tune predictions.
    pub tune: f64,
    /// A sub-tree which predicts how the controller would behave
    /// if the control value was changed.
    /// These sub-controllers advice the prediction.
    pub children: Vec<DistanceHeuristicController>,
}

impl DistanceHeuristicController {
    pub fn start(&mut self, value: f64, distance: [f64; 2]) {
        self.value = value;
        self.old_distance = distance;
        self.started = true;
    }

    /// Sets new value.
    pub fn set_value(&mut self, value: f64) {
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
                swap(&mut self.old_distance, &mut child.old_distance);
                swap(&mut self.error, &mut child.error);
                self.value = value;
                return;
            }
        }
        if self.children.len() < 1 {
            // Remember what is learned so far.
            let new_child = DistanceHeuristicController {children: vec![], ..*self};
            self.children.push(new_child);
        }

        self.value = value;
    }

    /// Learns when moving to a new distance.
    /// The control value might have changed in the mean time.
    pub fn learn(&mut self, new_distance: [f64; 2], dt: f64) {
        use vecmath::vec2_add as add;
        use vecmath::vec2_scale as scale;

        let eps = [
            0.0000000000001 + rand::random::<f64>() * self.tune * self.error,
            0.0000000000001 + rand::random::<f64>() * self.tune * self.error
        ];

        let error_middle = self.error(new_distance, dt);
        self.prediction = add(self.prediction, eps);
        let error_up = self.error(new_distance, dt);
        self.prediction = add(self.prediction, scale(eps, -2.0));
        let error_down = self.error(new_distance, dt);
        if error_middle < error_up && error_middle < error_down {
            // Change back.
            self.prediction = add(self.prediction, eps);
        } else if error_up < error_down {
            // Go up.
            self.prediction = add(self.prediction, scale(eps, 2.0));
        }
        self.prediction = [
            self.prediction[0].min(10.0).max(-10.0),
            self.prediction[1].min(10.0).max(-10.0),
        ];

        let eps = 0.0000000000001 + rand::random::<f64>() * self.tune * self.error;

        let error_middle = self.error(new_distance, dt);
        self.error += eps;
        let error_up = self.error(new_distance, dt);
        self.error -= 2.0 * eps;
        let error_down = self.error(new_distance, dt);
        if error_middle < error_up && error_middle < error_down {
            self.error += eps;
        }  else if error_up < error_down {
            self.error += 2.0 * eps;
        }
        self.error = self.error.min(20.0).max(0.0);

        self.old_distance = new_distance;
    }

    pub fn error(&self, new_distance: [f64; 2], dt: f64) -> f64 {
        use vecmath::vec2_sub as sub;
        use vecmath::vec2_scale as scale;
        use vecmath::vec2_len as len;

        // The actual change in distance per unit of time.
        let change = scale(sub(new_distance, self.old_distance), 1.0 / dt);
        let error = len(sub(change, self.predict(self.value, dt))).powi(2);
        error + (self.error - error).abs()
    }

    /// Predicts a change in distance from a new value.
    pub fn predict(&self, new_value: f64, dt: f64) -> [f64; 2] {
        use vecmath::vec2_add as add;
        use vecmath::vec2_scale as scale;
        use vecmath::vec2_len as len;

        // Use weighted average of predictions of itself and children.
        // Compute weight by taking into account how close they are to the new value,
        // how close their last distance is to extrapolated half distance,
        // and how far into the future and the predicted error.
        // Estimate the predicted distance given no change.
        let dist = len(add(self.old_distance, scale(self.prediction, 0.5 * dt)));
        let mut weight = 1.0 / ((self.value - new_value) * (self.value - new_value) + dt + self.error);
        let mut prediction = scale(self.prediction, weight);
        for i in 0..self.children.len() {
            let ref child = self.children[i];
            // Compute the predicted distance of child.
            let child_dist = len(add(child.old_distance, scale(child.prediction, 0.5 * dt)));
            let w = 1.0 /
                    ((child.value - new_value) * (child.value - new_value) +
                     (child_dist - dist) * (child_dist - dist) + dt + child.error);
            prediction = add(prediction, scale(child.prediction, w));
            weight += w;
        }
        scale(prediction, dt / weight)
    }

    /// Set new value out of some candidates.
    pub fn pick<F: Fn(f64) -> bool>(&mut self, dt: f64, values: &[f64], filter: F) {
        use vecmath::vec2_len as len;

        let mut min: Option<(usize, f64)> = None;
        for i in 0..values.len() {
            if filter(values[i]) {
                /*
                if rand::random::<f64>() < 1.0 * dt {
                    self.set_value(values[i]);
                    return;
                }
                */

                let pred = len(self.predict(values[i], dt));
                if min.is_none() || min.unwrap().1 > pred {
                    min = Some((i, pred));
                }
            }
        }
        if let Some((ind, _)) = min {
            self.set_value(values[ind]);
        }
    }
}

/// The Bee's decision maker.
pub struct DecisionMaker {
    pub sub_goal: SubGoal,
    // The flap state of left wing before trying another flap.
    pub left_flap_state: FlapState,
    // The flapt state of right wing before trying another flap.
    pub right_flap_state: FlapState,
    // How long to wait before next flap next time for left wing.
    pub left_repeat: f64,
    // How long to wait before next flap next time for right wing.
    pub right_repeat: f64,
    // Minimum time to repeat for left wing.
    // This is the time it takes for the left wing actuator
    // to prepare itself for another flap.
    pub left_min_repeat: f64,
    // Minimum time to repeat for right wing.
    // This is the time it takes for the right wing actuator
    // to prepare itself for another flap.
    pub right_min_repeat: f64,
    /// Controls left wing.
    pub left_wing_controller: DistanceHeuristicController,
    /// Controls right wing.
    pub right_wing_controller: DistanceHeuristicController,
    /// Current target position.
    pub target: [f64; 2],
    /// Previously measured position.
    pub position: [f64; 2],
    /// Moving avarage distance from target.
    pub avg: [f64; 2],
}

impl asi::DecisionMaker<Memory> for DecisionMaker {
    fn next_action(&mut self, memory: &[Memory], dt: f64) -> asi::Action {
        use asi::Action::*;
        use vecmath::vec2_sub as sub;
        use vecmath::vec2_add as add;
        use vecmath::vec2_scale as scale;

        for memory in memory {
            match *memory {
                Memory::Dummy => {}
                Memory::Position(pos) => {
                    self.position = pos;
                }
            }
        }

        let s = sub(self.target, self.position);
        self.avg = add(self.avg, scale(sub(s, self.avg), 1.0 * dt));
        let avg = self.avg;
        if !self.left_wing_controller.started {
            self.left_wing_controller.start(self.left_repeat, avg);
        }
        if !self.right_wing_controller.started {
            self.right_wing_controller.start(self.right_repeat, avg);
        }
        self.left_wing_controller.learn(avg, dt);
        self.right_wing_controller.learn(avg, dt);

        let mut arr = [0.0; 20];

        let ch = 0.16;
        for i in 0..arr.len() {arr[i] = self.left_repeat + ch * (i as f64 / arr.len() as f64 - 0.5)}
        self.left_wing_controller.pick(dt, &arr, |val| val >= 1.4 && val < 2.0);
        for i in 0..arr.len() {arr[i] = self.right_repeat + ch * (i as f64 / arr.len() as f64 - 0.5)}
        self.right_wing_controller.pick(dt, &arr, |val| val >= 1.4 && val < 2.0);

        self.left_repeat = self.left_wing_controller.value;
        self.right_repeat = self.right_wing_controller.value;
        // println!("TEST left_repeat {} right_repeat {} dist {}", self.left_repeat, self.right_repeat, dist);
        println!("TEST err {} pred {:?}", self.left_wing_controller.error, self.left_wing_controller.prediction);

        self.left_flap_state.update(dt);
        self.right_flap_state.update(dt);
        if self.left_flap_state.wait <= 0.0 {
            self.left_flap_state.wait += self.left_repeat;
            return Output {
                actuator: FLAP_LEFT,
                memory: DUMMY,
            }
        }
        if self.right_flap_state.wait <= 0.0 {
            self.right_flap_state.wait += self.right_repeat;
            return asi::Action::Output {
                actuator: FLAP_RIGHT,
                memory: DUMMY,
            }
        }
        Input {sensor: POSITION, memory: MEMORY_POSITION}
    }
    fn feedback(&mut self, feedback: Result<asi::Feedback, asi::Error>) {
        use asi::Feedback::*;

        match feedback {
            Ok(WaitingPeriodComplete) => {}
            Ok(InputStored {..}) => {}
            Ok(OutputReceived {..}) => {}
            Err(asi::Error::Actuator {actuator: FLAP_LEFT, ..}) => {
                self.left_flap_state.wait += self.left_repeat;
            }
            Err(asi::Error::Actuator {actuator: FLAP_RIGHT, ..}) => {
                self.right_flap_state.wait += self.right_repeat;
            }
            x => {
                println!("TEST {:?}", x);
            }
        }
    }
    fn shut_down(&mut self, dt: f64) -> f64 {
        dt
    }
}

pub mod math {
    /// Update position and velocity of particle.
    pub fn particle(pos: &mut [f64; 2], vel: &mut [f64; 2], acc: [f64; 2], dt: f64) {
        use vecmath::vec2_add as add;
        use vecmath::vec2_scale as scale;

        *vel = add(*vel, scale(acc, 0.5 * dt));
        *pos = add(*pos, scale(*vel, dt));
        *vel = add(*vel, scale(acc, 0.5 * dt));
    }
}


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
