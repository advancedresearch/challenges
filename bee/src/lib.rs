extern crate asi_core0 as asi;
extern crate vecmath;
extern crate interpolation;
extern crate graphics;
extern crate piston;
extern crate rand;

use asi::{ActuatorId, MemoryId, SensorId, Runtime};
use piston::input::*;
use piston::event_loop::*;
use piston::window::AdvancedWindow;
use interpolation::EaseFunction;
use graphics::*;
use std::error::Error;
use std::fmt;

const BEE_TEXTURE: usize = 0;

/// Stores settings for environment.
pub struct EnvironmentSettings {
    /// Starting position for bee.
    pub hive_pos: [f64; 2],
    /// The gravity vector.
    pub gravity: [f64; 2],
    /// How long a flap lasts.
    pub flap_repeat: f64,
    /// The maximum impulse for left wing.
    pub impulse_left: [f64; 2],
    /// The maximum impulse for right wing.
    pub impulse_right: [f64; 2],
    /// The starting repeat interval for both wings.
    pub start_repeat: f64,
    /// The minimum repeat interval for both wings.
    pub min_repeat: f64,
    /// The maximum repeat interval for both wings.
    pub max_repeat: f64,
    /// The amount of different when adjusting flap repeat.
    pub side_step: f64,
    /// The number of options to consider.
    pub options: usize,
    /// Whether to run the event loop in benchmark mode.
    pub bench_mode: bool,
    /// The Updates Per Frame (UPS) for event loop.
    /// Decreasing this might lead to faster training,
    /// but more inaccurate simulation.
    pub ups: u64,
    /// The maximum Frames Per Second (FPS) for event loop.
    /// Reducing FPS might help speed up benchmark mode.
    pub max_fps: u64,
    /// The spring coefficient for moving average.
    pub avg_spring: f64,
}

impl EnvironmentSettings {
    pub fn new() -> EnvironmentSettings {
        EnvironmentSettings {
            hive_pos: [200.0; 2],
            gravity: [0.0, 20.0],
            flap_repeat: 0.25,
            impulse_left: [30.0, -30.0],
            impulse_right: [-30.0, -30.0],
            start_repeat: 1.4,
            min_repeat: 1.4,
            max_repeat: 2.0,
            side_step: 0.15,
            options: 20,
            bench_mode: false,
            ups: 30,
            max_fps: 30,
            avg_spring: 1.0,
        }
    }
}

pub trait Backend {
    type Graphics: Graphics<Texture = Self::Texture>;
    type Texture: ImageSize;

    fn draw<F: FnMut(Context, &mut Self::Graphics)>(
        &mut self,
        render_args: RenderArgs,
        f: F
    );

    fn load_bee_texture(&self) -> Self::Texture;
}

pub fn run<W: AdvancedWindow, WC: WingController, B: Backend>(
    window: &mut W, left: WC, right: WC, mut backend: B, settings: EnvironmentSettings,
) {
    // Run as fast as possible to train the bee.
    let event_settings = EventSettings::new()
        .bench_mode(settings.bench_mode)
        .ups(settings.ups)
        .max_fps(settings.max_fps);
    let mut events = Events::new(event_settings);
    let hive_pos = settings.hive_pos;
    let flower_pos = [200.0, 100.0];
    let bee_texture = backend.load_bee_texture();
    let mut app = App {
        hive: Hive {
            pos: hive_pos
        },
        flower: Flower {
            pos: flower_pos
        },
        bee: Bee {
            pos: hive_pos,
            vel: [0.0; 2],
            state: BeeState::InAir
        },
        textures: vec![
            bee_texture,
        ],
        render_settings: AppRenderSettings {
            draw_bee_pos: true,
            draw_target_pos: true,
        },
        physics_settings: AppPhysicsSettings {
            gravity: settings.gravity,
        },
        runtime: asi::StandardRuntime::new(),
    };
    let flap_repeat = settings.flap_repeat;
    app.runtime.load(asi::Agent {
        actuators: vec![
            Actuator::FlapLeft(FlapWing {
                received: true,
                // When flapping left wing, move right-up.
                impulse: settings.impulse_left,
                force_function: EaseFunction::QuadraticInOut,
                remaining: 0.0,
                repeat_delay: flap_repeat,
            }),
            Actuator::FlapRight(FlapWing {
                received: true,
                // When flapping right wing, move left-up.
                impulse: settings.impulse_right,
                force_function: EaseFunction::QuadraticInOut,
                remaining: 0.0,
                repeat_delay: flap_repeat,
            }),
        ],
        sensors: vec![
            Sensor::Position([0.0; 2]),
        ],
        memory: vec![
            Memory::Dummy,
            Memory::Dummy,
        ],
        decision_maker: DecisionMaker {
            sub_goal: SubGoal::GoToFlower,
            left_flap_state: FlapState {wait: 0.0},
            right_flap_state: FlapState {wait: 0.25},
            left_repeat: settings.start_repeat,
            right_repeat: settings.start_repeat,
            left_min_repeat: settings.min_repeat,
            left_max_repeat: settings.max_repeat,
            right_min_repeat: settings.max_repeat,
            right_max_repeat: settings.max_repeat,
            left_wing_controller: left,
            right_wing_controller: right,
            target: hive_pos,
            position: hive_pos,
            avg: [0.0; 2],
            avg_spring: settings.avg_spring,
            side_step: settings.side_step,
            options: vec![0.0; settings.options],
        },
    });
    app.runtime.start();
    let mut tries = 0;
    let mut time = 0.0;
    let mut record = 0.0;
    while let Some(e) = events.next(window) {
        app.event(&e);
        if let Some(args) = e.render_args() {
            backend.draw(args, |c, g| {
                clear(color::hex("ffffff"), g);
                app.draw(&c, g);
            })
        }

        if let Some(args) = e.update_args() {
            time += args.dt;
        }

        let reset = e.press_args().is_some() ||
                    vecmath::vec2_len(vecmath::vec2_sub(app.bee.pos, hive_pos)) > 200.0;

        if reset {
            app.bee.pos = hive_pos;
            app.bee.vel = [0.0; 2];
            let ref mut dm = app.runtime.agent.as_mut().unwrap().decision_maker;
            dm.left_wing_controller.set_started(false);
            dm.right_wing_controller.set_started(false);
            dm.avg = [0.0; 2];
            tries += 1;
            window.set_title(format!("asi_core0: bee ({}, {:.2})", tries, record));
            if time > record {
                record = time;
            }
            time = 0.0;
        }
    }
}

pub struct AppRenderSettings {
    pub draw_bee_pos: bool,
    pub draw_target_pos: bool,
}

pub struct AppPhysicsSettings {
    pub gravity: [f64; 2],
}

pub struct App<Texture, W> where W: WingController {
    pub hive: Hive,
    pub flower: Flower,
    pub bee: Bee,
    pub textures: Vec<Texture>,
    pub render_settings: AppRenderSettings,
    pub physics_settings: AppPhysicsSettings,
    pub runtime: asi::StandardRuntime<Sensor, Actuator, Memory, DecisionMaker<W>>,
}

impl<Texture: ImageSize, W: WingController> App<Texture, W> {
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

pub trait WingController {
    /// Returns `true` if the wing controller has started.
    fn get_started(&self) -> bool;
    /// Sets whether the wing controller has started.
    fn set_started(&mut self, value: bool);
    /// Get the value of the wing controller.
    fn get_value(&self) -> f64;
    /// Set value of the wing controller.
    fn set_value(&mut self, value: f64);
    /// Get predicted vector.
    fn get_prediction(&self) -> [f64; 2];
    /// Set predicted vector.
    fn set_prediction(&mut self, [f64; 2]);
    /// Get old vector from previous learning.
    fn get_old_vector(&self) -> [f64; 2];
    /// Sets old vector from previous learning.
    fn set_old_vector(&mut self, value: [f64; 2]);
    /// Get expected error.
    fn get_error(&self) -> f64;
    /// Set expected error.
    fn set_error(&mut self, value: f64);
    /// Get the tuning spring parameter.
    fn get_tune(&self) -> f64;
    /// Predict future vector given that `new_value` is used.
    fn predict(&self, new_value: f64, dt: f64) -> [f64; 2];

    /// Called at beginning or when resetting position.
    fn start(&mut self, value: f64, vector: [f64; 2]) {
        self.set_value(value);
        self.set_old_vector(vector);
        self.set_started(true);
    }

    /// Returns error from current prediction.
    ///
    /// The error includes a term for predicted error.
    fn error(&self, new_vector: [f64; 2], dt: f64) -> f64 {
        use vecmath::vec2_sub as sub;
        use vecmath::vec2_scale as scale;
        use vecmath::vec2_len as len;

        // The actual change in distance per unit of time.
        let change = scale(sub(new_vector, self.get_old_vector()), 1.0 / dt);
        let error = len(sub(change, self.predict(self.get_value(), dt))).powi(2);
        error + (self.get_error() - error).abs()
    }

    /// Learns when moving to a new distance.
    /// The control value might have changed in the mean time.
    fn learn(&mut self, new_vector: [f64; 2], dt: f64) {
        use vecmath::vec2_add as add;
        use vecmath::vec2_scale as scale;

        let eps = [
            0.0000000000001 + rand::random::<f64>() * self.get_tune() * self.get_error(),
            0.0000000000001 + rand::random::<f64>() * self.get_tune() * self.get_error()
        ];

        let error_middle = self.error(new_vector, dt);
        let pred = self.get_prediction();
        self.set_prediction(add(pred, eps));
        let error_up = self.error(new_vector, dt);
        let pred = self.get_prediction();
        self.set_prediction(add(pred, scale(eps, -2.0)));
        let error_down = self.error(new_vector, dt);
        if error_middle < error_up && error_middle < error_down {
            // Change back.
            let pred = self.get_prediction();
            self.set_prediction(add(pred, eps));
        } else if error_up < error_down {
            // Go up.
            let pred = self.get_prediction();
            self.set_prediction(add(pred, scale(eps, 2.0)));
        }

        // Clamp prediction.
        let pred = self.get_prediction();
        self.set_prediction([
            pred[0].min(10.0).max(-10.0),
            pred[1].min(10.0).max(-10.0),
        ]);

        let eps = 0.0000000000001 + rand::random::<f64>() * self.get_tune() * self.get_error();

        let error_middle = self.error(new_vector, dt);
        let err = self.get_error();
        self.set_error(err + eps);
        let error_up = self.error(new_vector, dt);
        let err = self.get_error();
        self.set_error(err - 2.0 * eps);
        let error_down = self.error(new_vector, dt);
        if error_middle < error_up && error_middle < error_down {
            let err = self.get_error();
            self.set_error(err + eps);
        }  else if error_up < error_down {
            let err = self.get_error();
            self.set_error(err + 2.0 * eps);
        }

        // Clamp the error.
        let err = self.get_error();
        self.set_error(err.min(20.0).max(0.0));

        self.set_old_vector(new_vector);
    }

    /// Set new value out of some candidates.
    fn pick<F: Fn(f64) -> bool>(&mut self, dt: f64, values: &[f64], filter: F) {
        use vecmath::vec2_len as len;

        let mut min: Option<(usize, f64)> = None;
        for i in 0..values.len() {
            if filter(values[i]) {
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
pub struct DecisionMaker<W> where W: WingController {
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
    /// Maximum time to repeat for left wing.
    pub left_max_repeat: f64,
    // Minimum time to repeat for right wing.
    // This is the time it takes for the right wing actuator
    // to prepare itself for another flap.
    pub right_min_repeat: f64,
    /// Maximum time to repeat for right wing.
    pub right_max_repeat: f64,
    /// Controls left wing.
    pub left_wing_controller: W,
    /// Controls right wing.
    pub right_wing_controller: W,
    /// Current target position.
    pub target: [f64; 2],
    /// Previously measured position.
    pub position: [f64; 2],
    /// Moving avarage distance from target.
    pub avg: [f64; 2],
    /// Spring coefficient to moving average.
    pub avg_spring: f64,
    /// The amount of change in repeat time.
    pub side_step: f64,
    /// Stores options to consider when predicting possible futures.
    pub options: Vec<f64>,
}

impl<W: WingController> asi::DecisionMaker<Memory> for DecisionMaker<W> {
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
        self.avg = add(self.avg, scale(sub(s, self.avg), self.avg_spring * dt));
        let avg = self.avg;
        if !self.left_wing_controller.get_started() {
            self.left_wing_controller.start(self.left_repeat, avg);
        }
        if !self.right_wing_controller.get_started() {
            self.right_wing_controller.start(self.right_repeat, avg);
        }
        self.left_wing_controller.learn(avg, dt);
        self.right_wing_controller.learn(avg, dt);

        let ref mut arr = self.options;

        let min = self.left_min_repeat;
        let max = self.left_max_repeat;
        for i in 0..arr.len() {arr[i] = self.left_repeat + self.side_step * (i as f64 / arr.len() as f64 - 0.5)}
        self.left_wing_controller.pick(dt, &arr, |val| val >= min && val < max);
        let min = self.right_min_repeat;
        let max = self.right_max_repeat;
        for i in 0..arr.len() {arr[i] = self.right_repeat + self.side_step * (i as f64 / arr.len() as f64 - 0.5)}
        self.right_wing_controller.pick(dt, &arr, |val| val >= min && val < max);

        self.left_repeat = self.left_wing_controller.get_value();
        self.right_repeat = self.right_wing_controller.get_value();
        println!("TEST err {} pred {:?}",
                 self.left_wing_controller.get_error(),
                 self.left_wing_controller.get_prediction());

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
