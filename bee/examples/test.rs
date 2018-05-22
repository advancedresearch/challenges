extern crate asi_core0 as asi;
extern crate piston;
extern crate sdl2_window;
extern crate opengl_graphics;
extern crate graphics;
extern crate vecmath;
extern crate interpolation;
extern crate rand;
extern crate bee_challenge;

use asi::Runtime;
use opengl_graphics::*;
use sdl2_window::Sdl2Window;
use piston::window::*;
use piston::event_loop::*;
use piston::input::*;
use graphics::*;
use interpolation::EaseFunction;
use bee_challenge::{
    Actuator,
    App,
    AppPhysicsSettings,
    AppRenderSettings,
    Bee,
    BeeState,
    DecisionMaker,
    DistanceHeuristicController,
    FlapState,
    FlapWing,
    Flower,
    Hive,
    Memory,
    Sensor,
    SubGoal,
};

fn main() {
    let opengl = OpenGL::V3_2;
    let settings = WindowSettings::new("asi_core0: bee", [640, 480])
        .exit_on_esc(true)
        .opengl(opengl);
    let mut window: Sdl2Window = settings.build().unwrap();
    // Run as fast as possible to train the bee.
    let event_settings = EventSettings::new().bench_mode(true).ups(10).max_fps(2);
    let mut events = Events::new(event_settings);
    let mut gl = GlGraphics::new(opengl);
    let hive_pos = [200.0, 200.0];
    let flower_pos = [200.0, 100.0];
    let bee_texture: Texture = Texture::from_path(
        "assets/bee.png",
        &TextureSettings::new()
    ).unwrap();
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
            gravity: [0.0, 20.0],
        },
        runtime: asi::StandardRuntime::new(),
    };
    let flap_repeat = 0.25;
    let tune = 0.1;
    app.runtime.load(asi::Agent {
        actuators: vec![
            Actuator::FlapLeft(FlapWing {
                received: true,
                // When flapping left wing, move right-up.
                impulse: [30.0, -30.0],
                force_function: EaseFunction::QuadraticInOut,
                remaining: 0.0,
                repeat_delay: flap_repeat,
            }),
            Actuator::FlapRight(FlapWing {
                received: true,
                // When flapping right wing, move left-up.
                impulse: [-30.0, -30.0],
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
            left_repeat: 1.4,
            right_repeat: 1.4,
            left_min_repeat: flap_repeat,
            right_min_repeat: flap_repeat,
            left_wing_controller: DistanceHeuristicController {
                started: false,
                value: 0.0,
                old_distance: [0.0; 2],
                prediction: [1.0; 2],
                error: 20.0,
                tune,
                children: vec![],
            },
            right_wing_controller: DistanceHeuristicController {
                started: false,
                value: 0.0,
                old_distance: [0.0; 2],
                prediction: [1.0; 2],
                error: 20.0,
                tune,
                children: vec![],
            },
            target: hive_pos,
            position: hive_pos,
            avg: [0.0; 2],
        },
    });
    app.runtime.start();
    let mut tries = 0;
    let mut time = 0.0;
    let mut record = 0.0;
    while let Some(e) = events.next(&mut window) {
        app.event(&e);
        if let Some(args) = e.render_args() {
            gl.draw(args.viewport(), |c, g| {
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
            dm.left_wing_controller.started = false;
            dm.right_wing_controller.started = false;
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
