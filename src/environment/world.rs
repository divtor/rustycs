//! This is the main entrypoint for any program using this library.<br>
//! Simply define a world, fill it with entities, and call world.update() to advance a discrete physics step.<br>
//! <br>Simulated physics world that contains:
//! - bodies
//! - attractors
//! - forces<br>
//! <br>

use std::{collections::HashMap, fmt::Display, mem, time::Instant};

use crate::{
    collision::{
        correct_position, detect_collision, detection::hitboxes_collide, resolve_collision,
        Manifold,
    },
    entities::{
        attractor::Attractor,
        body::{Body, BodyType::*},
    },
    environment::force::Force,
    math::{vector2::ZERO, Vector2},
};

#[derive(Default)]
pub struct World {
    bodies: Vec<Body>,
    forces: Vector2,
    attractors: Vec<Attractor>,
    manifolds: HashMap<(usize, usize), Manifold>,
    possible_collisions: Vec<(usize, usize)>,
    pub collision_points: Vec<Vector2>,
    tick_rate: f32,
    delta_time: f32,
    collision_precision: usize,
    pixel_to_meter: f32,
    inv_pixel_to_meter: f32,
    last_step_duration: f32,
}

impl World {
    pub fn new(tick_rate: f32, pixel_to_meter: f32) -> World {
        World {
            tick_rate,
            delta_time: 1.0 / tick_rate,
            pixel_to_meter,
            inv_pixel_to_meter: 1.0 / pixel_to_meter,
            collision_precision: 1,
            ..Default::default()
        }
    }
}

impl World {
    pub fn add_body(&mut self, body: Body) {
        self.bodies.push(body);
    }

    pub fn remove_body(&mut self, body_idx: usize) -> Option<Body> {
        if body_idx >= self.bodies.len() {
            return None;
        }

        Some(self.bodies.remove(body_idx))
    }

    pub fn add_force(&mut self, force: Force) {
        self.forces += force.acceleration;
    }

    pub fn remove_force(&mut self, force: Force) {
        self.forces -= force.acceleration;
    }

    pub fn add_attractor(&mut self, attractor: Attractor) {
        self.attractors.push(attractor);
    }

    pub fn remove_attractor(&mut self, attractor_idx: usize) -> Option<Attractor> {
        if attractor_idx >= self.attractors.len() {
            return None;
        }

        Some(self.attractors.remove(attractor_idx))
    }

    pub fn add_bodies(&mut self, bodies: Vec<Body>) {
        for body in bodies {
            self.add_body(body);
        }
    }

    pub fn add_forces(&mut self, forces: Vec<Force>) {
        for force in forces {
            self.add_force(force);
        }
    }

    pub fn add_attractors(&mut self, attractors: Vec<Attractor>) {
        for a in attractors {
            self.add_attractor(a);
        }
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.attractors.clear();
        self.forces = ZERO;
        self.manifolds.clear();
        self.possible_collisions.clear();
        self.collision_points.clear();
    }
}

impl World {
    // GETTERS
    pub fn get_bodies(&self) -> &Vec<Body> {
        &self.bodies
    }

    pub fn get_attractors(&self) -> &Vec<Attractor> {
        &self.attractors
    }

    pub fn get_delta_time(&self) -> f32 {
        self.delta_time
    }

    pub fn get_last_update_duration(&self) -> f32 {
        self.last_step_duration
    }

    pub fn get_collision_precision(&self) -> usize {
        self.collision_precision
    }

    // SETTERS
    pub fn set_tick_rate(&mut self, tick_rate: f32) {
        self.tick_rate = tick_rate;
        self.delta_time = 1.0 / tick_rate;
    }

    /// Defines how many iterations the collision pipeline should execute.<br>
    /// This is a tradeoff of accuracy and computiation complexity.<br><br>
    /// The accuracy range is 10-100 and gets clamped upon setting it.<br>
    /// Higher accuracy means more expensive resolution.
    pub fn set_collision_precision(&mut self, precision: usize) {
        self.collision_precision = precision.clamp(10, 100);
    }
}

// World <-> Screen projections
impl World {
    pub fn get_ptm_ratio(&self) -> f32 {
        self.pixel_to_meter
    }

    pub fn set_ptm_ratio(&mut self, pixel_to_meter: f32) {
        self.pixel_to_meter = pixel_to_meter;
        self.inv_pixel_to_meter = 1. / pixel_to_meter;
    }

    pub fn change_ptm_ratio(&mut self, change: f32) {
        self.pixel_to_meter *= change;
        self.pixel_to_meter = self.pixel_to_meter.clamp(1.0, 1000.0);
        self.inv_pixel_to_meter = 1. / self.pixel_to_meter;
    }

    pub fn screen_to_world(&self, x: f32, y: f32, w: f32, h: f32) -> Vector2 {
        let x = (x - w * 0.5) * self.inv_pixel_to_meter;
        let y = -(y - h * 0.5) * self.inv_pixel_to_meter;
        Vector2::new(x, y)
    }

    pub fn world_to_screen(&self, coordinate: Vector2, w: f32, h: f32) -> (f32, f32) {
        let x = (coordinate.x * self.pixel_to_meter) + w * 0.5;
        let y = (-coordinate.y * self.pixel_to_meter) + h * 0.5;
        (x, y)
    }
}

// physics update (step)
impl World {
    pub fn update(&mut self) {
        let update_start = Instant::now();

        self.broad_phase();
        self.narrow_phase();

        for body in self.bodies.iter_mut().filter(|b| b.body_type == Dynamic) {
            let mut f = self.forces;

            self.attractors.iter().for_each(|a| {
                f += a.get_attraction(body);
            });

            body.transform.velocity += f * self.delta_time;
        }

        if !self.manifolds.is_empty() {
            self.setup_resolutions();

            for _ in 0..self.collision_precision {
                self.resolve_collisions();
            }

            self.correct_positions();
            self.manifolds.clear();
        }

        for body in self.bodies.iter_mut().filter(|b| b.body_type == Dynamic) {
            body.transform.location += body.transform.velocity * self.delta_time;
            body.rotate(self.delta_time);
        }

        self.last_step_duration = update_start.elapsed().as_secs_f32() * 1000.;
    }
}

// collisions
impl World {
    // More efficient way of doing this would be a QuadTree algortithm
    fn broad_phase(&mut self) {
        self.collision_points.clear();

        let body_count = self.bodies.len();

        for a_idx in 0..body_count {
            let a = &self.bodies[a_idx];

            for b_idx in (a_idx + 1)..body_count {
                let b = &self.bodies[b_idx];

                if a.body_type == Static && b.body_type == Static {
                    continue;
                }

                if hitboxes_collide(a, b) {
                    self.possible_collisions.push((a_idx, b_idx));
                }
            }
        }
    }

    fn narrow_phase(&mut self) {
        let pc = mem::take(&mut self.possible_collisions);

        for coll in pc {
            let (a_idx, b_idx) = coll;
            let (a, b) = (&self.bodies[a_idx], &self.bodies[b_idx]);

            if let Some(manifold) = detect_collision(a, a_idx, b, b_idx) {
                for idx in 0..manifold.contact_count {
                    self.collision_points.push(manifold.contacts[idx].location);
                }
                self.manifolds.insert((a_idx, b_idx), manifold);
            }
        }
    }

    fn setup_resolutions(&mut self) {
        for m in self.manifolds.values_mut() {
            let a = &self.bodies[m.a_idx];
            let b = &self.bodies[m.b_idx];
            m.setup(a, b, self.forces * self.delta_time);
        }
    }

    fn resolve_collisions(&mut self) {
        for m in self.manifolds.values_mut() {
            resolve_collision(m, &mut self.bodies);
        }
    }

    fn correct_positions(&mut self) {
        for m in self.manifolds.values_mut() {
            correct_position(m, &mut self.bodies);
        }
    }
}

impl Display for World {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PTM Ratio: {}\nTickrate: {}",
            self.pixel_to_meter, self.tick_rate
        )
    }
}
