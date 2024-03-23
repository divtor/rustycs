//! Entity that emits it's own gravity.<br>
//! Pulls other bodies in the scene towards it according to their mass, relative to the attractor's mass.

use crate::{
    body::Body,
    maths::{vector2::ZERO, Vector2},
};

#[derive(PartialEq, Clone, Debug)]
pub enum AttractorType {
    Global,
    Local,
}

impl Default for AttractorType {
    fn default() -> Self {
        Self::Global
    }
}

#[derive(Default)]
/// This attractor produces a pulling force towards it, that gets stronger the bodies are to it.<br>
/// It also has a maximum range that can be defined (as in bodies beyond the range do not get affected at all).
pub struct Attractor {
    pub location: Vector2,
    pub mass: f32,
    pub r: f32,
    pub d_min: f32,
    pub d_max: f32,
    pub name: Option<&'static str>,
    pub a_type: AttractorType,
}

const DEFAULT_MASS: f32 = 1000.;
const DEFAULT_RANGE: (f32, f32) = (10., 20.);

impl Attractor {
    pub fn new(
        x: f32,
        y: f32,
        r: f32,
        a_type: AttractorType,
        name: Option<&'static str>,
    ) -> Attractor {
        Attractor {
            location: Vector2::new(x, y),
            mass: DEFAULT_MASS,
            r,
            d_min: DEFAULT_RANGE.0,
            d_max: DEFAULT_RANGE.1,
            a_type,
            name,
        }
    }

    pub fn mass(mut self, mass: f32) -> Self {
        self.mass = mass;
        self
    }

    pub fn clamp_distance(mut self, d_min: f32, d_max: f32) -> Self {
        self.d_min = d_min;
        self.d_max = d_max;

        self
    }
}

const G: f32 = 1.0;

impl Attractor {
    pub fn get_attraction(&self, body: &Body) -> Vector2 {
        let dir = self.location - body.transform.location;
        let d2 = dir.len_squared();

        if self.a_type == AttractorType::Local && d2 > self.r * self.r {
            return ZERO;
        }

        let n = dir.normalize_or_zero();
        let strength = (G * self.mass * body.mass) / d2.clamp(self.d_min, self.d_max);

        n * strength.abs()
    }
}
