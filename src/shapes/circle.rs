//! Circle shape.

use crate::{math::Vector2, prelude::Hitbox};
use std::{f32::consts::PI, fmt::Display};

pub type CircleVertices = Vector2;

#[derive(Clone, Debug)]
pub struct Circle {
    pub r: f32,
    pub visual_point: CircleVertices,
    pub area: f32,
}

impl Circle {
    pub fn new(r: f32) -> Self {
        let visual_point = Vector2::new(0.0, r);

        let area: f32 = r * r * PI;

        Self {
            r,
            visual_point,
            area,
        }
    }

    pub fn get_hitbox(&self) -> Hitbox {
        Hitbox::new(Vector2::new(-self.r, -self.r), Vector2::new(self.r, self.r))
    }
}

impl Display for Circle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Circle with radius: {}m", self.r)
    }
}
