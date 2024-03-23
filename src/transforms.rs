//! Contains all transform relevant information of a specific body

use crate::maths::{vector2::ZERO, Vector2};
use std::fmt::Display;

#[derive(Clone, Debug, Default)]
pub struct Transform {
    pub location: Vector2,
    pub velocity: Vector2,
    pub angular_velocity: f32,
}

impl Transform {
    pub fn new(x: f32, y: f32) -> Self {
        Transform {
            location: Vector2::new(x, y),
            velocity: ZERO,
            angular_velocity: 0.0,
        }
    }
}

impl Display for Transform {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Location: {}\nVelocity: {}\nAngle Velocity: {}\n",
            self.location, self.velocity, self.angular_velocity,
        )
    }
}
