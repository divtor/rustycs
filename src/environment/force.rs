//! Forces that affect the physics world.<br>
//! Defined as constant accelerations in the form of a Vector2.

use std::fmt::Display;

use crate::math::Vector2;

#[derive(Clone)]
pub struct Force {
    pub acceleration: Vector2,
}

// constructors
impl Force {
    pub const fn new(x: f32, y: f32) -> Force {
        Force {
            acceleration: Vector2 { x, y },
        }
    }
}

pub const GRAVITY_EARTH: Force = Force::new(0.0, -9.81);
pub const GRAVITY_MARS: Force = Force::new(0.0, -3.71);
pub const GRAVITY_MOON: Force = Force::new(0.0, -1.62);
pub const RIGHTSIDE_WIND: Force = Force::new(2.0, 0.0);

impl Display for Force {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Force: {}", self.acceleration)
    }
}
