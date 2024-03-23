//! Axis-aligned bounding box shape, essentially non-rotatable rectangles.

use crate::{collision::Hitbox, maths::Vector2};
use std::fmt::Display;

pub type AABBVertices = [Vector2; 4];

/// The min and max Vectors describe the lower-left and upper-right corners of the Rectangle in world-coordinates.<br>
#[derive(Clone, Debug)]
pub struct AABB {
    pub min: Vector2,
    pub max: Vector2,

    pub area: f32,
    pub corners: AABBVertices,
}

impl AABB {
    pub fn corners_mut(&mut self) -> &mut AABBVertices {
        &mut self.corners
    }

    pub fn corners(&self) -> &AABBVertices {
        &self.corners
    }
}

impl AABB {
    pub fn new(width: f32, height: f32) -> Self {
        let corners = AABB::generate_corners(width, height);

        let min = Vector2::new(corners[3].x, corners[3].y); // bottom-left
        let max = Vector2::new(corners[1].x, corners[1].y); // top-right
        let area = width * height;

        Self {
            min,
            max,
            area,
            corners,
        }
    }
}

impl AABB {
    pub fn generate_corners(width: f32, height: f32) -> AABBVertices {
        let left = -width / 2.0;
        let right = width / 2.0;
        let top = height / 2.0;
        let bottom = -height / 2.0;

        [
            (left, top).into(),
            (right, top).into(),
            (right, bottom).into(),
            (left, bottom).into(),
        ]
    }

    pub fn get_hitbox(&self) -> Hitbox {
        Hitbox::new(self.min, self.max)
    }
}

impl Display for AABB {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Rectangle with dimensions: {}m/{}m",
            self.max.x * 2.0,
            self.max.y * 2.0
        )
    }
}
