//! Available shapes:
//! - Circle
//! - Axis-aligned bounding boxes (AABB)
//! - Oriented bounding boxes (OBB) in the form of polygons
//! - Convex polygons

use std::fmt::Display;

pub mod aabb;
pub mod circle;
pub mod polygon;

pub use aabb::AABB;
pub use circle::Circle;
pub use polygon::Polygon;

use crate::{
    body::{Vertices, MAX_VERTICE_COUNT},
    maths::vector2::ZERO,
};

/// OBBs are defined as Polygons in this engine.
#[derive(Clone, Debug)]
pub enum Shape {
    Circle(Circle),
    AABB(AABB),
    Polygon(Polygon),
}

// translators
impl Shape {
    pub fn as_circle(&self) -> Circle {
        match self {
            Shape::Circle(c) => Circle::new(c.r),
            _ => {
                panic!("Shape is not a circle!");
            }
        }
    }

    pub fn is_circle(&self) -> bool {
        matches!(self, Shape::Circle(_))
    }

    pub fn as_aabb(&self) -> AABB {
        match self {
            Shape::AABB(r) => AABB::new(r.max.x * 2.0, r.max.y * 2.0),
            _ => {
                panic!("Shape is not an AABB!");
            }
        }
    }

    pub fn is_aabb(&self) -> bool {
        matches!(self, Shape::AABB(_))
    }

    pub fn as_polygon(&self) -> Polygon {
        match self {
            Shape::Polygon(p) => Polygon::new(p.verts_to_vec()).unwrap(),
            _ => {
                panic!("Shape is not a polygon!");
            }
        }
    }

    pub fn is_polygon(&self) -> bool {
        matches!(self, Shape::Polygon(_))
    }
}

impl Shape {
    pub fn area(&self) -> f32 {
        use Shape::*;
        match self {
            Circle(c) => c.area,
            AABB(r) => r.area,
            Polygon(p) => p.area,
        }
    }

    pub fn get_vertices(&self) -> (Vertices, usize) {
        use Shape::*;

        let mut vertices: Vertices = [ZERO; MAX_VERTICE_COUNT];
        let count;

        match self {
            Circle(c) => {
                vertices[0] = c.visual_point;
                count = 1;
            }
            AABB(r) => {
                vertices[0..4].copy_from_slice(&r.corners()[0..4]);
                count = 4;
            }
            Polygon(p) => {
                let (verts, nr_of_verts) = p.vertices;
                vertices[0..nr_of_verts].copy_from_slice(&verts[0..nr_of_verts]);
                count = nr_of_verts;
            }
        }

        (vertices, count)
    }
}

impl Display for Shape {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use Shape::*;
        match self {
            Circle(c) => write!(f, "{}", c),
            AABB(r) => write!(f, "{}", r),
            Polygon(p) => write!(f, "{}", p),
        }
    }
}

impl Default for Shape {
    fn default() -> Self {
        Shape::Circle(Circle::new(1.0))
    }
}
