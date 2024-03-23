//! Polygon shape, defined via a collection of vertices, in clock-wise order.

use std::fmt::Display;

use crate::{
    body::Vertices,
    maths::{vector2::ZERO, Vector2},
};

pub type PolygonVertices = (Vertices, usize);

#[derive(Clone, Debug)]
pub struct Polygon {
    pub area: f32,
    pub vertices: PolygonVertices,
}

impl Polygon {
    pub fn new(mut verts: Vec<Vector2>) -> Option<Polygon> {
        let v_count = verts.len();
        let area = Polygon::area(&verts);

        let centroid = centroid(&verts);
        let shift = ZERO - centroid;

        for vert in &mut verts {
            *vert += shift;
        }

        let mut vertices: Vertices = [ZERO; 8];

        vertices[0..v_count].copy_from_slice(&verts[0..v_count]);

        let vertices = (vertices, v_count);

        let poly = Polygon { area, vertices };

        if !poly.is_convex() {
            return None;
        }

        Some(poly)
    }
}

// average point within convex polygon, for better rotation
pub fn centroid(vertices: &Vec<Vector2>) -> Vector2 {
    let inv_nr_of_vertices = 1. / vertices.len() as f32;

    let mut x: f32 = 0.0;
    let mut y: f32 = 0.0;

    for vert in vertices {
        x += vert.x;
        y += vert.y;
    }

    x *= inv_nr_of_vertices;
    y *= inv_nr_of_vertices;

    Vector2::new(x, y)
}

impl Polygon {
    /// uses shoelace method
    /// https://www.youtube.com/watch?v=FSWPX0XB7a0
    pub fn area(vertices: &[Vector2]) -> f32 {
        let nr_of_vertices = vertices.len();
        let mut area = 0.0;

        for i in 0..nr_of_vertices {
            let i_next = (i + 1) % nr_of_vertices;
            area += vertices[i].x * vertices[i_next].y - vertices[i_next].x * vertices[i].y;
        }

        area = f32::abs(area) * 0.5;
        area
    }

    pub fn is_convex(&self) -> bool {
        let (vertices, nr_of_verts) = self.vertices;

        let mut state: i8 = 0;

        for idx in 0..nr_of_verts {
            let a = vertices[idx];
            let b = vertices[(idx + 1) % nr_of_verts];
            let c = vertices[(idx + 2) % nr_of_verts];

            let axis_a = b - a;
            let axis_b = c - b;

            let crossproduct = axis_a.crossed(axis_b);

            match state {
                0 => {
                    if crossproduct < 0.0 {
                        state = -1;
                    }

                    if crossproduct > 0.0 {
                        state = 1;
                    }
                }
                1 => {
                    if crossproduct < 0.0 {
                        return false;
                    }
                }
                -1 => {
                    if crossproduct > 0.0 {
                        return false;
                    }
                }
                _ => (),
            }
        }

        true
    }

    pub fn verts_to_vec(&self) -> Vec<Vector2> {
        let mut vec = Vec::new();
        let (verts, nr_of_verts) = self.vertices;

        for vert in verts.into_iter().take(nr_of_verts) {
            vec.push(vert);
        }

        vec
    }
}

impl Display for Polygon {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Polygon with vertices: {:?}", self.vertices)
    }
}
