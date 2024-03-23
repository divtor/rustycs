//! Bodies that exist within a simulated world.<br>
//! They are defined via their:
//! - name (if given any)
//! - shape
//! - body type (Static or Dynamic)
//! - material
//! - transform information (rotation, position, ...)
//! - mass
//! - inertia

use std::fmt::Display;

use crate::{
    collision::Hitbox,
    material::Material,
    maths::{vector2::ZERO, Vector2},
    shapes::Shape,
    shapes::*,
    transforms::Transform,
};

#[derive(PartialEq, Clone, Debug)]
pub enum BodyType {
    Static,
    Dynamic,
}

impl Display for BodyType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BodyType::Static => write!(f, "Static"),
            BodyType::Dynamic => write!(f, "Dynamic"),
        }
    }
}

use BodyType::*;

const RESOLUTION_VELOCITY_CONSTRAINT: f32 = 0.0008831703;

/// DO NOT CHOOSE VALUE BELOW 4
pub const MAX_VERTICE_COUNT: usize = 8;

pub type Vertices = [Vector2; MAX_VERTICE_COUNT];

#[derive(Clone, Debug)]
pub struct Body {
    pub name: Option<&'static str>,

    pub shape: Shape,
    pub transform: Transform,
    pub material: Material,
    pub hitbox: Hitbox,
    pub vertices: Vertices,
    pub vertice_count: usize,
    pub body_type: BodyType,
    pub mass: f32,
    pub inverse_mass: f32,
    pub inertia: f32,
    pub inverse_inertia: f32,
}

// --------------------------------- GENERIC CONSTRUCTOR ---------------------------------
impl Body {
    pub fn new(
        x: f32,
        y: f32,
        shape: Shape,
        body_type: BodyType,
        material: Material,
        name: Option<&'static str>,
    ) -> Body {
        let mut mass: f32 = 0.0;
        let mut inverse_mass: f32 = 0.0;
        let mut inertia: f32 = 0.0;
        let mut inverse_inertia: f32 = 0.0;

        if body_type == Dynamic {
            (mass, inverse_mass) = calc_mass(shape.area(), material.density);
            (inertia, inverse_inertia) = calc_inertia(&shape, mass, material.density);
        }

        let (vertices, vertice_count) = shape.get_vertices();

        let mut b = Body {
            vertices,
            vertice_count,
            name,
            shape,
            transform: Transform::new(x, y),
            material,
            body_type,
            mass,
            inverse_mass,
            inertia,
            inverse_inertia,
            ..Default::default()
        };

        b.generate_hitbox();

        b
    }
}

fn calc_mass(area: f32, density: f32) -> (f32, f32) {
    let mass = area * density;

    (mass, 1.0 / mass)
}

fn calc_inertia(shape: &Shape, mass: f32, density: f32) -> (f32, f32) {
    match &shape {
        Shape::Circle(c) => {
            let inertia = 0.5 * mass * c.r.powf(2.0);
            (inertia, 1. / inertia)
        }
        Shape::Polygon(p) => {
            let mut inertia = 0.0;

            let (verts, nr_of_verts) = p.vertices;

            for idx in 0..nr_of_verts {
                let a = verts[idx];
                let b = verts[(idx + 1) % nr_of_verts];

                let mass_tri = density * 0.5 * a.crossed(b).abs();
                inertia += mass_tri * (a.len_squared() + b.len_squared() + a.dotted(b)) / 6.0;
            }

            (inertia, 1. / inertia)
        }
        _ => (0.0, 0.0),
    }
}

// --------------------------------- PHYSICS UPDATE ---------------------------------
impl Body {
    pub fn rotate(&mut self, dt: f32) {
        let angle = self.transform.angular_velocity;

        if !self.shape.is_aabb() && angle != 0.0 {
            let (vertices, len) = self.get_vertices();
            for (idx, v) in vertices.into_iter().enumerate().take(len) {
                self.vertices[idx] = Vector2::rotated(v, angle * dt);
            }
        }

        self.update_hitbox();
    }

    fn update_hitbox(&mut self) {
        if self.shape.is_circle() || self.shape.is_aabb() {
            return;
        }

        self.generate_hitbox();
    }

    fn generate_hitbox(&mut self) {
        if self.shape.is_circle() {
            self.hitbox = self.shape.as_circle().get_hitbox();
            return;
        }

        if self.shape.is_aabb() {
            self.hitbox = self.shape.as_aabb().get_hitbox();
            return;
        }

        let mut min = Vector2::new(f32::MAX, f32::MAX);
        let mut max = Vector2::new(f32::MIN, f32::MIN);

        let (vertices, len) = self.get_vertices();

        for vertice in vertices.into_iter().take(len) {
            if vertice.x < min.x {
                min.x = vertice.x;
            }

            if vertice.x > max.x {
                max.x = vertice.x
            }

            if vertice.y < min.y {
                min.y = vertice.y;
            }

            if vertice.y > max.y {
                max.y = vertice.y
            }
        }

        self.hitbox = Hitbox::new(min, max);
    }

    pub fn get_vertices(&self) -> (Vertices, usize) {
        let mut vertices: Vertices = [ZERO; MAX_VERTICE_COUNT];
        let c = self.vertice_count;

        vertices[0..c].copy_from_slice(&self.vertices[0..c]);

        (vertices, c)
    }

    pub fn get_moved_vertices(&self) -> (Vertices, usize) {
        let location = self.transform.location;
        let (vertices, nr_of_verts) = self.get_vertices();

        (vertices.map(|v| v + location), nr_of_verts)
    }

    pub fn get_vertices_as_vec(&self) -> Vec<Vector2> {
        let mut vertices = Vec::with_capacity(self.vertice_count);

        for idx in 0..self.vertice_count {
            vertices.push(self.vertices[idx]);
        }

        vertices
    }
}

// --------------------------------- METHODS ---------------------------------
impl Body {
    pub fn apply_impulse(&mut self, impulse: Vector2) {
        if impulse.abs().len_squared() > RESOLUTION_VELOCITY_CONSTRAINT {
            self.transform.velocity += impulse;
        }
    }

    pub fn set_mass(&mut self, mass: f32) {
        self.mass = mass;
        self.inverse_mass = 1.0 / mass;
        (self.inertia, self.inverse_inertia) =
            calc_inertia(&self.shape, mass, self.material.density);
    }

    pub fn encloses(&self, p: Vector2) -> bool {
        use Shape::*;
        match &self.shape {
            Circle(c) => (self.transform.location - p).len_squared() < c.r * c.r,
            AABB(_) => {
                let hb = &self.hitbox + self.transform.location;
                if hb.min.x >= p.x || hb.max.x <= p.x {
                    return false;
                }

                if hb.min.y >= p.y || hb.max.y <= p.y {
                    return false;
                }

                true
            }
            Polygon(_) => {
                let mut count: usize = 0;
                let (verts, nr_of_verts) = self.get_moved_vertices();

                for idx in 0..nr_of_verts {
                    let a: &Vector2 = &verts[idx];
                    let b: &Vector2 = &verts[(idx + 1) % nr_of_verts];

                    let y_in_between: bool = (p.y < a.y) != (p.y < b.y);
                    let x_on_left_side: bool =
                        p.x < (a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x));

                    if y_in_between && x_on_left_side {
                        count += 1;
                    }
                }

                count % 2 == 1
            }
        }
    }

    pub fn rotate_fixed_angle(&mut self, angle: f32) {
        if !self.shape.is_aabb() && angle != 0.0 {
            let (vertices, len) = self.get_vertices();
            for (idx, v) in vertices.iter().enumerate().take(len) {
                self.vertices[idx] = Vector2::rotated(*v, angle);
            }
        }
        self.update_hitbox();
    }
}

// --------------------------------- EXPERIMENTAL ---------------------------------
impl Body {
    pub fn halt(&mut self) {
        if self.body_type == Static {
            return;
        }

        self.transform.velocity.x = 0.0;
        self.transform.velocity.y = 0.0;
        self.transform.angular_velocity = 0.0;
    }

    pub fn clamp_small_movement(&mut self) {
        let abs_v = self.transform.velocity.abs();

        if abs_v.x <= 0.003 && abs_v.y <= 0.003 {
            self.transform.velocity = ZERO;
        }

        if self.transform.angular_velocity.abs() < 0.0004 {
            self.transform.angular_velocity = 0.;
        }
    }
}

// --------------------------------- DYNAMIC CONSTRUCTORS ---------------------------------
impl Body {
    pub fn circle(x: f32, y: f32, r: f32, material: Material) -> Body {
        Body::new(x, y, Shape::Circle(Circle::new(r)), Dynamic, material, None)
    }

    pub fn aabb(x: f32, y: f32, width: f32, height: f32, material: Material) -> Body {
        Body::new(
            x,
            y,
            Shape::AABB(AABB::new(width, height)),
            Dynamic,
            material,
            None,
        )
    }

    pub fn obb(x: f32, y: f32, width: f32, height: f32, material: Material) -> Body {
        let corners = AABB::generate_corners(width, height);

        Body::new(
            x,
            y,
            Shape::Polygon(
                Polygon::new(corners.to_vec())
                    .expect("shape is a rectangle, should not be detected as concave!"),
            ),
            Dynamic,
            material,
            None,
        )
    }

    pub fn polygon(x: f32, y: f32, vertices: Vec<Vector2>, material: Material) -> Option<Body> {
        if let Some(poly) = Polygon::new(vertices.clone()) {
            let b = Body::new(x, y, Shape::Polygon(poly), Dynamic, material, None);

            Some(b)
        } else {
            println!(
                "User tried to add concave polygon with vertices: {:?}",
                vertices
            );
            None
        }
    }
}

// --------------------------------- STATIC CONSTRUCTORS ---------------------------------
impl Body {
    pub fn platform_rectangle_obb(
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        rotation: f32,
        material: Material,
    ) -> Body {
        let corners = AABB::generate_corners(width, height);

        let mut pr = Body::new(
            x,
            y,
            Shape::Polygon(
                Polygon::new(corners.to_vec())
                    .expect("shape is a rectangle, should not be detected as concave!"),
            ),
            Static,
            material,
            None,
        );

        for v in &mut pr.vertices {
            *v = Vector2::rotated(*v, rotation);
        }

        pr.generate_hitbox();

        pr
    }

    pub fn platform_rectangle_aabb(
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        material: Material,
    ) -> Body {
        Body::new(
            x,
            y,
            Shape::AABB(AABB::new(width, height)),
            Static,
            material,
            None,
        )
    }

    pub fn platform_circle(x: f32, y: f32, r: f32, material: Material) -> Self {
        Body::new(x, y, Shape::Circle(Circle::new(r)), Static, material, None)
    }

    pub fn platform_polygon(
        x: f32,
        y: f32,
        vertices: Vec<Vector2>,
        rotation: f32,
        material: Material,
    ) -> Option<Body> {
        if let Some(poly) = Polygon::new(vertices.clone()) {
            let mut pp = Body::new(x, y, Shape::Polygon(poly), Static, material, None);

            for v in &mut pp.vertices {
                *v = Vector2::rotated(*v, rotation);
            }

            pp.generate_hitbox();

            Some(pp)
        } else {
            println!(
                "User tried to add concave polygon with vertices: {:?}",
                vertices
            );
            None
        }
    }
}

// --------------------------------- TRAITS ---------------------------------
impl Display for Body {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{dyn_or_stat} {shape}",
            dyn_or_stat = self.body_type,
            shape = self.shape
        )?;

        write!(f, "\n\tlocation: {}", self.transform.location)?;
        write!(f, "\n\tarea: {}", self.shape.area())?;
        write!(f, "\n\tmass: {}", self.mass)?;
        write!(f, "\n\tinertia: {}", self.inertia)
    }
}

impl Default for Body {
    fn default() -> Self {
        Self {
            name: None,
            shape: Default::default(),
            transform: Default::default(),
            material: Default::default(),
            hitbox: Default::default(),
            vertices: [ZERO; 8],
            vertice_count: 1,
            body_type: Dynamic,
            mass: 0.0,
            inverse_mass: 0.0,
            inertia: 0.0,
            inverse_inertia: 0.0,
        }
    }
}
