//! A custom made Vector2 class, specialized for rustycs.

use rand::Rng;
use std::{cmp::PartialEq, f32::consts::PI, fmt::Display, ops};

pub const ZERO: Vector2 = Vector2::new(0., 0.);
pub const NORMAL_UP: Vector2 = Vector2::new(0., 1.);
pub const NORMAL_DOWN: Vector2 = Vector2::new(0., -1.);
pub const NORMAL_LEFT: Vector2 = Vector2::new(-1., 0.);
pub const NORMAL_RIGHT: Vector2 = Vector2::new(1., 0.);

pub fn cross(f: f32, vec: Vector2) -> Vector2 {
    Vector2::new(-f * vec.y, f * vec.x)
}

pub fn dot(a: Vector2, b: Vector2) -> f32 {
    a.x * b.x + a.y * b.y
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

// constructors
impl Vector2 {
    /// create a vector with given values
    pub const fn new(x: f32, y: f32) -> Vector2 {
        Vector2 { x, y }
    }

    /// create vector with random values<br>
    /// limit is essentially the upper bound we want the random vector values to have<br>
    /// 0.0 ..= limit<br>
    pub fn rand(limit: f32) -> Vector2 {
        let mut rng = rand::thread_rng();

        let random_x = rng.gen_range(-1.0..1.0);
        let random_y = rng.gen_range(-1.0..1.0);

        let mut rand_vec = Vector2::new(random_x, random_y);
        rand_vec *= limit;

        rand_vec
    }

    pub fn rand_normal() -> Vector2 {
        let mut rng = rand::thread_rng();
        let angle = rng.gen_range(0.0..2. * PI);

        Vector2::rotated(Vector2::new(1., 0.), angle)
    }
}

// mutating methods
impl Vector2 {
    pub fn rotate(&mut self, angle: f32) {
        let x = self.x;
        let y = self.y;

        self.x = x.mul_add(angle.cos(), -y * angle.sin());
        self.y = x.mul_add(angle.sin(), y * angle.cos());
    }
}

// non mutating methods
impl Vector2 {
    /// computes representation with absolute x and y values
    pub fn abs(self) -> Vector2 {
        Vector2::new(self.x.abs(), self.y.abs())
    }

    /// computes the length of this vector
    pub fn len(self) -> f32 {
        self.dotted(self).sqrt()
    }

    /// computes the squared length of this vector, this reduces computational complexity and is often sufficient for use cases
    pub fn len_squared(self) -> f32 {
        self.dotted(self)
    }

    /// computes a normalized version of this vector
    pub fn normalize(self) -> Option<Self> {
        if self.len() == 0.0 {
            // Ideally, we should return Result<> instead of Option<>.
            // However, this suffices for the usecase and is simpler.
            return None;
        }

        Some(self * (1.0 / self.len()))
    }

    /// computes a normalized version of this vector or returns the zero vector if not possible
    pub fn normalize_or_zero(self) -> Self {
        if self.len() == 0.0 {
            return ZERO;
        }

        self * (1.0 / self.len())
    }

    /// computes a normalized version of this vector or returns a random normal vector if not possible
    pub fn normalize_or_random(self) -> Self {
        if self.len() == 0.0 {
            return Vector2::rand_normal();
        }

        self * (1.0 / self.len())
    }

    /// computes the dot product of this vector and another vector
    pub fn dotted(self, other: Vector2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// computes the crossproduct of this vector and another vector
    pub fn crossed(self, other: Vector2) -> f32 {
        self.x * other.y - self.y * other.x
    }

    /// computes the tangent of this vector
    pub fn tangent(self) -> Vector2 {
        Vector2::new(-self.y, self.x)
    }

    /// computes the angle between this and another vector
    pub fn angle_to(self, other: Vector2) -> f32 {
        (self.dotted(other) / (self.len() * other.len())).acos()
    }

    /// computes a clamped vector with given min and max vectors
    pub fn clamp(self, min: Vector2, max: Vector2) -> Vector2 {
        Vector2::max(Vector2::min(max, self), min)
    }
}

// associated functions
impl Vector2 {
    /// computes the distance between 2 given vectors
    pub fn distance(v1: Vector2, v2: Vector2) -> f32 {
        (v1 - v2).len()
    }

    /// computes the squared distance between 2 given vectors,
    /// this reduces computational complexity and is often sufficient for use cases
    pub fn distance_squared(v1: Vector2, v2: Vector2) -> f32 {
        (v1 - v2).len_squared()
    }

    /// computes the angle between 2 given vectors in radians
    pub fn angle_between(v1: Vector2, v2: Vector2) -> f32 {
        (v1.dotted(v2) / (v1.len() * v2.len())).acos()
    }

    /// computes a rotated vector from a given vector and the given angle to rotate
    pub fn rotated(v: Vector2, angle: f32) -> Self {
        Vector2::new(
            v.x.mul_add(angle.cos(), -v.y * angle.sin()),
            v.x.mul_add(angle.sin(), v.y * angle.cos()),
        )
    }

    /// computes a vector where the x and y values are the respective minimums of 2 given vectors
    pub fn min(v1: Vector2, v2: Vector2) -> Vector2 {
        Vector2::new(f32::min(v1.x, v2.x), f32::min(v1.y, v2.y))
    }

    /// computes a vector where the x and y values are the respective maximums of 2 given vectors
    pub fn max(v1: Vector2, v2: Vector2) -> Vector2 {
        Vector2::new(f32::max(v1.x, v2.x), f32::max(v1.y, v2.y))
    }
}

// traits
impl Display for Vector2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({:.3}/{:.3})", self.x, self.y)
    }
}

impl PartialEq for Vector2 {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl ops::Index<usize> for Vector2 {
    type Output = f32;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.y,
            other => panic!("Vec2D index {other} out of bounds!! Range is 0..=2."),
        }
    }
}

impl ops::IndexMut<usize> for Vector2 {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            other => panic!("Vector2 index {other} out of bounds!! Range is 0..=2."),
        }
    }
}

impl ops::Add for Vector2 {
    type Output = Vector2;

    fn add(self, rhs: Vector2) -> Self::Output {
        Vector2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl ops::AddAssign for Vector2 {
    fn add_assign(&mut self, rhs: Vector2) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl ops::Sub for Vector2 {
    type Output = Vector2;

    fn sub(self, rhs: Vector2) -> Self::Output {
        Vector2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl ops::SubAssign for Vector2 {
    fn sub_assign(&mut self, rhs: Vector2) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl ops::Mul<f32> for Vector2 {
    type Output = Vector2;

    fn mul(self, rhs: f32) -> Self::Output {
        Vector2::new(self.x * rhs, self.y * rhs)
    }
}

impl ops::Mul<Vector2> for f32 {
    type Output = Vector2;

    fn mul(self, rhs: Vector2) -> Self::Output {
        Vector2::new(self * rhs.x, self * rhs.y)
    }
}

impl ops::MulAssign<f32> for Vector2 {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl From<(f32, f32)> for Vector2 {
    fn from(value: (f32, f32)) -> Self {
        Vector2::new(value.0, value.1)
    }
}

impl From<Vector2> for (f32, f32) {
    fn from(value: Vector2) -> Self {
        (value.x, value.y)
    }
}
