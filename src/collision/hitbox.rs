//! Data structure used to wrap bodies in the physics world.<br>
//! If the hitboxes do not intersect, two bodies cannot possible intersect or collide at all.

use crate::maths::Vector2;

#[derive(Clone, Debug, Default)]
pub struct Hitbox {
    pub min: Vector2,
    pub max: Vector2,
}

impl Hitbox {
    pub fn new(min: Vector2, max: Vector2) -> Hitbox {
        Hitbox { min, max }
    }
}

impl std::ops::Add<Vector2> for &Hitbox {
    type Output = Hitbox;

    fn add(self, rhs: Vector2) -> Self::Output {
        Hitbox::new(self.min + rhs, self.max + rhs)
    }
}

impl std::ops::AddAssign<Vector2> for Hitbox {
    fn add_assign(&mut self, rhs: Vector2) {
        self.max += rhs;
        self.min += rhs;
    }
}
