//! Material struct that defines how exactly a body reacts to impulses and friction etc.

/// Each material has a density. <br>
/// To get the mass of a object, "simply" calculate "area times density".<br>
/// This is only simulated mass, so the actual units do not matter for now.
#[derive(Clone, Debug)]
pub struct Material {
    pub density: f32,
    pub friction: f32,
    pub restitution: f32,
    pub name: &'static str,
}

impl Material {
    pub const fn new(density: f32, friction: f32, restitution: f32, t: &'static str) -> Self {
        Self {
            density,
            friction,
            restitution,
            name: t,
        }
    }
}

impl Default for Material {
    fn default() -> Self {
        DEFAULT
    }
}

pub const DEFAULT: Material = Material::new(0.1, 0.6, 0.5, "default");
pub const RUBBER: Material = Material::new(0.9, 0.8, 0.6, "rubber");
pub const PLASTIC: Material = Material::new(1.2, 0.6, 0.5, "plastic");
pub const STONE: Material = Material::new(2.5, 0.4, 0.2, "stone");
pub const METAL: Material = Material::new(7.8, 0.2, 0.1, "metal");
