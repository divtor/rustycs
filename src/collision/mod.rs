//! Submodules contain all relevant collision mechanics.
//! - detection
//! - resolution
//! - manifolds
//! - hitboxes

pub mod detection;
pub mod hitbox;
pub mod manifold;
pub mod resolution;

pub use detection::detect_collision;
pub use hitbox::*;
pub use manifold::Manifold;
pub use resolution::{correct_position, resolve_collision};
