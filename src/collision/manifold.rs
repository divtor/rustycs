//! Stores information about individual collisions to properly resolve them.<br>

use crate::{
    body::Body,
    maths::{
        vector2::{cross, dot, ZERO},
        Vector2,
    },
};

#[derive(Clone, Copy, Debug, Default)]
pub struct Contact {
    pub location: Vector2,
    pub diff_to_a: Vector2,
    pub diff_to_b: Vector2,
    pub normal_magnitude: f32,
    pub tangent_magnitude: f32,
}

#[derive(Debug)]
pub struct Manifold {
    pub a_idx: usize,
    pub b_idx: usize,
    pub normal: Vector2,
    pub tangent: Vector2,
    pub depth: f32,
    pub contact_count: usize,
    pub inv_contact_count: f32,
    pub bounce_factor: f32,
    pub friction: f32,
    pub contacts: [Contact; 2],
}

impl Manifold {
    pub fn new(a: &Body, a_idx: usize, b: &Body, b_idx: usize) -> Manifold {
        let restitution = (a.material.restitution + b.material.restitution) * 0.5;
        let bounce_factor = -(1. + restitution);
        let friction = (a.material.friction + b.material.friction) * 0.5;

        Manifold {
            a_idx,
            b_idx,
            normal: ZERO,
            tangent: ZERO,
            depth: 0.,
            contact_count: 0,
            inv_contact_count: 0.,
            contacts: [Contact::default(); 2],
            bounce_factor,
            friction,
        }
    }
}

const BOUNCE_THRESHHOLD: f32 = 0.0001;

impl Manifold {
    pub fn setup(&mut self, a: &Body, b: &Body, scaled_world_force: Vector2) {
        self.tangent = self.normal.tangent();
        self.inv_contact_count = 1. / self.contact_count as f32;

        for contact in self.contacts.iter_mut().take(self.contact_count) {
            contact.diff_to_a = contact.location - a.transform.location;
            contact.diff_to_b = contact.location - b.transform.location;

            let ac = contact.diff_to_a;
            let bc = contact.diff_to_b;

            let a_normal = dot(ac, self.normal);
            let b_normal = dot(bc, self.normal);

            let mut factor_n = a.inverse_mass + b.inverse_mass;
            factor_n += (dot(ac, ac) - a_normal * a_normal) * a.inverse_inertia;
            factor_n += (dot(bc, bc) - b_normal * b_normal) * b.inverse_inertia;
            contact.normal_magnitude = 1. / factor_n;

            let a_tangent = dot(ac, self.tangent);
            let b_tangent = dot(bc, self.tangent);

            let mut factor_t = a.inverse_mass + b.inverse_mass;
            factor_t += (dot(ac, ac) - a_tangent * a_tangent) * a.inverse_inertia;
            factor_t += (dot(bc, bc) - b_tangent * b_tangent) * b.inverse_inertia;
            contact.tangent_magnitude = 1. / factor_t;

            let mut v_rel = b.transform.velocity + cross(b.transform.angular_velocity, bc);
            v_rel -= a.transform.velocity + cross(a.transform.angular_velocity, ac);

            // do not add bounce factor if only world forces move the body
            if v_rel.len_squared() < scaled_world_force.len_squared() + BOUNCE_THRESHHOLD {
                self.bounce_factor = -1.;
            }
        }
    }
}
