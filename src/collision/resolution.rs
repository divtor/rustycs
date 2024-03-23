//! Resolves a provided collision between 2 bodies.<br>
//! Needs additional information in the form of a manifold.

use crate::{
    body::Body,
    collision::Manifold,
    maths::{vector2::cross, Vector2},
};

pub fn resolve_collision(m: &mut Manifold, bodies: &mut [Body]) {
    let (a, b) = collision_bodies(m.a_idx, m.b_idx, bodies);
    resolve(m, a, b);
}

fn resolve(m: &mut Manifold, a: &mut Body, b: &mut Body) {
    let mut v_rel;

    for c in m.contacts.into_iter().take(m.contact_count) {
        // rotational impulse - normal
        v_rel = b.transform.velocity + cross(b.transform.angular_velocity, c.diff_to_b);
        v_rel -= a.transform.velocity + cross(a.transform.angular_velocity, c.diff_to_a);

        let v_rel_n = v_rel.dotted(m.normal);

        // if normal impulse is in wrong direction -> apply nothing instead
        let mut jn = c.normal_magnitude * v_rel_n * m.bounce_factor * m.inv_contact_count;
        jn = f32::max(jn, 0.);

        apply_impulses(a, b, jn * m.normal, c.diff_to_a, c.diff_to_b);

        // friction impulse - tangent
        v_rel = b.transform.velocity + cross(b.transform.angular_velocity, c.diff_to_b);
        v_rel -= a.transform.velocity + cross(a.transform.angular_velocity, c.diff_to_a);

        let v_rel_t = v_rel.dotted(m.tangent);

        let max_friction = jn * m.friction;
        let mut jt = c.tangent_magnitude * -v_rel_t * m.inv_contact_count;

        jt = jt.clamp(-max_friction, max_friction);

        apply_impulses(a, b, jt * m.tangent, c.diff_to_a, c.diff_to_b);
    }
}

// --------------------------------- UTILITY FUNCTIONS ---------------------------------
fn apply_impulses(a: &mut Body, b: &mut Body, impulse: Vector2, ac: Vector2, bc: Vector2) {
    a.transform.velocity -= impulse * a.inverse_mass;
    a.transform.angular_velocity -= ac.crossed(impulse) * a.inverse_inertia;

    b.transform.velocity += impulse * b.inverse_mass;
    b.transform.angular_velocity += bc.crossed(impulse) * b.inverse_inertia;
}

/// Necessary function, since we store the body index instead of the body reference.<br>
/// It provides us with the mutable references to both bodies via a mutable iterator.<br><br>
/// Storing the reference would more efficient, but way more complex due to rust.<br>
/// We would have to annotate lifetimes for the references, which I tried, but failed to do.<br>
/// For time reasons, i will implement this after I finished my thesis.
fn collision_bodies(a_idx: usize, b_idx: usize, bodies: &mut [Body]) -> (&mut Body, &mut Body) {
    let mut ref_iter = bodies.iter_mut().enumerate();

    let (a, b);

    if a_idx < b_idx {
        a = ref_iter
            .find(|(idx, _)| *idx == a_idx)
            .expect("Body was removed, but still detected in the collision pipeline!")
            .1;
        b = ref_iter
            .find(|(idx, _)| *idx == b_idx)
            .expect("Body was removed, but still detected in the collision pipeline!")
            .1;
    } else {
        b = ref_iter
            .find(|(idx, _)| *idx == b_idx)
            .expect("Body was removed, but still detected in the collision pipeline!")
            .1;
        a = ref_iter
            .find(|(idx, _)| *idx == a_idx)
            .expect("Body was removed, but still detected in the collision pipeline!")
            .1;
    }

    (a, b)
}

const CORRECTION_FACTOR: f32 = 0.6;
const ALLOWED_INTERSECTION: f32 = 0.005;

pub fn correct_position(m: &Manifold, bodies: &mut [Body]) {
    let (a, b) = collision_bodies(m.a_idx, m.b_idx, bodies);
    let correction = f32::max(m.depth - ALLOWED_INTERSECTION, 0.)
        / (a.inverse_mass + b.inverse_mass)
        * CORRECTION_FACTOR
        * m.normal;
    a.transform.location -= correction * a.inverse_mass;
    b.transform.location += correction * b.inverse_mass;
}
