//! Deals with all things collision detection.<br>
//!
//! Provides ability to check for:<br>
//! - Possible collisions in broad phase
//! - Actual collisions in narrow phase, that produce a manifold

use crate::{
    collision::{Hitbox, Manifold},
    entities::body::Body,
    math::{
        vector2::{dot, NORMAL_DOWN, NORMAL_LEFT, NORMAL_RIGHT, NORMAL_UP, ZERO},
        Vector2,
    },
    shapes::Shape::*,
};

// --------------------------------- BROAD PHASE ---------------------------------
/// Detects if the hitboxes of given bodies intersect, if not no further checks are necessary,
/// because they cannot possibly collide.
pub fn hitboxes_collide(a: &Body, b: &Body) -> bool {
    let hitbox_a: Hitbox = &a.hitbox + a.transform.location;
    let hitbox_b: Hitbox = &b.hitbox + b.transform.location;

    if hitbox_a.min.x >= hitbox_b.max.x || hitbox_b.min.x >= hitbox_a.max.x {
        return false;
    }

    if hitbox_a.min.y >= hitbox_b.max.y || hitbox_b.min.y >= hitbox_a.max.y {
        return false;
    }

    true
}

// --------------------------------- NARROW PHASE ---------------------------------
/// Detects collision between 2 bodies (a, b) living in the world.<br>
/// If a collision is detected, the function generates a manifold that
/// can be used to resolve the collision later on.
pub fn detect_collision(a: &Body, a_idx: usize, b: &Body, b_idx: usize) -> Option<Manifold> {
    match a.shape {
        Circle(_) => match b.shape {
            Circle(_) => circle_circle(a, a_idx, b, b_idx),
            AABB(_) => circle_aabb(a, a_idx, b, b_idx),
            Polygon(_) => circle_polygon(a, a_idx, b, b_idx),
        },

        AABB(_) => match b.shape {
            Circle(_) => circle_aabb(b, b_idx, a, a_idx),
            AABB(_) => aabb_aabb(a, a_idx, b, b_idx),
            Polygon(_) => aabb_polygon(a, a_idx, b, b_idx),
        },

        Polygon(_) => match b.shape {
            Circle(_) => circle_polygon(b, b_idx, a, a_idx),
            AABB(_) => aabb_polygon(b, b_idx, a, a_idx),
            Polygon(_) => polygon_polygon(a, a_idx, b, b_idx),
        },
    }
}

// --------------------------------- CASE HANDLING ---------------------------------
fn circle_circle(a: &Body, a_idx: usize, b: &Body, b_idx: usize) -> Option<Manifold> {
    let ra = a.shape.copy_as_circle().r;
    let rb = b.shape.copy_as_circle().r;

    let direction = b.transform.location - a.transform.location;
    let r = ra + rb;

    if direction.len_squared() >= r * r {
        return None;
    }

    let mut m = Manifold::new(a, a_idx, b, b_idx);

    let normal = direction.normalize_or_random();
    let contact = a.transform.location + (normal * ra);

    m.normal = normal;
    m.depth = r - direction.len();
    m.contacts[0].location = contact;
    m.contact_count = 1;

    Some(m)
}

fn circle_aabb(circle: &Body, circle_idx: usize, aabb: &Body, aabb_idx: usize) -> Option<Manifold> {
    let r = circle.shape.copy_as_circle().r;

    let aabb_max = aabb.hitbox.max + aabb.transform.location;
    let aabb_min = aabb.hitbox.min + aabb.transform.location;

    let clamped_circle_location = circle.transform.location.clamp(aabb_min, aabb_max);
    let difference_to_clamped = clamped_circle_location - circle.transform.location;
    let location_to_clamped_squared = difference_to_clamped.len_squared();

    if r * r <= location_to_clamped_squared {
        return None;
    }

    let mut m: Manifold = Manifold::new(circle, circle_idx, aabb, aabb_idx);
    m.contact_count = 1;

    if location_to_clamped_squared != 0.0 {
        let distance = location_to_clamped_squared.sqrt();
        let normal = difference_to_clamped
            .normalize()
            .expect("should be impossible!");
        let depth = r - distance;
        let contact = circle.transform.location + normal * distance;

        m.depth = depth;
        m.normal = normal;
        m.contacts[0].location = contact;

        return Some(m);
    }

    let distance = circle.transform.location - aabb.transform.location;
    let abs_distance = distance.abs();
    let x_overlap = aabb.hitbox.max.x - abs_distance.x;
    let y_overlap = aabb.hitbox.max.y - abs_distance.y;

    let normal;
    let depth;
    let contact;

    if x_overlap < y_overlap {
        depth = r + x_overlap;

        if distance.x > 0.0 {
            normal = NORMAL_LEFT;
        } else {
            normal = NORMAL_RIGHT;
        }

        contact = circle.transform.location + normal * x_overlap;
    } else {
        depth = r + y_overlap;

        if distance.y > 0.0 {
            normal = NORMAL_DOWN;
        } else {
            normal = NORMAL_UP;
        }

        contact = circle.transform.location + normal * y_overlap;
    }

    m.depth = depth;
    m.normal = normal;
    m.contacts[0].location = contact;

    Some(m)
}

fn circle_polygon(
    circle: &Body,
    circle_idx: usize,
    polygon: &Body,
    polygon_idx: usize,
) -> Option<Manifold> {
    let r = circle.shape.copy_as_circle().r;

    let contact_candidate = contacts_single(circle.transform.location, polygon);

    let direction = contact_candidate - circle.transform.location;
    let distance_squared = direction.len_squared();

    let mut m = Manifold::new(circle, circle_idx, polygon, polygon_idx);
    m.contact_count = 1;
    m.contacts[0].location = contact_candidate;

    if is_outside_polygon(circle.transform.location, polygon) {
        if distance_squared > r * r {
            return None;
        }

        m.normal = direction.normalize().unwrap_or(
            (polygon.transform.location - circle.transform.location).normalize_or_random(),
        );

        m.depth = r - distance_squared.sqrt();
    } else {
        m.normal = direction.normalize().unwrap_or(
            (polygon.transform.location - circle.transform.location).normalize_or_random(),
        );

        m.normal *= -1.0;
        m.depth = r + distance_squared.sqrt();
    }

    Some(m)
}

fn aabb_aabb(a: &Body, a_idx: usize, b: &Body, b_idx: usize) -> Option<Manifold> {
    let mut m = Manifold::new(a, a_idx, b, b_idx);

    let direction = b.transform.location - a.transform.location;
    let aabb_1 = a.shape.copy_as_aabb();
    let aabb_2 = b.shape.copy_as_aabb();

    let x_overlap = aabb_1.max.x + aabb_2.max.x - direction.x.abs();
    let y_overlap = aabb_1.max.y + aabb_2.max.y - direction.y.abs();

    let normal;
    let depth;

    if y_overlap < x_overlap {
        if direction.y > 0.0 {
            normal = NORMAL_UP;
        } else {
            normal = NORMAL_DOWN;
        }
        depth = y_overlap;
    } else {
        if direction.x > 0.0 {
            normal = NORMAL_RIGHT;
        } else {
            normal = NORMAL_LEFT;
        }
        depth = x_overlap;
    }

    let (contact_1, contact_2) = contacts_double(a, b);

    m.contact_count = 1;
    m.depth = depth;
    m.normal = normal;
    m.contacts[0].location = contact_1;

    if let Some(p) = contact_2 {
        m.contact_count = 2;
        m.contacts[1].location = p;
    }

    Some(m)
}

fn aabb_polygon(
    aabb: &Body,
    aabb_idx: usize,
    polygon: &Body,
    polygon_idx: usize,
) -> Option<Manifold> {
    polygon_polygon(aabb, aabb_idx, polygon, polygon_idx)
}

fn polygon_polygon(a: &Body, a_idx: usize, b: &Body, b_idx: usize) -> Option<Manifold> {
    let (vertices_a, len_a) = a.get_moved_vertices();
    let (vertices_b, len_b) = b.get_moved_vertices();

    let mut depth = f32::MAX;
    let mut normal = ZERO;

    for idx in 0..len_a {
        let edge = vertices_a[(idx + 1) % len_a] - vertices_a[idx];
        let axis = edge
            .tangent()
            .normalize()
            .expect("polygon {a} with id {a_idx} was created with stacked vertices!");

        let (a_min, a_max) = sat_projection(a, axis);
        let (b_min, b_max) = sat_projection(b, axis);

        if a_min >= b_max || b_min >= a_max {
            return None;
        }

        let d = f32::min(b_max - a_min, a_max - b_min);

        if d < depth {
            depth = d;
            normal = axis;
        }
    }

    for idx in 0..len_b {
        let edge = vertices_b[(idx + 1) % len_b] - vertices_b[idx];
        let axis = edge
            .tangent()
            .normalize()
            .expect("polygon {b} with id {b_idx} was created with stacked vertices!");

        let (a_min, a_max) = sat_projection(a, axis);
        let (b_min, b_max) = sat_projection(b, axis);

        if a_min >= b_max || b_min >= a_max {
            return None;
        }

        let d = f32::min(b_max - a_min, a_max - b_min);

        if d < depth {
            depth = d;
            normal = axis;
        }
    }

    let mut m = Manifold::new(a, a_idx, b, b_idx);

    let direction = b.transform.location - a.transform.location;
    let (contact_1, contact_2) = contacts_double(a, b);

    if direction.dotted(normal) < 0.0 {
        normal *= -1.;
    }

    m.contact_count = 1;
    m.depth = depth;
    m.normal = normal;
    m.contacts[0].location = contact_1;

    if let Some(p) = contact_2 {
        m.contact_count = 2;
        m.contacts[1].location = p;
    }

    Some(m)
}

// --------------------------------- DETECTION UTILITY FUNCTIONS ---------------------------------
fn sat_projection(body: &Body, axis: Vector2) -> (f32, f32) {
    let mut min = f32::MAX;
    let mut max = f32::MIN;

    let (vertices, len) = body.get_moved_vertices();

    for vertice in vertices.into_iter().take(len) {
        let projected = dot(vertice, axis);

        if projected < min {
            min = projected;
        }

        if projected > max {
            max = projected;
        }
    }

    (min, max)
}

fn contacts_single(p: Vector2, body: &Body) -> Vector2 {
    let mut min_d2 = f32::MAX;
    let mut contact = ZERO;

    let (vertices, len) = body.get_moved_vertices();

    for idx in 0..len {
        let v1 = vertices[idx];
        let v2 = vertices[(idx + 1) % len];

        let contact_candidate = project_onto_line(v1, v2, p);
        let d2 = Vector2::distance_squared(p, contact_candidate);

        // contact candidate with least distance to given point is contact
        if d2 < min_d2 {
            min_d2 = d2;
            contact = contact_candidate;
        }
    }

    contact
}

fn contacts_double(a: &Body, b: &Body) -> (Vector2, Option<Vector2>) {
    let (vertices_a, len_a) = a.get_moved_vertices();
    let (vertices_b, len_b) = b.get_moved_vertices();

    let mut min_d2 = f32::MAX;
    let mut contact_1 = None;
    let mut contact_2 = None;

    // a -> b
    for va in vertices_a.into_iter().take(len_a) {
        for idx_b in 0..len_b {
            let vb_1 = vertices_b[idx_b];
            let vb_2 = vertices_b[(idx_b + 1) % len_b];

            let contact_candidate: Vector2 = project_onto_line(vb_1, vb_2, va);
            let d2 = Vector2::distance_squared(va, contact_candidate);

            if similar(d2, min_d2) {
                if contact_1.is_some() && !similar_vector2(contact_1.unwrap(), contact_candidate) {
                    contact_2 = Some(contact_candidate);
                }
            } else if d2 < min_d2 {
                min_d2 = d2;
                contact_1 = Some(contact_candidate);
            }
        }
    }

    // b -> a
    for vb in vertices_b.into_iter().take(len_b) {
        for idx_a in 0..len_a {
            let va_1 = vertices_a[idx_a];
            let va_2 = vertices_a[(idx_a + 1) % len_a];

            let contact_candidate = project_onto_line(va_1, va_2, vb);
            let d2 = Vector2::distance_squared(vb, contact_candidate);

            if similar(d2, min_d2) {
                if contact_1.is_some() && !similar_vector2(contact_1.unwrap(), contact_candidate) {
                    contact_2 = Some(contact_candidate);
                }
            } else if d2 < min_d2 {
                min_d2 = d2;
                contact_1 = Some(contact_candidate);
            }
        }
    }

    (
        contact_1.expect("collision detected when there is none"),
        contact_2,
    )
}

fn project_onto_line(a: Vector2, b: Vector2, p: Vector2) -> Vector2 {
    let line = b - a;
    let ap = p - a;
    let projection_factor = dot(line, ap) / dot(line, line);

    // of point gets projected outside of line segment:
    // clamp it to the line bounds (a or b)
    if projection_factor >= 1.0 {
        return b;
    }

    if projection_factor <= 0.0 {
        return a;
    }

    a + (line * projection_factor)
}

// Source: https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm
fn is_outside_polygon(p: Vector2, polygon: &Body) -> bool {
    let mut count = 0;
    let (verts, len) = polygon.get_moved_vertices();

    for idx in 0..len {
        let a = verts[idx];
        let b = verts[(idx + 1) % len];

        let y_in_between = (p.y < a.y) != (p.y < b.y);
        let x_on_left_side = p.x < (a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x));

        if y_in_between && x_on_left_side {
            count += 1;
        }
    }

    count % 2 == 0
}

const THRESHHOLD: f32 = 0.0001;

pub fn similar(f1: f32, f2: f32) -> bool {
    (f1 - f2).abs() <= THRESHHOLD
}

fn similar_vector2(v1: Vector2, v2: Vector2) -> bool {
    similar(v1.x, v2.x) && similar(v1.y, v2.y)
}
