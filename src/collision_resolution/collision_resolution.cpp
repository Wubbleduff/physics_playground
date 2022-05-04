
#include "collision_resolution/collision_resolution.h"
#include "graphics.h"
#include <math.h>


using namespace GameMath;



namespace CollisionResolution
{

static float EPSILON = 0.0001f;
static bool EQ(float a, float b)
{
    return (GameMath::abs(a - b) < EPSILON);
}
static bool EQ(v2 a, v2 b)
{
    return (GameMath::abs(a.x - b.x) < EPSILON) && (GameMath::abs(a.y - b.y) < EPSILON);
}

bool box_box(Box *a, Box *b, Collision *collision)
{
    v2 a_vertices[] =
    {
        a->center - rotate_vector(a->half_extents, a->rotation), // bl
        a->center + rotate_vector(v2(a->half_extents.x, -a->half_extents.y), a->rotation), // br
        a->center + rotate_vector(a->half_extents, a->rotation), // tr
        a->center + rotate_vector(v2(-a->half_extents.x, a->half_extents.y), a->rotation), // tl
    };
    v2 b_vertices[] =
    {
        b->center - rotate_vector(b->half_extents, b->rotation), // bl
        b->center + rotate_vector(v2(b->half_extents.x, -b->half_extents.y), b->rotation), // br
        b->center + rotate_vector(b->half_extents, b->rotation), // tr
        b->center + rotate_vector(v2(-b->half_extents.x, b->half_extents.y), b->rotation), // tl
    };

    // SAT to collide.
    // We may early out here.
    v2 *v_pairs[2][2] = {
        {&(a_vertices[0]), &(b_vertices[0])},
        {&(b_vertices[0]), &(a_vertices[0])}
    };
    // Check A against B and B against A.
    for(int pair_index = 0; pair_index < 2; pair_index++)
    {
        v2 *i_vertices = v_pairs[pair_index][0];
        v2 *j_vertices = v_pairs[pair_index][1];

        // Loop through each plane for the first shape.
        for(int i = 0; i < 4; i++)
        {
            v2 i_plane = i_vertices[(i + 1) % 4] - i_vertices[i];

            // Since this is a OBB, we know the min and max of the first shape
            // is just the 2 vertices we picked for the current edge axis.
            float min_i = 0.0f;
            float max_i = dot(i_plane, i_plane);

            // Find min and max of second shape.
            float min_j = INFINITY;
            float max_j = -INFINITY;
            for(int j = 0; j < 4; j++)
            {
                float j_vertex_measured_along_i_plane = dot(i_plane, j_vertices[j] - i_vertices[i]);
                min_j = min(min_j, j_vertex_measured_along_i_plane);
                max_j = max(max_j, j_vertex_measured_along_i_plane);
            }

            // If there's no overlap, there' can't be a collision.
            if(min_i > max_j || max_i < min_j) return false;
        }

    }

    // From this point forward, the boxes are assumed to be colliding.
    // Given the boxes are colliding:
    // Find the minimum separating normal for the collision.

    v2 min_separating_normal = v2();
    v2 min_contact_points[2] = {v2(), v2()};
    bool resolve_as_a_in_b = false; // Used for the orientation of the collision.
    bool initial_check = true; // Used for always using the first separation normal check when finding minimum.

    // Check A against B and B against A.
    for(int pair_index = 0; pair_index < 2; pair_index++)
    {
        v2 *i_vertices = v_pairs[pair_index][0];
        v2 *j_vertices = v_pairs[pair_index][1];

        // Loop through edges of the first shape.
        for(int i = 0; i < 4; i++)
        {
            v2 i_normal = -find_ccw_normal(i_vertices[(i + 1) % 4] - i_vertices[i]);

            // Find the maximum penetration of the second shape's vertices. We can
            // only resolve the collision if we use the vertex that's deepest in the
            // first shape's edges.
            float max_pen = 0.0f;
            int max_pen_index = 0;
            for(int j = 0; j < 4; j++)
            {
                float j_vertex_pen_depth = dot(i_normal, j_vertices[j] - i_vertices[i]);
                if(j_vertex_pen_depth < max_pen)
                {
                    max_pen = j_vertex_pen_depth;
                    max_pen_index = j;
                }
            }

            // Check if this separation is the best (minimum) separation so far.
            float j_pen_depth = abs(signed_distance_to_plane(j_vertices[max_pen_index], i_vertices[i], i_normal));
            if(initial_check || (j_pen_depth < length(min_separating_normal)))
            {
                min_separating_normal = normalize(i_normal) * j_pen_depth;
                min_contact_points[0] = j_vertices[max_pen_index];
                min_contact_points[1] = min_contact_points[0] + min_separating_normal;
                resolve_as_a_in_b = (bool)pair_index;
                initial_check = false;
            }
        }
    }

    // Check for the orientation of the collision.
    if(resolve_as_a_in_b)
    {
        collision->a_in_b = min_contact_points[0];
        collision->b_in_a = min_contact_points[1];
    }
    else
    {
        collision->b_in_a = min_contact_points[0];
        collision->a_in_b = min_contact_points[1];
    }

    return true;
}

bool box_circle(Box *box, Circle *circle, Collision *collision)
{
    // Rotate the system so it's AABB vs circle collision where the box is at the origin.
    v2 box_center = v2();
    v2 circle_center = circle->center;
    circle_center -= box->center;
    circle_center = rotate_vector(circle_center, -box->rotation);

    //v2 box_center = box->center;
    //v2 circle_center = circle->center;

    v2 b_min = box_center - box->half_extents;
    v2 b_max = box_center + box->half_extents;

    v2 clamped = v2(
            clamp(circle_center.x, b_min.x, b_max.x),
            clamp(circle_center.y, b_min.y, b_max.y));

    if(length(circle_center - clamped) < circle->radius)
    {
        // Collision
        if(EQ(circle_center, clamped))
        {
            // Center in box
            v2 c_min = circle_center - v2(circle->radius, circle->radius);
            v2 c_max = circle_center + v2(circle->radius, circle->radius);

            float left_diff = c_max.x - b_min.x;
            float right_diff = b_max.x - c_min.x;
            float bottom_diff = c_max.y - b_min.y;
            float top_diff = b_max.y - c_min.y;
            float min_diff = min(left_diff, min(right_diff, min(bottom_diff, top_diff)));

            v2 normal;
            if(min_diff == left_diff) normal = v2(-1.0f, 0.0f);
            else if(min_diff == right_diff) normal = v2(1.0f, 0.0f);
            else if(min_diff == top_diff) normal = v2(0.0f, 1.0f);
            else if(min_diff == bottom_diff) normal = v2(0.0f, -1.0f);

            collision->a_in_b = circle_center + normal * (min_diff - circle->radius);
            collision->b_in_a = circle_center - normal * circle->radius;

            // Un-rotate system
            collision->a_in_b = rotate_vector(collision->a_in_b, box->rotation);
            collision->b_in_a = rotate_vector(collision->b_in_a, box->rotation);
            collision->a_in_b += box->center;
            collision->b_in_a += box->center;
            return true;
        }
        else
        {
            // Center outside of box
            collision->a_in_b = clamped;
            collision->b_in_a = circle_center + normalize(clamped - circle_center) * circle->radius;

            collision->a_in_b = rotate_vector(collision->a_in_b, box->rotation);
            collision->b_in_a = rotate_vector(collision->b_in_a, box->rotation);
            collision->a_in_b += box->center;
            collision->b_in_a += box->center;
            return true;
        }
    }
    
    return false;
}

bool circle_circle(Circle *a, Circle *b, Collision *collision)
{
    v2 diff = b->center - a->center;
    float l = length(diff);
    if(l > a->radius + b->radius)
    {
        return false;
    }
    else if(EQ(a->center, b->center))
    {
        collision->a_in_b = a->center;
        collision->b_in_a = a->center;
        return true;
    }
    else
    {
        v2 ndiff = normalize(diff);
        collision->a_in_b = a->center + ndiff * a->radius;
        collision->b_in_a = b->center - ndiff * b->radius;
        return true;
    }
}








static void update_body(RigidBodyBox *body, float time_step)
{
    static const float air_drag = 3.0f;
    static const float rotational_air_drag = 1.0f;

    v2 drag_force = 0.5f * -body->velocity * air_drag;
    body->apply_force(drag_force, v2());

    body->velocity += body->acceleration_sum * time_step;
    body->shape.center += body->velocity * time_step;

    float angular_drag_force = -body->angular_velocity * rotational_air_drag;
    body->apply_torque(angular_drag_force);

    body->angular_velocity += body->angular_acceleration_sum * time_step;
    body->shape.rotation += body->angular_velocity * time_step;

    body->acceleration_sum = v2();
    body->angular_acceleration_sum = 0.0;
}

static void update_body(RigidBodyCircle *body, float time_step)
{
    static const float air_drag = 3.0f;

    v2 drag_force = 0.5f * -body->velocity * air_drag;
    body->apply_force(drag_force, v2());

    body->velocity += body->acceleration_sum * time_step;
    body->shape.center += body->velocity * time_step;

    float angular_drag_force = -body->angular_velocity * air_drag;
    body->apply_torque(angular_drag_force);

    body->angular_velocity += body->angular_acceleration_sum * time_step;
    body->shape.rotation += body->angular_velocity * time_step;

    body->acceleration_sum = v2();
    body->angular_acceleration_sum = 0.0;
}

void LevelCollisionResolution::init()
{
    Graphics::Camera::width() = 16.0f;

    b1.mass = 1.0f;
    b1.moment_of_inertia = 1.0f;
    b2.mass = 1.0f;
    b2.moment_of_inertia = 1.0f;

    b1.shape.half_extents = v2(0.5f, 0.5f);
    b2.shape.radius = 0.5f;

    v2 initial_vel = v2(15.0f, 0.0f);

    b1.shape.center = v2(-5.0f, 0.25f);
    b1.velocity = initial_vel;

    b2.shape.center = v2( 5.0f, -0.25f);
    b2.velocity = -initial_vel;
}

void LevelCollisionResolution::step(float time_step)
{
    update_body(&b1, time_step);
    update_body(&b2, time_step);

    Collision c;
    bool collided = box_circle(&b1.shape, &b2.shape, &c);
    if(collided)
    {
        //
        // Collision resolution is based on the following contraints:
        // * We can simplify a tiny window of large forces into an impulse (just an instant change in velocity).
        //   We need to figure out how much of a change in velocity we need to apply.
        // * With a frictionless collision, we only apply an impulse in the axis of the collision normal. This
        //   simplifies the collision into a 1 dimensional resolution problem: we can look at the relative
        //   velocities along the collision normal.
        // * Newton's law of restituion: relative_velocity_after = e * -relative_velocity_before
        // * Applying the impluse: velocity_after = velocity_before + impulse   ->
        //                         velocity_after = velocity_before + (j / M)*n
        // 
        // Combining these equations, we can figure out what impulse to apply:
        // impulse = direction of the collision * impulse magnitude
        // We know the direction of the impulse: the collision normal. The magnitude of the impulse is (j / M).
        // In otherwords, we need to find j.
        // Using the equations above:
        // * relative_velocity_after*n = e * -relative_velocity_before*n ->
        //   (va_after - vb_after)*n = e * -(va_before - vb_before)*n
        // and
        // * va_after = va_before + (j / M_a)*n
        //   vb_after = vb_before - (j / M_b)*n
        // algebra.exe
        // j = -(1 + e)*(va_before - vb_before) * n
        //     ------------------------------------
        //            n * n*(1/M_a + 1/M_b)        
        // 
        // Plug j back into equations for applying impulse and collision is resolved.
        //
        // For rotations, we follow a similar set of constarints:
        // * wa_after = wa_before + (r_ap * j*n) / Ia
        // * wb_after = wb_before - (r_bp * j*n) / Ib
        // w is rotational velocity.
        // rp_ab is perp-dot-product of the point of application that same as in dynamics.
        // I is moment of inertia.
        //
        // j =           -(1 + e)*(va_before - vb_before) * n
        //     --------------------------------------------------------
        //     n * n*(1/M_a + 1/M_b) + (r_ap * n)^2/Ia + (r_bp * n)^2/Ia
        //

        // Check if we're moving towards each other. Otherwise, we're moving out and we shouldn't change velocities anyways.
        if(dot(b2.shape.center - b1.shape.center, b1.velocity) > 0.0f)
        {
            static const float e = 0.75f;
            v2 n = c.a_in_b - c.b_in_a;
            v2 relative_velocity = b1.velocity - b2.velocity;

            {
                float j = (-(1.0f + e) * dot(relative_velocity, n)) /
                          (dot(n, n * (1.0f / b1.mass + 1.0f / b2.mass)));

                b1.velocity = b1.velocity + (j / b1.mass) * n;
                b2.velocity = b2.velocity - (j / b2.mass) * n;
            }

            {
                v2 r_ap = find_ccw_normal(c.a_in_b - b1.shape.center);
                v2 r_bp = find_ccw_normal(c.b_in_a - b2.shape.center);
                float j = (-(1.0f + e) * dot(relative_velocity, n)) /
                          (dot(n, n * (1.0f / b1.mass + 1.0f / b2.mass)) + (squared(dot(r_ap, n)) / b1.moment_of_inertia) + (squared(dot(r_bp, n)) / b2.moment_of_inertia));

                b1.angular_velocity = b1.angular_velocity + dot(r_ap, j * n) / b1.moment_of_inertia;
                b2.angular_velocity = b2.angular_velocity - dot(r_bp, j * n) / b2.moment_of_inertia;
            }

        }
    }

    Graphics::quad(b1.shape.center, b1.shape.half_extents, b1.shape.rotation, Color::BLUE);
    Graphics::arrow(b1.shape.center, b1.shape.center + rotate_vector(b1.shape.half_extents, b1.shape.rotation), 0.02f, v4(Color::RED));

    Graphics::circle(b2.shape.center, b2.shape.radius, Color::YELLOW);
    Graphics::arrow(b2.shape.center, b2.shape.center + rotate_vector(v2(1.0f, 0.0f) * b2.shape.radius, b2.shape.rotation), 0.02f, v4(Color::RED));

}

}
