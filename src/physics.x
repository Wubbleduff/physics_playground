
#include "physics.h"

#include <vector>
#include <cassert>

using namespace GameMath;



static float EPSILON = 0.0001f;



static bool EQ(float a, float b)
{
    return (GameMath::abs(a - b) < EPSILON);
}

static bool EQ(v2 a, v2 b)
{
    return (GameMath::abs(a.x - b.x) < EPSILON) && (GameMath::abs(a.y - b.y) < EPSILON);
}



static bool circle_circle(v2 a_center, float a_radius, v2 b_center, float b_radius, Collision::Manifold *m)
{
    assert(a_radius > EPSILON);
    assert(b_radius > EPSILON);

    v2 diff = b_center - a_center;
    float l = length(diff);
    if(l > a_radius + b_radius)
    {
        return false;
    }
    else if(EQ(a_center, b_center))
    {
        m->a_in_b = a_center;
        m->b_in_a = a_center;
        return true;
    }
    else
    {
        v2 ndiff = normalize(diff);
        m->a_in_b = a_center + ndiff * a_radius;
        m->b_in_a = b_center - ndiff * b_radius;
        return true;
    }
}

static bool circle_box(v2 c_center, float c_radius, v2 b_center, v2 b_half_extents, Collision::Manifold *m)
{
    assert(c_radius > EPSILON);
    assert(!EQ(b_half_extents, v2()));
    if(EQ(c_center, b_center))
    {
        m->a_in_b = c_center;
        m->b_in_a = c_center;
        return true;
    }

    v2 b_min = b_center - b_half_extents;
    v2 b_max = b_center + b_half_extents;

    v2 clamped = v2(
            clamp(c_center.x, b_min.x, b_max.x),
            clamp(c_center.y, b_min.y, b_max.y));

    if(length(c_center - clamped) < c_radius)
    {
        // Collision
        if(EQ(c_center, clamped))
        {
            // Center in box
            v2 c_min = c_center - v2(c_radius, c_radius);
            v2 c_max = c_center + v2(c_radius, c_radius);

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

            m->b_in_a = c_center - normal * c_radius;
            m->a_in_b = c_center + normal * (min_diff - c_radius);

            return true;
        }
        else
        {
            // Center outside of box
            v2 diff = c_center - clamped;
            v2 normal = normalize(diff);
            m->b_in_a = c_center - normal * c_radius;
            m->a_in_b = clamped;
            return true;
        }
    }
    
    return false;
}

static bool circle_plane(v2 a_center, float a_radius, v2 b_pos, v2 b_normal, Collision::Manifold *m)
{
    assert(a_radius > EPSILON);
    assert(!EQ(b_normal, v2()));
    assert(EQ(length(b_normal), 1.0f));

    float dist_from_plane = signed_distance_to_plane(a_center, b_pos, b_normal);
    float pen_depth = dist_from_plane - a_radius;
    if(pen_depth > EPSILON)
    {
        return false;
    }
    else
    {
        m->a_in_b = a_center - b_normal * a_radius;
        m->b_in_a = project_onto_plane(a_center, b_pos, b_normal);
        return true;
    }
}

static bool box_box(v2 a_center, v2 a_half_extents,
                    v2 b_center, v2 b_half_extents,
                    Collision::Manifold *m)
{
    v2 a_up = v2(0.0f, a_half_extents.y);
    v2 a_right = v2(a_half_extents.x, 0.0f);
    v2 b_up = v2(0.0f, b_half_extents.y);
    v2 b_right = v2(b_half_extents.x, 0.0f);
    assert(!EQ(a_up, v2()));
    assert(!EQ(a_right, v2()));
    assert(!EQ(b_up, v2()));
    assert(!EQ(b_right, v2()));
    assert(EQ(dot(a_up, a_right), 0.0f));
    assert(EQ(dot(b_up, b_right), 0.0f));

    v2 a_vertices[4] =
    {
        a_center - a_right - a_up,
        a_center + a_right - a_up,
        a_center + a_right + a_up,
        a_center - a_right + a_up };
    v2 a_normals[4] = { -a_up, a_right, a_up, -a_right };
    for(int i = 0; i < 4; i++) a_normals[i] = normalize(a_normals[i]);

    v2 b_vertices[4] =
    {
        b_center - b_right - b_up,
        b_center + b_right - b_up,
        b_center + b_right + b_up,
        b_center - b_right + b_up
    };
    v2 b_normals[4] = { -b_up, b_right, b_up, -b_right };
    for(int i = 0; i < 4; i++) b_normals[i] = normalize(b_normals[i]);

    bool collided = false;
    for(int a_i = 0; a_i < 4; a_i++)
    {
        v2 a_v = a_vertices[a_i];

        bool in_all = true;
        for(int b_i = 0; b_i < 4; b_i++)
        {
            v2 b_v = b_vertices[b_i];
            v2 b_n = b_normals[b_i];

            float depth = dot(a_v - b_v, b_n);
            if(depth > 0.0f)
            {
                in_all = false;
                break; // Point is outside
            }
        }

        if(in_all)
        {
            collided = true;
            break;
        }
    }

    for(int b_i = 0; b_i < 4; b_i++)
    {
        v2 b_v = b_vertices[b_i];

        bool in_all = true;
        for(int a_i = 0; a_i < 4; a_i++)
        {
            v2 a_v = a_vertices[a_i];
            v2 a_n = a_normals[a_i];

            float depth = dot(b_v - a_v, a_n);
            if(depth > 0.0f)
            {
                in_all = false;
                break; // Point is outside
            }
        }

        if(in_all)
        {
            collided = true;
            break;
        }
    }

    if(!collided) return false;

    float min_separation_depth = infinity();
    v2 min_separating_axis;
    v2 pen_point;
    bool is_pen_point_a = false;;

    for(int i = 0; i < 4; i++)
    {
        v2 n = a_normals[i];
        v2 p = a_vertices[i];

        float max_depth = 0.0f;
        v2 max_point;
        for(int j = 0; j < 4; j++)
        {
            v2 other_p = b_vertices[j];
            float depth = -dot(other_p - p, n);

            if(depth > max_depth)
            {
                max_depth = depth;
                max_point = other_p;
            }
        }

        if(max_depth < min_separation_depth)
        {
            min_separation_depth = max_depth;
            min_separating_axis = n;
            pen_point = max_point;
            is_pen_point_a = true;
        }
    }

    for(int i = 0; i < 4; i++)
    {
        v2 n = b_normals[i];
        v2 p = b_vertices[i];

        float max_depth = 0.0f;
        v2 max_point;
        for(int j = 0; j < 4; j++)
        {
            v2 other_p = a_vertices[j];
            float depth = -dot(other_p - p, n);

            if(depth > max_depth)
            {
                max_depth = depth;
                max_point = other_p;
            }
        }

        if(max_depth < min_separation_depth)
        {
            min_separation_depth = max_depth;
            min_separating_axis = n;
            pen_point = max_point;
        }
    }

    if(is_pen_point_a)
    {
        m->b_in_a = pen_point;
        m->a_in_b = pen_point + min_separating_axis * min_separation_depth;
    }
    else
    {
        m->a_in_b = pen_point;
        m->b_in_a = pen_point + min_separating_axis * min_separation_depth;
    }

    return true;
}

static bool box_plane(v2 a_center, v2 a_half_extents,
                      v2 b_pos, v2 b_normal,
                      Collision::Manifold *m)
{
    v2 a_up = v2(0.0f, a_half_extents.y);
    v2 a_right = v2(a_half_extents.x, 0.0f);
    assert(!EQ(a_up, v2()));
    assert(!EQ(a_right, v2()));
    assert(EQ(dot(a_up, a_right), 0.0f));
    assert(!EQ(b_normal, v2()));

    v2 a_vertices[4] =
    {
        a_center - a_right - a_up,
        a_center + a_right - a_up,
        a_center + a_right + a_up,
        a_center - a_right + a_up
    };

    float most_negative_dist = 0.0f;
    int most_negative_i = -1;
    for(int i = 0; i < 4; i++)
    {
        v2 v = a_vertices[i];
        float dist_from_plane = signed_distance_to_plane(v, b_pos, b_normal);
        if(dist_from_plane < most_negative_dist) {
            most_negative_dist = dist_from_plane;
            most_negative_i = i;
        }
    }

    if(most_negative_i == -1)
    {
        return false;
    }

    m->a_in_b = a_vertices[most_negative_i];
    m->b_in_a = project_onto_plane(a_vertices[most_negative_i], b_pos, b_normal);
    return true;
}



bool collide_rigid_bodies(RigidBody *a, RigidBody *b, Collision::Manifold *manifold)
{
    if(a->shape.type == Shape::CIRCLE)
    {
        if(b->shape.type == Shape::CIRCLE)
        {
            return circle_circle(a->position, a->shape.circle.radius, b->position, b->shape.circle.radius, manifold);
        }
        if(b->shape.type == Shape::BOX)
        {
            return circle_box(a->position, a->shape.circle.radius, b->position, v2(b->shape.box.half_extents), manifold);
        }
        if(b->shape.type == Shape::PLANE)
        {
            return circle_plane(a->position, a->shape.circle.radius, b->position, b->shape.plane.normal, manifold);
        }
    }

    if(a->shape.type == Shape::BOX)
    {
        if(b->shape.type == Shape::CIRCLE)
        {
            return circle_box(b->position, b->shape.circle.radius, a->position, v2(a->shape.box.half_extents), manifold);
        }
        if(b->shape.type == Shape::BOX)
        {
            return box_box(a->position, a->shape.box.half_extents, b->position, b->shape.box.half_extents, manifold);
        }
        if(b->shape.type == Shape::PLANE)
        {
            return box_plane(a->position, a->shape.box.half_extents, b->position, b->shape.plane.normal, manifold);
        }
    }

    if(a->shape.type == Shape::PLANE)
    {
        if(b->shape.type == Shape::CIRCLE)
        {
            return circle_plane(b->position, b->shape.circle.radius, b->position, a->shape.plane.normal, manifold);
        }
        if(b->shape.type == Shape::BOX)
        {
            return box_plane(b->position, b->shape.box.half_extents, b->position, a->shape.plane.normal, manifold);
        }
        if(b->shape.type == Shape::PLANE)
        {
            return false;
        }
    }

    assert(false);
    return false;
}

void solve_rigid_body_physics(RigidBody *rigid_bodies, uint32_t num_rigid_bodies, float time_step, uint32_t iters)
{
    for(int i = 0 ; i < iters; i++)
    {
        float divided_time_step = time_step / iters;

        // Kinematics
        for(RigidBody *body = rigid_bodies; body < rigid_bodies + num_rigid_bodies; body++)
        {
            if(body->is_static) continue;

            static const float SMALLEST_SPEED = 0.00001f;
            if(length(body->velocity) < SMALLEST_SPEED)
            {
                body->velocity = v2();
            }

            body->velocity.y += -9.81f * divided_time_step * divided_time_step;
            body->velocity -= body->velocity * 1.0f * divided_time_step;
            body->position += body->velocity;
        }

        // Collision detection
        std::vector<Collision> collisions;
        for(int i = 0; i < num_rigid_bodies; i++)
        {
            for(int j = i + 1; j < num_rigid_bodies; j++)
            {
                RigidBody *a = rigid_bodies + i;
                RigidBody *b = rigid_bodies + j;

                //if(!a->shape || !b->shape) continue;

                bool hit = false;
                Collision collision = {};

                //hit = a->shape->collide(&a->transform, b->shape, &b->transform, &collision.manifold);
                hit = collide_rigid_bodies(a, b, &collision.manifold);

                if(hit)
                {
                    collision.a = a;
                    collision.b = b;
                    collisions.push_back(collision);

                    //d_col = collision;
                }
            }
        }

#if 0
        // Rope
        {
            RigidBody *body = rigid_bodies + (num_rigid_bodies - 1);
            v2 mouse_diff = body->transform.position - mouse_pos;
            float rope_len = 1.0f;
            if(length(mouse_diff) >= rope_len)
            {
                v2 target = mouse_pos + normalize(mouse_diff) * rope_len;
                v2 body_to_target = target - body->transform.position;
                body->velocity += body_to_target * (1.0f / iters);

                float percent = 0.8f;
                body->transform.position += body_to_target * percent;
            }
            Graphics::line(mouse_pos, body->transform.position, 0.01f, Color::YELLOW);
        }
#endif

        // Collision resolution
        // Impulse
        for(const Collision &collision : collisions)
        {
            float a_inv_mass = collision.a->is_static ? 0.0f : 1.0f / collision.a->mass;
            float b_inv_mass = collision.b->is_static ? 0.0f : 1.0f / collision.b->mass;
            v2 relative_velocity = collision.b->velocity - collision.a->velocity;
            v2 normal = normalize(collision.manifold.a_in_b - collision.manifold.b_in_a);
            float n_speed = dot(relative_velocity, normal);

            if(n_speed < 0.0f)
            {
                continue;
            }

            float e = 1.0f * 0.5f;
            // float e = collision.a->restitution * collision.b->restitution;
            float j = -(1.0f + e) * n_speed / (a_inv_mass + b_inv_mass);
            v2 impulse = j * normal;

            if(!collision.a->is_static)
            {
                collision.a->velocity -= impulse * a_inv_mass;
            }

            if(!collision.b->is_static)
            {
                collision.b->velocity += impulse * b_inv_mass;
            }
        }

        // Position correction
        for(const Collision &collision : collisions)
        {
            float percent = 0.8f;
            float slop = 0.01f;

            float a_factor;
            float b_factor;
            if(collision.a->is_static && collision.b->is_static)
            {
                continue;
            }
            else if(collision.a->is_static && !collision.b->is_static)
            {
                a_factor = 0.0f;
                b_factor = 1.0f;
            }
            else if(collision.b->is_static && !collision.a->is_static)
            {
                a_factor = 1.0f;
                b_factor = 0.0f;
            }
            else
            {
                float sum = collision.a->mass + collision.b->mass;
                a_factor = collision.a->mass / sum;
                b_factor = collision.b->mass / sum;
            }

            v2 diff = collision.manifold.b_in_a - collision.manifold.a_in_b;
            v2 normal = -normalize(diff);
            float pen_depth = length(diff);
            pen_depth = max(pen_depth - slop, 0.0f);
            v2 correction = normal * pen_depth * percent;
            collision.a->position += correction * a_factor;
            collision.b->position -= correction * b_factor;
        }
    }
}

