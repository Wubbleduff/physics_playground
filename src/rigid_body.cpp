
#include "rigid_body.h"
#include "graphics.h"

#include <assert.h>


using namespace GameMath;
static float EPSILON = 0.0001f;



static bool EQ(float a, float b)
{
    return (abs(a - b) < EPSILON);
}

static bool EQ(v2 a, v2 b)
{
    return (abs(a.x - b.x) < EPSILON) && (abs(a.y - b.y) < EPSILON);
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



bool Circle::collide(Transform *transform, Shape *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return other->collide(other_transform, this, transform, manifold);
}

bool Circle::collide(Transform *transform, Circle *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return circle_circle(transform->position, radius, other_transform->position, other->radius, manifold);
}

bool Circle::collide(Transform *transform, Box *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return circle_box(transform->position, radius, other_transform->position, other->half_extents, manifold);
}

bool Circle::collide(Transform *transform, Plane *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return circle_plane(transform->position, radius, other_transform->position, other->normal, manifold);
}

void Circle::draw(Transform *transform)
{
    Graphics::circle(transform->position, radius, v4(0.5f, 0.0f, 1.0f, 1.0f));
}

bool Box::collide(Transform *transform, Shape *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return other->collide(other_transform, this, transform, manifold);
}

bool Box::collide(Transform *transform, Circle *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return circle_box(other_transform->position, other->radius, transform->position, half_extents, manifold);
}

bool Box::collide(Transform *transform, Box *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return box_box(transform->position, half_extents, other_transform->position, other->half_extents, manifold);
}

bool Box::collide(Transform *transform, Plane *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return box_plane(transform->position, half_extents, other_transform->position, other->normal, manifold);
}

void Box::draw(Transform *transform)
{
    Graphics::quad(transform->position, half_extents, 0.0f, v4(0.5f, 0.0f, 1.0f, 1.0f));
}

bool Plane::collide(Transform *transform, Shape *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return other->collide(other_transform, this, transform, manifold);
}

bool Plane::collide(Transform *transform, Circle *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return circle_plane(other_transform->position, other->radius, transform->position, normal, manifold);
}

bool Plane::collide(Transform *transform, Box *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return box_plane(other_transform->position, other->half_extents, transform->position, normal, manifold);
}

bool Plane::collide(Transform *transform, Plane *other, Transform *other_transform, Collision::Manifold *manifold)
{
    return false;
}

void Plane::draw(Transform *transform)
{
    const float PLANE_SCALE = 1000.0f;
    float plane_rotation = angle_between(normal, v2(0.0f, 1.0f));
    if(dot(normal, v2(1.0f, 0.0f)) > 0.0f) plane_rotation *= -1.0f;
    Graphics::quad(transform->position - normal * PLANE_SCALE / 2.0f,
            v2(0.5f, 0.5f) * PLANE_SCALE,
            plane_rotation,
            v4(0.4f, 0.4f, 0.0f, 1.0f));
}

void Body::draw()
{
    if(shape)
    {
        shape->draw(&transform);
    }
}

