
#include "collision_detection.h"
#include "graphics.h"
#include <math.h>


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








void LevelCollisionDetection::init()
{
    Graphics::Camera::width() = 8.0f;

}

void LevelCollisionDetection::step(float time_step)
{
    v2 mouse_pos = Graphics::mouse_world_position();

    int scenario = 0;

    // Box box
    if(scenario == 0)
    {
        static float r = 0.0f;
        r += 0.1f * time_step;
        Box b1{v2(), v2(0.5f, 0.5f), r};
        Box b2{v2(), v2(1.5f, 1.5f), 0.0f};
        b1.center = mouse_pos;

        v4 color = Color::BLUE;

        Collision c = {};
        bool colliding = box_box(&b1, &b2, &c);
        if(colliding)
        {
            color = Color::YELLOW;

            Graphics::arrow(c.a_in_b, c.b_in_a, 0.01f, Color::BLUE);

//            v2 c_normal = c.b_in_a - c.a_in_b;
//            b1.center += c_normal / 2.0f;
//            b2.center -= c_normal / 2.0f;
        }

        Graphics::quad(b1.center, b1.half_extents, b1.rotation, color);
        Graphics::quad(b2.center, b2.half_extents, b2.rotation, color);
    }

    // Box circle
    if(scenario == 1)
    {
        static float r = 0.0f;
        r += 0.1f * time_step;
        Box b1{mouse_pos, v2(0.5f, 0.5f), r};
        Circle c1{v2(), 0.5f};

        v4 color = Color::BLUE;

        Collision c = {};
        bool colliding = box_circle(&b1, &c1, &c);
        if(colliding)
        {
            color = Color::YELLOW;
            Graphics::arrow(c.a_in_b, c.b_in_a, 0.01f, Color::BLUE);

//            v2 c_normal = c.b_in_a - c.a_in_b;
//            b1.center += c_normal / 2.0f;
//            c1.center -= c_normal / 2.0f;
        }

        Graphics::quad(b1.center, b1.half_extents, b1.rotation, color);
        Graphics::circle(c1.center, c1.radius, color);
    }

    // Circle circle
    if(scenario == 2)
    {
        v4 color = Color::BLUE;

        Circle c1{mouse_pos, 0.5f};
        Circle c2{v2(), 0.5f};

        Collision c = {};
        bool colliding = circle_circle(&c1, &c2, &c);
        if(colliding)
        {
            color = Color::YELLOW;
            Graphics::arrow(c.a_in_b, c.b_in_a, 0.01f, Color::BLUE);

//            v2 c_normal = c.b_in_a - c.a_in_b;
//            c1.center += c_normal / 2.0f;
//            c2.center -= c_normal / 2.0f;
        }

        Graphics::circle(c1.center, c1.radius, color);
        Graphics::circle(c2.center, c2.radius, color);
    }

}


