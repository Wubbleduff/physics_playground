
#include "sandbox/sandbox.h"
#include "graphics.h"

#include "imgui.h"

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

using namespace GameMath;

namespace Sandbox
{
    static float MIN_SEPARATION_THRESHOLD = 0.001f;
    static bool EQ(float a, float b)
    {
        return (GameMath::abs(a - b) < MIN_SEPARATION_THRESHOLD);
    }
    static bool EQ(v2 a, v2 b)
    {
        return (GameMath::abs(a.x - b.x) < MIN_SEPARATION_THRESHOLD) && (GameMath::abs(a.y - b.y) < MIN_SEPARATION_THRESHOLD);
    }
    
    inline v2 cross(float s, v2 a)
    {
        return v2(-s * a.y, s * a.x);
    }

    inline float cross(v2 a, v2 b)
    {
        return a.x * b.y - a.y * b.x;
    }

    // n assumed to be normalized.
    v2 clip_line_to_plane(v2 v0, v2 v1, v2 p, v2 n)
    {
        float v0n = dot(v0 - p, n);
        float v1n = dot(v1 - p, n);
        if(v0n > 0.0f && v1n > 0.0f)
        {
            return (v0n < v1n) ? v0 : v1;
        }
        if(v0n < 0.0f && v1n < 0.0f)
        {
            return (v0n > v1n) ? v0 : v1;
        }

        float ratio = v0n / (v0n + (-v1n));
        return v0 + (v1 - v0) * ratio;
    }
    
    bool box_box(Box *a, Box *b, Collision *collision)
    {
        v2 vertices[] =
        {
            a->center - rotate_vector(a->half_extents, a->rotation), // bl
            a->center + rotate_vector(v2(a->half_extents.x, -a->half_extents.y), a->rotation), // br
            a->center + rotate_vector(a->half_extents, a->rotation), // tr
            a->center + rotate_vector(v2(-a->half_extents.x, a->half_extents.y), a->rotation), // tl

            b->center - rotate_vector(b->half_extents, b->rotation), // bl
            b->center + rotate_vector(v2(b->half_extents.x, -b->half_extents.y), b->rotation), // br
            b->center + rotate_vector(b->half_extents, b->rotation), // tr
            b->center + rotate_vector(v2(-b->half_extents.x, b->half_extents.y), b->rotation), // tl
        };
        v2 *v_start = &(vertices[0]);
        
        v2i reference_edge; // Edge with minimum separating axis.
        v2i incident_edge;  // Edge with potential contact vertices.
        // Which body contain the reference edge. Necessary for determining the orientation of the collision.
        // false: A, true: B
        bool body_containing_reference_edge = false;
        float min_pen = INFINITY;
        
        // Check A against B and B against A.
        v2 *index_pairs[2][2] = {{v_start, v_start + 4}, {v_start + 4, v_start}};
        for(int pair_index = 0; pair_index < 2; pair_index++)
        {
            v2 *i_vertices = index_pairs[pair_index][0];
            v2 *j_vertices = index_pairs[pair_index][1];
            
            // Loop through edges of the first shape.
            for(int i = 0; i < 4; i++)
            {
                v2 i_n = -find_ccw_normal(i_vertices[(i + 1) % 4] - i_vertices[i]);
                
                // Find the maximum penetration of the second shape's vertices. We can
                // only resolve the collision if we use the vertex that's deepest in the
                // first shape's edges.
                float max_j_pen = 0.0f;
                int max_j_pen_index = 0;
                for(int j = 0; j < 4; j++)
                {
                    float j_pen = -dot(i_n, j_vertices[j] - i_vertices[i]);
                    if(j_pen > max_j_pen)
                    {
                        max_j_pen = j_pen;
                        max_j_pen_index = j;
                    }
                }
                
                // Check if this separation is the best (minimum) separation so far.
                float pen = -signed_distance_to_plane(j_vertices[max_j_pen_index], i_vertices[i], i_n);

                // Can early out here. Negative penetration between 2 boxes necessarily means they're not colliding.
                if(pen < 0.0f)
                {
                    return false;
                }

                if(pen < min_pen)
                {
                    reference_edge = {(int)(&(i_vertices[i]) - v_start), (int)(&(i_vertices[(i + 1) % 4]) - v_start)};
                    body_containing_reference_edge = (bool)pair_index;
                    int j = max_j_pen_index;
                    int j_prev = (j == 0) ? 3 : j - 1;
                    int j_next = (j + 1) % 4;

                    float j_prev_d = dot(j_vertices[j_prev] - i_vertices[i], i_n);
                    float j_next_d = dot(j_vertices[j_next] - i_vertices[i], i_n);
                    if(j_prev_d < j_next_d)
                    {
                        incident_edge = {(int)(&(j_vertices[j_prev]) - v_start), (int)(&(j_vertices[j]) - v_start)};
                    }
                    else
                    {
                        incident_edge = {(int)(&(j_vertices[j]) - v_start), (int)(&(j_vertices[j_next]) - v_start)};
                    }

                    min_pen = pen;
                }
            }
        }

        v2 normal = find_ccw_normal(vertices[reference_edge[0]] - vertices[reference_edge[1]]);

        v2 c0 = clip_line_to_plane(vertices[incident_edge[0]], vertices[incident_edge[1]], vertices[reference_edge[0]], normalize(find_ccw_normal(normal)));
        v2 c1 = clip_line_to_plane(vertices[incident_edge[1]], vertices[incident_edge[0]], vertices[reference_edge[1]], normalize(-find_ccw_normal(normal)));

        collision->num_contacts = 0;
        float c0_pen = -signed_distance_to_plane(c0, vertices[reference_edge[0]], normal);
        float c1_pen = -signed_distance_to_plane(c1, vertices[reference_edge[0]], normal);
        if(c0_pen > 0.0f)
        {
            collision->contacts[collision->num_contacts].position = c0;
            collision->contacts[collision->num_contacts].pen = c0_pen;
            collision->contacts[collision->num_contacts].normal = normal;
            collision->num_contacts++;
        }
        if(c1_pen > 0.0f)
        {
            collision->contacts[collision->num_contacts].position = c1;
            collision->contacts[collision->num_contacts].pen = c1_pen;
            collision->contacts[collision->num_contacts].normal = normal;
            collision->num_contacts++;
        }

        collision->body_containing_reference_edge = body_containing_reference_edge;

        return true;
    }
    
    bool box_circle(Box *box, Circle *circle, Collision *collision)
    {
        // Rotate the system so it's AABB vs circle collision where the box is at the origin.
        v2 box_center = v2();
        v2 circle_center = circle->center;
        circle_center -= box->center;
        circle_center = rotate_vector(circle_center, -box->rotation);
        
        v2 b_min = box_center - box->half_extents;
        v2 b_max = box_center + box->half_extents;
        
        v2 clamped = v2(
                        clamp(circle_center.x, b_min.x, b_max.x),
                        clamp(circle_center.y, b_min.y, b_max.y));

        collision->body_containing_reference_edge = false;

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
                
                collision->num_contacts = 1;
                collision->contacts[0].position = circle_center + normal * (min_diff - circle->radius);
                
                // Un-rotate system
                collision->contacts[0].position = rotate_vector(collision->contacts[0].position, box->rotation);
                collision->contacts[0].normal = rotate_vector(normal, box->rotation);
                collision->contacts[0].pen = min_diff;
                collision->contacts[0].position += box->center;
                
                return true;
            }
            else
            {
                // Center outside of box
                collision->num_contacts = 1;
                collision->contacts[0].position = clamped;
                collision->contacts[0].position = rotate_vector(collision->contacts[0].position, box->rotation);
                collision->contacts[0].position += box->center;

                collision->contacts[0].normal = normalize(circle->center - collision->contacts[0].position);
                collision->contacts[0].pen = circle->radius - length(circle->center - collision->contacts[0].position);
                
                return true;
            }
        }
        
        return false;
    }
    
    bool circle_circle(Circle *a, Circle *b, Collision *collision)
    {
        v2 diff = b->center - a->center;
        float l = length(diff);
        collision->body_containing_reference_edge = false;
        if((l > a->radius + b->radius) || EQ(l, a->radius + b->radius))
        {
            // No collision.
            return false;
        }
        else 
        {

            // Collision.
            if(EQ(a->center, b->center))
            {
                // Circles are exactly on top of each other. Build a fake collision normal.
                diff = v2(1.0f, 0.0f);
            }
            
            v2 ndiff = normalize(diff);

            collision->num_contacts = 1;
            collision->contacts[0].normal = ndiff;
            collision->contacts[0].pen = a->radius + b->radius - l;
            collision->contacts[0].position = a->center + ndiff * a->radius;
            
            return true;
        }
    }
    
    static void update_body(Kinematics *kinematics, v2 *position, float *rotation, float time_step)
    {
        static const float gravity = 9.81f;

        if(kinematics->is_static) return;
        
        kinematics->velocity += (v2(0.0f, -gravity) + kinematics->acceleration_sum * kinematics->inv_mass) * time_step;
        *position += kinematics->velocity * time_step;
        
        kinematics->angular_velocity += kinematics->angular_acceleration_sum * kinematics->inv_moment_of_inertia * time_step;
        *rotation += kinematics->angular_velocity * time_step;
        
        kinematics->acceleration_sum = v2();
        kinematics->angular_acceleration_sum = 0.0f;
    }
    
    void LevelSandbox::init()
    {
        box_list.kinematics.clear();
        box_list.geometry.clear();
        circle_list.kinematics.clear();
        circle_list.geometry.clear();

        Graphics::Camera::position() = v2(0.0f, 0.0f);
        Graphics::Camera::width() = 16.0f;
        
        //time_t t;
        //seed_random((unsigned)time(&t));
        seed_random(2);
#if 1
        for(int i = 0; i < 100; i++)
        {
            v2 position = v2(random_range(-2.0f, 2.0f), random_range(-2.0f, 2.0f));
            v2 scale = v2(random_range(0.1f, 0.3f), random_range(0.1f, 0.3f));
            float mass = scale.x + scale.y;
            float moment_of_inertia = mass * (scale.x * scale.x + scale.y * scale.y) / 12.0f;
            float inv_mass = 1.0f / mass;
            float inv_moment_of_inertia = 1.0f / moment_of_inertia;
            
            //v2 velocity = v2(random_range(-10.0f, 10.0f), random_range(-10.0f, 10.0f));
            float angular_velocity = random_range(-1.0f, 1.0f);
            v2 velocity = v2();
            //float angular_velocity = 0.0f;
            make_body_box(
                          Kinematics { velocity, angular_velocity, v2(), 0.0f, inv_mass, inv_moment_of_inertia, false, },
                          Box { position, scale, 0.0f }
                          );
        }
#endif
        
#if 1
        int num_circles = 100;
        for(int i = 0; i < num_circles; i++)
        {
            v2 position = v2(random_range(-2.0f, 2.0f), random_range(-2.0f, 2.0f));
            float r = random_range(0.1f, 0.3f);
            float mass = PI * r*r;
            float moment_of_inertia = (PI * r*r*r*r) * (1.0f / 4.0f);
            float inv_mass = 1.0f/mass;
            float inv_moment_of_inertia= 1.0f/moment_of_inertia;

            //v2 velocity = v2(random_range(-10.0f, 10.0f), random_range(-10.0f, 10.0f));
            //float angular_velocity = random_range(-10.0f, 10.0f);
            v2 velocity = v2();
            float angular_velocity = 0.0f;
            make_body_circle(
                             Kinematics { velocity, angular_velocity, v2(), 0.0f, inv_mass, inv_moment_of_inertia, false, },
                             Circle{ position, r, 0.0f }
                             );
        }
#endif

// Rotate in to each other towards the outside
#if 0
        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), -1.0f, v2(), 0.0f, 1.0f, 1.0f, false, },
                      Box { v2( 0.25f, 1.0f), v2(2.0f, 0.25f), 0.0f }
                      );

        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), 0.0f, v2(), 0.0f, 1.0f, 1.0f, false, },
                      Box { v2(-0.25f, -1.0f), v2(2.0f, 0.25f), PI/2.0f }
                      );
#endif

// Rotate and hit center of mass
#if 0
        float w = 0.10f;
        float h = 2.0f;
        float av = -0.5f;
        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), av, v2(), 0.0f, 1.0f, 1.0f, false, },
                      Box { v2( 0.0f, 0.0f), v2(w, h), PI/2.0f }
                      );

        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), av, v2(), 0.0f, 1.0f, 1.0f, false, },
                      Box { v2(w + w, h + w), v2(w, h), PI/2.0f }
                      );
#endif

// box2d compare
#if 0
        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), 0.0f, v2(), 0.0f, 1.0f, 0.118811876f, false, },
                      Box { v2( 0.0f, 5.0f), v2(0.5f, 5.0f), 0.0f }
                      );

        make_body_box(
                      Kinematics { v2(1.0f, 0.0f), 0.0f, v2(), 0.0f, 1.0f, 6.0f, false, },
                      Box { v2(-5.0f, 10.4f), v2(0.5f, 0.5f), 0.0f }
                      );
#endif

// Box standing on corner
#if 0
        v2 scale = v2(0.1f, 0.3f);
        float mass = 0.1f;
        float moment_of_inertia = mass * (scale.x * scale.x + scale.y * scale.y) / 12.0f; // TODO: Scale may be double here?
        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), -2.85f, v2(), 0.0f, 1.0f/mass, 1.0f/moment_of_inertia, false, },
                      Box { v2( 2.0f, 0.0f), scale, 0.4f }
                      );
#endif

// Circle spinning on landing
#if 0
        make_body_circle(
                      Kinematics { v2(0.0f, 0.0f), 10.0f, v2(), 0.0f, 1.0f, 10.0f, false, },
                      Circle { v2(0.0f, 0.0f), 1.0f, 0.0f }
                      );
#endif
        
        
#if 1
        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), 0.0f, v2(), 0.0f, 0.0f, 0.0f, true, },
                      Box { v2(0.0f, -14.0f), v2(10.0f, 10.0f), 0.0f }
                      );
        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), 0.0f, v2(), 0.0f, 0.0f, 0.0f, true, },
                      Box { v2(0.0f, 14.0f), v2(10.0f, 10.0f), 0.0f }
                      );
        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), 0.0f, v2(), 0.0f, 0.0f, 0.0f, true, },
                      Box { v2(-17.0f, 0.0f), v2(10.0f, 10.0f), 0.0f }
                      );
        make_body_box(
                      Kinematics { v2(0.0f, 0.0f), 0.0f, v2(), 0.0f, 0.0f, 0.0f, true, },
                      Box { v2(17.0f, 0.0f), v2(10.0f, 10.0f), 0.0f }
                      );
#endif
    }
    
    void LevelSandbox::step(float time_step)
    {
        if(ImGui::Button("Reset"))
        {
            init();
            return;
        }

        static float time_scale = 1.0f;
        ImGui::InputFloat("Time scale", &time_scale, 0.1f);
        time_step *= time_scale;

        int step_count = 8;
        for(int step_i = 0; step_i < step_count; step_i++)
        {
            float sub_time_step = time_step / step_count;

            for(int i = 0; i < box_list.kinematics.size(); i++)
            {
                Kinematics &kinematics = box_list.kinematics[i];
                Box &geometry = box_list.geometry[i];
                update_body(&kinematics, &geometry.center, &geometry.rotation, sub_time_step);
            }
            for(int i = 0; i < circle_list.kinematics.size(); i++)
            {
                Kinematics &kinematics = circle_list.kinematics[i];
                Circle &geometry = circle_list.geometry[i];
                update_body(&kinematics, &geometry.center, &geometry.rotation, sub_time_step);
            }
            
            std::vector<Collision> collisions;
            for(int box1i = 0; box1i < box_list.geometry.size(); box1i++)
            {
                for(int box2i = box1i + 1; box2i < box_list.geometry.size(); box2i++)
                {
                    Box *box1 = &(box_list.geometry[box1i]);
                    Box *box2 = &(box_list.geometry[box2i]);
                    
                    // TODO
                    // Probably put static objects in their own list.
                    if(box_list.kinematics[box1i].is_static && box_list.kinematics[box2i].is_static)
                    {
                        continue;
                    }
                    
                    Collision c;

                    bool collided = box_box(box1, box2, &c);
                    if(collided)
                    {
                        c.a_kinematics = &(box_list.kinematics[box1i]);
                        c.b_kinematics = &(box_list.kinematics[box2i]);
                        c.a_center_of_mass = &(box_list.geometry[box1i].center);
                        c.b_center_of_mass = &(box_list.geometry[box2i].center);
                        collisions.push_back(c);
                    }
                }
            }
            
            for(int boxi = 0; boxi < box_list.geometry.size(); boxi++)
            {
                for(int circlei = 0; circlei < circle_list.geometry.size(); circlei++)
                {
                    Box *box = &(box_list.geometry[boxi]);
                    Circle *circle = &(circle_list.geometry[circlei]);
                    Collision c;
                    bool collided = box_circle(box, circle, &c);
                    if(collided)
                    {
                        c.a_kinematics = &(box_list.kinematics[boxi]);
                        c.b_kinematics = &(circle_list.kinematics[circlei]);
                        c.a_center_of_mass = &(box_list.geometry[boxi].center);
                        c.b_center_of_mass = &(circle_list.geometry[circlei].center);
                        collisions.push_back(c);
                    }
                }
            }
            
            for(int circle1i = 0; circle1i < circle_list.geometry.size(); circle1i++)
            {
                for(int circle2i = circle1i + 1; circle2i < circle_list.geometry.size(); circle2i++)
                {
                    Circle *circle1 = &(circle_list.geometry[circle1i]);
                    Circle *circle2 = &(circle_list.geometry[circle2i]);
                    Collision c;
                    bool collided = circle_circle(circle1, circle2, &c);
                    if(collided)
                    {
                        c.a_kinematics = &(circle_list.kinematics[circle1i]);
                        c.b_kinematics = &(circle_list.kinematics[circle2i]);
                        c.a_center_of_mass = &(circle_list.geometry[circle1i].center);
                        c.b_center_of_mass = &(circle_list.geometry[circle2i].center);
                        collisions.push_back(c);
                    }
                }
            }

            for(int collision_index = 0; collision_index < collisions.size(); collision_index++)
            {
                Collision &collision = collisions[collision_index];
                for(int contact_index = 0; contact_index < collision.num_contacts; contact_index++)
                {
                    v2 a_center_of_mass;
                    Kinematics *a;
                    v2 b_center_of_mass;
                    Kinematics *b;
                    if(collision.body_containing_reference_edge)
                    {
                        a_center_of_mass = *collision.b_center_of_mass;
                        a = collision.b_kinematics;
                        b_center_of_mass = *collision.a_center_of_mass;
                        b = collision.a_kinematics;
                    }
                    else
                    {
                        a_center_of_mass = *collision.a_center_of_mass;
                        a = collision.a_kinematics;
                        b_center_of_mass = *collision.b_center_of_mass;
                        b = collision.b_kinematics;
                    }

                    float pen = collision.contacts[contact_index].pen;
                    assert(!EQ(collision.contacts[contact_index].normal, v2()));
                    v2 n = normalize(collision.contacts[contact_index].normal);
                    v2 contact_position = collision.contacts[contact_index].position;

                    v2 ra = contact_position - a_center_of_mass;
                    v2 rb = contact_position - b_center_of_mass;
                    v2 relative_velocity = -(a->velocity + cross(a->angular_velocity, ra)) + (b->velocity + cross(b->angular_velocity, rb));

                    float vn = dot(n, relative_velocity);

                    static const float beta_coeff = -0.4f;
                    static const float minimum_pen = 0.01f;

                    float beta = (beta_coeff / sub_time_step) * min(0.0f, -pen + minimum_pen);

                    float rna = dot(ra, n);
                    float rnb = dot(rb, n);
                    float k_normal = a->inv_mass + b->inv_mass;
                    k_normal += a->inv_moment_of_inertia * (dot(ra, ra) - rna * rna) + b->inv_moment_of_inertia * (dot(rb, rb) - rnb * rnb);
                    float effective_mass = 1.0f / k_normal;

                    float lambda = effective_mass * (-vn + beta);
                    lambda = max(lambda, 0.0f);
                    v2 impulse = lambda * n;

                    a->velocity += -(a->inv_mass * impulse);
                    a->angular_velocity += -(a->inv_moment_of_inertia * cross(ra, impulse));
                    b->velocity += (b->inv_mass * impulse);
                    b->angular_velocity += (b->inv_moment_of_inertia * cross(rb, impulse));

                    // Friction
                    v2 tangent = v2(n.y, -n.x);
                    relative_velocity = -(a->velocity + cross(a->angular_velocity, ra)) + (b->velocity + cross(b->angular_velocity, rb));

                    float vt = dot(tangent, relative_velocity);

                    float rta = dot(ra, tangent);
                    float rtb = dot(rb, tangent);
                    float k_tangent = a->inv_mass + b->inv_mass;
                    k_tangent += a->inv_moment_of_inertia * (dot(ra, ra) - rta * rta) + b->inv_moment_of_inertia * (dot(rb, rb) - rtb * rtb);
                    float mass_tangent = 1.0f / k_tangent;
                    float dpt = mass_tangent * -vt;

                    const static float friction = 0.1f;
                    float maxPt = friction * lambda;
                    dpt = clamp(dpt, -maxPt, maxPt);
                    v2 friction_impulse = dpt * tangent;

                    a->velocity += -(a->inv_mass * friction_impulse);
                    a->angular_velocity += -(a->inv_moment_of_inertia * cross(ra, friction_impulse));
                    b->velocity += (b->inv_mass * friction_impulse);
                    b->angular_velocity += (b->inv_moment_of_inertia * cross(rb, friction_impulse));
                }
            }
            last_collisions = collisions;
        }

        for(int i = 0; i < box_list.geometry.size(); i++)
        {
            Box &geometry = box_list.geometry[i];
            Graphics::quad_outline(geometry.center, geometry.half_extents, geometry.rotation, Color::BLUE, 0.02f);
        }
        for(int i = 0; i < circle_list.geometry.size(); i++)
        {
            Circle &geometry = circle_list.geometry[i];
            Graphics::circle_outline(geometry.center, geometry.radius, Color::YELLOW, 0.01f);
            Graphics::arrow(geometry.center, geometry.center + rotate_vector(v2(1.0f, 0.0f) * geometry.radius, geometry.rotation), 0.01f, v4(Color::RED));
        }
    }
    
    void LevelSandbox::make_body_box(const Kinematics &kinematics, const Box &shape)
    {
        box_list.kinematics.emplace_back(kinematics);
        box_list.geometry.emplace_back(shape);
    }
    
    void LevelSandbox::make_body_circle(const Kinematics &kinematics, const Circle &shape)
    {
        circle_list.kinematics.emplace_back(kinematics);
        circle_list.geometry.emplace_back(shape);
    }
}

