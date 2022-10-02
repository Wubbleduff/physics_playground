
#include "sandbox/sandbox.h"
#include "graphics.h"

#include "imgui.h"

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

#include <windows.h>

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

    bool soa_box_circle(
       float box_x, float box_y, float box_hw, float box_hh, float box_rot,
       float circle_x, float circle_y, float circle_r,
       SoaCollision *collision)
    {
        collision->body_containing_reference_edge = false;

        // Rotate the system so it's AABB vs circle collision where the box is at the origin.

        //circle_center -= box_center;
        //circle_center = rotate_vector(circle_center, -box->rotation);
        float t_circle_x = (circle_x - box_x) * cosf(-box_rot) - (circle_y - box_y) * sinf(-box_rot);
        float t_circle_y = (circle_x - box_x) * sinf(-box_rot) + (circle_y - box_y) * cosf(-box_rot);
        
        float clamped_x = GameMath::clamp(t_circle_x, -box_hw, box_hw);
        float clamped_y = GameMath::clamp(t_circle_y, -box_hh, box_hh);

        float diff_x = t_circle_x - clamped_x;
        float diff_y = t_circle_y - clamped_y;
        //float ll = diff_x*diff_x + diff_y*diff_y;
        float l = sqrtf(diff_x * diff_x + diff_y * diff_y);
        //if(ll < circle_r*circle_r)
        if(l < circle_r)
        {
            // Collision
            if(EQ(t_circle_x, clamped_x) && EQ(t_circle_y, clamped_y))
            {
                // Center in box
                float c_min_x = t_circle_x - circle_r;
                float c_min_y = t_circle_y - circle_r;
                float c_max_x = t_circle_x + circle_r;
                float c_max_y = t_circle_y + circle_r;
                
                float left_diff = c_max_x - (-box_hw);
                float right_diff = box_hw - c_min_x;
                float bottom_diff = c_max_y - (-box_hh);
                float top_diff = box_hh - c_min_y;
                float min_diff = min(left_diff, min(right_diff, min(bottom_diff, top_diff)));
                
                float n_x;
                float n_y;
                if(min_diff == left_diff)       { n_x=-1.0f; n_y= 0.0f; }
                else if(min_diff == right_diff) { n_x= 1.0f; n_y= 0.0f; }
                else if(min_diff == top_diff)   { n_x= 0.0f; n_y= 1.0f; }
                else                            { n_x= 0.0f; n_y=-1.0f; }
                
                // Un-transform the system.
                collision->num_contacts = 1;
                /*
                collision->contacts[0].position = circle_center + normal * (min_diff - circle->radius);
                collision->contacts[0].position = rotate_vector(collision->contacts[0].position, box->rotation);
                collision->contacts[0].position += box->center;
                collision->contacts[0].normal = rotate_vector(normal, box->rotation);
                collision->contacts[0].pen = min_diff;
                */
                float contact_x = t_circle_x + n_x * (min_diff - circle_r);
                float contact_y = t_circle_y + n_y * (min_diff - circle_r);

                float tmp_contact_x = contact_x * cosf(box_rot) - contact_y * sinf(box_rot);
                float tmp_contact_y = contact_x * sinf(box_rot) + contact_y * cosf(box_rot);
                contact_x = tmp_contact_x;
                contact_y = tmp_contact_y;
                contact_x += box_x;
                contact_y += box_y;
                collision->contacts[0].position.x = contact_x;
                collision->contacts[0].position.y = contact_y;

                float tmp_n_x = n_x * cosf(box_rot) - n_y * sinf(box_rot);
                float tmp_n_y = n_x * sinf(box_rot) + n_y * cosf(box_rot);
                n_x = tmp_n_x;
                n_y = tmp_n_y;
                collision->contacts[0].normal.x = n_x;
                collision->contacts[0].normal.y = n_y;

                collision->contacts[0].pen = min_diff;
                
                return true;
            }
            else
            {
                // Center outside of box
                collision->num_contacts = 1;
                /*
                collision->contacts[0].position = clamped;
                collision->contacts[0].position = rotate_vector(collision->contacts[0].position, box->rotation);
                collision->contacts[0].position += box->center;
                collision->contacts[0].normal = normalize(circle->center - collision->contacts[0].position);
                collision->contacts[0].pen = circle->radius - length(circle->center - collision->contacts[0].position);
                */
                float contact_x = clamped_x;
                float contact_y = clamped_y;
                float tmp_contact_x = contact_x * cosf(box_rot) - contact_y * sinf(box_rot);
                float tmp_contact_y = contact_x * sinf(box_rot) + contact_y * cosf(box_rot);
                contact_x = tmp_contact_x;
                contact_y = tmp_contact_y;
                contact_x += box_x;
                contact_y += box_y;
                collision->contacts[0].position.x = contact_x;
                collision->contacts[0].position.y = contact_y;

                //float l = sqrtf(ll);
                l = sqrtf((circle_x - contact_x) * (circle_x - contact_x) + (circle_y - contact_y) * (circle_y - contact_y));
                collision->contacts[0].normal.x = (circle_x - contact_x) / l;
                collision->contacts[0].normal.y = (circle_y - contact_y) / l;

                collision->contacts[0].pen = circle_r - l;
                
                return true;
            }
        }
        
        return false;
    }
    
#if 0
    // Baseline 36 ms
    bool circle_circle(Circle *a, Circle *b, Collision *collision)
    {
        collision->body_containing_reference_edge = false;

        v2 diff = b->center - a->center;
        float l = length(diff);
        if((l > a->radius + b->radius) || EQ(l, a->radius + b->radius))
        {
            // No collision.
            return false;
        }
        else 
        {

            // Collision.
            v2 n = v2(1.0f, 0.0f);
            if(EQ(a->center, b->center))
            {
                // Circles are exactly on top of each other. Use fake collision normal.
            }
            else
            {
                n = normalize(diff);
            }
            

            collision->num_contacts = 1;
            collision->contacts[0].normal = n;
            collision->contacts[0].pen = a->radius + b->radius - l;
            collision->contacts[0].position = a->center + n * a->radius;
            
            return true;
        }
    }
#else
    // 12 ms
    bool circle_circle(Circle *a, Circle *b, Collision *collision)
    {
        collision->body_containing_reference_edge = false;

        v2 diff = b->center - a->center;
        float l = sqrtf(diff.x*diff.x + diff.y*diff.y);
        float radius_sum = a->radius + b->radius;
        float pen = radius_sum - l;
        if(pen < 0)
        {
            // No collision.
            return false;
        }
        else 
        {

            // Collision.
            v2 n = v2(1.0f, 0.0f);
            if(EQ(a->center, b->center))
            {
                // Circles are exactly on top of each other. Use fake collision normal.
            }
            else
            {
                n = diff / l;
            }

            collision->num_contacts = 1;
            collision->contacts[0].normal = n;
            collision->contacts[0].pen = pen;
            collision->contacts[0].position = a->center + n * a->radius;
            
            return true;
        }
    }
#endif

#if 1
    bool soa_circle_circle(
        float a_x, float a_y, float a_r, 
        float b_x, float b_y, float b_r, 
        SoaCollision *collision)

    {
        float diff_x = b_x - a_x;
        float diff_y = b_y - a_y;
        float l = sqrtf(diff_x*diff_x + diff_y*diff_y);
        float radius_sum = a_r + b_r;
        float pen = radius_sum - l;
        if(pen < 0)
        {
            // No collision.
            return false;
        }
        else 
        {

            // Collision.
            float n_x = 1.0f;
            float n_y = 0.0f;
            if(l < MIN_SEPARATION_THRESHOLD)
            {
                // Circles are exactly on top of each other. Use fake collision normal.
            }
            else
            {
                n_x = diff_x / l;
                n_y = diff_y / l;
            }

            collision->num_contacts = 1;
            collision->contacts[0].normal.x = n_x;
            collision->contacts[0].normal.y = n_y;
            collision->contacts[0].pen = pen;
            collision->contacts[0].position.x = a_x + n_x * a_r;
            collision->contacts[0].position.y = a_y + n_y * a_r;
            
            return true;
        }
    }
#endif
    
    static void update_body(Kinematics *kinematics, v2 *position, float *rotation, float time_step)
    {
        if(kinematics->is_static) return;
        
        kinematics->velocity.y += -9.81f * time_step;
        *position += kinematics->velocity * time_step;
        *rotation += kinematics->angular_velocity * time_step;
    }

    void LevelSandbox::init()
    {
        box_list.kinematics.clear();
        box_list.geometry.clear();
        circle_list.kinematics.clear();
        circle_list.geometry.clear();
        soa_circle_list.num_circles = 0;

        Graphics::Camera::position() = v2(0.0f, 0.0f);
        Graphics::Camera::width() = 16.0f;
        
        //time_t t;
        //seed_random((unsigned)time(&t));
        seed_random(2);
#if 0
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
        int num_circles = 1000;
        //int num_circles = 10;
        for(int i = 0; i < num_circles; i++)
        {
            v2 position = v2(random_range(-2.0f, 2.0f), random_range(-2.0f, 2.0f));
            float r = random_range(0.05f, 0.1f);
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
            make_body_soa_circle(
                             Kinematics { velocity, angular_velocity, v2(), 0.0f, inv_mass, inv_moment_of_inertia, false, },
                             Circle{ position, r, 0.0f }
                             );
        }
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

        make_body_soa_box(
                      Kinematics { v2(0.0f, 0.0f), 0.0f, v2(), 0.0f, 0.0f, 0.0f, true, },
                      Box { v2(0.0f, -14.0f), v2(10.0f, 10.0f), 0.0f }
                      );
        make_body_soa_box(
                      Kinematics { v2(0.0f, 0.0f), 0.0f, v2(), 0.0f, 0.0f, 0.0f, true, },
                      Box { v2(0.0f, 14.0f), v2(10.0f, 10.0f), 0.0f }
                      );
        make_body_soa_box(
                      Kinematics { v2(0.0f, 0.0f), 0.0f, v2(), 0.0f, 0.0f, 0.0f, true, },
                      Box { v2(-17.0f, 0.0f), v2(10.0f, 10.0f), 0.0f }
                      );
        make_body_soa_box(
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

        static bool use_soa = false;
        ImGui::Checkbox("use soa", &use_soa);
        //if(use_soa)
        {
            aos_step(time_step);
        }
        //else
        {
            soa_step(time_step);
        }

#if 1
        for(int c1i = 0; c1i < circle_list.kinematics.size(); c1i++)
        {
            const Kinematics &c1k = circle_list.kinematics[c1i];
            const Circle &c1g = circle_list.geometry[c1i];

            assert(c1k.velocity.x == soa_circle_list.vx[c1i]);
            assert(c1k.velocity.y == soa_circle_list.vy[c1i]);
            assert(c1k.angular_velocity == soa_circle_list.av[c1i]);
            assert(c1k.inv_mass == soa_circle_list.inv_mass[c1i]);
            assert(c1k.inv_moment_of_inertia == soa_circle_list.inv_moment_of_inertia[c1i]);

            assert(c1g.center.x == soa_circle_list.px[c1i]);
            assert(c1g.center.y == soa_circle_list.py[c1i]);
            assert(c1g.radius == soa_circle_list.r[c1i]);
            assert(c1g.rotation == soa_circle_list.rotation[c1i]);
        }
#endif
    }

    
    void LevelSandbox::aos_step(float time_step)
    {

        static float time_scale = 1.0f;
        ImGui::InputFloat("Time scale", &time_scale, 0.1f);
        time_step *= time_scale;

        int step_count = 1;
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
                    
                    Collision c = {};

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
                    Collision c = {};
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
            

            LARGE_INTEGER start_circle_circle;
            QueryPerformanceCounter(&start_circle_circle);

            for(int circle1i = 0; circle1i < circle_list.geometry.size(); circle1i++)
            {
                for(int circle2i = circle1i + 1; circle2i < circle_list.geometry.size(); circle2i++)
                {
                    Circle *circle1 = &(circle_list.geometry[circle1i]);
                    Circle *circle2 = &(circle_list.geometry[circle2i]);
                    Collision c = {};
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
            
            LARGE_INTEGER end_circle_circle;
            QueryPerformanceCounter(&end_circle_circle);

            uint64_t duration_circle_circle = end_circle_circle.QuadPart - start_circle_circle.QuadPart;

            LARGE_INTEGER freq;
            QueryPerformanceFrequency(&freq);
            ImGui::Text("circle circle ms %f", ((double)duration_circle_circle / freq.QuadPart) * 1000.0);

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

    void LevelSandbox::soa_step(float time_step)
    {
        int step_count = 1;
        for(int step_i = 0; step_i < step_count; step_i++)
        {
            float sub_time_step = time_step / step_count;

            // SOA
            std::vector<SoaCollision> collisions;
            for(int i = 0; i < soa_circle_list.num_circles; i++)
            {
                soa_circle_list.vy[i] += -9.81f * sub_time_step;
                soa_circle_list.px[i] += soa_circle_list.vx[i] * sub_time_step;
                soa_circle_list.py[i] += soa_circle_list.vy[i] * sub_time_step;
                soa_circle_list.rotation[i] += soa_circle_list.av[i] * sub_time_step;
            }


            for(int i1 = 0; i1 < soa_box_list.num_boxes; i1++)
            {
                for(int i2 = 0; i2 < soa_circle_list.num_circles; i2++)
                {
                    SoaCollision c = {};
                    bool collided = soa_box_circle(
                        soa_box_list.px[i1], soa_box_list.py[i1], soa_box_list.hw[i1], soa_box_list.hh[i1], soa_box_list.rotation[i1],
                        soa_circle_list.px[i2], soa_circle_list.py[i2], soa_circle_list.r[i2], 
                        &c);
                    if(collided)
                    {
                        c.a_index = i1;
                        c.a_is_circle = false;
                        c.b_index = i2;
                        c.b_is_circle = true;
                        collisions.push_back(c);
                    }

#if 0
                    Collision check_collision = {};
                    Box cb1 = {v2(soa_box_list.px[i1], soa_box_list.py[i1]), v2(soa_box_list.hw[i1], soa_box_list.hh[i1]), soa_box_list.rotation[i1]};
                    Circle cc2 = {v2(soa_circle_list.px[i2], soa_circle_list.py[i2]), soa_circle_list.r[i2], 0.0f};
                    bool check_collided = box_circle(&cb1, &cc2, &check_collision);
                    assert(check_collided == collided);
                    assert(check_collision.num_contacts == c.num_contacts);
                    for(int ic = 0; ic < check_collision.num_contacts; ic++)
                    {
                        assert(check_collision.contacts[ic].pen == c.contacts[ic].pen);
                        assert(check_collision.contacts[ic].normal == c.contacts[ic].normal);
                        assert(check_collision.contacts[ic].position == c.contacts[ic].position);

                        check_collided = box_circle(&cb1, &cc2, &check_collision);
                        collided = soa_box_circle(
                            soa_box_list.px[i1], soa_box_list.py[i1], soa_box_list.hw[i1], soa_box_list.hh[i1], soa_box_list.rotation[i1],
                            soa_circle_list.px[i2], soa_circle_list.py[i2], soa_circle_list.r[i2],
                            &c);
                    }
#endif
                }
            }

            LARGE_INTEGER start_soa_circle_circle;
            QueryPerformanceCounter(&start_soa_circle_circle);
            for(int i1 = 0; i1 < soa_circle_list.num_circles; i1++)
            {
                for(int i2 = i1 + 1; i2 < soa_circle_list.num_circles; i2++)
                {
                    SoaCollision c = {};
                    bool collided = soa_circle_circle(
                        soa_circle_list.px[i1], soa_circle_list.py[i1], soa_circle_list.r[i1], 
                        soa_circle_list.px[i2], soa_circle_list.py[i2], soa_circle_list.r[i2], 
                        &c);
                    if(collided)
                    {
                        c.a_index = i1;
                        c.a_is_circle = true;
                        c.b_index = i2;
                        c.b_is_circle = true;
                        collisions.push_back(c);
                    }

#if 0
                    Collision check_collision = {};
                    Circle cc1 = {v2(soa_circle_list.px[i1], soa_circle_list.py[i1]), soa_circle_list.r[i1], 0.0f};
                    Circle cc2 = {v2(soa_circle_list.px[i2], soa_circle_list.py[i2]), soa_circle_list.r[i2], 0.0f};
                    bool check_collided = circle_circle(&cc1, &cc2, &check_collision);
                    assert(check_collided == collided);
                    assert(check_collision.num_contacts == c.num_contacts);
                    for(int ic = 0; ic < check_collision.num_contacts; ic++)
                    {
                        assert(check_collision.contacts[ic].pen == c.contacts[ic].pen);
                        assert(check_collision.contacts[ic].normal == c.contacts[ic].normal);
                        assert(check_collision.contacts[ic].position == c.contacts[ic].position);
                    }
#endif
                }
            }
            LARGE_INTEGER end_soa_circle_circle;
            QueryPerformanceCounter(&end_soa_circle_circle);
            uint64_t duration_soa_circle_circle = end_soa_circle_circle.QuadPart - start_soa_circle_circle.QuadPart;
            LARGE_INTEGER freq;
            QueryPerformanceFrequency(&freq);
            ImGui::Text("soa circle circle ms %f", ((double)duration_soa_circle_circle / freq.QuadPart) * 1000.0);

            for(int collision_index = 0; collision_index < collisions.size(); collision_index++)
            {
                SoaCollision &collision = collisions[collision_index];

#if 1
                float *a_vx = (collision.a_is_circle) ? (soa_circle_list.vx + collision.a_index) : (soa_box_list.vx + collision.a_index);
                float *a_vy = (collision.a_is_circle) ? (soa_circle_list.vy + collision.a_index) : (soa_box_list.vy + collision.a_index);
                float *a_av = (collision.a_is_circle) ? (soa_circle_list.av + collision.a_index) : (soa_box_list.av + collision.a_index);
                float *a_inv_mass = (collision.a_is_circle) ? (soa_circle_list.inv_mass + collision.a_index) : (soa_box_list.inv_mass + collision.a_index);
                float *a_inv_moment_of_inertia = (collision.a_is_circle) ? (soa_circle_list.inv_moment_of_inertia + collision.a_index) : (soa_box_list.inv_moment_of_inertia + collision.a_index);
                float *a_px = (collision.a_is_circle) ? (soa_circle_list.px + collision.a_index) : (soa_box_list.px + collision.a_index);
                float *a_py = (collision.a_is_circle) ? (soa_circle_list.py + collision.a_index) : (soa_box_list.py + collision.a_index);

                float *b_vx = (collision.b_is_circle) ? (soa_circle_list.vx + collision.b_index) : (soa_box_list.vx + collision.b_index);
                float *b_vy = (collision.b_is_circle) ? (soa_circle_list.vy + collision.b_index) : (soa_box_list.vy + collision.b_index);
                float *b_av = (collision.b_is_circle) ? (soa_circle_list.av + collision.b_index) : (soa_box_list.av + collision.b_index);
                float *b_inv_mass = (collision.b_is_circle) ? (soa_circle_list.inv_mass + collision.b_index) : (soa_box_list.inv_mass + collision.b_index);
                float *b_inv_moment_of_inertia = (collision.b_is_circle) ? (soa_circle_list.inv_moment_of_inertia + collision.b_index) : (soa_box_list.inv_moment_of_inertia + collision.b_index);
                float *b_px = (collision.b_is_circle) ? (soa_circle_list.px + collision.b_index) : (soa_box_list.px + collision.b_index);
                float *b_py = (collision.b_is_circle) ? (soa_circle_list.py + collision.b_index) : (soa_box_list.py + collision.b_index);
#endif

                if(collision.body_containing_reference_edge)
                {
                    std::swap(a_vx, b_vx);
                    std::swap(a_vy, b_vy);
                    std::swap(a_av, b_av);
                    std::swap(a_inv_mass, b_inv_mass);
                    std::swap(a_inv_moment_of_inertia, b_inv_moment_of_inertia);
                    std::swap(a_px, b_px);
                    std::swap(a_py, b_py);
                }

                for(int contact_index = 0; contact_index < collision.num_contacts; contact_index++)
                {
                    float pen = collision.contacts[contact_index].pen;
                    assert(!EQ(collision.contacts[contact_index].normal, v2()));
                    v2 n = normalize(collision.contacts[contact_index].normal);
                    v2 contact_position = collision.contacts[contact_index].position;

                    v2 ra = contact_position - v2(*a_px, *a_py);
                    v2 rb = contact_position - v2(*b_px, *b_py);
                    v2 relative_velocity = -(v2(*a_vx, *a_vy) + cross(*a_av, ra)) + (v2(*b_vx, *b_vy) + cross(*b_av, rb));

                    float vn = dot(n, relative_velocity);

                    static const float beta_coeff = -0.4f;
                    static const float minimum_pen = 0.01f;

                    float beta = (beta_coeff / sub_time_step) * min(0.0f, -pen + minimum_pen);

                    float rna = dot(ra, n);
                    float rnb = dot(rb, n);
                    float k_normal = *a_inv_mass + *b_inv_mass;
                    k_normal += *a_inv_moment_of_inertia * (dot(ra, ra) - rna * rna) + *b_inv_moment_of_inertia * (dot(rb, rb) - rnb * rnb);
                    float effective_mass = 1.0f / k_normal;

                    float lambda = effective_mass * (-vn + beta);
                    lambda = max(lambda, 0.0f);
                    v2 impulse = lambda * n;

                    *a_vx += -(*a_inv_mass * impulse).x;
                    *a_vy += -(*a_inv_mass * impulse).y;
                    *a_av += -(*a_inv_moment_of_inertia * cross(ra, impulse));

                    *b_vx += (*b_inv_mass * impulse).x;
                    *b_vy += (*b_inv_mass * impulse).y;
                    *b_av += (*b_inv_moment_of_inertia * cross(rb, impulse));

                    // Friction
                    v2 tangent = v2(n.y, -n.x);
                    relative_velocity = -(v2(*a_vx, *a_vy) + cross(*a_av, ra)) + (v2(*b_vx, *b_vy) + cross(*b_av, rb));

                    float vt = dot(tangent, relative_velocity);

                    float rta = dot(ra, tangent);
                    float rtb = dot(rb, tangent);
                    float k_tangent = *a_inv_mass + *b_inv_mass;
                    k_tangent += *a_inv_moment_of_inertia * (dot(ra, ra) - rta * rta) + *b_inv_moment_of_inertia * (dot(rb, rb) - rtb * rtb);
                    float mass_tangent = 1.0f / k_tangent;
                    float dpt = mass_tangent * -vt;

                    const static float friction = 0.1f;
                    float maxPt = friction * lambda;
                    dpt = clamp(dpt, -maxPt, maxPt);
                    v2 friction_impulse = dpt * tangent;

                    *a_vx += -(*a_inv_mass * friction_impulse).x;
                    *a_vy += -(*a_inv_mass * friction_impulse).y;
                    *a_av += -(*a_inv_moment_of_inertia * cross(ra, friction_impulse));

                    *b_vx += (*b_inv_mass * friction_impulse).x;
                    *b_vy += (*b_inv_mass * friction_impulse).y;
                    *b_av += (*b_inv_moment_of_inertia * cross(rb, friction_impulse));
                }
            }
        }


        for(int i = 0; i < soa_box_list.num_boxes; i++)
        {
            v2 center = v2(soa_box_list.px[i], soa_box_list.py[i]);
            v2 he = v2(soa_box_list.hw[i], soa_box_list.hh[i]);
            float rotation = soa_box_list.rotation[i];
            Graphics::quad_outline(center, he, rotation, Color::BLUE, 0.01f);
            //Graphics::arrow(center, center + rotate_vector(v2(1.0f, 0.0f) * r, rotation), 0.01f, v4(Color::RED));
        }

        for(int i = 0; i < soa_circle_list.num_circles; i++)
        {
            v2 center = v2(soa_circle_list.px[i], soa_circle_list.py[i]);
            float r = soa_circle_list.r[i];
            float rotation = soa_circle_list.rotation[i];
            Graphics::circle_outline(center, r, Color::GREEN, 0.01f);
            Graphics::arrow(center, center + rotate_vector(v2(1.0f, 0.0f) * r, rotation), 0.01f, v4(Color::RED));
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

    void LevelSandbox::make_body_soa_box(const Kinematics &kinematics, const Box &shape)
    {
        assert(soa_box_list.num_boxes <= SoaBoxList::MAX_BOXES);
        uint32_t &n = soa_box_list.num_boxes;
        soa_box_list.vx[n] = kinematics.velocity.x;
        soa_box_list.vy[n] = kinematics.velocity.y;
        soa_box_list.av[n] = kinematics.angular_velocity;
        soa_box_list.inv_mass[n] = kinematics.inv_mass;
        soa_box_list.inv_moment_of_inertia[n] = kinematics.inv_moment_of_inertia;
        soa_box_list.px[n] = shape.center.x;
        soa_box_list.py[n] = shape.center.y;
        soa_box_list.hw[n] = shape.half_extents.x;
        soa_box_list.hh[n] = shape.half_extents.y;
        soa_box_list.rotation[n] = shape.rotation;
        n++;
    }

    void LevelSandbox::make_body_soa_circle(const Kinematics &kinematics, const Circle &shape)
    {
        assert(soa_circle_list.num_circles <= SoaCircleList::MAX_CIRCLES);
        uint32_t &n = soa_circle_list.num_circles;
        soa_circle_list.vx[n] = kinematics.velocity.x;
        soa_circle_list.vy[n] = kinematics.velocity.y;
        soa_circle_list.av[n] = kinematics.angular_velocity;
        soa_circle_list.inv_mass[n] = kinematics.inv_mass;
        soa_circle_list.inv_moment_of_inertia[n] = kinematics.inv_moment_of_inertia;
        soa_circle_list.px[n] = shape.center.x;
        soa_circle_list.py[n] = shape.center.y;
        soa_circle_list.r[n] = shape.radius;
        soa_circle_list.rotation[n] = shape.rotation;
        n++;
    }
}

