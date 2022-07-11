
#include "sandbox/sandbox.h"
#include "graphics.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>


using namespace GameMath;



namespace Sandbox
{
    struct CollisionResponse
    {
        v2 a_vel = v2();
        v2 b_vel = v2();
        float a_angular_vel = 0.0f;
        float b_angular_vel = 0.0f;
        v2 a_pos = v2();
        v2 b_pos = v2();
    };
    
    static float EPSILON = 0.001f;
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
                float j_pen_depth = GameMath::abs(signed_distance_to_plane(j_vertices[max_pen_index], i_vertices[i], i_normal));
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
        
        // If the contact points are approximately equal, treat this as no collision.
        if(EQ(min_contact_points[0], min_contact_points[1]))
        {
            return false;
        }
        
        // Check for the orientation of the collision.
        if(resolve_as_a_in_b)
        {
            collision->a_in_b = min_contact_points[0];
            collision->b_in_a = min_contact_points[1];
            collision->normal = -normalize(min_separating_normal);
            collision->depth = length(min_separating_normal);
        }
        else
        {
            collision->b_in_a = min_contact_points[0];
            collision->a_in_b = min_contact_points[1];
            collision->normal = normalize(min_separating_normal);
            collision->depth = length(min_separating_normal);
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
                collision->normal = rotate_vector(normal, box->rotation);
                collision->depth = min_diff;
                collision->a_in_b += box->center;
                collision->b_in_a += box->center;
                
                if(EQ(collision->a_in_b, collision->b_in_a))
                {
                    // Contact points are exactly the same. Return no collision or else we'll have a malformed collision normal.
                    return false;
                }
                
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
                
                collision->normal = normalize(collision->a_in_b - collision->b_in_a);
                collision->depth = length(collision->a_in_b - collision->b_in_a);
                
                if(EQ(collision->a_in_b, collision->b_in_a))
                {
                    // Contact points are exactly the same. Return no collision or else we'll have a malformed collision normal.
                    return false;
                }
                
                return true;
            }
        }
        
        return false;
    }
    
    bool circle_circle(Circle *a, Circle *b, Collision *collision)
    {
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
            if(EQ(a->center, b->center))
            {
                // Circles are exactly on top of each other. Build a fake collision normal.
                diff = v2(1.0f, 0.0f);
            }
            
            v2 ndiff = normalize(diff);

            collision->normal = ndiff;
            collision->depth = a->radius + b->radius - l;
            collision->a_in_b = a->center + ndiff * a->radius;
            collision->b_in_a = b->center - ndiff * b->radius;
            
            return true;
        }
    }
    
    
    
    
    
    
    
    
    static void update_body(Kinematics *kinematics, v2 *position, float *rotation, float time_step)
    {
        static const float air_drag = 0.2f;
        static const float rotational_air_drag = 1.0f;
        static const float gravity = 9.81f;

        if(kinematics->is_static) return;
        
        v2 drag_force = 0.5f * -kinematics->velocity * air_drag;
        kinematics->apply_force(drag_force, v2());
        
        kinematics->apply_force(v2(0.0f, -gravity * (1.0f / kinematics->inv_mass)), v2());
        
        kinematics->velocity += kinematics->acceleration_sum * time_step;
        *position += kinematics->velocity * time_step;
        
        float angular_drag_force = -kinematics->angular_velocity * rotational_air_drag;
        kinematics->apply_torque(angular_drag_force);
        
        kinematics->angular_velocity += kinematics->angular_acceleration_sum * time_step;
        *rotation += kinematics->angular_velocity * time_step;
        
        kinematics->acceleration_sum = v2();
        kinematics->angular_acceleration_sum = 0.0;
        
        // If velocity is very close to zero, set it to 0.
        if(EQ(kinematics->velocity, v2()))
        {
            kinematics->velocity = v2();
        }
        if(EQ(kinematics->angular_velocity, 0.0f))
        {
            kinematics->angular_velocity = 0.0f;
        }
    }
    
    void LevelSandbox::init()
    {
        Graphics::Camera::width() = 16.0f;
        
        //time_t t;
        //seed_random((unsigned)time(&t));
        seed_random(42);
#if 1
        for(int i = 0; i < 2; i++)
        {
            v2 position = v2(random_range(-2.0f, 2.0f), random_range(-2.0f, 2.0f));
            v2 scale = v2(random_range(0.1f, 0.8f), random_range(0.1f, 0.8f));
            float inv_mass = 1.0f / (scale.x * scale.y);
            float inv_moment_of_inertia = 1.0f;
            
            v2 velocity = v2(random_range(-10.0f, 10.0f), random_range(-10.0f, 10.0f));
            //float angular_velocity = random_range(-10.0f, 10.0f);
            float angular_velocity = 0.0f;
            make_body_box(
                          Kinematics { velocity, angular_velocity, v2(), 0.0f, inv_mass, inv_moment_of_inertia, false, },
                          Box { position, scale, 0.0f }
                          );
        }
#endif
        
#if 1
        int num_circles = 2;
        for(int i = 0; i < num_circles; i++)
        {
            v2 position = v2(random_range(-2.0f, 2.0f), random_range(-2.0f, 2.0f));
            //float radius = random_range(0.1f, 0.8f);
            //float inv_mass = 1.0f / (PI * radius*radius);
            //float inv_moment_of_inertia = inv_mass;

            float radius = 0.5f;
            float inv_mass = 1.0f;
            float inv_moment_of_inertia = 5.0f;
            
            v2 velocity = v2(random_range(-10.0f, 10.0f), random_range(-10.0f, 10.0f));
            float angular_velocity = random_range(-10.0f, 10.0f);
            //float angular_velocity = 0.0f;
            make_body_circle(
                             Kinematics { velocity, angular_velocity, v2(), 0.0f, inv_mass, inv_moment_of_inertia, false, },
                             Circle{ position, radius, 0.0f }
                             );
        }
#endif
        
        
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
    }
    
    void LevelSandbox::step(float time_step)
    {
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
            
#if 1
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
                        assert(!EQ(c.normal, v2()));
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
                        assert(!EQ(c.normal, v2()));
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
                        assert(!EQ(c.normal, v2()));
                    }
                }
            }
            
            std::vector<CollisionResponse> collision_responses;
            collision_responses.resize(collisions.size());
            for(int collisioni = 0; collisioni < collisions.size(); collisioni++)
            {
                Collision &collision = collisions[collisioni];
                CollisionResponse &collision_response = collision_responses[collisioni];
                
                v2 a_in_b = collision.a_in_b;
                v2 b_in_a = collision.b_in_a;
                
                v2 *a_center_of_mass = collision.a_center_of_mass;
                v2 a_velocity = collision.a_kinematics->velocity;
                float a_angular_velocity = collision.a_kinematics->angular_velocity;
                float a_inv_mass = collision.a_kinematics->inv_mass;
                float a_inv_moment_of_inertia = collision.a_kinematics->inv_moment_of_inertia;
                
                v2 *b_center_of_mass = collision.b_center_of_mass;
                v2 b_velocity = collision.b_kinematics->velocity;
                float b_angular_velocity = collision.b_kinematics->angular_velocity;
                float b_inv_mass = collision.b_kinematics->inv_mass;
                float b_inv_moment_of_inertia = collision.b_kinematics->inv_moment_of_inertia;

                float depth = collision.depth;
                
                v2 n = normalize(collision.normal);
                assert(!EQ(n, v2()));
                
                // Used for later calculations for determining linear velocity from rotational velocity.
                v2 ra = a_in_b - *a_center_of_mass;
                v2 rb = b_in_a - *b_center_of_mass;
                v2 r_ap = find_ccw_normal(ra);
                v2 r_bp = find_ccw_normal(rb);
                v2 relative_velocity = (a_velocity + r_ap * a_angular_velocity) - (b_velocity + r_bp * b_angular_velocity);
                
                if(dot(relative_velocity, n) < 0.0f)
                {
                    // The bodies are leaving each other. No resolution necessary.
                    continue;
                }
                
                // Calculate jacobian
                v2 j_va = -n;
                v3 j_wa = -cross(v3(ra, 0.0f), v3(n, 0.0f)); // TODO I don't get what this is.
                v2 j_vb = n;
                v3 j_wb = cross(v3(rb, 0.0f), v3(n, 0.0f)); // TODO I don't get what this is.
                float k =
                    a_inv_mass +
                    dot(j_wa, a_inv_moment_of_inertia * j_wa) + // TODO I don't get what this is.
                    b_inv_mass +
                    dot(j_wb, b_inv_moment_of_inertia * j_wb); // TODO I don't get what this is.
                float effective_mass = 1.0f / k;

                float jv =
                    dot(j_va, a_velocity) +
                    dot(j_wa, v3(0.0f, 0.0f, a_angular_velocity)) +
                    dot(j_vb, b_velocity) +
                    dot(j_wb, v3(0.0f, 0.0f, b_angular_velocity));

                float beta = 0.2f;
                float b = (beta / sub_time_step) * -depth;

                float lambda = effective_mass * -(jv + b);

                collision_response.a_vel = j_va * lambda * a_inv_mass;
                collision_response.a_angular_vel = (j_wa * lambda * a_inv_moment_of_inertia).z;
                collision_response.b_vel = j_vb * lambda * b_inv_mass;
                collision_response.b_angular_vel = (j_wb * lambda * b_inv_moment_of_inertia).z;
                
            }
            
            // Do accumulation of impluse and position correction.
            for(int collisioni = 0; collisioni < collisions.size(); collisioni++)
            {
                Collision &collision = collisions[collisioni];
                CollisionResponse &collision_response = collision_responses[collisioni];
                
                collision.a_kinematics->velocity += collision_response.a_vel;
                collision.b_kinematics->velocity += collision_response.b_vel;
                collision.a_kinematics->angular_velocity += collision_response.a_angular_vel;
                collision.b_kinematics->angular_velocity += collision_response.b_angular_vel;
                //*collision.a_center_of_mass += collision_response.a_pos;
                //*collision.b_center_of_mass += collision_response.b_pos;
            }
#endif
            
            
        }
        
        for(int i = 0; i < box_list.geometry.size(); i++)
        {
            Box &geometry = box_list.geometry[i];
            Graphics::quad(geometry.center, geometry.half_extents, geometry.rotation, Color::BLUE);
            Graphics::arrow(geometry.center, geometry.center + rotate_vector(geometry.half_extents, geometry.rotation), 0.02f, v4(Color::RED));
        }
        for(int i = 0; i < circle_list.geometry.size(); i++)
        {
            Circle &geometry = circle_list.geometry[i];
            Graphics::circle(geometry.center, geometry.radius, Color::YELLOW);
            Graphics::arrow(geometry.center, geometry.center + rotate_vector(v2(1.0f, 0.0f) * geometry.radius, geometry.rotation), 0.02f, v4(Color::RED));
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

