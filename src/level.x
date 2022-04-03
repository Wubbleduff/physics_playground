
#include "level.h"
#include "graphics.h"

#include <cstdio>
#include <cassert>
#include <cstdlib> // rand
#include <ctime> // rand

using namespace GameMath;




template<typename T>
static void swap(T &a, T &b)
{
    T temp = a;
    a = b;
    b = a;
}



#if 0
static void draw_debug_scenarios()
{
    v2 mouse_pos = Graphics::mouse_world_position();


    Body a;
    Box *box = new Box();
    a.shape = box;

    Body b;
    Circle *circle = new Circle;
    b.shape = circle;

    Graphics::quad_outline(a.transform.position, box->half_extents, 0.0f, Color::YELLOW, 0.01f);

    b.transform.position = mouse_pos;
    Graphics::circle_outline(b.transform.position, circle->radius, Color::YELLOW, 0.01f);

    Graphics::line(v2(), mouse_pos, 0.1f, Color::YELLOW);
}
#endif






uint32_t Level::make_body()
{
    RigidBody body;

    body.is_static = false;

    body.position = v2();
    body.velocity = v2();
    body.mass = 1.0f;

    body.shape.type = Shape::NONE;

    bodies.push_back(body);

    return bodies.size() - 1;
}

/*
void Level::destroy_body(Body *body)
{
    delete body;
}
*/

void Level::init()
{
    reset();
}

void Level::uninit()
{
}

void Level::reset()
{
    bodies.clear();

    for(int i = 0; i < 4; i++)
    {
        uint32_t body_id = make_body();
        RigidBody *body = &(bodies[body_id]);

        body->is_static = true;
        body->velocity = v2();
        body->mass = 0.0f;

        body->shape.type = Shape::PLANE;
        Shape::Plane *plane = &(body->shape.plane);

        float pad = 0.1f;
        switch(i)
        {
            case 0:
                body->position = v2(0.0f, -Graphics::Camera::height() / 2.0f + pad);
                plane->normal[0] = 0.0f;
                plane->normal[1] = 1.0f;
                break;
            case 1:
                body->position = v2(Graphics::Camera::width() / 2.0f - pad, 0.0f);
                plane->normal[0] = -1.0f;
                plane->normal[1] = 0.0f;
                break;
            case 2:
                body->position = v2(0.0f,  Graphics::Camera::height() / 2.0f - pad);
                plane->normal[0] = 0.0f;
                plane->normal[1] = -1.0f;
                break;
            case 3:
                body->position = v2(-Graphics::Camera::width() / 2.0f + pad, 0.0f);
                plane->normal[0] = 1.0f;
                plane->normal[1] = 0.0f;
                break;
        }
    }

    srand(1);

    for(int i = 0; i < 100; i++)
    {
        uint32_t body_id = make_body();
        RigidBody *body = &(bodies[body_id]);

        body->position = v2(random_range(-2.0f, 2.0f), random_range(-2.0f, 2.0f));
        //body->velocity = v2(random_range(-5.0f, 5.0f), random_range(-5.0f, 5.0f));
        body->velocity = v2();
        body->mass = 0.1f;

        body->shape.type = Shape::CIRCLE;
        body->shape.circle.radius = 0.25f;
    }
}

void Level::step(float time_step)
{
    static v2 last_mouse_pos = v2();
    v2 mouse_pos = Graphics::mouse_world_position();

    /*
    if(Input::key_down('R'))
    {
        reset();
        return;
    }
    */

    //bodies.back()->position = mouse_pos;

/*
    quad_tree->clear();
    for(const Body *body : bodies)
    {
        quad_tree->add(body);
    }

*/

    
    solve_rigid_body_physics(bodies.data(), bodies.size(), time_step, 8);
    return;



#if 0
    static Collision d_col;
    //if((last_mouse_pos.x < 0.0f && mouse_pos.x > 0.0f) || mouse_pos.y > 5.0f)
    {
        int iters = 8;
        for(int i = 0 ; i < iters; i++)
        {
            float divided_time_step = time_step / iters;

            // Kinematics
            for(Body *&body : bodies)
            {
                if(body->is_static) continue;

                static const float SMALLEST_SPEED = 0.00001f;
                if(length(body->velocity) < SMALLEST_SPEED)
                {
                    body->velocity = v2();
                }

                body->velocity.y += -9.81f * divided_time_step * divided_time_step;
                body->velocity -= body->velocity * 1.0f * divided_time_step;
                body->transform.position += body->velocity;
            }

            // Collision detection
            std::vector<Collision> collisions;
            for(int i = 0; i < bodies.size(); i++)
            {
                for(int j = i + 1; j < bodies.size(); j++)
                {
                    Body *a = bodies[i];
                    Body *b = bodies[j];

                    if(!a->shape || !b->shape) continue;

                    bool hit = false;
                    Collision collision = {};

                    hit = a->shape->collide(&a->transform, b->shape, &b->transform, &collision.manifold);

                    if(hit)
                    {
                        collision.a = a;
                        collision.b = b;
                        collisions.push_back(collision);

                        d_col = collision;
                    }
                }
            }

#if 1
            // Rope
            {
                Body *body = bodies[bodies.size() - 1];
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
                collision.a->transform.position += correction * a_factor;
                collision.b->transform.position -= correction * b_factor;
            }
        }
    }

    last_mouse_pos = mouse_pos;
#endif
}

void Level::draw()
{
    for(RigidBody &body : bodies)
    {
        if(body.shape.type == Shape::CIRCLE)
        {
            Graphics::circle(body.position, body.shape.circle.radius, v4(0.2f, 0.0, 0.8f, 1.0f));
        }
    }

#if 0
    draw_debug_scenarios();
#endif
};

