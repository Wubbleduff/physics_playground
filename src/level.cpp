
#include "level.h"
#include "graphics.h"
#include "rigid_body.h"

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



#if 1
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






Body *Level::make_body()
{
    Body *body = new Body;

    body->is_static = false;

    body->transform.position = v2();
    body->velocity = v2();
    body->mass = 1.0f;

    body->shape = nullptr;

    bodies.push_back(body);

    return body;
}

void Level::destroy_body(Body *body)
{
    delete body;
}

void Level::init()
{
    reset();
}

void Level::uninit()
{
}

void Level::reset()
{
    for(Body *b : bodies) delete b;
    bodies.clear();

    for(int i = 0; i < 4; i++)
    {
        Body *body = make_body();

        body->is_static = true;
        body->velocity = v2();
        body->mass = 0.0f;

        Plane *plane = new Plane();
        body->shape = plane;

        float pad = 0.1f;
        switch(i)
        {
            case 0:
                body->transform.position = v2(0.0f, -Graphics::Camera::height() / 2.0f + pad);
                plane->normal = v2(0.0f, 1.0f);
                break;
            case 1:
                body->transform.position = v2(Graphics::Camera::width() / 2.0f - pad, 0.0f);
                plane->normal = v2(-1.0f, 0.0f);
                break;
            case 2:
                body->transform.position = v2(0.0f,  Graphics::Camera::height() / 2.0f - pad);
                plane->normal = v2(0.0f, -1.0f);
                break;
            case 3:
                body->transform.position = v2(-Graphics::Camera::width() / 2.0f + pad, 0.0f);
                plane->normal = v2(1.0f, 0.0f);
                break;
        }
    }


    //srand((unsigned)time(NULL));
    srand(1);
#if 1
    for(int i = 0; i < 500; i++)
    {
        Body *body = make_body();
        body->transform.position = v2(random_range(-2.0f, 2.0f), random_range(-2.0f, 2.0f));
        body->velocity = v2(random_range(-5.0f, 5.0f), random_range(-5.0f, 5.0f));
        body->mass = 1.0f;

        Circle *circle = new Circle();
        body->shape = circle;
        circle->radius = 0.05f;
    }
#endif

    Body *mouse_body = make_body();
    Circle *mouse_shape = new Circle();
    mouse_shape->radius = 0.1f;
    mouse_body->shape = mouse_shape;
    mouse_body_index = bodies.size() - 1;

#if 1
    for(int i = 0; i < 500; i++)
    {
        Body *body = make_body();
        body->transform.position = v2(random_range(-2.0f, 2.0f), random_range(-2.0f, 2.0f));
        body->velocity = v2(random_range(-5.0f, 5.0f), random_range(-5.0f, 5.0f));
        body->mass = 1.0f;

        Box *box = new Box();
        body->shape = box;
        box->half_extents = v2(0.05f, 0.05f);
    }
#endif

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

                static const float SMALLEST_SPEED = 0.001f;
                if(length(body->velocity) < SMALLEST_SPEED)
                {
                    body->velocity = v2();
                }

                body->velocity.y += -9.81f * divided_time_step;
                body->velocity -= body->velocity * 1.0f * divided_time_step;
                body->transform.position += body->velocity * divided_time_step;
            }

            // Collision detection
            std::vector<Collision> collisions;
#if 1
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
#else
            for(Body *body : bodies)
            {
                quad_tree->query(body->shape.bounding_box, &collisions);
            }
#endif

            {
                Body *mouse_body = bodies[mouse_body_index]; 
                v2 mouse_velocity = (mouse_pos - mouse_body->transform.position) * 1.0f / time_step;
                mouse_body->velocity = mouse_velocity;

                /*
                Body *body = bodies[bodies.size() - 2];
                v2 mouse_diff = body->transform.position - mouse_body->transform.position;
                if(length(mouse_diff) >= 3.0f)
                {
                    Collision col;
                    col.manifold.a_in_b = body->transform.position;
                    col.manifold.b_in_a = mouse_pos + normalize(mouse_diff) * 3.0f;
                    col.a = mouse_body;
                    col.b = body;
                    collisions.push_back(col);
                }
                */
            }

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

                // Friction
#if 0
                relative_velocity = collision.a->velocity - collision.b->velocity;

                v2 tangent = relative_velocity - dot(relative_velocity, normal) * normal;
                tangent = normalize(tangent);

                float f_vel = dot(relative_velocity, tangent);

                float a_sf = 0.1f;
                float b_sf = 0.1f;
                float a_df = 0.1f;
                float b_df = 0.1f;
                float mu  = length(v2(a_sf, b_sf));

                float f  = -f_vel / (a_inv_mass + b_inv_mass);

                v2 friction;
                if(abs(f) < j * mu)
                {
                    friction = f * tangent;
                }
                else
                {
                    mu = length(v2(a_df, b_df));
                    friction = -j * tangent * mu;
                }

                if(!collision.a->is_static)
                {
                    collision.a->velocity = collision.a->velocity - friction * a_inv_mass;
                }
                if(!collision.b->is_static)
                {
                    collision.b->velocity = collision.b->velocity + friction * b_inv_mass;
                }
#endif
            }

            // Position correction
#if 1
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
#endif
        }
    }
#endif

#if 0
    Graphics::circle(d_col.manifold.a_in_b, 0.1, v4(1.0f, 0.0f, 0.0f, 1.0f), 1);
    Graphics::circle(d_col.manifold.b_in_a, 0.1, v4(0.0f, 1.0f, 0.0f, 1.0f), 1);

#endif

    last_mouse_pos = mouse_pos;
}

void Level::draw()
{
#if 0
    for(Body *body : bodies)
    {
        body->draw();
    }
#endif

#if 1
    draw_debug_scenarios();
#endif
};

