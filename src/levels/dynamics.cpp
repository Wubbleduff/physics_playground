
#include "levels/dynamics.h"

#include "graphics.h"

using namespace GameMath;

void LevelDynamics::init()
{
    Graphics::Camera::width() = 64.0f;

    body = new RigidBody();
    body->mass = 1.0f;
    body->moment_of_inertia = body->mass; // TODO Derive moment of inertia.

    body->apply_force(v2(500.0f, 0.0f), v2(0.0f, 1.0f));
}

void LevelDynamics::step(float time_step)
{
    v2 drag_force = 0.5f * -body->velocity * air_drag;
    body->apply_force(drag_force, v2());

    body->velocity += body->acceleration_sum * time_step;
    body->position += body->velocity * time_step;

    float angular_drag_force = -body->angular_velocity * air_drag;
    body->apply_torque(angular_drag_force);

    body->angular_velocity += body->angular_acceleration_sum * time_step;
    body->rotation += body->angular_velocity * time_step;

    body->acceleration_sum = v2();
    body->angular_acceleration_sum = 0.0;

    Graphics::quad(body->position, body->scale, body->rotation, v4(1, 1, 1, 1));
}


