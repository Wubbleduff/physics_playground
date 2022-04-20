
#pragma once

#include "level.h"
#include "game_math.h"

struct RigidBody
{
    GameMath::v2 position = GameMath::v2(); // m
    GameMath::v2 scale = GameMath::v2(1.0f, 1.0f); // m
    float rotation = 0.0f; // rad

    GameMath::v2 velocity = GameMath::v2(); // m / s
    float angular_velocity = 0.0f; // rad / s

    GameMath::v2 acceleration_sum = GameMath::v2(0.0f, 0.0f);
    float angular_acceleration_sum = 0.0f;

    float mass = 1.0f; // kg
    float moment_of_inertia = 0.0f; // kg * m^2 To be derived

    void apply_torque(float torque)
    {
        angular_acceleration_sum += torque / moment_of_inertia;
    }

    void apply_force(GameMath::v2 force, GameMath::v2 point_of_application)
    {
        acceleration_sum += force / mass;
        
        GameMath::v2 center_of_mass = GameMath::v2();
        GameMath::v2 perp = GameMath::find_ccw_normal(point_of_application - center_of_mass);
        apply_torque(GameMath::dot(perp, force));
    }
};

struct LevelDynamics : public Level
{
    RigidBody *body;

    float air_drag = 1.0f;

    void init();
    void step(float time_step);
};

