
#pragma once

#include "level_base.h"
#include "game_math.h"

#include <vector>


namespace Sandbox
{

struct Box
{
    GameMath::v2 center = GameMath::v2();
    GameMath::v2 half_extents = GameMath::v2(0.5f, 0.5f);
    float rotation = 0.0f;
};

struct Circle
{
    GameMath::v2 center = GameMath::v2();
    float radius = 0.5f;
    float rotation = 0.0f;
};

struct Collision
{
    GameMath::v2 a_in_b;
    GameMath::v2 b_in_a;
};

struct Kinematics
{
    GameMath::v2 velocity = GameMath::v2(); // m / s
    float angular_velocity = 0.0f; // rad / s

    GameMath::v2 acceleration_sum = GameMath::v2(0.0f, 0.0f);
    float angular_acceleration_sum = 0.0f;

    float mass = 1.0f; // kg
    float moment_of_inertia = 1.0f; // kg * m^2 To be derived

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

bool box_box(Box *a, Box *b, Collision *collision);
bool box_circle(Box *a, Circle *b, Collision *collision);
bool circle_circle(Circle *a, Circle *b, Collision *collision);

struct LevelSandbox : public Level
{
    void init();
    void step(float time_step);

    void make_body_box(const Kinematics &body, const Box &shape);
    void make_body_circle(const Kinematics &body, const Circle &shape);

    struct BoxList
    {
        std::vector<Kinematics> kinematics;
        std::vector<Box> geometry;
    } box_list;

    struct CircleList
    {
        std::vector<Kinematics> kinematics;
        std::vector<Circle> geometry;
    } circle_list;

};

}
