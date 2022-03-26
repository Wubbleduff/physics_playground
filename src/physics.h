
#pragma once

#include "game_math.h"

#include <cstdint>

struct Shape
{
    enum Type
    {
        NONE,
        CIRCLE,
        BOX,
        PLANE,
    } type;

    struct Circle
    {
        float radius;
    };

    struct Box
    {
        float half_extents[2];
    };

    struct Plane
    {
        float normal[2];
    };

    union
    {
        Circle circle;
        Box box;
        Plane plane;
    };
};

struct RigidBody
{
    GameMath::v2 position = GameMath::v2();
    GameMath::v2 scale = GameMath::v2(1.0f, 1.0f);
    float rotation = 0.0f;

    GameMath::v2 velocity;
    float mass;
    bool is_static;

    Shape shape;
};

struct Collision
{
    struct Manifold
    {
        GameMath::v2 a_in_b;
        GameMath::v2 b_in_a;
    } manifold;
    RigidBody *a = nullptr;
    RigidBody *b = nullptr;
};

bool collide_shapes(Shape *a, Shape *b, Collision *out_collision);

void solve_rigid_body_physics(RigidBody *rigid_bodies, uint32_t num_rigid_bodies, float time_step, uint32_t iters);

