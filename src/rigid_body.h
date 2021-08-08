
#pragma once

#include "game_math.h"



struct Collision
{
    struct Manifold
    {
        GameMath::v2 a_in_b;
        GameMath::v2 b_in_a;
    } manifold;
    struct Body *a = nullptr;
    struct Body *b = nullptr;
};

struct Shape
{
    virtual bool collide(GameMath::Transform *transform, struct Shape *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) = 0;
    virtual bool collide(GameMath::Transform *transform, struct Circle *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) = 0;
    virtual bool collide(GameMath::Transform *transform, struct Box *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) = 0;
    virtual bool collide(GameMath::Transform *transform, struct Plane *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) = 0;

    virtual void draw(GameMath::Transform *transform) = 0;
};

struct Circle : public Shape
{
    float radius = 1.0f;

    bool collide(GameMath::Transform *transform, struct Shape *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;
    bool collide(GameMath::Transform *transform, struct Circle *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;
    bool collide(GameMath::Transform *transform, struct Box *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;
    bool collide(GameMath::Transform *transform, struct Plane *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;

    void draw(GameMath::Transform *transform) override;
};

struct Box : public Shape
{
    GameMath::v2 half_extents = GameMath::v2(1.0f, 1.0f);

    bool collide(GameMath::Transform *transform, struct Shape *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;
    bool collide(GameMath::Transform *transform, struct Circle *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;
    bool collide(GameMath::Transform *transform, struct Box *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;
    bool collide(GameMath::Transform *transform, struct Plane *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;

    void draw(GameMath::Transform *transform) override;
};

struct Plane : public Shape
{
    GameMath::v2 normal = GameMath::v2(0.0f, 1.0f);

    bool collide(GameMath::Transform *transform, struct Shape *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;
    bool collide(GameMath::Transform *transform, struct Circle *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;
    bool collide(GameMath::Transform *transform, struct Box *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;
    bool collide(GameMath::Transform *transform, struct Plane *other, GameMath::Transform *other_transform, Collision::Manifold *manifold) override;

    void draw(GameMath::Transform *transform) override;
};

struct Body
{
    GameMath::Transform transform;

    GameMath::v2 velocity;
    float mass;

    bool is_static;

    Shape *shape;

    void draw();
};

