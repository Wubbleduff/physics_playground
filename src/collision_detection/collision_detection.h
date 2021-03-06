
#pragma once

#include "level_base.h"
#include "game_math.h"


struct Box
{
    GameMath::v2 center;
    GameMath::v2 half_extents;
    float rotation;
};

struct Circle
{
    GameMath::v2 center;
    float radius;
};

struct Plane
{
    GameMath::v2 point;
    GameMath::v2 normal;
};

struct Collision
{
    GameMath::v2 a_in_b;
    GameMath::v2 b_in_a;
};

bool box_box(Box *a, Box *b, Collision *collision);
bool box_circle(Box *a, Circle *b, Collision *collision);
bool box_plane(Box *a, Plane *b, Collision *collision);
bool circle_circle(Circle *a, Circle *b, Collision *collision);
bool circle_plane(Circle *a, Plane *b, Collision *collision);


struct LevelCollisionDetection : public Level
{
    void init();
    void step(float time_step);
};

