
#pragma once

enum LevelID
{
    DYNAMICS,
    COLLISION_DETECTION,
    COLLISION_RESOLUTION,
    SANDBOX,
};

struct Level
{
    virtual void init() = 0;
    virtual void step(float time_step) = 0;
};

