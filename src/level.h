
#pragma once

#include "game_math.h"

#include <vector>



struct Body;

struct Level
{
    std::vector<Body *> bodies;

    void init();
    void uninit();
    void reset();
    void step(float time_step);
    void draw();

    Body *make_body();
    void destroy_body(Body *body);
};

