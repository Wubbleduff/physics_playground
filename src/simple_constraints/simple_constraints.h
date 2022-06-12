
#pragma once

#include "level_base.h"
#include "game_math.h"

namespace SimpleConstraints
{
    struct LevelSimpleConstraints : public Level
    {
        GameMath::v2 ball_position;
        GameMath::v2 ball_velocity;

        GameMath::v2 last_mouse_position;
        
        void init();
        void step(float time_step);
    };
}


