
#pragma once

#include "level_base.h"
#include "game_math.h"

namespace Chain
{
    struct LevelChain : public Level
    {
        GameMath::v2 last_mouse_position;
        
        static const u32 CHAIN_POINTS_CAPACITY = 256;
        u32 num_chain_points;
        GameMath::v2 chain_points[CHAIN_POINTS_CAPACITY];
        GameMath::v2 chain_point_vels[CHAIN_POINTS_CAPACITY];
        
        void init();
        void step(float time_step);
    };
}


