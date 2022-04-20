
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "graphics.h"
#include "game_math.h"

#include "level.h"
#include "levels/dynamics.h"
#include "levels/collision_detection.h"

#include <windows.h>
#include <GLFW/glfw3.h>

using namespace GameMath;

struct LevelState
{
    Level *level;

    bool do_swap;
    LevelID next_level;
};

static LevelState level_state = {};


float get_time()
{
    return glfwGetTime();
}

void set_next_level(LevelID next_level_id)
{
    level_state.next_level = next_level_id;
    level_state.do_swap = true;
}

static void switch_level()
{
    delete level_state.level;

    switch(level_state.next_level)
    {
        case DYNAMICS:
            level_state.level = new LevelDynamics();
            level_state.level->init();
            break;
        case COLLISION_DETECTION:
            level_state.level = new LevelCollisionDetection();
            level_state.level->init();
            break;
    }

    level_state.do_swap = false;
}

INT WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR lpCmdLine, INT nCmdShow)
{
    bool success = Graphics::init();
    if(!success)
    {
        return 1;
    }

    set_next_level(COLLISION_DETECTION);

    // Main loop
    float timer = 0.0f;
    float last_time = 0.0f;
    const static float STEP_TIME = 0.016f;
    while(!Graphics::wants_to_close())
    {
        float current_time = get_time();
        timer += current_time - last_time;
        last_time = current_time;

        if(timer >= STEP_TIME)
        {
            timer -= STEP_TIME;

            if(level_state.do_swap)
            {
                switch_level();
            }

            level_state.level->step(STEP_TIME);

            Graphics::clear_frame(v4(0.0f, 0.0f, 0.05f, 1.0f));
            Graphics::render();
            Graphics::swap_frames();
        }
    }

    Graphics::uninit();

    return 0;
}
