
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "graphics.h"
#include "game_math.h"

#include "level_base.h"
#include "dynamics/dynamics.h"
#include "collision_detection/collision_detection.h"
#include "collision_resolution/collision_resolution.h"
#include "simple_constraints/simple_constraints.h"
#include "sandbox/sandbox.h"
#include "chain/chain.h"

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
        case COLLISION_RESOLUTION:
        level_state.level = new CollisionResolution::LevelCollisionResolution();
        level_state.level->init();
        break;
        case SIMPLE_CONSTRAINTS:
        level_state.level = new SimpleConstraints::LevelSimpleConstraints();
        level_state.level->init();
        break;
        case SANDBOX:
        level_state.level = new Sandbox::LevelSandbox();
        level_state.level->init();
        break;
        case CHAIN:
        level_state.level = new Chain::LevelChain();
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

    Graphics::ImGuiImplementation::init();
    
    //set_next_level(DYNAMICS);
    //set_next_level(COLLISION_DETECTION);
    //set_next_level(COLLISION_RESOLUTION);
    set_next_level(SANDBOX);
    //set_next_level(SIMPLE_CONSTRAINTS);
    //set_next_level(CHAIN);
    
    // Main loop
    float timer = 0.0f;
    float last_time = 0.0f;
    const static float STEP_TIME = 0.008f;
    while(!Graphics::wants_to_close())
    {
        float current_time = get_time();
        timer += current_time - last_time;
        last_time = current_time;
        
        if(timer >= STEP_TIME)
        {
            timer -= STEP_TIME;

            glfwPollEvents();
            
            if(level_state.do_swap)
            {
                switch_level();
            }
            
            Graphics::ImGuiImplementation::new_frame();
            level_state.level->step(STEP_TIME);
            
            Graphics::clear_frame(v4(0.0f, 0.0f, 0.05f, 1.0f));
            Graphics::render();
            Graphics::ImGuiImplementation::end_frame();
            Graphics::swap_frames();
        }
    }
    
    Graphics::uninit();
    
    return 0;
}
