
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "graphics.h"
#include "game_math.h"

#include "level.h"

#include <windows.h>
#include <GLFW/glfw3.h>

using namespace GameMath;

float get_time()
{
    return glfwGetTime();
}

INT WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR lpCmdLine, INT nCmdShow)
{
    bool success = Graphics::init();
    if(!success)
    {
        return 1;
    }

    Level *level = new Level();
    level->init();

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

            level->step(STEP_TIME);
            level->draw();

            Graphics::clear_frame(v4(0.0f, 0.0f, 0.05f, 1.0f));
            Graphics::render();
            Graphics::swap_frames();
        }
    }

    level->uninit();
    Graphics::uninit();

    return 0;
}
