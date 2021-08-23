
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "graphics.h"
#include "game_math.h"

#include "level.h"

#include <windows.h>

using namespace GameMath;

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
    while(!Graphics::wants_to_close())
    {
        Graphics::clear_frame(v4(0.0f, 0.0f, 0.05f, 1.0f));

        level->step(0.016f);
        level->draw();

        Graphics::render();

        Graphics::swap_frames();
    }

    level->uninit();
    Graphics::uninit();

    return 0;
}
