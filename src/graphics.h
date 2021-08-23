
#pragma once

#include "game_math.h"

struct Graphics
{
    static struct GraphicsState *instance;

    static bool init();
    static void uninit();

    static void clear_frame(GameMath::v4 color);
    static void render();
    static void swap_frames();

    static bool wants_to_close();

    static void quad(GameMath::v2 position, GameMath::v2 half_extents, float rotation, GameMath::v4 color, int layer = 0);
    static void circle(GameMath::v2 position, float radius, GameMath::v4 color, int layer = 0);

    static GameMath::mat4 view_m_world();
    static GameMath::mat4 world_m_view();
    static GameMath::mat4 ndc_m_world();
    static GameMath::mat4 world_m_ndc();

    static GameMath::v2 mouse_world_position();

    struct Camera
    {
        static struct CameraState *instance;

        static GameMath::v2 &position();
        static float &width();
        static float height();
        static float aspect_ratio();
    };

    struct ImGuiImplementation
    {
        static void init();
        static void new_frame();
        static void end_frame();
        static void shutdown();
    };
};

