
#pragma once

#include "game_math.h"

struct Color
{
    static const GameMath::v4 RED;
    static const GameMath::v4 ORANGE;
    static const GameMath::v4 YELLOW;
    static const GameMath::v4 GREEN;
    static const GameMath::v4 BLUE;
    static const GameMath::v4 PURPLE;
    static const GameMath::v4 WHITE;
};

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
    static void quad_outline(GameMath::v2 position, GameMath::v2 half_extents, float rotation, GameMath::v4 color, float thickness, int layer = 0);
    static void circle(GameMath::v2 position, float radius, GameMath::v4 color, int layer = 0);
    static void circle_outline(GameMath::v2 position, float radius, GameMath::v4 color, float thickness, int layer = 0);
    static void line(GameMath::v2 a, GameMath::v2 b, float half_width, GameMath::v4 color, int layer = 0);
    static void arrow(GameMath::v2 start, GameMath::v2 end, float half_width, GameMath::v4 color, int layer = 0);
    
    static GameMath::mat4 view_m_world();
    static GameMath::mat4 world_m_view();
    static GameMath::mat4 ndc_m_world();
    static GameMath::mat4 world_m_ndc();
    
    static GameMath::v2 mouse_world_position();

    static struct GLFWwindow *get_graphics_window();
    
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

