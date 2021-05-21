
#include "level.h"
#include "memory.h"
#include "my_math.h"
#include "my_algorithms.h"
#include "renderer.h"
#include "engine.h"
#include "input.h"
#include "quad_tree.h"

#include "imgui.h"

#include <stdio.h>
#include <map>


typedef void (*ScenarioFn)(Level *, float);

enum Scenario
{
    RENDER,
    POINT_CIRCLE,
    CIRCLE_CIRCLE,
    POINT_AABB,
    AABB_AABB,
    AABB_CIRCLE,
    LINE_CIRCLE,
    RAY_CIRCLE,
    QUAD_TREE
};

struct Level
{
    Scenario scenario;
};
static const float EPSILON = 0.0001f;

static const v4 RED    = v4(0.6f,  0.0f,  0.0f, 1.0f);
static const v4 YELLOW = v4(0.75f, 0.75f, 0.0f, 1.0f);
static const v4 PURPLE = v4(0.5f,  0.0f,  0.8f, 1.0f);
static const v4 GREEN  = v4(0.0f,  0.6f,  0.0f, 1.0f);

Level *create_level()
{
    Level *level = (Level *)my_allocate(sizeof(Level));

    level->scenario = RENDER;

    return level;
}

static void point_circle_step(Level *level, float dt)
{
    static v2 center = v2();
    static float radius = 1.5f;
    static float line_width = 0.01f;

    InputState *input_state = get_engine_input_state();

    char text[64];

    v2 mouse_pos = mouse_world_position(input_state);

    v4 color = RED;
    float diff = length(mouse_pos - center);
    if(diff < radius)
    {
        color = GREEN;

        if(key_state(input_state, ' '))
        {
            float depth = radius - diff;
            v2 a = mouse_pos;
            v2 b = mouse_pos + normalize(mouse_pos - center) * (depth);
            add_line_immediate({a, b, 0.001f, YELLOW});
            snprintf(text, 64, "%.3f", depth);
            //draw_text(text, (a + b) / 2.0f, 32.0f, YELLOW);
        }
    }


    add_circle_immediate({center, radius, color});
    add_line_immediate({center, mouse_pos, line_width, PURPLE});

    v2 mid_point = (center + mouse_pos) / 2.0f;

    snprintf(text, 64, "%.3f", diff);
    //draw_text(text, mid_point, 32.0f, PURPLE);
    //draw_text("Yaga, this is a line of text... Yay! ~~~", v2(), 32.0f, v4(1.0f, 1.0f, 1.0f, 1.0f));

    //draw_line(center, center + v2(radius, 0.0f), YELLOW);
    add_line_immediate({center, center + v2(radius, 0.0f), line_width, YELLOW});

    snprintf(text, 64, "%.3f", radius);
    //draw_text(text, center + v2(radius / 2.0f, 0.0f), 32.0f, YELLOW);


}

static void circle_circle_step(Level *level, float dt)
{
    static v2 a_center = v2(1.0f, 1.0f);
    static float a_radius = 0.5f;
    static v2 b_center = v2();
    static float b_radius = 1.5f;

    InputState *input_state = get_engine_input_state();

    char text[64];

    v2 mouse_pos = mouse_world_position(input_state);
    a_center = mouse_pos;

    v4 color = RED;
    float diff = length(a_center - b_center);
    if(diff < a_radius + b_radius)
    {
        color = GREEN;

        if(key_state(input_state, ' '))
        {
            v2 direction = a_center - b_center;
            float depth = absf(diff - (a_radius + b_radius));

            v2 a = a_center;
            v2 b = a_center + normalize(direction) * depth;

            //draw_line(a, b, YELLOW);
            add_circle_immediate({b, a_radius, YELLOW});

            snprintf(text, 64, "%.3f", depth);
            //draw_text(text, (a + b) / 2.0f, 32.0f, YELLOW);
        }
    }


    add_circle_immediate({a_center, a_radius, color});
    add_circle_immediate({b_center, b_radius, color});
    //draw_line(a_center, b_center, PURPLE);

    //draw_line(a_center, a_center + v2(a_radius, 0.0f), YELLOW);
    snprintf(text, 64, "%.3f", a_radius);
    //draw_text(text, a_center + v2(a_radius / 2.0f, 0.0f), 32.0f, YELLOW);

    //draw_line(b_center, b_center + v2(b_radius, 0.0f), YELLOW);
    snprintf(text, 64, "%.3f", b_radius);
    //draw_text(text, b_center + v2(b_radius / 2.0f, 0.0f), 32.0f, YELLOW);

    snprintf(text, 64, "%.3f", diff);
    //draw_text(text, (a_center + b_center) / 2.0f, 32.0f, PURPLE);

}

static void point_aabb_step(Level *level, float dt)
{
    static v2 a_center = v2();
    static float a_size = 1.5f;

    static v2 big_center;
    static float big_size = 100.0f;
    static v4 big_color = v4(1.0f, 0.0f, 0.0f, 0.25f);

    InputState *input_state = get_engine_input_state();

    v2 mouse_pos = mouse_world_position(input_state);

    v4 color = GREEN;
    bool hit = true;

    float left = a_center.x - a_size;
    float right = a_center.x + a_size;
    float bottom = a_center.y - a_size;
    float top = a_center.y + a_size;

    char text[64];

    if(mouse_pos.x < left)
    {
        float diff = left - mouse_pos.x;
        big_center = v2((mouse_pos.x + left) / 2.0f, 0.0f);
        add_rect_immediate({big_center, v2(diff, big_size), 0.0f, big_color});

        snprintf(text, 64, "%.3f", diff);
        //draw_text(text, big_center, 32.0f, YELLOW);

        hit = false;
    }
    if(mouse_pos.x > right)
    {
        float diff = mouse_pos.x - right;
        big_center = v2((mouse_pos.x + right) / 2.0f, 0.0f);
        add_rect_immediate({big_center, v2(diff, big_size), 0.0f, big_color});

        snprintf(text, 64, "%.3f", diff);
        //draw_text(text, big_center, 32.0f, YELLOW);

        hit = false;
    }
    if(mouse_pos.y < bottom)
    {
        float diff = bottom - mouse_pos.y;
        big_center = v2(0.0f, (mouse_pos.y + bottom) / 2.0f);
        add_rect_immediate({big_center, v2(big_size, diff), 0.0f, big_color});

        snprintf(text, 64, "%.3f", diff);
        //draw_text(text, big_center, 32.0f, YELLOW);

        hit = false;
    }
    if(mouse_pos.y > top)
    {
        float diff = mouse_pos.y - top;
        big_center = v2(0.0f, (mouse_pos.y + top) / 2.0f);
        add_rect_immediate({big_center, v2(big_size, mouse_pos.y - top), 0.0f, big_color});

        snprintf(text, 64, "%.3f", diff);
        //draw_text(text, big_center, 32.0f, YELLOW);

        hit = false;
    }

    bool show_resolution = false;
    if(hit && key_state(input_state, ' '))
    {
        show_resolution = true;
    }


    color = (hit) ? GREEN : RED;
    add_rect_immediate({a_center, v2(a_size, a_size) * 2.0f, 0.0f, color});

    if(show_resolution)
    {
        float left_diff  = mouse_pos.x - left;
        float right_diff = right - mouse_pos.x;
        float bottom_diff = mouse_pos.y - bottom;
        float top_diff = top - mouse_pos.y;
        float depth = min_float(left_diff, min_float(right_diff, min_float(bottom_diff, top_diff)));
        v2 direction;
        if(depth == left_diff) direction = v2(-1.0f, 0.0f);
        else if(depth == right_diff) direction = v2(1.0f, 0.0f);
        else if(depth == bottom_diff) direction = v2(0.0f, -1.0f);
        else if(depth == top_diff) direction = v2(0.0f, 1.0f);

        add_rect_immediate({mouse_pos + direction * depth, v2(1.0f, 1.0f) * 0.025f, 0.0f, YELLOW});

        //draw_line(mouse_pos, mouse_pos + direction * depth, RED);
    }

    ImGui::Begin("Point AABB Collision");

    ImGui::Text("Left: %.3f", left);
    ImGui::Text("Right: %.3f", right);
    ImGui::Text("Top: %.3f", bottom);
    ImGui::Text("Bottom: %.3f", top);
    ImGui::Text("Mouse X: %.3f", mouse_pos.x);
    ImGui::Text("Mouse Y: %.3f", mouse_pos.y);

    ImGui::End();
}

static void aabb_aabb_step(Level *level, float dt)
{
    static v2 a_center = v2(1.0f, 1.0f);
    static float a_size = 0.5f;
    static v2 b_center = v2();
    static float b_size = 1.5f;

    static float big_size = 100.0f;
    static v4 big_color = v4(1.0f, 0.0f, 0.0f, 0.25f);

    InputState *input_state = get_engine_input_state();

    v2 mouse_pos = mouse_world_position(input_state);
    a_center = mouse_pos;

    v4 color = GREEN;
    bool hit = true;

    float a_left   = a_center.x - a_size;
    float a_right  = a_center.x + a_size;
    float a_bottom = a_center.y - a_size;
    float a_top    = a_center.y + a_size;

    float b_left   = b_center.x - b_size;
    float b_right  = b_center.x + b_size;
    float b_bottom = b_center.y - b_size;
    float b_top    = b_center.y + b_size;

    char text[64];

    if(a_right < b_left)
    {
        v2 big_center = v2((a_right + b_left) / 2.0f, 0.0f);
        v2 big_scale = v2(b_left - a_right, big_size);
        add_rect_immediate({big_center, big_scale, 0.0f, big_color});

        snprintf(text, 64, "%.3f", big_scale.x);
        //draw_text(text, big_center, 32.0f, YELLOW);

        hit = false;
    }
    if(a_left > b_right)
    {
        v2 big_center = v2((a_left + b_right) / 2.0f, 0.0f);
        v2 big_scale = v2(a_left - b_right, big_size);
        add_rect_immediate({big_center, big_scale, 0.0f, big_color});

        snprintf(text, 64, "%.3f", big_scale.x);
        //draw_text(text, big_center, 32.0f, YELLOW);

        hit = false;
    }
    if(a_top < b_bottom)
    {
        v2 big_center = v2(0.0f, (a_top + b_bottom) / 2.0f);
        v2 big_scale = v2(big_size, b_bottom - a_top);
        add_rect_immediate({big_center, big_scale, 0.0f, big_color});

        snprintf(text, 64, "%.3f", big_scale.y);
        //draw_text(text, big_center, 32.0f, YELLOW);

        hit = false;
    }
    if(a_bottom > b_top)
    {
        v2 big_center = v2(0.0f, (a_bottom + b_top) / 2.0f);
        v2 big_scale = v2(big_size, a_bottom - b_top);
        add_rect_immediate({big_center, big_scale, 0.0f, big_color});

        snprintf(text, 64, "%.3f", big_scale.y);
        //draw_text(text, big_center, 32.0f, YELLOW);

        hit = false;
    }

    bool show_resolution = false;
    if(hit && key_state(input_state, ' '))
    {
        show_resolution = true;
    }

    color = (hit) ? GREEN : RED;
    color.a = 0.75f;
    add_rect_immediate({b_center, v2({b_size, b_size}) * 2.0f, 0.0f, color});
    add_rect_immediate({b_center, v2(b_size, b_size) * 2.0f, 0.0f});
    add_rect_immediate({a_center, v2(a_size, a_size) * 2.0f, 0.0f, color});
    add_rect_immediate({a_center, v2(a_size, a_size) * 2.0f, 0.0f});

    if(show_resolution)
    {
        float left_diff  = a_right - b_left;
        float right_diff = b_right - a_left;
        float bottom_diff = a_top - b_bottom;
        float top_diff = b_top - a_bottom;
        float depth = min_float(left_diff, min_float(right_diff, min_float(bottom_diff, top_diff)));
        v2 direction;
        if(depth == left_diff) direction = v2(-1.0f, 0.0f);
        else if(depth == right_diff) direction = v2(1.0f, 0.0f);
        else if(depth == bottom_diff) direction = v2(0.0f, -1.0f);
        else if(depth == top_diff) direction = v2(0.0f, 1.0f);

        add_rect_immediate({mouse_pos + direction * depth, v2(2.0f, 2.0f) * a_size, 0.0f, YELLOW});
        //draw_line(mouse_pos, mouse_pos + direction * depth, RED);
    }

    ImGui::Begin("AABB AABB Collision");

    ImGui::Text("B left - A right = %.3f", b_left - a_right);
    ImGui::Text("A left - B right = %.3f", a_left - b_right);
    ImGui::Text("B bottom - A top = %.3f", b_bottom - a_top);
    ImGui::Text("A bottom - B top = %.3f", a_bottom - b_top);

    ImGui::End();
}

static void aabb_circle_step(Level *level, float dt)
{
    static v2 a_center = v2(1.0f, 1.0f);
    static float a_radius = 0.5f;
    static v2 b_center = v2();
    static float b_size = 1.5f;

    InputState *input_state = get_engine_input_state();

    v2 mouse_pos = mouse_world_position(input_state);
    a_center = mouse_pos;

    v4 color = RED;

    float b_left   = b_center.x - b_size;
    float b_right  = b_center.x + b_size;
    float b_bottom = b_center.y - b_size;
    float b_top    = b_center.y + b_size;

    char text[64];

    v2 clamped_center = v2(clamp(a_center.x, b_left, b_right), clamp(a_center.y, b_bottom, b_top));


    if(length(clamped_center - a_center) < a_radius)
    {
        color = GREEN;
        color.a = 0.5f;
        add_rect_immediate({b_center, v2(b_size, b_size) * 2.0f, 0.0f, color});

        add_rect_immediate({clamped_center, v2(1.0f, 1.0f) * 0.025f, 0.0f, YELLOW});

        add_circle_immediate({a_center, a_radius, color});


        if(key_state(input_state, ' '))
        {
            v2 direction;
            float depth;
            bool inside = false;
            if(length(clamped_center - a_center) <= EPSILON) inside = true;

            if(inside)
            {
                float a_left = a_center.x - a_radius;
                float a_right = a_center.x + a_radius;
                float a_bottom = a_center.y - a_radius;
                float a_top = a_center.y + a_radius;

                float left_diff  = a_right - b_left;
                float right_diff = b_right - a_left;
                float bottom_diff = a_top - b_bottom;
                float top_diff = b_top - a_bottom;
                depth = min_float(left_diff, min_float(right_diff, min_float(bottom_diff, top_diff)));
                if(depth == left_diff) direction = v2(-1.0f, 0.0f);
                else if(depth == right_diff) direction = v2(1.0f, 0.0f);
                else if(depth == bottom_diff) direction = v2(0.0f, -1.0f);
                else if(depth == top_diff) direction = v2(0.0f, 1.0f);
            }
            else
            {
                direction = a_center - clamped_center;
                depth = a_radius - length(direction);
                direction = normalize(direction);
            }

            add_circle_immediate({a_center + direction * depth, a_radius, YELLOW});
            v2 a = a_center;
            v2 b = a_center + direction * depth;
            //draw_line(a, b, YELLOW);

            snprintf(text, 64, "%.3f", depth);
            //draw_text(text, (a + b) / 2.0f, 32.0f, YELLOW);
        }
    }
    else
    {
        //v2 diff = a_center - clamped_center;
        //v2 on_circle = clamped_center + normalize(diff) * (length(diff) - a_radius);
        //draw_line(clamped_center, a_center, PURPLE);
        add_circle_immediate({a_center, a_radius, color});
        add_rect_immediate({b_center, v2(b_size, b_size) * 2.0f, 0.0f, color});

        add_rect_immediate({clamped_center, v2(1.0f, 1.0f) * 0.025f, 0.0f, YELLOW});

        float diff_length = length(a_center - clamped_center);

        //draw_line(a_center, a_center + v2(a_radius, 0.0f), YELLOW);
        snprintf(text, 64, "%.3f", a_radius);
        //draw_text(text, a_center + v2(a_radius / 2.0f, 0.0f), 32.0f, YELLOW);

        snprintf(text, 64, "%.3f", diff_length);
        //draw_text(text, (clamped_center + a_center) / 2.0f, 32.0f, PURPLE);
    }



    ImGui::Begin("AABB Circle Collision");

    ImGui::Text("Clamped circle center: (%.3f, %.3f)", clamped_center.x, clamped_center.y);
    ImGui::Text("Distance between clamped center and actual center: %.3f", length(clamped_center - a_center));
    ImGui::Text("Circle radius: %.3f", a_radius);

    ImGui::End();
}

void line_circle_step(Level *level, float dt)
{
    InputState *input_state = get_engine_input_state();
    v2 mouse_pos = mouse_world_position(input_state);

    static v2 p = {0.0f, 0.0f};
    if(key_state(input_state, ' '))
    {
        p = mouse_pos;
    }
    //v2 q = {1.0f, 0.0f};
    v2 q = mouse_pos;
    v2 c = {1.0f, 1.0f};
    float r = 1.0f;


    v2 ray = normalize(q - p);
    v2 diff = c - p;

    float distance_along_line = dot(diff, ray);

    float distance_center_to_line = sqrtf( length_squared(diff) - distance_along_line*distance_along_line  );


    char text[256];
    //snprintf(text, 256, "%.3f", radicand);
    //draw_text(text, {0.0f, 0.0f}, 32.0f, PURPLE);
    
    bool hit = false;
    if(distance_center_to_line < r)
    {
        hit = true;

        float radicand = r*r - distance_center_to_line*distance_center_to_line;
        float x = sqrtf( radicand );
        float d1 = distance_along_line - x;
        float d2 = distance_along_line + x;

        v2 intersection1 = p + ray * d1;
        v2 intersection2 = p + ray * d2;
        add_circle_immediate({intersection1, 0.1f, RED});
        add_circle_immediate({intersection2, 0.1f, RED});
    }
    else
    {
        // No collision
    }

    v4 color = YELLOW;
    if(hit)
    {
        color = PURPLE;
    }
    //draw_line(p - ray * 100.0f, q + ray * 100.0f, color);
    add_circle_immediate({c, r, color});
}

void ray_circle_step(Level *level, float dt)
{
    InputState *input_state = get_engine_input_state();
    v2 mouse_pos = mouse_world_position(input_state);

    v2 ray_start = {-4.0f, 0.0f};
    v2 ray_dir = v2(5.0f, 1.0f) - ray_start;
    //v2 circle_center = {2.0f, 1.0f};
    v2 circle_center = mouse_pos;
    float circle_radius = 0.2f;

    float l = length(ray_dir);


    bool hit = false;
    // If laser dir doesn't have a direction, early return false;
    if(l == 0.0f)
    {
        hit = false;
    }
    else
    {
        // Normalize laser dir
        //ray_dir.x /= l;
        //ray_dir.y /= l;
        ray_dir = normalize(ray_dir);

        v2 diff = circle_center - ray_start;

        float diffAlongLaserDir = dot(diff, ray_dir);
        v2 normal = (circle_center - (ray_dir * diffAlongLaserDir)) - ray_start;
        float distFromLine = length(normal);

        //draw_line(ray_start, circle_center, YELLOW);
        //draw_line(ray_start, ray_start + normal, YELLOW);
        //draw_line(circle_center, circle_center - (ray_dir * diffAlongLaserDir), RED);

        char text[256];
        snprintf(text, 256, "%.3f", distFromLine);
        //draw_text(text, {0.0f, 0.0f}, 32.0f, PURPLE);

        // If the sphere is too far away from the line made by the ray, return false
        if(distFromLine > circle_radius)
        {
            hit = false;
        }
        else
        {
            if(diffAlongLaserDir >= 0.0f)
            {
                // If the sphere is in front of the ray, the ray will hit the sphere
                hit = true;
            }
            else
            {
                // If the sphere is behind the ray, we need to test if the sphere hits the start point
                if(length(diff) <= circle_radius)
                {
                    hit = true;
                }
                else
                {
                    hit = false;
                }
            }
        }

    }
    

    v4 color = YELLOW;
    if(hit) color = PURPLE;

    add_circle_immediate({circle_center, circle_radius, color});
    //draw_line(ray_start, ray_start + ray_dir * 1000.0f, YELLOW);
    //draw_line(ray_start, ray_start + normalize(ray_dir), RED);
}


static QuadTree *quad_tree = nullptr;
#if 0
static v2 tree_points[] =
{
    { 0.8f,  0.8f},
    {-0.8f,  0.8f},
    {-0.8f, -0.8f},
    { 0.8f, -0.8f}

    //{ 0.3f, 0.3f},
    //{ 0.4f, 0.4f},
    //{ 0.5f, 0.5f},
    //{ 0.6f, 0.6f}
};
#endif
static int num_tree_points = 0;
static v2 *tree_points = nullptr;
void draw_tree(QuadTree *tree)
{
    v2 bl = tree->bl;
    v2 tr = tree->tr;
    v2 tl = {tree->bl.x, tree->tr.y};
    v2 br = {tree->tr.x, tree->bl.y};
    //draw_line(bl, br, YELLOW);
    //draw_line(br, tr, YELLOW);
    //draw_line(tr, tl, YELLOW);
    //draw_line(tl, bl, YELLOW);

    for(int i = 0; i < tree->num_points; i++)
    {
        add_circle_immediate({*(tree->points[i]), 0.05f, YELLOW});
    }

    for(int i = 0; i < 4; i++)
    {
        if(tree->quadrants[i])
        {
            Rect rect;
            rect.center = (tree->bl + tree->tr) / 2.0f;
            rect.half_extents = v2((tree->bl.x - tree->tr.x), 0.005f);
            rect.rotation = 0.0f;
            rect.color = YELLOW;
            add_rect_immediate(rect);
            rect.rotation = PI / 2.0f;
            add_rect_immediate(rect);

            draw_tree(tree->quadrants[i]);
        }
    }
}
void quad_tree_step(Level *level, float dt)
{
    if(quad_tree == nullptr)
    {
        quad_tree = new QuadTree({-2.0f, -2.0f}, {2.0f, 2.0f});

        tree_points = new v2[1000]();
        /*
        tree_points[num_tree_points++] = { -1.0,  1.0 };
        tree_points[num_tree_points++] = {  0.0,  1.0 };
        tree_points[num_tree_points++] = {  1.0,  1.0 };
        tree_points[num_tree_points++] = { -1.0,  0.0 };
        tree_points[num_tree_points++] = {  0.0,  0.0 };
        tree_points[num_tree_points++] = {  1.0,  0.0 };
        tree_points[num_tree_points++] = { -1.0, -1.0 };
        tree_points[num_tree_points++] = {  0.0, -1.0 };
        tree_points[num_tree_points++] = {  1.0, -1.0 };
        */

        
        for(int i = 0; i < num_tree_points; i++)
        {
            QuadTree *tree = quad_tree->insert(&(tree_points[i]));
        }

        std::vector<v2 *> results;
        quad_tree->get_in_range(&results, { -1.5, -1.5 }, { 1.5, 1.5f } );

        int num = quad_tree->debug_count();
        assert(num == num_tree_points);

        results.push_back(nullptr);
    }

    InputState *input_state = get_engine_input_state();

    v2 mouse_pos = mouse_world_position(input_state);

#if 1
    if(key_toggled_down(input_state, ' '))
    {
        tree_points[num_tree_points++] = mouse_pos;
        QuadTree *tree = quad_tree->insert(&(tree_points[num_tree_points - 1]));
    }
#endif

    draw_tree(quad_tree);
}

void stress_test_iterate()
{
    static std::vector<Rect *> rects;
    static std::vector<Triangle *> triangles;
    static std::vector<Circle *> circles;
    static std::vector<Line *> lines;

    int rand_num = random_range(0, 8);
    if(rand_num == 0)
    {
        v2 pos = v2(random_range(-5.0f, 5.0f), random_range(-5.0f, 5.0f));
        v2 scale = v2(random_range(0.2f, 1.0f), random_range(0.2f, 1.0f));
        v4 color = v4(random_range(0.2f, 1.0f), random_range(0.2f, 1.0f), random_range(0.2f, 1.0f), random_range(0.2f, 1.0f));
        float rotation = random_range(0.0f, 2 * PI);
        rects.push_back(add_rect_retained({pos, scale, rotation, color}));
    }
    else if(rand_num == 1)
    {
        if(!rects.empty())
        {
            remove_rect(rects.back());
            rects.pop_back();
        }
    }
    else if(rand_num == 2)
    {
        v2 a = v2(random_range(-5.0f, 5.0f), random_range(-1.0f, 1.0f));
        v2 b = v2(random_range(-5.0f, 5.0f), random_range(-1.0f, 1.0f));
        v2 c = v2(random_range(-5.0f, 5.0f), random_range(-1.0f, 1.0f));
        v4 color = v4(random_range(0.2f, 1.0f), random_range(0.2f, 1.0f), random_range(0.2f, 1.0f), random_range(0.2f, 1.0f));
        triangles.push_back(add_triangle_retained({a, b, c, color}));
    }
    else if(rand_num == 3)
    {
        if(!triangles.empty())
        {
            remove_triangle(triangles.back());
            triangles.pop_back();
        }
    }
    else if(rand_num == 4)
    {
        v2 pos = v2(random_range(-5.0f, 5.0f), random_range(-5.0f, 5.0f));
        float scale = random_range(0.2f, 1.0f);
        v4 color = v4(random_range(0.2f, 1.0f), random_range(0.2f, 1.0f), random_range(0.2f, 1.0f), random_range(0.2f, 1.0f));
        circles.push_back(add_circle_retained({pos, scale, color}));
    }
    else if(rand_num == 5)
    {
        if(!circles.empty())
        {
            remove_circle(circles.back());
            circles.pop_back();
        }
    }
    else if(rand_num == 6)
    {
        v2 a = v2(random_range(-5.0f, 5.0f), random_range(-5.0f, 5.0f));
        v2 b = v2(random_range(-5.0f, 5.0f), random_range(-5.0f, 5.0f));
        float width = random_range(0.01f, 0.1f);
        v4 color = v4(random_range(0.2f, 1.0f), random_range(0.2f, 1.0f), random_range(0.2f, 1.0f), random_range(0.2f, 1.0f));
        lines.push_back(add_line_retained({a, b, width, color}));
    }
    else if(rand_num == 7)
    {
        if(!lines.empty())
        {
            remove_line(lines.back());
            lines.pop_back();
        }
    }
}
void test_render_step(Level *level, float dt)
{
    static std::vector<Rect *> rects;
    static std::vector<Triangle *> triangles;
    static std::vector<Circle *> circles;
    static bool initted = false;
    if(!initted)
    {

        initted = true;
    }

    InputState *input_state = get_engine_input_state();
    v2 mouse_pos = mouse_world_position(input_state);


    //stress_test_iterate();
    
    v2 a = v2(-1.0f, -1.0f);
    v2 b = mouse_pos;
    add_line_immediate({a, b, 0.5f, v4(1.0f, 1.0f, 1.0f, 1.0f)});
}

void level_step(Level *level, float dt)
{
    InputState *input_state = get_engine_input_state();

    if(key_toggled_down(input_state, '1')) level->scenario = POINT_CIRCLE;
    if(key_toggled_down(input_state, '2')) level->scenario = CIRCLE_CIRCLE;
    if(key_toggled_down(input_state, '3')) level->scenario = POINT_AABB;
    if(key_toggled_down(input_state, '4')) level->scenario = AABB_AABB;
    if(key_toggled_down(input_state, '5')) level->scenario = AABB_CIRCLE;
    if(key_toggled_down(input_state, '6')) level->scenario = LINE_CIRCLE;
    if(key_toggled_down(input_state, '7')) level->scenario = RAY_CIRCLE;
    if(key_toggled_down(input_state, 'Q')) level->scenario = QUAD_TREE;

    ScenarioFn scenario_fns[] =
    {
        &test_render_step,
        &point_circle_step,
        &circle_circle_step,
        &point_aabb_step,
        &aabb_aabb_step,
        &aabb_circle_step,
        &line_circle_step,
        &ray_circle_step,
        &quad_tree_step,
    };

    (scenario_fns[(int)level->scenario])(level, dt);
}

