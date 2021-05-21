
#pragma once

#include "my_math.h"

struct RendererState;

struct Rect
{
    v2 center;
    v2 half_extents;
    float rotation;
    v4 color;
    const char *texture;
};
struct WireframeRect
{
    v2 center;
    v2 half_extents;
    float rotation;
    v4 color;
    const char *texture;
    float edge_size;
};

struct Circle
{
    v2 position;
    float radius;
    v4 color;
};
struct WireframeCircle
{
    v2 position;
    float radius;
    v4 color;
    float edge_size;
};

struct Line
{
    v2 a, b;
    float width;
    v4 color;
    //bool rounded_ends;
};

struct Triangle
{
    v2 a, b, c;
    v4 color;
};
struct WireframeTriangle
{
    v2 a, b, c;
    v4 color;
    float edge_size;
};

RendererState *init_renderer();
void clear_frame(v4 color);
void render();
void swap_frames();



void  add_rect_immediate(Rect rect = {}, int layer = 0);
Rect *add_rect_retained(Rect rect = {}, int layer = 0);
void  remove_rect(Rect *rect);

void    add_circle_immediate(Circle circle = {}, int layer = 0);
Circle *add_circle_retained(Circle circle = {}, int layer = 0);
void    remove_circle(Circle *circle);

void      add_triangle_immediate(Triangle tri = {}, int layer = 0);
Triangle *add_triangle_retained(Triangle tri = {}, int layer = 0);
void      remove_triangle(Triangle *tri);

void  add_line_immediate(Line line = {}, int layer = 0);
Line *add_line_retained(Line line = {}, int layer = 0);
void  remove_line(Line *line);

//void draw_line(v2 a, v2 b, v4 color);

//void draw_text(const char *text, v2 position, float size, v4 color);
// Text *add_text();








// Misc
v2 ndc_point_to_world(v2 ndc);


// Camera
void set_camera_position(v2 position);
void set_camera_width(float width);

