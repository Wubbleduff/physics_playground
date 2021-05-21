
#include "../src/my_math.h"


static float min(float a, float b) { return 0; }

static bool operator==(v2 a, v2 b) { return false; }



struct CollisionRecord
{
    v2 normal;
    float depth;

    // Entity *a;
    // Entity *b;
    // ...
};

struct Circle
{
    v2 center;
    float radius;
};

struct Rect
{
    v2 center;
    float half_width;
    float half_height;
};








bool point_circle_collision(v2 point, Circle circle, CollisionRecord *record)
{
    v2 diff = point - circle.center;
    float depth = circle.radius - length(diff);

    // length(diff) < circle.radius  ===  length(diff) - circle.radius < 0.0f  ===  circle.radius - length(diff) > 0.0f

    if( depth > 0.0f )
    {
        record->normal = normalize(diff);
        record->depth  = depth;
        // record->... = circle....

        return true;
    }

    return false;
}

bool circle_circle_collision(Circle a, Circle b, CollisionRecord *record)
{
    v2 diff = b.center - a.center;
    float depth = (a.radius + b.radius) - length(diff);
    
    if( depth > 0.0f )
    {
        record->normal = normalize(diff);
        record->depth  = depth;
        // record->a = a;
        // record->b = b;
        // ...

        return true;
    }

    return false;
}

bool point_aabb_collision(v2 point, Rect rect, CollisionRecord *record)
{
    float rect_left   = rect.center.x - rect.half_width;
    float rect_right  = rect.center.x + rect.half_width;
    float rect_bottom = rect.center.y - rect.half_height;
    float rect_top    = rect.center.y + rect.half_height;

    float left_depth   = point.x - rect_left;
    float right_depth  = rect_right - point.x;
    float bottom_depth = point.y - rect_bottom;
    float top_depth    = rect_top - point.y;

    if( (left_depth < 0.0f) || (right_depth < 0.0f) || (bottom_depth < 0.0f) || (top_depth < 0.0f) )
    {
        return false;
    }

    float min_depth = min( left_depth, min(right_depth, min(bottom_depth, top_depth) ) );

    if( min_depth == left_depth )   { record->normal = v2(-1.0f,  0.0f); }
    if( min_depth == right_depth )  { record->normal = v2( 1.0f,  0.0f); }
    if( min_depth == bottom_depth ) { record->normal = v2( 0.0f, -1.0f); }
    if( min_depth == top_depth )    { record->normal = v2( 0.0f,  1.0f); }
    record->depth = min_depth;
    // ...

    return true;
}

bool aabb_aabb_collision(Rect a, Rect b, CollisionRecord *record)
{
    float a_left   = a.center.x - a.half_width;
    float a_right  = a.center.x + a.half_width;
    float a_bottom = a.center.y - a.half_height;
    float a_top    = a.center.y + a.half_height;

    float b_left   = b.center.x - b.half_width;
    float b_right  = b.center.x + b.half_width;
    float b_bottom = b.center.y - b.half_height;
    float b_top    = b.center.y + b.half_height;

    float left_depth   = a_right - b_left;
    float right_depth  = b_right - a_left;
    float bottom_depth = a_top - b_bottom;
    float top_depth    = b_top - a_bottom;

    if( (left_depth < 0.0f) || (right_depth < 0.0f) || (bottom_depth < 0.0f) || (top_depth < 0.0f) )
    {
        return false;
    }

    float min_depth = min( left_depth, min(right_depth, min(bottom_depth, top_depth) ) );

    if( min_depth == left_depth )   { record->normal = v2(-1.0f,  0.0f); }
    if( min_depth == right_depth )  { record->normal = v2( 1.0f,  0.0f); }
    if( min_depth == bottom_depth ) { record->normal = v2( 0.0f, -1.0f); }
    if( min_depth == top_depth )    { record->normal = v2( 0.0f,  1.0f); }
    record->depth = min_depth;
    // ...

    return true;
}

bool circle_aabb_collision(Circle circle, Rect rect, CollisionRecord *record)
{
    float rect_left   = rect.center.x - rect.half_width;
    float rect_right  = rect.center.x + rect.half_width;
    float rect_bottom = rect.center.y - rect.half_height;
    float rect_top    = rect.center.y + rect.half_height;

    float clamped_x = clamp(circle.center.x, rect_left, rect_right);
    float clamped_y = clamp(circle.center.y, rect_bottom, rect_top);
    v2 clamped_center = v2(clamped_x, clamped_y);

    if( length(circle.center - clamped_center) > circle.radius )
    {
        return false;
    }

    if( clamped_center == circle.center )
    {
        // Circle center is inside the rect
        Rect circle_as_rect = { circle.center, circle.radius, circle.radius };
        return aabb_aabb_collision(circle_as_rect, rect, record);
    }
    else
    {
        // Circle center is outside the rect
        return point_circle_collision(clamped_center, circle, record);
    }
}


int main()
{
    return 0;
}




