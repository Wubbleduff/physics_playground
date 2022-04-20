
#include "collision_detection.h"
#include "graphics.h"


using namespace GameMath;


bool box_box(Box *a, Box *b, Collision *collision)
{
    v2 a_vertices[] =
    {
        a->center - a->half_extents, // bl
        a->center + v2(a->half_extents.x, -a->half_extents.y), // br
        a->center + a->half_extents, // tr
        a->center + v2(-a->half_extents.x, a->half_extents.y), // tl
    };
    v2 b_vertices[] =
    {
        b->center - b->half_extents, // bl
        b->center + v2(b->half_extents.x, -b->half_extents.y), // br
        b->center + b->half_extents, // tr
        b->center + v2(-b->half_extents.x, b->half_extents.y), // tl
    };

    // SAT to collide

    // Loop over all A axes
    for(int i = 0; i < 4; i++)
    {
        v2 a_plane = a_vertices[(i + 1) % 4] - a_vertices[i]; // Outward normal
    }

    // Given the boxes are colliding
    // Find the minimum separating normal for the collision
}

bool box_circle(Box *a, Circle *b, Collision *collision)
{
}

bool box_plane(Box *a, Plane *b, Collision *collision)
{
}

bool circle_circle(Circle *a, Circle *b, Collision *collision)
{
}

bool circle_plane(Circle *a, Plane *b, Collision *collision)
{
}








void LevelCollisionDetection::init()
{
}

void LevelCollisionDetection::step(float time_step)
{
    Graphics::quad(v2(), v2(1.0f, 1.0f), 0.0f, v4(1,1,1,1));
}


