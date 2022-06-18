
#include "simple_constraints/simple_constraints.h"

#include "graphics.h"

using namespace GameMath;

namespace SimpleConstraints
{
    void LevelSimpleConstraints::init()
    {
        Graphics::Camera::width() = 48.0f;
        
        v2 mouse_position = Graphics::mouse_world_position();
        last_mouse_position = mouse_position;
        ball_position = mouse_position;
        ball_velocity = v2(0.0f, 0.0f);
    }
    
    void LevelSimpleConstraints::step(float time_step)
    {
        v2 mouse_position = Graphics::mouse_world_position();
        v2 mouse_velocity = (mouse_position - last_mouse_position) / time_step;
        last_mouse_position = mouse_position;
        
        // Physics
        {
            float m = 10.0f;
            float rope_length = 10.0f;
            
            ball_velocity += v2(0.0f, -9.81f) * m * time_step;
            ball_velocity -= (ball_velocity * 0.4f) * time_step;
            
            v2 n = normalize(mouse_position - ball_position);
            v2 v = (ball_velocity - mouse_velocity);
            if(dot(v, n) < 0.0f && length(ball_position - mouse_position) > rope_length)
            {
                float d = (length(ball_position - mouse_position) - rope_length);
                float b = 0.5f*d / time_step;
                float impulse_magnitude = (dot(-v, n) + b)*m / dot(n, n);
                ball_velocity += n * impulse_magnitude / m;
            }
            
            ball_position += ball_velocity * time_step;
        }
        
        // Draw
        {
            Graphics::circle(ball_position, 0.2f, Color::WHITE, 10);
            static const int segs = 128;
            static int trail_points_start = 0;
            static int trail_points_end = 0;
            static v2 trail_points[segs] = {};
            for(int i = trail_points_start; i < trail_points_end; i++)
            {
                int a = i % segs;
                int b = (i + 1) % segs;
                float t = (float)(i-trail_points_start) / (trail_points_end - trail_points_start);
                float thicc = lerp(0.01f, 0.2f, t);
                float transparency = lerp(0.f, 1.0f, t);
                v2 diff = normalize(trail_points[b] - trail_points[a]);
                Graphics::line(trail_points[a] - diff*0.1f, trail_points[b] + diff*0.1f, thicc, v4(0.4f, 0.0f, 0.8f, transparency*transparency));
            }
            if(trail_points_end >= 64)
            {
                trail_points_start++;
            }
            trail_points_end++;
            trail_points[trail_points_end % segs] = ball_position;
        }
    }
}



