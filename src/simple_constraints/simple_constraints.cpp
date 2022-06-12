
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
        //ball_position = v2(0.0f, 10.0f);
        ball_position = mouse_position;
        ball_velocity = v2(0.0f, 0.0f);
    }
    
    void LevelSimpleConstraints::step(float time_step)
    {
        // Gravity
        float mass = 2.0f;
        float inv_mass = 1.0f/mass;
        ball_velocity += v2(0.0f, -9.81) * mass * time_step;
        
        v2 drag_force = v2();
        drag_force = -ball_velocity * mass * 0.2f;
        if(length(drag_force) * time_step < length(ball_velocity))
        {
            ball_velocity += drag_force * time_step;
        }
        
        v2 mouse_position = Graphics::mouse_world_position();
        v2 mouse_velocity = (mouse_position - last_mouse_position) * (1.0f/time_step);
        
        
        float rope_length = 5.0f;
        if(length(ball_position - mouse_position) > rope_length)
        {
            v2 contact1 = ball_position;
            v2 contact2 = mouse_position + normalize(ball_position - mouse_position) * min(rope_length, length(ball_position - mouse_position));
            
            v2 n = normalize(contact2 - contact1);
            
            v2 v1 = ball_velocity - mouse_velocity;
            
            if(dot(v1, n) < 0.0f)
            {
                float b = -(1.0f / time_step) * length(contact2 - contact1);
                //float b = 0.0f;
                
                float lambda = -(dot(n, v1) + b) / (dot(n, n) * inv_mass);
                
                v2 delta_velocity = n * lambda * inv_mass;
                
                ball_velocity += delta_velocity;
            }
            
            Graphics::circle(contact1, 0.1f, Color::RED, 10);
            Graphics::circle(contact2, 0.1f, Color::RED, 10);
        }
        
        // Update position
        ball_position += ball_velocity * time_step;
        
        last_mouse_position = mouse_position;
        
        Graphics::circle(ball_position, 0.3f, v4(1, 1, 1, 1), 1);
        Graphics::quad(v2(0.0f, -64.0f), v2(1000.0f, 64.0f), 0.0f, Color::BLUE);
        Graphics::arrow(mouse_position, mouse_position + mouse_velocity*time_step, 0.1f, Color::WHITE);
        
    }
}



