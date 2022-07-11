
#include "chain.h"
#include "graphics.h"

#include <GLFW/glfw3.h>

using namespace GameMath;

namespace Chain
{
    
    void LevelChain::init()
    {
        last_mouse_position = Graphics::mouse_world_position();
        
        num_chain_points = 64;
        for(u32 i = 0; i < num_chain_points; i++)
        {
            chain_points[i] = last_mouse_position;
        }
        for(u32 i = 0; i < num_chain_points; i++)
        {
            chain_point_vels[i] = v2();
        }
    }
    
    void LevelChain::step(float time_step)
    {
        if(num_chain_points == 0) return;

        static bool on = false;
        if(glfwGetKey(Graphics::get_graphics_window(), GLFW_KEY_SPACE))
        {
            on = true;

            init();
        }
        if(!on) return;
        
        v2 mouse_position = Graphics::mouse_world_position();
        v2 mouse_velocity = (mouse_position - last_mouse_position) / time_step;
        last_mouse_position = mouse_position;

        float m = 10.0f;
        
        for(u32 i = 0; i < num_chain_points; i++)
        {
            chain_point_vels[i] += v2(0.0f, -9.81f) * m * time_step;
            chain_point_vels[i] -= chain_point_vels[i] * time_step * 1.0f;
        }
        
        u32 num_sub_steps = 16;
        float chain_link_len = 0.01f;
        
        // First chain point constrain to mouse position
        for(u32 sub_step = 0; sub_step < num_sub_steps; sub_step++)
        {
            float sub_time_step = time_step / num_sub_steps;
            
            float beta = 0.1f;
            float d = length(chain_points[0] - mouse_position);
            if(d > 0.0001f)
            {
                v2 n = normalize(mouse_position - chain_points[0]);
                v2 v = chain_point_vels[0] - mouse_velocity;
                float b = beta*d / sub_time_step;
                float impulse_magnitude = (dot(-v, n) + b) / dot(n, n);
                chain_point_vels[0] += n * impulse_magnitude;
            }
            
            // All other chain points
            for(u32 i = 1; i < num_chain_points; i++)
            {
                v2 p0 = chain_points[i-1];
                v2 p1 = chain_points[i];
                v2 v0 = chain_point_vels[i-1];
                v2 v1 = chain_point_vels[i];
                float p0_pen_depth = length(p1 - p0) - chain_link_len;
                float p1_pen_depth = length(p0 - p1) - chain_link_len;
                float beta = 0.05f;
                
                v2 n = normalize(p1 - p0);
                v2 v = v0 - v1;
                if(dot(v, n) < 0.0f && length(p1 - p0) > chain_link_len)
                {
                    float d = p1_pen_depth;
                    float b = beta*d / sub_time_step;
                    float impulse_magnitude = (dot(-v, n) + b) * m / dot(n, n); // TODO Is thet mass here correct?
                    chain_point_vels[i-1] += n * impulse_magnitude * 0.5f * (1.0f/m);
                    chain_point_vels[i]   -= n * impulse_magnitude * 0.5f * (1.0f/m);
                }
            }
            
            for(u32 i = 0; i < num_chain_points; i++)
            {
                chain_points[i] += chain_point_vels[i] * sub_time_step;
            }
        }
        
        for(u32 i = 0; i < num_chain_points; i++)
        {
            Graphics::circle(chain_points[i], 0.04f, Color::WHITE);
        }
    }
    
}
