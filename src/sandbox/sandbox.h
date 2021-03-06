
#pragma once

#include "level_base.h"
#include "game_math.h"

#include <cstdint>
#include <vector>


namespace Sandbox
{
    
    struct Box
    {
        GameMath::v2 center = GameMath::v2();
        GameMath::v2 half_extents = GameMath::v2(0.5f, 0.5f);
        float rotation = 0.0f;
    };
    
    struct Circle
    {
        GameMath::v2 center = GameMath::v2();
        float radius = 0.5f;
        float rotation = 0.0f;
    };
    
    struct Kinematics
    {
        GameMath::v2 velocity = GameMath::v2(); // m / s
        float angular_velocity = 0.0f; // rad / s
        
        GameMath::v2 acceleration_sum = GameMath::v2(0.0f, 0.0f);
        float angular_acceleration_sum = 0.0f;
        
        float inv_mass = 1.0f; // kg
        float inv_moment_of_inertia = 1.0f; // kg * m^2 To be derived
        
        bool is_static = false;
    };

    struct Contact
    {
        float pen;
        GameMath::v2 normal;
        GameMath::v2 position;
    };
    
    struct Collision
    {
        uint32_t num_contacts;
        Contact contacts[2];

        Kinematics *a_kinematics;
        Kinematics *b_kinematics;
        GameMath::v2 *a_center_of_mass;
        GameMath::v2 *b_center_of_mass;
    };
    
    bool box_box(Box *a, Box *b, Collision *collision);
    bool box_circle(Box *a, Circle *b, Collision *collision);
    bool circle_circle(Circle *a, Circle *b, Collision *collision);
    
    struct LevelSandbox : public Level
    {
        void init();
        void step(float time_step);
        
        void make_body_box(const Kinematics &body, const Box &shape);
        void make_body_circle(const Kinematics &body, const Circle &shape);
        
        struct BoxList
        {
            std::vector<Kinematics> kinematics;
            std::vector<Box> geometry;
        } box_list;
        
        struct CircleList
        {
            std::vector<Kinematics> kinematics;
            std::vector<Circle> geometry;
        } circle_list;
        
    };
    
}
