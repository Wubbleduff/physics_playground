
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
        bool body_containing_reference_edge;

        Kinematics *a_kinematics;
        Kinematics *b_kinematics;
        GameMath::v2 *a_center_of_mass;
        GameMath::v2 *b_center_of_mass;
    };

    struct SoaCollision
    {
        uint32_t num_contacts;
        Contact contacts[2];
        bool body_containing_reference_edge;

        uint32_t a_index;
        bool a_is_circle;
        uint32_t b_index;
        bool b_is_circle;
    };
    
    bool box_box(Box *a, Box *b, Collision *collision);
    bool box_circle(Box *a, Circle *b, Collision *collision);
    bool circle_circle(Circle *a, Circle *b, Collision *collision);
    
    struct LevelSandbox : public Level
    {
        void init();
        void step(float time_step);
        void aos_step(float time_step);
        void soa_step(float time_step);
        
        void make_body_box(const Kinematics &body, const Box &shape);
        void make_body_circle(const Kinematics &body, const Circle &shape);
        void make_body_soa_box(const Kinematics &kinematics, const Box &shape);
        void make_body_soa_circle(const Kinematics &kinematics, const Circle &shape);

        std::vector<Collision> last_collisions;
        
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

        struct SoaBoxList
        {
            static const uint32_t MAX_BOXES = 1024;
            uint32_t num_boxes = 0;

            float vx[MAX_BOXES];
            float vy[MAX_BOXES];
            float av[MAX_BOXES];
            float inv_mass[MAX_BOXES];
            float inv_moment_of_inertia[MAX_BOXES];
            float px[MAX_BOXES];
            float py[MAX_BOXES];
            float hw[MAX_BOXES];
            float hh[MAX_BOXES];
            float rotation[MAX_BOXES];
        } soa_box_list;

        struct SoaCircleList
        {
            static const uint32_t MAX_CIRCLES = 1024;
            uint32_t num_circles = 0;

            float vx[MAX_CIRCLES];
            float vy[MAX_CIRCLES];
            float av[MAX_CIRCLES];
            float inv_mass[MAX_CIRCLES];
            float inv_moment_of_inertia[MAX_CIRCLES];
            float px[MAX_CIRCLES];
            float py[MAX_CIRCLES];
            float r[MAX_CIRCLES];
            float rotation[MAX_CIRCLES];
        } soa_circle_list;
        
    };
    
}
