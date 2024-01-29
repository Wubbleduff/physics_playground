
#define WIN32_MEAN_AND_LEAN
#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <cstdio>
#include <cassert>
#include <cmath>

#include "glad/glad.h"
#include <gl/glu.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "Glu32.lib")
#pragma comment(lib, "User32.lib")
#pragma comment(lib, "Gdi32.lib")
#pragma comment(lib, "Shell32.lib")
#pragma comment(lib, "lib/glfw/glfw.lib")

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;
typedef int64_t  s64;

#define ARRAY_COUNT(N) (sizeof(N) / sizeof(*(N)))
#define SQ(N) ((N)*(N))


#define MAX_QUADS 1024 * 1024
#define MAX_CIRCLES 1024 * 1024
#define SSBO_STRIDE 10 * 1024 * 4
#define NUM_QUAD_SSBO_FIELDS 11
#define NUM_CIRCLE_SSBO_FIELDS 10
struct GraphicsData
{
    u32 num_quads;
    float quad_x[MAX_QUADS];
    float quad_y[MAX_QUADS];
    float quad_w[MAX_QUADS];
    float quad_h[MAX_QUADS];
    float quad_r[MAX_QUADS];
    float quad_g[MAX_QUADS];
    float quad_b[MAX_QUADS];
    
    u32 num_circles;
    float circle_x[MAX_CIRCLES];
    float circle_y[MAX_CIRCLES];
    float circle_radius[MAX_CIRCLES];
    float circle_r[MAX_CIRCLES];
    float circle_g[MAX_CIRCLES];
    float circle_b[MAX_CIRCLES];
};
float g_quad_data[] = {
    // X
    -0.5f,  0.5f,  0.5f, -0.5f,
    // Y
    -0.5f, -0.5f,  0.5f,  0.5f,
};

enum ColorId
{
    WHITE,
    BLACK,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    ORANGE,
    GRAY,
    
    BLUE_0,
    BLUE_1,
    BLUE_2,
    BLUE_3,
    
    NUM_COLOR_ID
};
float g_color_data[NUM_COLOR_ID * 3] = {
    1.0f, 1.0f, 1.0f,
    0.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
    0.8f, 0.7f, 0.0f,
    0.8f, 0.4f, 0.2f,
    0.4f, 0.4f, 0.4f,
    
    0.05f, 0.025f, 0.2f,
    0.1f, 0.05f, 0.2f,
    0.05f, 0.05f, 0.4f,
    0.3f, 0.05f, 0.4f,
};



struct v2
{
    float x, y;

    v2 operator+(const v2 v) const { return v2{x + v.x, y + v.y}; }
    v2 operator-(const v2 v) const { return v2{x - v.x, y - v.y}; }
    v2 operator*(float v) const { return v2{x*v, y*v}; }
    v2 operator/(float v) const { return v2{x/v, y/v}; }
    v2 operator-() const { return v2{-x, -y}; }
    v2 operator+=(v2 v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }
};
v2 operator*(float a, v2 b) { return v2{a*b.x, a*b.y}; }
float dot(v2 a, v2 b)
{
    return a.x*b.x + a.y*b.y;
}
float len_sq(v2 v)
{
    return dot(v, v);
}
float len(v2 v)
{
    return sqrtf(len_sq(v));
}
v2 normalize(v2 v)
{
    return v / len(v);
}
v2 normalize_safe(v2 v)
{
    const float l = len(v);
    return l == 0.0f ? v2{0.0f, 0.0f} : v / len(v);
}



u32 g_screen_width;
u32 g_screen_height;
FILE* g_log_file;
LARGE_INTEGER g_freq;

static inline u32 min_u32(u32 a, u32 b)
{
    return a < b ? a : b;
}

static inline float min_f32(float a, float b)
{
    return a < b ? a : b;
}
static inline float max_f32(float a, float b)
{
    return a > b ? a : b;
}

#define GL_ERR() check_gl_errors_fn(__FILE__, __LINE__);
static void check_gl_errors_fn(const char* file, int line)
{
    // https://registry.khronos.org/OpenGL-Refpages/gl4/html/glGetError.xhtml
    GLint error;
    u8 any_error = 0;
    do
    {
        error = glGetError();
        if(error)
        {
            fprintf(g_log_file, "%s:%i GL error: %i \"%s\"\n", file, line, error, gluErrorString(error));
            any_error = 1;
        }
    }
    while(error);
    
    if(any_error)
    {
        fflush(g_log_file);
        assert(0);
    }
}

void glfw_error_fn(s32 error, const char *message)
{
    (void)error;
    (void)message;
    fprintf(g_log_file, "%s:%i GLFW error: %i \"%s\"\n", __FILE__, __LINE__, error, message);
    fflush(g_log_file);
    assert(0);
}

static s64 get_time_ms()
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    const s64 ns = (s64)((t.QuadPart * 1'000LL) / g_freq.QuadPart);
                          return ns;
}

static GLuint make_shader_from_string(const char* vert_str, const char* frag_str)
{
    static char error_buf[1024];
    GLuint vert_shader = glCreateShader(GL_VERTEX_SHADER); GL_ERR();
    const GLchar* const* p_vert_source_v = &vert_str;
    glShaderSource(vert_shader, 1, p_vert_source_v, NULL); GL_ERR();
    glCompileShader(vert_shader); GL_ERR();
    GLint success;
    glGetShaderiv(vert_shader, GL_COMPILE_STATUS, &success); GL_ERR();
    if(!success)
    {
        glGetShaderInfoLog(vert_shader, sizeof(error_buf), NULL, error_buf);GL_ERR();
        fprintf(g_log_file, "%s:%i GL error compiling shader \"%s\"\n", __FILE__, __LINE__, error_buf);
        fflush(g_log_file);
        assert(0);
    }
    GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER); GL_ERR();
    const GLchar* const* p_frag_source_v = &frag_str;
    glShaderSource(frag_shader, 1, p_frag_source_v, NULL); GL_ERR();
    glCompileShader(frag_shader); GL_ERR();
    glGetShaderiv(frag_shader, GL_COMPILE_STATUS, &success); GL_ERR();
    if(!success)
    {
        glGetShaderInfoLog(frag_shader, sizeof(error_buf), NULL, error_buf);GL_ERR();
        fprintf(g_log_file, "%s:%i GL error compiling shader \"%s\"\n", __FILE__, __LINE__, error_buf);
        fflush(g_log_file);
        assert(0);
    }
    GLuint shader_program = glCreateProgram(); GL_ERR();
    glAttachShader(shader_program, vert_shader); GL_ERR();
    glAttachShader(shader_program, frag_shader); GL_ERR();
    glLinkProgram(shader_program); GL_ERR();
    glGetProgramiv(shader_program, GL_LINK_STATUS, &success); GL_ERR();
    assert(success);
    glDeleteShader(vert_shader); GL_ERR();
    glDeleteShader(frag_shader);  GL_ERR();
    
    return shader_program;
}

static void quad(
                 GraphicsData* graphics_data,
                 const float x,
                 const float y,
                 const float w,
                 const float h,
                 const enum ColorId color_id)
{
    const u32 num = graphics_data->num_quads;
    graphics_data->quad_x[num] = x;
    graphics_data->quad_y[num] = y;
    graphics_data->quad_w[num] = w;
    graphics_data->quad_h[num] = h;
    graphics_data->quad_r[num] = g_color_data[color_id*3 + 0];
    graphics_data->quad_g[num] = g_color_data[color_id*3 + 1];
    graphics_data->quad_b[num] = g_color_data[color_id*3 + 2];
    graphics_data->num_quads++;
}

static inline void quad(
                 GraphicsData* graphics_data,
                 const v2 pos,
                 const float w,
                 const float h,
                 const enum ColorId color_id)
{
    quad(graphics_data, pos.x, pos.y, w, h, color_id);
}

static void circle(
                   GraphicsData* graphics_data,
                   const float x,
                   const float y,
                   const float r,
                   const enum ColorId color_id)
{
    const u32 num = graphics_data->num_circles;
    graphics_data->circle_x[num] = x;
    graphics_data->circle_y[num] = y;
    graphics_data->circle_radius[num] = r;
    graphics_data->circle_r[num] = g_color_data[color_id*3 + 0];
    graphics_data->circle_g[num] = g_color_data[color_id*3 + 1];
    graphics_data->circle_b[num] = g_color_data[color_id*3 + 2];
    graphics_data->num_circles++;
}
static void circle(
                   GraphicsData* graphics_data,
                   const v2 pos,
                   const float r,
                   const enum ColorId color_id)
{
    circle(graphics_data, pos.x, pos.y, r, color_id);
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    (void)nCmdShow;
    (void)lpCmdLine;
    (void)hPrevInstance;
    (void)hInstance;
    
    g_log_file = fopen("log.txt", "w");
    
    const u32 monitor_width = GetSystemMetrics(SM_CXSCREEN);
    const u32 monitor_height = GetSystemMetrics(SM_CYSCREEN);
    
    g_screen_width = (u32)(monitor_width * 0.8f);
    g_screen_height = (u32)(monitor_height * 0.8f);
    
    glfwSetErrorCallback(glfw_error_fn);
    if(!glfwInit())
    {
        return 1;
    }
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    GLFWwindow* glfw_window = glfwCreateWindow(g_screen_width, g_screen_height, "physics", NULL, NULL);
    glfwMakeContextCurrent(glfw_window);
    if(!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
    {
        return 1;
    }
    //glfwSetKeyCallback(glfw_window, glfw_key_callback);
    //glfwSetMouseButtonCallback(glfw_window, glfw_mouse_key_callback);
    //glfwSetScrollCallback(glfw_window, glfw_mouse_scroll_callback);
    glfwSwapInterval(1);
    QueryPerformanceFrequency(&g_freq);
    
    // ImGui
    {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        ImGui::StyleColorsDark();
        ImGui_ImplOpenGL3_Init("#version 440 core");
        ImGui_ImplGlfw_InitForOpenGL(glfw_window, true);
    }
    
    GraphicsData* graphics_data = (GraphicsData*)malloc(sizeof(GraphicsData));
    assert(graphics_data);
    
    GLuint shader_program;
    GLuint color_shader_program;
    GLuint circle_shader_program;
    GLuint vao;
    GLuint vbo;
    GLuint quad_ssbo;
    GLuint circle_ssbo;
    {
        // VBO
        glGenBuffers(1, &vbo); GL_ERR();
        glBindBuffer(GL_ARRAY_BUFFER, vbo); GL_ERR();
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_data), g_quad_data, GL_STATIC_DRAW); GL_ERR();
        
        // VAO
        glGenVertexArrays(1, &vao); GL_ERR();
        glBindVertexArray(vao); GL_ERR();
        glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 0, (void *)(0*sizeof(float))); GL_ERR();
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, (void *)(4*sizeof(float))); GL_ERR();
        glEnableVertexAttribArray(0); GL_ERR();
        glEnableVertexAttribArray(1); GL_ERR();
        
        // SSBO
        // https://www.khronos.org/opengl/wiki/Shader_Storage_Buffer_Object
        glGenBuffers(1, &quad_ssbo); GL_ERR();
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, quad_ssbo); GL_ERR();
        glBufferData(GL_SHADER_STORAGE_BUFFER, NUM_QUAD_SSBO_FIELDS * SSBO_STRIDE, 0, GL_DYNAMIC_DRAW); GL_ERR();
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, quad_ssbo); GL_ERR();
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); GL_ERR();
        
        glGenBuffers(1, &circle_ssbo); GL_ERR();
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, circle_ssbo); GL_ERR();
        glBufferData(GL_SHADER_STORAGE_BUFFER, NUM_CIRCLE_SSBO_FIELDS * SSBO_STRIDE, 0, GL_DYNAMIC_DRAW); GL_ERR();
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, circle_ssbo); GL_ERR();
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); GL_ERR();
        
        // Shader
        assert(NUM_QUAD_SSBO_FIELDS == 11);
        shader_program = make_shader_from_string(
                                                 "#version 440 core\n"
                                                 "layout (location = 0) in float a_x;"
                                                 "layout (location = 1) in float a_y;"
                                                 "layout(std430, binding = 0) buffer voxel_data"
                                                 "{"
                                                 "    float ssbo_pos_x[10*1024];"
                                                 "    float ssbo_pos_y[10*1024];"
                                                 "    float ssbo_scale_x[10*1024];"
                                                 "    float ssbo_scale_y[10*1024];"
                                                 "    float ssbo_r[10*1024];"
                                                 "    float ssbo_g[10*1024];"
                                                 "    float ssbo_b[10*1024];"
                                                 "    float ssbo_tex_l[10*1024];"
                                                 "    float ssbo_tex_r[10*1024];"
                                                 "    float ssbo_tex_b[10*1024];"
                                                 "    float ssbo_tex_t[10*1024];"
                                                 "};"
                                                 "uniform mat4 u_view_proj_mat;"
                                                 "out float frag_u;"
                                                 "out float frag_v;"
                                                 "out vec4 frag_color;"
                                                 "void main()"
                                                 "{"
                                                 "    frag_u = mix(ssbo_tex_l[gl_InstanceID], ssbo_tex_r[gl_InstanceID], a_x);"
                                                 "    frag_v = mix(ssbo_tex_b[gl_InstanceID], ssbo_tex_t[gl_InstanceID], a_y);"
                                                 "    frag_color = vec4(ssbo_r[gl_InstanceID], ssbo_g[gl_InstanceID], ssbo_b[gl_InstanceID], 1.0f);"
                                                 "    vec4 world_position = vec4("
                                                 "            a_x*ssbo_scale_x[gl_InstanceID] + ssbo_pos_x[gl_InstanceID],"
                                                 "            a_y*ssbo_scale_y[gl_InstanceID] + ssbo_pos_y[gl_InstanceID],"
                                                 "            0.0f,"
                                                 "            1.0f);"
                                                 "    gl_Position = u_view_proj_mat * world_position;"
                                                 "};",
                                                 
                                                 "#version 440 core\n"
                                                 "in float frag_u;"
                                                 "in float frag_v;"
                                                 "in vec4 frag_color;"
                                                 "uniform sampler2D u_tex_sampler;"
                                                 "out vec4 frag_result;"
                                                 "void main()"
                                                 "{"
                                                 "    float tex = texture(u_tex_sampler, vec2(frag_u, frag_v)).r;"
                                                 "    vec4 tex_color = vec4(tex, tex, tex, tex);"
                                                 "    frag_result = frag_color * tex_color;"
                                                 "}"
                                                 );
        
        color_shader_program = make_shader_from_string(
                                                       "#version 440 core\n"
                                                       "layout (location = 0) in float a_x;"
                                                       "layout (location = 1) in float a_y;"
                                                       "layout(std430, binding = 0) buffer voxel_data"
                                                       "{"
                                                       "    float ssbo_pos_x[10*1024];"
                                                       "    float ssbo_pos_y[10*1024];"
                                                       "    float ssbo_scale_x[10*1024];"
                                                       "    float ssbo_scale_y[10*1024];"
                                                       "    float ssbo_r[10*1024];"
                                                       "    float ssbo_g[10*1024];"
                                                       "    float ssbo_b[10*1024];"
                                                       "    float ssbo_tex_l[10*1024];"
                                                       "    float ssbo_tex_r[10*1024];"
                                                       "    float ssbo_tex_b[10*1024];"
                                                       "    float ssbo_tex_t[10*1024];"
                                                       "};"
                                                       "uniform mat4 u_view_proj_mat;"
                                                       "out vec4 frag_color;"
                                                       "void main()"
                                                       "{"
                                                       "    frag_color = vec4(ssbo_r[gl_InstanceID], ssbo_g[gl_InstanceID], ssbo_b[gl_InstanceID], 1.0f);"
                                                       "    vec4 world_position = vec4("
                                                       "            a_x*ssbo_scale_x[gl_InstanceID] + ssbo_pos_x[gl_InstanceID],"
                                                       "            a_y*ssbo_scale_y[gl_InstanceID] + ssbo_pos_y[gl_InstanceID],"
                                                       "            0.0f,"
                                                       "            1.0f);"
                                                       "    gl_Position = u_view_proj_mat * world_position;"
                                                       "};",
                                                       
                                                       "#version 440 core\n"
                                                       "in vec4 frag_color;"
                                                       "out vec4 frag_result;"
                                                       "void main()"
                                                       "{"
                                                       "    frag_result = frag_color;"
                                                       "}"
                                                       );
        
        assert(NUM_CIRCLE_SSBO_FIELDS == 10);
        circle_shader_program = make_shader_from_string(
                                                        "#version 440 core\n"
                                                        "layout (location = 0) in float a_x;"
                                                        "layout (location = 1) in float a_y;"
                                                        "layout(std430, binding = 0) buffer voxel_data"
                                                        "{"
                                                        "    float ssbo_pos_x[10*1024];"
                                                        "    float ssbo_pos_y[10*1024];"
                                                        "    float ssbo_scale_r[10*1024];"
                                                        "    float ssbo_r[10*1024];"
                                                        "    float ssbo_g[10*1024];"
                                                        "    float ssbo_b[10*1024];"
                                                        "    float ssbo_tex_l[10*1024];"
                                                        "    float ssbo_tex_r[10*1024];"
                                                        "    float ssbo_tex_b[10*1024];"
                                                        "    float ssbo_tex_t[10*1024];"
                                                        "};"
                                                        "uniform mat4 u_view_proj_mat;"
                                                        "out float frag_u;"
                                                        "out float frag_v;"
                                                        "out vec4 frag_color;"
                                                        "void main()"
                                                        "{"
                                                        "    frag_u = a_x;"
                                                        "    frag_v = a_y;"
                                                        "    frag_color = vec4(ssbo_r[gl_InstanceID], ssbo_g[gl_InstanceID], ssbo_b[gl_InstanceID], 1.0f);"
                                                        "    vec4 world_position = vec4("
                                                        "            a_x*ssbo_scale_r[gl_InstanceID]*2.0f + ssbo_pos_x[gl_InstanceID],"
                                                        "            a_y*ssbo_scale_r[gl_InstanceID]*2.0f + ssbo_pos_y[gl_InstanceID],"
                                                        "            0.0f,"
                                                        "            1.0f);"
                                                        "    gl_Position = u_view_proj_mat * world_position;"
                                                        "};",
                                                        
                                                        "#version 440 core\n"
                                                        "in float frag_u;"
                                                        "in float frag_v;"
                                                        "in vec4 frag_color;"
                                                        "out vec4 frag_result;"
                                                        "void main()"
                                                        "{"
                                                        "    vec2 delta = vec2(frag_u, frag_v);"
                                                        "    if(dot(delta, delta) > 0.707f* 0.707f*0.5f)"
                                                        "    {"
                                                        "        discard;"
                                                        "    }"
                                                        "    frag_result = frag_color;"
                                                        "}"
                                                        );
    }
    
    
    u32 num_circles = 3;
    v2 circle_pos_m[] = {
        {-5.0f, 0.25f},
        {5.0f, -0.25f},
        {0.0f, 5.0f}
    };
    float circle_radius_m[] = {
        0.5f,
        0.5f,
        0.5f
    };
    const float VEL_MAG = 8.0f;
    v2 circle_vel_mps[] = {
        {VEL_MAG, 0.0f},
        {-VEL_MAG, 0.0f},
        {0.0f, -VEL_MAG}
    };
    float circle_inv_mass_kg[] = {
        1.0f,
        1.0f,
        1.0f,
    };
    
    const s64 frame_time_ms = 16LL;
    const s64 engine_start_ms = get_time_ms();
    const float dt_s = 0.016f;
    s64 last_time_ms = engine_start_ms;
    while(1)
    {
        ////////////////////////////////////////////////////////////////////////////////
        // Frame start
        
        while(get_time_ms() - last_time_ms < frame_time_ms)
        {
            _mm_pause();
        }
        const s64 time_frame_start_ms = get_time_ms();
        last_time_ms = time_frame_start_ms;
        const s64 engine_time_ms = time_frame_start_ms - engine_start_ms;
        
        glfwPollEvents();
        if(glfwWindowShouldClose(glfw_window))
        {
            break;
        }
        
        glViewport(0, 0, g_screen_width, g_screen_height); GL_ERR();
        glClearColor(
                     15.0f / 256.0f,
                     5.0f / 256.0f,
                     40.0f / 256.0f,
                     0); GL_ERR();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); GL_ERR();
        glDisable(GL_CULL_FACE); GL_ERR();
        glDisable(GL_DEPTH_TEST); GL_ERR();
        glEnable(GL_BLEND); GL_ERR();
        glEnable(GL_MULTISAMPLE);
        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        ////////////////////////////////////////////////////////////////////////////////
        
        ////////////////////////////////////////////////////////////////////////////////
        // Update and render world
        
        const float aspect_ratio = (float)g_screen_width / (float)g_screen_height;
        float camera_width = 40.0f;
        const float camera_height = camera_width / aspect_ratio;
        float camera_pos_x = 0.0f;
        float camera_pos_y = 0.0f;
        
        static float gravity_mps2 = -9.8f;
        //static float gravity_mps2 = 0.0f;
        ImGui::DragFloat("gravity mps2", &gravity_mps2, 0.01f);
        static float friction_coeff = 2.0f;
        ImGui::DragFloat("friction_coeff", &friction_coeff, 0.01f);
        
        
        v2 accel_mps2[] = {
            {0.0f, gravity_mps2},
            {0.0f, gravity_mps2},
            {0.0f, gravity_mps2}
        };

        ColorId color = BLUE;
        
        // 1. Compute forces
        // Friction
        for(u64 i = 0; i < num_circles; i++)
        {
            accel_mps2[i] += -normalize_safe(circle_vel_mps[i]) * friction_coeff;
        }
        // 2. Integrate forces into velocity.
        for(u64 i = 0; i < num_circles; i++)
        {
            const v2 vel_mps = circle_vel_mps[i];
            const v2 result_vel_mps = vel_mps + accel_mps2[i] * dt_s;
            circle_vel_mps[i] = result_vel_mps;
        }

        // 2. Collision impulses
        static float e = 0.8f;
        ImGui::DragFloat("e", &e, 0.001f);
        for(u64 iter = 0; iter < 8; iter++)
        {
            for(u64 ai = 0; ai < num_circles; ai++)
            {
                const v2 a_pos_m = circle_pos_m[ai];
                const v2 a_vel_mps = circle_vel_mps[ai];
                const float a_radius_m = circle_radius_m[ai];
                const float a_inv_mass_kg = circle_inv_mass_kg[ai];

                // Other circles
                for(u64 bi = ai + 1; bi < num_circles; bi++)
                {
                    const v2 b_pos_m = circle_pos_m[bi];
                    const v2 b_vel_mps = circle_vel_mps[bi];
                    const float b_radius_m = circle_radius_m[bi];
                    const float b_inv_mass_kg = circle_inv_mass_kg[bi];

                    if(len_sq(a_pos_m - b_pos_m) < SQ(a_radius_m + b_radius_m))
                    {
                        v2 n = a_pos_m - b_pos_m;
                        const v2 relv_mps = a_vel_mps - b_vel_mps;

                        if(dot(relv_mps, n) < 0)
                        {
                            float j = dot(-(1.0f + e) * relv_mps, n) /
                                (dot(n, n) * (a_inv_mass_kg + b_inv_mass_kg));


                            const v2 a_result_vel_mps = a_vel_mps + j * a_inv_mass_kg * n;
                            const v2 b_result_vel_mps = b_vel_mps - j * b_inv_mass_kg * n;
                            circle_vel_mps[ai] = a_result_vel_mps;
                            circle_vel_mps[bi] = b_result_vel_mps;

                            color = RED;
                        }
                    }
                }

                // Walls
                if(a_pos_m.y < -5.0f)
                {
                    const v2 n = {0.0f, 1.0f};
                    const v2 relv_mps = a_vel_mps;
                    if(dot(relv_mps, n) < 0)
                    {
                        float j = dot(-(1.0f + e) * relv_mps, n) /
                                  (dot(n, n) * (a_inv_mass_kg));
                        const v2 a_result_vel_mps = a_vel_mps + j * a_inv_mass_kg * n;
                        circle_vel_mps[ai] = a_result_vel_mps;
                    }
                }
                if(a_pos_m.y > 5.0f)
                {
                    const v2 n = {0.0f, -1.0f};
                    const v2 relv_mps = a_vel_mps;
                    if(dot(relv_mps, n) < 0)
                    {
                        float j = dot(-(1.0f + e) * relv_mps, n) /
                                  (dot(n, n) * (a_inv_mass_kg));
                        const v2 a_result_vel_mps = a_vel_mps + j * a_inv_mass_kg * n;
                        circle_vel_mps[ai] = a_result_vel_mps;
                    }
                }
                if(a_pos_m.x < -5.0f)
                {
                    const v2 n = {1.0f, 0.0f};
                    const v2 relv_mps = a_vel_mps;
                    if(dot(relv_mps, n) < 0)
                    {
                        float j = dot(-(1.0f + e) * relv_mps, n) /
                                  (dot(n, n) * (a_inv_mass_kg));
                        const v2 a_result_vel_mps = a_vel_mps + j * a_inv_mass_kg * n;
                        circle_vel_mps[ai] = a_result_vel_mps;
                    }
                }
                if(a_pos_m.x > 5.0f)
                {
                    const v2 n = {-1.0f, 0.0f};
                    const v2 relv_mps = a_vel_mps;
                    if(dot(relv_mps, n) < 0)
                    {
                        float j = dot(-(1.0f + e) * relv_mps, n) /
                                  (dot(n, n) * (a_inv_mass_kg));
                        const v2 a_result_vel_mps = a_vel_mps + j * a_inv_mass_kg * n;
                        circle_vel_mps[ai] = a_result_vel_mps;
                    }
                }
            }
        }

        // 3. Integrate velocity into position.
        for(u64 i = 0; i < num_circles; i++)
        {
            const v2 pos_m = circle_pos_m[i];
            const v2 vel_mps = circle_vel_mps[i];
            const v2 result_pos_m = pos_m + vel_mps * dt_s + 0.5f * accel_mps2[i] * SQ(dt_s);
            circle_pos_m[i] = result_pos_m;
        }

        // Metrics
        v2 total_momentum = {};
        for(u64 i = 0; i < num_circles; i++)
        {
            total_momentum += circle_vel_mps[i] * (1.0f / circle_inv_mass_kg[i]);
            ImGui::Text("    vel %i: (%.3f, %.3f) : |%.3f|", i, circle_vel_mps[i].x, circle_vel_mps[i].y, len(circle_vel_mps[i]));
        }
        ImGui::Text("Total momentum: (%.3f, %.3f) : |%.3f|", total_momentum.x, total_momentum.y, len(total_momentum));
        
        graphics_data->num_quads = 0;
        graphics_data->num_circles = 0;
        
        for(u64 i = 0; i < num_circles; i++)
        {
            circle(graphics_data, circle_pos_m[i], circle_radius_m[i], color);
        }
        
        ImGui::Text("Time: %.3f s", double(engine_time_ms) / 1000.0);
        
        for(u64 i = 0; i < 100; i++) quad(graphics_data, -50.0f + float(i), 0.0f, 0.01f, 1000.0f, GRAY);
        for(u64 i = 0; i < 100; i++) quad(graphics_data, 0.0f, -50.0f + float(i), 1000.0f, 0.01f, GRAY);
        
        
        static const u32 BATCH_SIZE = 1024;
        for(u32 batch_i = 0; batch_i < graphics_data->num_quads; batch_i += BATCH_SIZE)
        {
            u32 batch_size = min_u32(graphics_data->num_quads - batch_i, BATCH_SIZE);
            glUseProgram(color_shader_program); GL_ERR();
            const float view_proj_mat[] = {
                2.0f / camera_width, 0.0f, 0.0f, -camera_pos_x / (camera_width*0.5f),
                0.0f, 2.0f / camera_height, 0.0f, camera_pos_y / (camera_height*0.5f),
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f,
            };
            GLint loc = glGetUniformLocation(color_shader_program, "u_view_proj_mat"); GL_ERR();
            assert(loc != -1);
            glUniformMatrix4fv(loc, 1, 1, &(view_proj_mat[0])); GL_ERR();
            
            glBindVertexArray(vao); GL_ERR();
            
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, quad_ssbo); GL_ERR();
            
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  0 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->quad_x + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  1 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->quad_y + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  2 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->quad_w + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  3 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->quad_h + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  4 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->quad_r + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  5 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->quad_g + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  6 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->quad_b + batch_i); GL_ERR();
            
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, quad_ssbo); GL_ERR();
            glBindBuffer(GL_ARRAY_BUFFER, vbo); GL_ERR();
            glBindTexture(GL_TEXTURE_2D, 0); GL_ERR();
            
            glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4, batch_size); GL_ERR();
        }
        for(u32 batch_i = 0; batch_i < graphics_data->num_circles; batch_i += BATCH_SIZE)
        {
            u32 batch_size = min_u32(graphics_data->num_circles - batch_i, BATCH_SIZE);
            glUseProgram(circle_shader_program); GL_ERR();
            const float view_proj_mat[] = {
                2.0f / camera_width, 0.0f, 0.0f, -camera_pos_x / (camera_width*0.5f),
                0.0f, 2.0f / camera_height, 0.0f, camera_pos_y / (camera_height*0.5f),
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f,
            };
            GLint loc = glGetUniformLocation(circle_shader_program, "u_view_proj_mat"); GL_ERR();
            assert(loc != -1);
            glUniformMatrix4fv(loc, 1, 1, &(view_proj_mat[0])); GL_ERR();
            
            glBindVertexArray(vao); GL_ERR();
            
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, circle_ssbo); GL_ERR();
            
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  0 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->circle_x + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  1 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->circle_y + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  2 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->circle_radius + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  3 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->circle_r + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  4 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->circle_g + batch_i); GL_ERR();
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,  5 * SSBO_STRIDE, sizeof(float) * batch_size, graphics_data->circle_b + batch_i); GL_ERR();
            
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, circle_ssbo); GL_ERR();
            glBindBuffer(GL_ARRAY_BUFFER, vbo); GL_ERR();
            glBindTexture(GL_TEXTURE_2D, 0); GL_ERR();
            
            glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4, batch_size); GL_ERR();
        }
        ////////////////////////////////////////////////////////////////////////////////
        
        ////////////////////////////////////////////////////////////////////////////////
        // Frame end
        
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        //glFlush(); GL_ERR();
        //glFinish(); GL_ERR();
        glfwSwapBuffers(glfw_window);
        
        ////////////////////////////////////////////////////////////////////////////////
    }
    
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwTerminate();
    }
    
    return 0;
}


