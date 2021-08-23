
#include "graphics.h"
#include "game_math.h"

#include <cstdio>  // fopen
#include <cstring> // strncmp
#include <vector>
#include <map>
#include <cassert>

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>



using namespace GameMath;



struct CameraState
{
    v2 position = v2();
    float width = 10.0f;
};

struct ObjectBuffer
{
    GLuint vao;
    GLuint vbo;
    int bytes_capacity;
};

struct Texture
{
    GLuint handle;
};

struct Shader
{
    GLuint program;
};

struct GraphicsState
{
    GLFWwindow *window;

    float screen_aspect_ratio;

    Texture *white_texture;

    v2 mouse_screen_position;

    Shader *batch_quad_shader;
    ObjectBuffer *batch_quad_buffer;
    struct QuadRenderingData
    {
        v2 position;
        v2 half_extents;
        v4 color;
        float rotation;
    };

    Shader *batch_circle_shader;
    ObjectBuffer *batch_circle_buffer;
    struct CircleRenderingData
    {
        v2 position;
        float radius;
        v4 color;
    };

    void render_quad_batch(std::vector<QuadRenderingData> *packed_data);
    void render_circle_batch(std::vector<CircleRenderingData> *packed_data);

    struct LayerGroup
    {
        std::vector<GraphicsState::QuadRenderingData> quads_packed_buffer;
        void pack_quad(QuadRenderingData *quad)
        {
            quads_packed_buffer.push_back({quad->position, quad->half_extents, quad->color, quad->rotation});
        }

        std::vector<GraphicsState::CircleRenderingData> circles_packed_buffer;
        void pack_circle(CircleRenderingData *circle)
        {
            circles_packed_buffer.push_back({circle->position, circle->radius, circle->color});
        }
    };
    std::map<int, LayerGroup *> *objects_to_render;
};
GraphicsState *Graphics::instance = nullptr;
CameraState *Graphics::Camera::instance = nullptr;



////////////////////////////////////////////////////////////////////////////////
// GLFW
////////////////////////////////////////////////////////////////////////////////
static void check_gl_errors(const char *desc)
{
    GLint error = glGetError();
    if(error)
    {
        fprintf(stderr, "Error %i: %s\n", error, desc);
        assert(false);
    }
}

static void reshape(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, (GLint)width, (GLint)height);
    Graphics::instance->screen_aspect_ratio = (float)width / height;
}

static void key(GLFWwindow* window, int k, int s, int action, int mods)
{
    if(action != GLFW_PRESS) return;

    switch (k) {
        case GLFW_KEY_Z:
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            break;
        case GLFW_KEY_DOWN:
            break;
        case GLFW_KEY_LEFT:
            break;
        case GLFW_KEY_RIGHT:
            break;
        default:
            return;
    }
}



////////////////////////////////////////////////////////////////////////////////
// Shaders
////////////////////////////////////////////////////////////////////////////////

static bool is_newline(char *c)
{
    return (c[0] == '\n' || c[0] == '\r' || (c[0] == '\r' && c[1] == '\n'));
}

static char *read_file_into_string(const char *path)
{
    FILE *file = fopen(path, "rb");
    if(file == nullptr) return nullptr;
    fseek(file, 0L, SEEK_END);
    int size = ftell(file);
    fseek(file, 0L, SEEK_SET);
    char *buffer = new char[size + 1]();
    fread(buffer, size, 1, file);
    buffer[size] = '\0';
    return buffer;
}

static bool read_shader_file(const char *path, char **vert_source, char **geom_source, char **frag_source)
{
    *vert_source = nullptr;
    *geom_source = nullptr;
    *frag_source = nullptr;

    char *file = read_file_into_string(path);
    if(file == nullptr) return false;

    int current_tag_length = 0;
    char *current_tag = nullptr;
    char *current_source_start = nullptr;

    char *character = file;
    while(*character != '\0')
    {
        if(*character == '@')
        {
            // Finish reading current shader source
            if(current_tag == nullptr)
            {
            }
            else if(strncmp("vertex", current_tag, current_tag_length) == 0)
            {
                *vert_source = current_source_start;
            }
            else if(strncmp("geometry", current_tag, current_tag_length) == 0)
            {
                *geom_source = current_source_start;
            }
            else if(strncmp("fragment", current_tag, current_tag_length) == 0)
            {
                *frag_source = current_source_start;
            }


            // Null terminate previous shader string
            *character = '\0';

            // Read tag
            character++;
            char *tag = character;

            // Move past tag
            while(!is_newline(character))
            {
                character++;
            }
            char *one_past_end_tag = character;
            while(is_newline(character)) character++;

            current_tag_length = one_past_end_tag - tag;
            current_tag = tag;
            current_source_start = character;
        }
        else
        {
            character++;
        }
    }

    // Finish reading current shader source
    if(current_tag == nullptr)
    {
    }
    else if(strncmp("vertex", current_tag, current_tag_length) == 0)
    {
        *vert_source = current_source_start;
    }
    else if(strncmp("geometry", current_tag, current_tag_length) == 0)
    {
        *geom_source = current_source_start;
    }
    else if(strncmp("fragment", current_tag, current_tag_length) == 0)
    {
        *frag_source = current_source_start;
    }

    return true;
}

Shader *make_shader(const char *shader_path)
{
    Shader *result = new Shader();

    int  success;
    char info_log[512];

    char *vert_source = nullptr;
    char *geom_source = nullptr;
    char *frag_source = nullptr;
    bool read = read_shader_file(shader_path, &vert_source, &geom_source, &frag_source);
    if(!read)
    {
        fprintf(stderr, "Could not open shader file %s\n", shader_path);
        return nullptr;
    }

    unsigned int vert_shader;
    vert_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert_shader, 1, &vert_source, NULL);
    glCompileShader(vert_shader);
    glGetShaderiv(vert_shader, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(vert_shader, 512, NULL, info_log);
        fprintf(stderr, "VERTEX SHADER ERROR : %s\n%s\n", shader_path, info_log);
        return nullptr;
    }

    unsigned int frag_shader;
    frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag_shader, 1, &frag_source, NULL);
    glCompileShader(frag_shader);
    glGetShaderiv(frag_shader, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(frag_shader, 512, NULL, info_log);
        fprintf(stderr, "VERTEX SHADER ERROR : %s\n%s\n", shader_path, info_log);
        return nullptr;
    }


    unsigned int geom_shader;
    if(geom_source != nullptr)
    {
        geom_shader = glCreateShader(GL_GEOMETRY_SHADER);
        glShaderSource(geom_shader, 1, &geom_source, NULL);
        glCompileShader(geom_shader);
        glGetShaderiv(geom_shader, GL_COMPILE_STATUS, &success);
        if(!success)
        {
            glGetShaderInfoLog(geom_shader, 512, NULL, info_log);
            fprintf(stderr, "VERTEX SHADER ERROR : %s\n%s\n", shader_path, info_log);
            return nullptr;
        }
    }

    check_gl_errors("compiling shaders");

    GLuint program = glCreateProgram();
    check_gl_errors("making program");

    glAttachShader(program, vert_shader);
    glAttachShader(program, frag_shader);
    if(geom_source != nullptr) glAttachShader(program, geom_shader);
    glLinkProgram(program);
    check_gl_errors("linking program");

    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if(!success)
    {
        glGetProgramInfoLog(program, 512, NULL, info_log);
        fprintf(stderr, "SHADER LINKING ERROR : %s\n%s\n", shader_path, info_log);
        return nullptr;
    }

    glDeleteShader(vert_shader);
    glDeleteShader(frag_shader); 
    check_gl_errors("deleting shaders");

    result->program = program;

    return result;
}

void use_shader(Shader *shader)
{
    glUseProgram(shader->program);
    check_gl_errors("use program");
}

void set_uniform(Shader *shader, const char *name, v2 value)
{
    GLint loc = glGetUniformLocation(shader->program, name);
    glUniform2f(loc, value.x, value.y);
    if(loc == -1) fprintf(stderr, "Could not find uniform \"%s\"", name);
}

void set_uniform(Shader *shader, const char *name, v4 value)
{
    GLint loc = glGetUniformLocation(shader->program, name);
    glUniform4f(loc, value.x, value.y, value.z, value.w);
    if(loc == -1) fprintf(stderr, "Could not find uniform \"%s\"", name);
}

void set_uniform(Shader *shader, const char *name, mat4 value)
{
    GLint loc = glGetUniformLocation(shader->program, name);
    glUniformMatrix4fv(loc, 1, true, &(value[0][0]));
    if(loc == -1) fprintf(stderr, "Could not find uniform \"%s\"", name);
}



////////////////////////////////////////////////////////////////////////////////
// Textures
////////////////////////////////////////////////////////////////////////////////
static Texture *make_texture_from_bitmap(int width, int height, char *bitmap)
{
    Texture *texture = new Texture();

    glGenTextures(1, &texture->handle);
    glBindTexture(GL_TEXTURE_2D, texture->handle);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    check_gl_errors("make texture");

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, bitmap);
    check_gl_errors("send texture data");

    glGenerateMipmap(GL_TEXTURE_2D);
    check_gl_errors("generate mipmap");

    return texture;
}

static void use_texture(Texture *texture)
{
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture->handle);
}

void GraphicsState::render_quad_batch(std::vector<GraphicsState::QuadRenderingData> *packed_data)
{
    if(packed_data->empty()) return;

    Shader *shader = batch_quad_shader;
    ObjectBuffer *object_buffer = batch_quad_buffer;

    use_shader(shader);
    mat4 project_m_world = Graphics::ndc_m_world();
    set_uniform(shader, "vp", project_m_world);

    glBindVertexArray(object_buffer->vao);
    check_gl_errors("use vao");
    glBindBuffer(GL_ARRAY_BUFFER, object_buffer->vbo);
    int bytes = sizeof((*packed_data)[0]) * packed_data->size();
    assert(bytes <= (object_buffer->bytes_capacity));
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, packed_data->data());
    check_gl_errors("send quad data");

    use_texture(white_texture);

    glFinish();

    glDrawArrays(GL_POINTS, 0, packed_data->size());
}

void GraphicsState::render_circle_batch(std::vector<CircleRenderingData> *packed_data)
{
    if(packed_data->empty()) return;

    Shader *shader = batch_circle_shader;
    ObjectBuffer *object_buffer = batch_circle_buffer;

    use_shader(shader);
    mat4 project_m_world = Graphics::ndc_m_world();
    set_uniform(shader, "vp", project_m_world);

    glBindVertexArray(object_buffer->vao);
    check_gl_errors("use vao");
    glBindBuffer(GL_ARRAY_BUFFER, object_buffer->vbo);
    int bytes = sizeof((*packed_data)[0]) * packed_data->size();
    assert(bytes <= (object_buffer->bytes_capacity));
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, packed_data->data());
    check_gl_errors("send circle data");

    glFinish();

    glDrawArrays(GL_POINTS, 0, packed_data->size());
}




////////////////////////////////////////////////////////////////////////////////
// Rendering
////////////////////////////////////////////////////////////////////////////////
static GraphicsState::LayerGroup *get_or_add_layer_group(int layer)
{
    GraphicsState::LayerGroup *group = (*Graphics::instance->objects_to_render)[layer];
    if(group == nullptr)
    {
        group = new GraphicsState::LayerGroup();
        (*Graphics::instance->objects_to_render)[layer] = group;
    }

    return group;
}

void Graphics::quad(v2 position, v2 half_extents, float rotation, v4 color, int layer)
{
    GraphicsState::LayerGroup *group = get_or_add_layer_group(layer);
    v2 scale = half_extents * 2.0f;
    GraphicsState::QuadRenderingData data = {position, scale, color, rotation};
    group->pack_quad(&data);
}

void Graphics::circle(GameMath::v2 position, float radius, GameMath::v4 color, int layer)
{
    GraphicsState::LayerGroup *group = get_or_add_layer_group(layer);
    GraphicsState::CircleRenderingData data = {position, radius, color};
    group->pack_circle(&data);
}





void glfw_error_fn(int error, const char *message)
{
    printf("%i: %s\n", error, message);
}

bool Graphics::init()
{
    instance = new GraphicsState();

    {
        glfwSetErrorCallback(glfw_error_fn);
        if(!glfwInit())
        {
            fprintf(stderr, "Failed to initialize GLFW\n");
            return false;
        }

        instance->window = glfwCreateWindow(300, 300, "My Game", NULL, NULL);
        if(!instance->window)
        {
            fprintf(stderr, "Failed to open GLFW window\n");
            glfwTerminate();
            return false;
        }

        // Set callback functions
        glfwSetFramebufferSizeCallback(instance->window, reshape);
        glfwSetKeyCallback(instance->window, key);

        glfwMakeContextCurrent(instance->window);

        if(!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
        {
            fprintf(stderr, "Failed to initialize OpenGL context\n");
            return false;
        }

        glfwSwapInterval(1);

        int framebuffer_width, framebuffer_height;
        glfwGetFramebufferSize(instance->window, &framebuffer_width, &framebuffer_height);
        reshape(instance->window, framebuffer_width, framebuffer_height);
    }

    //Graphics::ImGuiImplementation::init();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    Camera::instance = new CameraState();

    int framebuffer_width, framebuffer_height;
    glfwGetFramebufferSize(instance->window, &framebuffer_width, &framebuffer_height);
    instance->screen_aspect_ratio = (float)framebuffer_width / framebuffer_height;

    instance->objects_to_render = new std::map<int, GraphicsState::LayerGroup *>();

    instance->batch_quad_shader = make_shader("assets/shaders/batch_quad.shader");
    instance->batch_circle_shader = make_shader("assets/shaders/batch_circle.shader");

    // Make quad object buffer
    {
        instance->batch_quad_buffer = new ObjectBuffer();
        glGenVertexArrays(1, &instance->batch_quad_buffer->vao);
        glBindVertexArray(instance->batch_quad_buffer->vao);
        check_gl_errors("making vao");

        glGenBuffers(1, &instance->batch_quad_buffer->vbo);
        check_gl_errors("making vbo");

        static const int MAX_QUADS = 1024 * 10;
        glBindBuffer(GL_ARRAY_BUFFER, instance->batch_quad_buffer->vbo);
        glBufferData(GL_ARRAY_BUFFER, MAX_QUADS * sizeof(GraphicsState::QuadRenderingData), nullptr, GL_DYNAMIC_DRAW);
        check_gl_errors("send vbo data");
        instance->batch_quad_buffer->bytes_capacity = MAX_QUADS * sizeof(GraphicsState::QuadRenderingData);

        float stride = sizeof(GraphicsState::QuadRenderingData);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, (void *)0); // Position
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void *)(1 * sizeof(v2))); // Scale
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride, (void *)(2 * sizeof(v2))); // Color
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, stride, (void *)(2 * sizeof(v2) + sizeof(v4))); // Rotation
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glEnableVertexAttribArray(3);
        check_gl_errors("vertex attrib pointer");
    }

    // Make circle object buffer
    {
        instance->batch_circle_buffer = new ObjectBuffer();
        glGenVertexArrays(1, &instance->batch_circle_buffer->vao);
        glBindVertexArray(instance->batch_circle_buffer->vao);
        check_gl_errors("making vao");

        glGenBuffers(1, &instance->batch_circle_buffer->vbo);
        check_gl_errors("making vbo");

        static const int MAX_CIRCLES = 1024 * 10;
        glBindBuffer(GL_ARRAY_BUFFER, instance->batch_circle_buffer->vbo);
        glBufferData(GL_ARRAY_BUFFER, MAX_CIRCLES * sizeof(GraphicsState::CircleRenderingData), nullptr, GL_DYNAMIC_DRAW);
        check_gl_errors("send vbo data");
        instance->batch_circle_buffer->bytes_capacity = MAX_CIRCLES * sizeof(GraphicsState::CircleRenderingData);

        float stride = sizeof(GraphicsState::CircleRenderingData);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, (void *)0); // Position
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, stride, (void *)(2 * sizeof(float))); // Radius
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride, (void *)(3 * sizeof(float))); // Color
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        check_gl_errors("vertex attrib pointer");
    }
    
    // Make texture
    {
        char bitmap[] = { (char)255, (char)255, (char)255, (char)255 };
        instance->white_texture = make_texture_from_bitmap(1, 1, bitmap);
    }

    return true;
}

void Graphics::uninit()
{
    // Terminate GLFW
    glfwTerminate();
}

void Graphics::clear_frame(v4 color)
{
    glClearColor(color.r, color.g, color.b, color.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Graphics::render()
{
    std::map<int, GraphicsState::LayerGroup *> *objects_to_render = instance->objects_to_render;

    for(std::pair<int, GraphicsState::LayerGroup *> pair : *objects_to_render)
    {
        GraphicsState::LayerGroup *group = pair.second;

        // Render all quads
        instance->render_quad_batch(&group->quads_packed_buffer);
        group->quads_packed_buffer.clear();

        // Render all circles
        instance->render_circle_batch(&group->circles_packed_buffer);
        group->circles_packed_buffer.clear();

    }

}

void Graphics::swap_frames()
{
    glFinish();
    glfwSwapBuffers(instance->window);

    // TODO: Probably move these
    glfwPollEvents();
    double x, y;
    glfwGetCursorPos(instance->window, &x, &y);
    instance->mouse_screen_position.x = (float)x;
    instance->mouse_screen_position.y = (float)y;
}

bool Graphics::wants_to_close()
{
    return glfwWindowShouldClose(instance->window);
}



GameMath::mat4 Graphics::view_m_world()
{
    return make_translation_matrix(v3(-Camera::instance->position, 0.0f));
}

GameMath::mat4 Graphics::world_m_view()
{
    return inverse(view_m_world());
}

GameMath::mat4 Graphics::ndc_m_world()
{
    mat4 ndc_m_view =
    {
        2.0f / Camera::width(), 0.0f, 0.0f, 0.0f,
        0.0f, (2.0f / Camera::width()) * instance->screen_aspect_ratio, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    return ndc_m_view * view_m_world();
}

GameMath::mat4 Graphics::world_m_ndc()
{
    return inverse(ndc_m_world());
}

GameMath::v2 Graphics::mouse_world_position()
{
    GameMath::v2 p = instance->mouse_screen_position;
    int framebuffer_width, framebuffer_height;
    glfwGetFramebufferSize(instance->window, &framebuffer_width, &framebuffer_height);
    float screen_width =(float)framebuffer_width; 
    float screen_height = (float)framebuffer_height;

    p.y = screen_height - p.y;

    GameMath::v2 ndc =
    {
        (p.x / screen_width)  * 2.0f - 1.0f,
        (p.y / screen_height) * 2.0f - 1.0f,
    };

    GameMath::v4 ndc4 = {ndc, 0.0f, 1.0f};
    GameMath::v4 world4 = Graphics::world_m_ndc() * ndc4;

    return GameMath::v2(world4.x, world4.y);
}



GameMath::v2 &Graphics::Camera::position()
{
    return instance->position;
}

float &Graphics::Camera::width()
{
    return instance->width;
}

float Graphics::Camera::height()
{
    return instance->width / Graphics::Camera::aspect_ratio();
}

float Graphics::Camera::aspect_ratio()
{
    return Graphics::instance->screen_aspect_ratio;
}






#if 0

// ImGui Implementation
void Graphics::ImGuiImplementation::init()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();
    
    // Setup Platform/Renderer bindings
    ImGui_ImplOpenGL3_Init("#version 440 core");
    ImGui_ImplWin32_Init(Windows::handle());
}

void Graphics::ImGuiImplementation::new_frame()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
}

void Graphics::ImGuiImplementation::end_frame()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Graphics::ImGuiImplementation::shutdown()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}


#endif
