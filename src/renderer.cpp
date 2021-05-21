
#include "renderer.h"
#include "engine.h"
#include "platform.h"
#include "memory.h"
#include "my_math.h"
#include "shader.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include "GL/glew.h"
#include "GL/wglew.h"
#include <windows.h>
#include <assert.h>

#include <string>
#include <map>
#include <vector>



struct Camera
{
    v2 position;
    float width;
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

struct RendererState
{
    HGLRC gl_context;

    Camera *camera;
    float screen_aspect_ratio;

    Shader *batch_rect_shader;
    ObjectBuffer *batch_rect_buffer;
    struct RectRenderingData
    {
        v2 position;
        v2 half_extents;
        v4 color;
        float rotation;
    };

    Shader *batch_triangle_shader;
    ObjectBuffer *batch_triangle_buffer;
    struct TriangleRenderingData
    {
        v2 a;
        v4 color0;
        v2 b;
        v4 color1;
        v2 c;
        v4 color2;
    };

    Shader *batch_circle_shader;
    ObjectBuffer *batch_circle_buffer;
    struct CircleRenderingData
    {
        v2 center;
        float radius;
        v4 color;
    };

    Shader *batch_line_shader;
    ObjectBuffer *batch_line_buffer;
    struct LineRenderingData
    {
        v2 a;
        v2 b;
        float width;
        v4 color;
    };

    std::map<std::string, Texture *> *texture_map;
    Texture *white_texture;
    
    struct LayerGroup
    {
        std::vector<Rect *> rects_retained;
        std::vector<RendererState::RectRenderingData> rects_packed_buffer;
        void pack_rect(Rect *rect)
        {
            rects_packed_buffer.push_back({rect->center, rect->half_extents, rect->color, rect->rotation});
        }

        std::vector<Triangle *> triangles_retained;
        std::vector<RendererState::TriangleRenderingData> triangles_packed_buffer;
        void pack_triangle(Triangle *tri)
        {
            triangles_packed_buffer.push_back({tri->a, tri->color, tri->b, tri->color, tri->c, tri->color});
        }

        std::vector<Circle *> circles_retained;
        std::vector<RendererState::CircleRenderingData> circles_packed_buffer;
        void pack_circle(Circle *circle)
        {
            circles_packed_buffer.push_back({circle->position, circle->radius, circle->color});
        }

        std::vector<Line *> lines_retained;
        std::vector<RendererState::LineRenderingData> lines_packed_buffer;
        void pack_line(Line *line)
        {
            lines_packed_buffer.push_back({line->a, line->b, line->width, line->color});
        }
    };
    std::map<int, LayerGroup *> *objects_to_render;

};
static const int TARGET_GL_VERSION[2] = { 4, 4 }; // {major, minor}
static const int MAX_TRIANGLES = 256;
static const int CIRCLE_MESH_RESOLUTION = 256;



#pragma region Helper Functions
static void check_gl_errors(const char *desc)
{
    GLint error = glGetError();
    if(error)
    {
        log_error("Error %i: %s\n", error, desc);
        assert(false);
    }
}

static void create_gl_context(RendererState *renderer_state)
{
    PlatformState *platform_state = get_engine_platform_state();
    HDC dc = get_device_context(platform_state);

    PIXELFORMATDESCRIPTOR pfd;
    memset(&pfd, 0, sizeof(PIXELFORMATDESCRIPTOR));
    pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DOUBLEBUFFER | PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 32;
    pfd.iLayerType = PFD_MAIN_PLANE;

    int pixel_format = ChoosePixelFormat(dc, &pfd);
    if(pixel_format == 0) return;

    BOOL result = SetPixelFormat(dc, pixel_format, &pfd);
    if(!result) return;

    HGLRC temp_context = wglCreateContext(dc);
    wglMakeCurrent(dc, temp_context);

    glewExperimental = true;
    GLenum error = glewInit();
    if(error != GLEW_OK)
    {
        OutputDebugString("GLEW is not initialized!");
    }

    int attribs[] =
    {
        WGL_CONTEXT_MAJOR_VERSION_ARB, TARGET_GL_VERSION[0],
        WGL_CONTEXT_MINOR_VERSION_ARB, TARGET_GL_VERSION[1],
        WGL_CONTEXT_FLAGS_ARB, 0,
        0
    };

    if(wglewIsSupported("WGL_ARB_create_context") == 1)
    {
        renderer_state->gl_context = wglCreateContextAttribsARB(dc, 0, attribs);
        wglMakeCurrent(NULL, NULL);
        wglDeleteContext(temp_context);
        wglMakeCurrent(dc, renderer_state->gl_context);
    }
    else
    {   //It's not possible to make a GL 3.x context. Use the old style context (GL 2.1 and before)
        renderer_state->gl_context = temp_context;
    }

    //Checking GL version
    const GLubyte *GLVersionString = glGetString(GL_VERSION);

    //Or better yet, use the GL3 way to get the version number
    int OpenGLVersion[2];
    glGetIntegerv(GL_MAJOR_VERSION, &OpenGLVersion[0]);
    glGetIntegerv(GL_MINOR_VERSION, &OpenGLVersion[1]);

    if(!renderer_state->gl_context) return; // Bad
}

static Texture *make_texture_from_bitmap(int width, int height, char *bitmap)
{
    Texture *texture = (Texture *)my_allocate(sizeof(Texture));

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

static Texture *make_texture(const char *texture_path)
{
    stbi_set_flip_vertically_on_load(true);
    int width, height, channels;
    unsigned char *bitmap = stbi_load(texture_path, &width, &height, &channels, 4);
    if(bitmap == NULL)
    {
        log_error("Could not load texture: %s", texture_path);
        return nullptr;
    }

    return make_texture_from_bitmap(width, height, (char *)bitmap);
}

static Texture *get_or_make_texture(const char *texture_path)
{
    RendererState *renderer_state = get_engine_renderer_state();
    if(renderer_state->texture_map->find(texture_path) != renderer_state->texture_map->end())
    {
        return (*renderer_state->texture_map)[texture_path]; 
    }
    else
    {
        Texture *new_texture = make_texture(texture_path);
        (*renderer_state->texture_map)[texture_path] = new_texture;
        return new_texture;
    }
}

static void use_texture(Texture *texture)
{
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture->handle);

    check_gl_errors("use texture");
}

static mat4 ndc_m_world(Camera *camera, float screen_aspect_ratio)
{
    mat4 view_m_world = make_translation_matrix(v3(-camera->position, 0.0f));
    mat4 ndc_m_view =
    {
        2.0f / camera->width, 0.0f, 0.0f, 0.0f,
        0.0f, (2.0f / camera->width) * screen_aspect_ratio, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    return ndc_m_view * view_m_world;
}
#pragma endregion

#pragma region Batch Rendering
static void render_rect_batch(std::vector<RendererState::RectRenderingData> *packed_data)
{
    if(packed_data->empty()) return;

    RendererState *renderer_state = get_engine_renderer_state();
    Shader *shader = renderer_state->batch_rect_shader;
    ObjectBuffer *object_buffer = renderer_state->batch_rect_buffer;

    use_shader(shader);
    mat4 project_m_world = ndc_m_world(renderer_state->camera, renderer_state->screen_aspect_ratio);
    set_uniform(shader, "vp", project_m_world);

    glBindVertexArray(object_buffer->vao);
    check_gl_errors("use vao");
    glBindBuffer(GL_ARRAY_BUFFER, object_buffer->vbo);
    int bytes = sizeof((*packed_data)[0]) * packed_data->size();
    assert(bytes <= (object_buffer->bytes_capacity));
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, packed_data->data());
    check_gl_errors("send rect data");

    use_texture(renderer_state->white_texture);

    glFinish();

    glDrawArrays(GL_POINTS, 0, packed_data->size());
}

static void render_triangle_batch(std::vector<RendererState::TriangleRenderingData> *packed_data)
{
    if(packed_data->empty()) return;

    RendererState *renderer_state = get_engine_renderer_state();
    Shader *shader = renderer_state->batch_triangle_shader;
    ObjectBuffer *object_buffer = renderer_state->batch_triangle_buffer;

    use_shader(shader);
    mat4 project_m_world = ndc_m_world(renderer_state->camera, renderer_state->screen_aspect_ratio);
    set_uniform(shader, "vp", project_m_world);

    glBindVertexArray(object_buffer->vao);
    check_gl_errors("use vao");
    glBindBuffer(GL_ARRAY_BUFFER, object_buffer->vbo);
    int bytes = sizeof((*packed_data)[0]) * packed_data->size();
    assert(bytes <= (object_buffer->bytes_capacity));
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, packed_data->data());
    check_gl_errors("send tri data");

    //use_texture(renderer_state->white_texture);

    glFinish();

    glDrawArrays(GL_TRIANGLES, 0, packed_data->size() * 3);
}

static void render_circle_batch(std::vector<RendererState::CircleRenderingData> *packed_data)
{
    if(packed_data->empty()) return;

    RendererState *renderer_state = get_engine_renderer_state();
    Shader *shader = renderer_state->batch_circle_shader;
    ObjectBuffer *object_buffer = renderer_state->batch_circle_buffer;

    use_shader(shader);
    mat4 project_m_world = ndc_m_world(renderer_state->camera, renderer_state->screen_aspect_ratio);
    set_uniform(shader, "vp", project_m_world);

    glBindVertexArray(object_buffer->vao);
    check_gl_errors("use vao");
    glBindBuffer(GL_ARRAY_BUFFER, object_buffer->vbo);
    int bytes = sizeof((*packed_data)[0]) * packed_data->size();
    assert(bytes <= (object_buffer->bytes_capacity));
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, packed_data->data());
    check_gl_errors("send circle data");

    //use_texture(renderer_state->white_texture);

    glFinish();

    glDrawArrays(GL_POINTS, 0, packed_data->size());
}

static void render_line_batch(std::vector<RendererState::LineRenderingData> *packed_data)
{
    if(packed_data->empty()) return;

    RendererState *renderer_state = get_engine_renderer_state();
    Shader *shader = renderer_state->batch_line_shader;
    ObjectBuffer *object_buffer = renderer_state->batch_line_buffer;

    use_shader(shader);
    mat4 project_m_world = ndc_m_world(renderer_state->camera, renderer_state->screen_aspect_ratio);
    set_uniform(shader, "vp", project_m_world);

    glBindVertexArray(object_buffer->vao);
    check_gl_errors("use vao");
    glBindBuffer(GL_ARRAY_BUFFER, object_buffer->vbo);
    int bytes = sizeof((*packed_data)[0]) * packed_data->size();
    assert(bytes <= (object_buffer->bytes_capacity));
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, packed_data->data());
    check_gl_errors("send line data");

    //use_texture(renderer_state->white_texture);

    glFinish();

    glDrawArrays(GL_POINTS, 0, packed_data->size());
}
#pragma endregion




static RendererState::LayerGroup *get_or_add_layer_group(int layer)
{
    RendererState::LayerGroup *group = (*get_engine_renderer_state()->objects_to_render)[layer];
    if(group == nullptr)
    {
        group = new RendererState::LayerGroup();
        (*get_engine_renderer_state()->objects_to_render)[layer] = group;
    }

    return group;
}

void add_rect_immediate(Rect rect, int layer)
{
    RendererState::LayerGroup *group = get_or_add_layer_group(layer);
    group->pack_rect(&rect);
}

Rect *add_rect_retained(Rect rect, int layer)
{
    RendererState::LayerGroup *group = get_or_add_layer_group(layer);
    Rect *new_rect = new Rect();
    *new_rect = rect;
    group->rects_retained.push_back(new_rect);
    return new_rect;
}

void remove_rect(Rect *rect)
{
    if(rect == nullptr) return;

    RendererState *renderer_state = get_engine_renderer_state();

    RendererState::LayerGroup *found_group = nullptr;
    Rect *rect_pos;
    for(std::pair<int, RendererState::LayerGroup *> pair : *(renderer_state->objects_to_render))
    {
        RendererState::LayerGroup *layer_group = pair.second;

        for(Rect *current_rect : layer_group->rects_retained)
        {
            if(current_rect == rect)
            {
                found_group = layer_group;
                rect_pos = current_rect;
                break;
            }
        }

        if(found_group != nullptr)
        {
            break;
        }
    }

    if(found_group)
    {
        // NOTE:
        // Need to remove the rect from this layer's list. While it shouldn't matter what order the rects are drawn in
        // on this layer, changing the existing order might cause objects to flicker in front and behind other objects.
        // For now, I'm doing a naive order-preserving erase from the list.
        std::vector<Rect *> &rects = found_group->rects_retained;
        rects.erase(std::find(rects.begin(), rects.end(), rect_pos));
    }
}



void add_circle_immediate(Circle circle, int layer)
{
    RendererState::LayerGroup *group = get_or_add_layer_group(layer);
    group->pack_circle(&circle);
}

Circle *add_circle_retained(Circle circle, int layer)
{
    RendererState::LayerGroup *group = get_or_add_layer_group(layer);
    Circle *new_circle = new Circle();
    *new_circle = circle;
    group->circles_retained.push_back(new_circle);
    return new_circle;
}

void remove_circle(Circle *circle)
{
    if(circle == nullptr) return;

    RendererState *renderer_state = get_engine_renderer_state();

    RendererState::LayerGroup *found_group = nullptr;
    Circle *circle_pos;
    for(std::pair<int, RendererState::LayerGroup *> pair : *(renderer_state->objects_to_render))
    {
        RendererState::LayerGroup *layer_group = pair.second;

        for(Circle *current_circle : layer_group->circles_retained)
        {
            if(current_circle == circle)
            {
                found_group = layer_group;
                circle_pos = current_circle;
                break;
            }
        }

        if(found_group != nullptr)
        {
            break;
        }
    }

    if(found_group)
    {
        // NOTE:
        // Need to remove the circle from this layer's list. While it shouldn't matter what order the circles are drawn in
        // on this layer, changing the existing order might cause objects to flicker in front and behind other objects.
        // For now, I'm doing a naive order-preserving erase from the list.
        std::vector<Circle *> &circles = found_group->circles_retained;
        circles.erase(std::find(circles.begin(), circles.end(), circle_pos));
    }
}



void add_triangle_immediate(Triangle tri, int layer)
{
    RendererState::LayerGroup *group = get_or_add_layer_group(layer);
    group->pack_triangle(&tri);
}

Triangle *add_triangle_retained(Triangle tri, int layer)
{
    RendererState::LayerGroup *group = get_or_add_layer_group(layer);
    Triangle *new_tri = new Triangle();
    *new_tri = tri;
    group->triangles_retained.push_back(new_tri);
    return new_tri;
}

void remove_triangle(Triangle *triangle)
{
    if(triangle == nullptr) return;

    RendererState *renderer_state = get_engine_renderer_state();

    RendererState::LayerGroup *found_group = nullptr;
    Triangle *triangle_pos;
    for(std::pair<int, RendererState::LayerGroup *> pair : *(renderer_state->objects_to_render))
    {
        RendererState::LayerGroup *layer_group = pair.second;

        for(Triangle *current_tri : layer_group->triangles_retained)
        {
            if(current_tri == triangle)
            {
                found_group = layer_group;
                triangle_pos = current_tri;
                break;
            }
        }

        if(found_group != nullptr)
        {
            break;
        }
    }

    if(found_group)
    {
        // NOTE:
        // Need to remove the triangle from this layer's list. While it shouldn't matter what order the triangles are drawn in
        // on this layer, changing the existing order might cause objects to flicker in front and behind other objects.
        // For now, I'm doing a naive order-preserving erase from the list.
        std::vector<Triangle *> &tris = found_group->triangles_retained;
        tris.erase(std::find(tris.begin(), tris.end(), triangle_pos));
    }
}



void add_line_immediate(Line line, int layer)
{
    RendererState::LayerGroup *group = get_or_add_layer_group(layer);
    group->pack_line(&line);
}

Line *add_line_retained(Line line, int layer)
{
    RendererState::LayerGroup *group = get_or_add_layer_group(layer);
    Line *new_line = new Line();
    *new_line = line;
    group->lines_retained.push_back(new_line);
    return new_line;
}

void remove_line(Line *line)
{
    if(line == nullptr) return;

    RendererState *renderer_state = get_engine_renderer_state();

    RendererState::LayerGroup *found_group = nullptr;
    Line *line_pos;
    for(std::pair<int, RendererState::LayerGroup *> pair : *(renderer_state->objects_to_render))
    {
        RendererState::LayerGroup *layer_group = pair.second;

        for(Line *current_line : layer_group->lines_retained)
        {
            if(current_line == line)
            {
                found_group = layer_group;
                line_pos = current_line;
                break;
            }
        }

        if(found_group != nullptr)
        {
            break;
        }
    }

    if(found_group)
    {
        // NOTE:
        // Need to remove the line from this layer's list. While it shouldn't matter what order the lines are drawn in
        // on this layer, changing the existing order might cause objects to flicker in front and behind other objects.
        // For now, I'm doing a naive order-preserving erase from the list.
        std::vector<Line *> &lines = found_group->lines_retained;
        lines.erase(std::find(lines.begin(), lines.end(), line_pos));
    }
}



#pragma region Fonts
#if 0
struct Font
{
    int first_character;
    int num_characters;
    stbtt_bakedchar *character_data;

    stbtt_fontinfo info;

    GLuint gl_texture;
    int texture_width;
    int texture_height;
};

void draw_text(const char *text, v2 position, float size, v4 color)
{
    RendererState *renderer_state = get_engine_renderer_state();
    static bool inited = false;
    static Font *font;
    if(!inited)
    {
        // Make quad mesh
        VertexUv quad_vs[] =
        {
            { {0.0f, 0.0f}, {0.0f, 0.0f} },
            { {1.0f, 0.0f}, {1.0f, 0.0f} },
            { {1.0f, 1.0f}, {1.0f, 1.0f} },
            { {0.0f, 1.0f}, {0.0f, 1.0f} },
        };
        int quad_is[] =
        {
            0, 1, 2,
            0, 2, 3
        };
        renderer_state->font_quad_mesh = make_uv_mesh(_countof(quad_vs), quad_vs, _countof(quad_is), quad_is);

        font = (Font *)my_allocate(sizeof(Font));
        font->first_character = ' ';
        font->num_characters = '~' - font->first_character + 1;
        font->character_data = (stbtt_bakedchar *)my_allocate(sizeof(stbtt_bakedchar) * font->num_characters);

        font->texture_width = 2048;
        font->texture_height = 2048;

        unsigned ttf_buffer_size;
        char *ttf_buffer = read_whole_file("fonts/arial.ttf", &ttf_buffer_size);

        stbtt_InitFont(&(font->info), (unsigned char *)ttf_buffer, 0);

        /*
           STBTT_DEF int stbtt_BakeFontBitmap(const unsigned char *data,
           int offset,  // font location (use offset=0 for plain .ttf)
           float pixel_height,                     // height of font in pixels
           unsigned char *pixels, int pw, int ph,  // bitmap to be filled in
           int first_char, int num_chars,          // characters to bake
           stbtt_bakedchar *chardata);             // you allocate this, it's num_chars long
           */
        int font_offset = 0;
        float pixel_height = 128;
        unsigned char *bitmap = (unsigned char *)my_allocate(font->texture_width * font->texture_height);

        stbtt_BakeFontBitmap((unsigned char *)ttf_buffer,
                font_offset,
                pixel_height,
                bitmap, font->texture_width, font->texture_height,
                font->first_character, font->num_characters,
                font->character_data); // no guarantee this fits!

        //free(ttf_buffer);

        glGenTextures(1, &(font->gl_texture));
        glBindTexture(GL_TEXTURE_2D, font->gl_texture);
        check_gl_errors("make font texture");
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, font->texture_width, font->texture_height, 0, GL_RED, GL_UNSIGNED_BYTE, bitmap);
        check_gl_errors("allocate font texture");
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        check_gl_errors("set paramters");

        free(bitmap);

        inited = true;
    }


    size *= 1.0f / 128.0f;


    Mesh *mesh = renderer_state->font_quad_mesh;
    Shader *shader = renderer_state->font_quad_shader;

    use_mesh(mesh);
    use_shader(shader);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, font->gl_texture);

    v2 pixels_position = v2();
    while(*text)
    {
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(font->character_data, font->texture_width, font->texture_height, *text-32,
                &(pixels_position.x), &(pixels_position.y), &q, 1);//1=opengl & d3d10+,0=d3d9

        VertexUv quad_vs[] =
        {
            { {0.0f, 0.0f}, {q.s0, q.t1} },
            { {1.0f, 0.0f}, {q.s1, q.t1} },
            { {1.0f, 1.0f}, {q.s1, q.t0} },
            { {0.0f, 1.0f}, {q.s0, q.t0} },
        };

        glBindBuffer(GL_ARRAY_BUFFER, mesh->vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, 4 * sizeof(VertexUv), quad_vs);
        glFinish();
        check_gl_errors("set mesh vertex buffer data");

        float screen_width = get_screen_width(get_engine_platform_state());
        float screen_height = get_screen_height(get_engine_platform_state());
        float screen_aspect_ratio = renderer_state->screen_aspect_ratio;
        float x =  q.x0 * size * (2.0f / screen_width);
        float y = -q.y1 * size * (2.0f / screen_height) * screen_aspect_ratio * 0.5f;
        float width  = size * (q.x1 - q.x0) * (2.0f / screen_width);
        float height = size * (q.y1 - q.y0) * (2.0f / screen_height) * screen_aspect_ratio * 0.5f;

        mat4 ndc_m_world_mat = ndc_m_world(renderer_state->camera, renderer_state->screen_aspect_ratio);
        v4 position_in_ndc_4 = ndc_m_world_mat * v4(position, 0.0f, 1.0f);
        v2 position_in_ndc = v2(position_in_ndc_4.x, position_in_ndc_4.y);

        mat4 mvp = 
            make_translation_matrix(v3(position_in_ndc + v2(x, y), 0.0f)) *
            make_scale_matrix(v3(width, height, 1.0f));
        set_uniform(shader, "mvp", mvp);
        set_uniform(shader, "blend_color", color);

        glDrawElements(mesh->primitive_type, mesh->num_indices, GL_UNSIGNED_INT, 0);

        text++;
    }
}
#endif
#pragma endregion





#pragma region Required API
RendererState *init_renderer()
{
    RendererState *renderer_state = (RendererState *)my_allocate(sizeof(RendererState));

    create_gl_context(renderer_state);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    renderer_state->camera = (Camera *)my_allocate(sizeof(Camera));
    renderer_state->camera->position = v2();
    renderer_state->camera->width = 10.0f;
    PlatformState *platform_state = get_engine_platform_state();
    renderer_state->screen_aspect_ratio = (float)get_screen_width(platform_state) / (float)get_screen_height(platform_state);

    renderer_state->texture_map = new std::map<std::string, Texture *>();

    renderer_state->objects_to_render = new std::map<int, RendererState::LayerGroup *>();
    
    {
        // Make shaders
        renderer_state->batch_rect_shader     = make_shader("shaders/batch_rect.shader");
        renderer_state->batch_triangle_shader = make_shader("shaders/batch_triangle.shader");
        renderer_state->batch_circle_shader   = make_shader("shaders/batch_circle.shader");
        renderer_state->batch_line_shader     = make_shader("shaders/batch_line.shader");

        {
            renderer_state->batch_rect_buffer = new ObjectBuffer();
            glGenVertexArrays(1, &renderer_state->batch_rect_buffer->vao);
            glBindVertexArray(renderer_state->batch_rect_buffer->vao);
            check_gl_errors("making vao");

            glGenBuffers(1, &renderer_state->batch_rect_buffer->vbo);
            check_gl_errors("making vbo");

            glBindBuffer(GL_ARRAY_BUFFER, renderer_state->batch_rect_buffer->vbo);
            glBufferData(GL_ARRAY_BUFFER, 256 * sizeof(RendererState::RectRenderingData), nullptr, GL_DYNAMIC_DRAW);
            check_gl_errors("send vbo data");
            renderer_state->batch_rect_buffer->bytes_capacity = 256 * sizeof(RendererState::RectRenderingData);

            float stride = sizeof(RendererState::RectRenderingData);
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

        {
            renderer_state->batch_triangle_buffer = new ObjectBuffer();
            glGenVertexArrays(1, &renderer_state->batch_triangle_buffer->vao);
            glBindVertexArray(renderer_state->batch_triangle_buffer->vao);
            check_gl_errors("making vao");

            glGenBuffers(1, &renderer_state->batch_triangle_buffer->vbo);
            check_gl_errors("making vbo");

            glBindBuffer(GL_ARRAY_BUFFER, renderer_state->batch_triangle_buffer->vbo);
            glBufferData(GL_ARRAY_BUFFER, 256 * sizeof(RendererState::TriangleRenderingData), nullptr, GL_DYNAMIC_DRAW);
            check_gl_errors("send vbo data");
            renderer_state->batch_triangle_buffer->bytes_capacity = 256 * sizeof(RendererState::TriangleRenderingData);

            float stride = sizeof(v2) + sizeof(v4);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, (void *)0);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride, (void *)(1 * sizeof(v2)));
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            check_gl_errors("vertex attrib pointer");
        }

        {
            renderer_state->batch_circle_buffer = new ObjectBuffer();
            glGenVertexArrays(1, &renderer_state->batch_circle_buffer->vao);
            glBindVertexArray(renderer_state->batch_circle_buffer->vao);
            check_gl_errors("making vao");

            glGenBuffers(1, &renderer_state->batch_circle_buffer->vbo);
            check_gl_errors("making vbo");

            glBindBuffer(GL_ARRAY_BUFFER, renderer_state->batch_circle_buffer->vbo);
            glBufferData(GL_ARRAY_BUFFER, 256 * sizeof(RendererState::CircleRenderingData), nullptr, GL_DYNAMIC_DRAW);
            check_gl_errors("send vbo data");
            renderer_state->batch_circle_buffer->bytes_capacity = 256 * sizeof(RendererState::CircleRenderingData);

            float stride = sizeof(RendererState::CircleRenderingData);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, (void *)0);
            glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, stride, (void *)(1 * sizeof(v2)));
            glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride, (void *)(1 * sizeof(v2) + sizeof(float)));
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            glEnableVertexAttribArray(2);
            check_gl_errors("vertex attrib pointer");
        }

        {
            renderer_state->batch_line_buffer = new ObjectBuffer();
            glGenVertexArrays(1, &renderer_state->batch_line_buffer->vao);
            glBindVertexArray(renderer_state->batch_line_buffer->vao);
            check_gl_errors("making vao");

            glGenBuffers(1, &renderer_state->batch_line_buffer->vbo);
            check_gl_errors("making vbo");

            glBindBuffer(GL_ARRAY_BUFFER, renderer_state->batch_line_buffer->vbo);
            glBufferData(GL_ARRAY_BUFFER, 256 * sizeof(RendererState::LineRenderingData), nullptr, GL_DYNAMIC_DRAW);
            check_gl_errors("send vbo data");
            renderer_state->batch_line_buffer->bytes_capacity = 256 * sizeof(RendererState::LineRenderingData);

            float stride = sizeof(RendererState::LineRenderingData);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, (void *)0);
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void *)(1 * sizeof(v2)));
            glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, (void *)(2 * sizeof(v2)));
            glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, stride, (void *)(2 * sizeof(v2) + sizeof(float)));
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            glEnableVertexAttribArray(2);
            glEnableVertexAttribArray(3);
            check_gl_errors("vertex attrib pointer");
        }
        

        // Make texture
        char bitmap[] = { (char)255, (char)255, (char)255, (char)255 };
        renderer_state->white_texture = make_texture_from_bitmap(1, 1, bitmap);

    }

    return renderer_state;
}

void clear_frame(v4 color)
{
    glClearColor(color.r, color.g, color.b, color.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void render()
{
    RendererState *renderer_state = get_engine_renderer_state();
    std::map<int, RendererState::LayerGroup *> *objects_to_render = renderer_state->objects_to_render;

    for(std::pair<int, RendererState::LayerGroup *> pair : *objects_to_render)
    {
        RendererState::LayerGroup *group = pair.second;

        // Render all rectangles
        for(Rect *o : group->rects_retained) group->pack_rect(o);
        render_rect_batch(&group->rects_packed_buffer);
        group->rects_packed_buffer.clear();

        // Render all triangles
        for(Triangle *o : group->triangles_retained) group->pack_triangle(o);
        render_triangle_batch(&group->triangles_packed_buffer);
        group->triangles_packed_buffer.clear();

        // Render all circles
        for(Circle *o : group->circles_retained) group->pack_circle(o);
        render_circle_batch(&group->circles_packed_buffer);
        group->circles_packed_buffer.clear();

        // Render all lines
        for(Line *o : group->lines_retained) group->pack_line(o);
        render_line_batch(&group->lines_packed_buffer);
        group->lines_packed_buffer.clear();
    }

}

void swap_frames()
{
    glFlush();
    glFinish();

    PlatformState *platform_state = get_engine_platform_state();
    HDC dc = get_device_context(platform_state);
    SwapBuffers(dc);
}
#pragma endregion


#pragma region Utility API
v2 ndc_point_to_world(v2 ndc)
{
    RendererState *renderer_state = get_engine_renderer_state();
    mat4 m_ndc_m_world = ndc_m_world(renderer_state->camera, renderer_state->screen_aspect_ratio);

    v4 ndc4 = v4(ndc, 0.0f, 1.0f);
    v4 world4 = inverse(m_ndc_m_world) * ndc4;

    return v2(world4.x, world4.y);
}

void set_camera_position(v2 position)
{
    RendererState *renderer_state = get_engine_renderer_state();
    renderer_state->camera->position = position;
}

void set_camera_width(float width)
{
    RendererState *renderer_state = get_engine_renderer_state();
    renderer_state->camera->width = width;
}
#pragma endregion

