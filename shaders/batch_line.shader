

@vertex



#version 440 core
layout (location = 0) in vec2 a_pos_a;
layout (location = 1) in vec2 a_pos_b;
layout (location = 2) in float a_width;
layout (location = 3) in vec4 a_color;

uniform mat4 vp;

out VS_OUT
{
    vec2 line_a;
    vec2 line_b;
    float width;
    vec4 color;
    mat4 vp;
} vs_out;

void main()
{
    vs_out.line_a = a_pos_a;
    vs_out.line_b = a_pos_b;
    vs_out.width = a_width;
    vs_out.color = a_color;
    vs_out.vp = vp;
    gl_Position = vec4(0.0f, 0.0f, 0.0f, 1.0);
};



@geometry



#version 440 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VS_OUT
{
    vec2 line_a;
    vec2 line_b;
    float width;
    vec4 color;
    mat4 vp;
} gs_in[];

out vec2  m_line_a;
out vec2  m_line_b;
out vec2  m_rect_pos;
out float m_rect_width;
out vec4  m_color;

void main() {
    vec4 line_a = vec4(gs_in[0].line_a, 0.0f, 1.0f);
    vec4 line_b = vec4(gs_in[0].line_b, 0.0f, 1.0f);
    float width = gs_in[0].width;
    vec4 v_color = gs_in[0].color;
    mat4 vp = gs_in[0].vp;

    vec4 line_d = normalize(line_b - line_a) * width / 2.0f;
    vec4 line_n = vec4(-line_d.y, line_d.x, 0.0f, 0.0f);

    m_line_a = gs_in[0].line_a;
    m_line_b = gs_in[0].line_b;
    m_rect_pos = (line_a + line_n - line_d).xy;
    m_rect_width = width;
    m_color = v_color;
    gl_Position = vp * (line_a + line_n - line_d);
    EmitVertex();

    m_line_a = gs_in[0].line_a;
    m_line_b = gs_in[0].line_b;
    m_rect_pos = (line_a - line_n - line_d).xy;
    m_rect_width = width;
    m_color = v_color;
    gl_Position = vp * (line_a - line_n - line_d);
    EmitVertex();

    m_line_a = gs_in[0].line_a;
    m_line_b = gs_in[0].line_b;
    m_rect_pos = (line_b + line_n + line_d).xy;
    m_rect_width = width;
    m_color = v_color;
    gl_Position = vp * (line_b + line_n + line_d);
    EmitVertex();

    m_line_a = gs_in[0].line_a;
    m_line_b = gs_in[0].line_b;
    m_rect_pos = (line_b - line_n + line_d).xy;
    m_rect_width = width;
    m_color = v_color;
    gl_Position = vp * (line_b - line_n + line_d);
    EmitVertex();

    EndPrimitive();
}



@fragment



#version 440 core
uniform sampler2D texture0;

in vec2  m_line_a;
in vec2  m_line_b;
in vec2  m_rect_pos;
in float m_rect_width;
in vec4  m_color;

out vec4 frag_color;

void main()
{
    vec2 diff = normalize(m_line_b - m_line_a);

    if(dot(diff, normalize(m_rect_pos - m_line_a)) < 0.0f)
    {
        if(length(m_rect_pos - m_line_a) > m_rect_width / 2.0f)
        {
            discard;
        }
    }

    if(dot(-diff, normalize(m_rect_pos - m_line_b)) < 0.0f)
    {
        if(length(m_rect_pos - m_line_b) > m_rect_width / 2.0f)
        {
            discard;
        }
    }

    frag_color = m_color;
};

