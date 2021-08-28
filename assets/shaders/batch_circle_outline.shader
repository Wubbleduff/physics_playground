
@vertex



#version 440 core
layout (location = 0) in vec2 a_pos;
layout (location = 1) in float a_radius;
layout (location = 2) in vec4 a_color;
layout (location = 3) in float a_thickness;

uniform mat4 vp;

out VS_OUT
{
    mat4 mvp;
    vec4 color;
    float radius;
    float thickness;
} vs_out;

void main()
{
    mat4 scaling = mat4(
       a_radius, 0.0f, 0.0f, 0.0f,
       0.0f, a_radius, 0.0f, 0.0f,
       0.0f, 0.0f, 1.0f, 0.0f,
       0.0f, 0.0f, 0.0f, 1.0f
       );
    mat4 translating = mat4(
       1.0f, 0.0f, 0.0f, 0.0f,
       0.0f, 1.0f, 0.0f, 0.0f,
       0.0f, 0.0f, 1.0f, 0.0f,
       a_pos.x, a_pos.y, 0.0f, 1.0f
       );
    vs_out.mvp = vp * translating * scaling;
    vs_out.color = a_color;
    vs_out.radius = a_radius;
    vs_out.thickness = a_thickness;
    gl_Position = vec4(0.0f, 0.0f, 0.0f, 1.0);
};



@geometry



#version 440 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in VS_OUT
{
    mat4 mvp;
    vec4 color;
    float radius;
    float thickness;
} gs_in[];

out vec2 uvs;
out vec4 color;
out float model_thickness;

void main() {
    mat4 mvp = gs_in[0].mvp;
    vec4 v_color = gs_in[0].color;
    float thickness = gs_in[0].thickness * (1.0f / gs_in[0].radius);

    uvs = vec2(0.0f, 0.0f);
    color = v_color;
    model_thickness = thickness;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(-1.0f, -1.0f, 0.0f, 0.0f));
    EmitVertex();

    uvs = vec2(1.0f, 0.0f);
    color = v_color;
    model_thickness = thickness;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4( 1.0f, -1.0f, 0.0f, 0.0f));
    EmitVertex();

    uvs = vec2(0.0f, 1.0f);
    color = v_color;
    model_thickness = thickness;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(-1.0f,  1.0f, 0.0f, 0.0f));
    EmitVertex();

    uvs = vec2(1.0f, 1.0f);
    color = v_color;
    model_thickness = thickness;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4( 1.0f,  1.0f, 0.0f, 0.0f));
    EmitVertex();

    EndPrimitive();
}



@fragment



#version 440 core
uniform sampler2D texture0;

in vec2 uvs;
in vec4 color;
in float model_thickness;

out vec4 frag_color;

void main()
{
    float l = length(uvs - vec2(0.5f, 0.5f));
    if((l <= 0.5f) && (l >= 0.5f - model_thickness))
    {
        frag_color = color;
    }
    else
    {
        frag_color = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    }
};

