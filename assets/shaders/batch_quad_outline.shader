
@vertex



#version 440 core
layout (location = 0) in vec2 a_pos;
layout (location = 1) in vec2 a_scale;
layout (location = 2) in vec4 a_color;
layout (location = 3) in float a_rotation;
layout (location = 4) in float a_thickness;

uniform mat4 vp;

out VS_OUT
{
    mat4 mvp;
    vec4 color;
    vec2 object_scale;
    float thickness;
} vs_out;

void main()
{
    float c = cos(a_rotation);
    float s = sin(a_rotation);
    mat4 scaling = mat4(
       a_scale.x, 0.0f, 0.0f, 0.0f,
       0.0f, a_scale.y, 0.0f, 0.0f,
       0.0f, 0.0f, 1.0f, 0.0f,
       0.0f, 0.0f, 0.0f, 1.0f
       );
    mat4 rotating = mat4(
       c, s, 0.0f, 0.0f,
       -s, c, 0.0f, 0.0f,
       0.0f, 0.0f, 1.0f, 0.0f,
       0.0f, 0.0f, 0.0f, 1.0f
       );
    mat4 translating = mat4(
       1.0f, 0.0f, 0.0f, 0.0f,
       0.0f, 1.0f, 0.0f, 0.0f,
       0.0f, 0.0f, 1.0f, 0.0f,
       a_pos.x, a_pos.y, 0.0f, 1.0f
       );
    vs_out.mvp = vp * translating * rotating * scaling;
    vs_out.color = a_color;
    vs_out.object_scale = a_scale;
    vs_out.thickness = a_thickness;
    gl_Position = vec4(0.0f, 0.0f, 0.0f, 1.0);
};



@geometry



#version 440 core
layout (points) in;
layout (triangle_strip, max_vertices = 10) out;

in VS_OUT
{
    mat4 mvp;
    vec4 color;
    vec2 object_scale;
    float thickness;
} gs_in[];

out vec4 color;

void main() {
    mat4 mvp = gs_in[0].mvp;
    vec4 v_color = gs_in[0].color;
    vec2 object_scale = gs_in[0].object_scale;
    vec2 model_thickness = gs_in[0].thickness * vec2(1.0f / object_scale.x, 1.0f / object_scale.y);

    vec2 _1 = vec2(-0.5f, -0.5f);
    vec2 _3 = vec2( 0.5f, -0.5f);
    vec2 _5 = vec2( 0.5f,  0.5f);
    vec2 _7 = vec2(-0.5f,  0.5f);

    vec2 _2 = _1 + vec2( model_thickness.x,  model_thickness.y);
    vec2 _4 = _3 + vec2(-model_thickness.x,  model_thickness.y);
    vec2 _6 = _5 + vec2(-model_thickness.x, -model_thickness.y);
    vec2 _8 = _7 + vec2( model_thickness.x, -model_thickness.y);
    /*

       7                       5
        *---------------------*
        | *-----------------* |
        | |8               6| |
        | |                 | |
        | |                 | |
        | |                 | |
        | |                 | |
        | |2               4| |
        | *-----------------* |
        *---------------------*
       1                       3

    */

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_1, 0.0f, 0.0f));
    EmitVertex();

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_2, 0.0f, 0.0f));
    EmitVertex();

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_3, 0.0f, 0.0f));
    EmitVertex();

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_4, 0.0f, 0.0f));
    EmitVertex();

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_5, 0.0f, 0.0f));
    EmitVertex();

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_6, 0.0f, 0.0f));
    EmitVertex();

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_7, 0.0f, 0.0f));
    EmitVertex();

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_8, 0.0f, 0.0f));
    EmitVertex();

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_1, 0.0f, 0.0f));
    EmitVertex();

    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(_2, 0.0f, 0.0f));
    EmitVertex();

    EndPrimitive();
}



@fragment



#version 440 core
uniform sampler2D texture0;

in vec4 color;

out vec4 frag_color;

void main()
{
    frag_color = color;
};

