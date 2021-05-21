

@vertex



#version 440 core
layout (location = 0) in vec2 a_pos;
layout (location = 1) in float a_radius;
layout (location = 2) in vec4 a_color;

uniform mat4 vp;

out VS_OUT
{
    mat4 mvp;
    vec4 color;
} vs_out;

void main()
{
    mat4 scaling = mat4(
       a_radius * 2.0f, 0.0f, 0.0f, 0.0f,
       0.0f, a_radius * 2.0f, 0.0f, 0.0f,
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
} gs_in[];

out vec2 uvs;
out vec4 color;

void main() {
    mat4 mvp = gs_in[0].mvp;
    vec4 v_color = gs_in[0].color;

    uvs = vec2(-0.5f, -0.5f);
    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(-0.5f, -0.5f, 0.0f, 0.0f));
    EmitVertex();

    uvs = vec2(0.5f, -0.5f);
    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4( 0.5f, -0.5f, 0.0f, 0.0f));
    EmitVertex();

    uvs = vec2(-0.5f, 0.5f);
    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4(-0.5f,  0.5f, 0.0f, 0.0f));
    EmitVertex();

    uvs = vec2(0.5f, 0.5f);
    color = v_color;
    gl_Position = mvp * (gl_in[0].gl_Position + vec4( 0.5f,  0.5f, 0.0f, 0.0f));
    EmitVertex();

    EndPrimitive();
}



@fragment



#version 440 core

in vec2 uvs;
in vec4 color;

out vec4 frag_color;

void main()
{
    if(length(uvs) > 0.5f)
    {
        discard;
    }

    frag_color = color;
};

