

@vertex



#version 440 core
layout (location = 0) in vec2 a_pos;
layout (location = 1) in vec4 a_color;

uniform mat4 vp;

out vec4 color;

void main()
{
    color = a_color;
    gl_Position = vp * vec4(a_pos, 0.0f, 1.0);
};



@fragment



#version 440 core

in vec4 color;

out vec4 frag_color;

void main()
{
    frag_color = color;
};

