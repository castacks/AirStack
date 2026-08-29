#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aTex;
uniform mat4 model, view, projection;
out vec2 TexCoord;
void main(){
    TexCoord = aTex;
    gl_Position = projection * view * model * vec4(aPos,1.0);
}
