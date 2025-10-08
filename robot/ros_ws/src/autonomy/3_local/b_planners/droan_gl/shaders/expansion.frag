#version 430

uniform sampler2DArray tex_array;
uniform float baseline_fx;
uniform float sign;

in float disparity;
in vec2 fragUV;

out float FragColor;

void main() {
  if(sign > 0)
    FragColor = disparity;
  else{
    float front_depth = baseline_fx / texture(tex_array, vec3(fragUV, 0)).r; // TODO: 0 is hardcoded texture index
    float back_depth = baseline_fx / disparity;
    if(abs(front_depth - back_depth) > 3.) // TODO dont do abs, instead have the correct order, remove hardcoded 3
      discard; // TODO double check whether discard always works or there is still some culling happening
    else
      FragColor = disparity;
  }
}
