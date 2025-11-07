#version 430

uniform sampler2DArray tex_array;
uniform sampler2D disparity_tex;
uniform float baseline_fx;
uniform float sign;

in float disparity;
in float back_disparity;
in float origin_disparity;
in float circle_depth;
in vec2 fragUV;

out float FragColor;

void main() {
  /*
  FragColor = origin_disparity;
  float depth = baseline_fx / FragColor + circle_depth;
  FragColor = baseline_fx / depth;
  float disp_curr = texture(disparity_tex, fragUV).r;
  float depth_curr = baseline_fx / disp_curr;

  if((depth_curr - depth) < 2.)
    FragColor = baseline_fx / (depth_curr + 2.);
  */
  
  //*
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
  //*/
}
