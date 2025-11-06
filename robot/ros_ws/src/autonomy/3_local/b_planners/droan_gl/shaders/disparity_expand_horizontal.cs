#version 450
layout(local_size_x = 16, local_size_y = 16) in;

layout(binding = 0, r32f) uniform readonly image2D disparityIn;

layout(binding = 1, r32i) uniform iimage2D fgHoriz;
layout(binding = 2, r32i) uniform iimage2D bgHoriz;

uniform float baseline;
uniform float fx;
uniform float fy;
uniform float cx;
uniform float cy;
uniform float expansion_radius;
uniform float discontinuityThresh;
uniform float scale;
uniform int downsample_scale;

const float PI = 3.14159265358979323846;

void main() {
  ivec2 coord = ivec2(gl_GlobalInvocationID.xy);
  ivec2 size = imageSize(disparityIn);
  if (coord.x >= size.x*downsample_scale || coord.y >= size.y*downsample_scale) return;
  //if (coord.x >= size.x || coord.y >= size.y) return;

  ivec2 in_coord = downsample_scale*coord;
  float center = 0.f;
  for(int i = 0; i < downsample_scale; i++)
    for(int j = 0; j < downsample_scale; j++)
      center = max(center, imageLoad(disparityIn, in_coord + ivec2(i, j)).r);
  center /= downsample_scale;
  

  //int centerInt = imageLoad(disparityIn, coord).r;
  if (center <= 0.f) return;
  float center_depth = fx*baseline / center;
  
  int radius = int(expansion_radius * center / baseline);

  float a_0 = (float(coord.x) - cx)/fx;
  float b = (float(coord.y) - cy)/fy;
  float b_0 = (float(coord.y) - cy)/fy;

  float Zc = center_depth;
  float Xc = a_0*Zc;
  
  for (int dx = -radius; dx <= radius; ++dx) {
    ivec2 p = coord + ivec2(dx, 0);
    if (p.x < 0 || p.x >= size.x) continue;

    float a = (float(p.x) - cx)/fx;
    float A = a*a + b*b + 1;
    float B = -2.*Zc*(a*a_0 + b*b_0 + 1);
    float C = Zc*Zc*(a_0*a_0 + b_0*b_0  + 1) - expansion_radius*expansion_radius;
    if((B*B - 4.*A*C) < 0.)
      break;
    float Zp = (-B - sqrt(B*B - 4.*A*C))/(2.*A);
    int new_disp = int(scale*fx*baseline / Zp);
    
    //float Zp_bg = (-B - sqrt(B*B - 4.*A*C))/(2.*A);
    //int new_disp_bg = int(scale*fx*baseline / Zp_bg);

    float Xp = a*Zp;
    float raw_angle = atan(Xp - Xc, Zp - Zc);
    int angle = int((raw_angle + PI/2.) / PI * 1000.);
    
    new_disp = (new_disp & ~0x3FF) | (angle & 0x3FF);
    
    float new_angle = float(new_disp & 0x3FF) / 1000. * PI - PI/2.;
    
    imageAtomicMax(fgHoriz, p, new_disp);
    //imageAtomicMin(bgHoriz, p, new_disp_bg);
  }
}
