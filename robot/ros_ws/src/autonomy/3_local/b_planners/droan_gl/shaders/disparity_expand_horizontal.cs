#version 450
layout(local_size_x = 16, local_size_y = 16) in;

// Input disparity (integer disparity * 10000)
layout(binding = 0, r32i) uniform readonly iimage2D disparityIn;

// Horizontal expansion outputs
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

const float PI = 3.14159265358979323846;

void main() {
  ivec2 coord = ivec2(gl_GlobalInvocationID.xy);
  ivec2 size = imageSize(disparityIn);
  if (coord.x >= size.x || coord.y >= size.y) return;

  int centerInt = imageLoad(disparityIn, coord).r;
  if (centerInt <= 0) return;
  float center_depth = fx*baseline / (float(centerInt) / scale);
  
  int radius = int(expansion_radius * float(centerInt) / scale / baseline);

  float a_0 = (float(coord.x) - cx)/fx;
  float b = (float(coord.y) - cy)/fy;
  float b_0 = b;

  float Zc = center_depth;
  float Xc = a_0*Zc;
  //float Yc = b_0*Zc;
  
  for (int dx = -radius; dx <= radius; ++dx) {
    ivec2 p = coord + ivec2(dx, 0);
    if (p.x < 0 || p.x >= size.x) continue;

    float a = (float(p.x) - cx)/fx;
    float A = a*a + b*b + 1;
    float B = -2.*Zc*(a*a_0 + b*b_0 + 1);
    float C = Zc*Zc*(a_0*a_0 + b_0*b_0  + 1) - expansion_radius*expansion_radius;
    float Zp = (-B - sqrt(B*B - 4.*A*C))/(2.*A);
    int new_disp = int(scale*fx*baseline / Zp);
    new_disp = (new_disp >> 10) << 10;

    float Xp = a*Zp;
    int angle = int((atan(Zp - Zc, Xp - Xc) + PI/2.) / PI * 1000.);

    new_disp += angle;
    
    //float sphere_depth = sqrt(expansion_radius*expansion_radius - pow(expansion_radius * int(dx)/int(radius), 2));
    //int new_disp = int(scale*fx*baseline / (center_depth - sphere_depth));
    
    imageAtomicMax(fgHoriz, p, new_disp);
    //imageAtomicMax(fgHoriz, p, centerInt);
    imageAtomicMin(bgHoriz, p, centerInt);
  }
}
