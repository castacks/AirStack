#version 450
layout(local_size_x = 16, local_size_y = 16) in;

// Input from horizontal pass
layout(binding = 0, r32i) uniform readonly iimage2D fgHoriz;
layout(binding = 1, r32i) uniform readonly iimage2D bgHoriz;

// Final outputs
layout(binding = 2, r32i) uniform iimage2D fgFinal;
layout(binding = 3, r32i) uniform iimage2D bgFinal;

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
  ivec2 size = imageSize(fgHoriz);
  if (coord.x >= size.x || coord.y >= size.y) return;
  
  // TODO remove
  imageStore(bgFinal, coord, imageLoad(bgHoriz, coord));
  //imageStore(fgFinal, coord, imageLoad(fgHoriz, coord));
  //return;

  int centerInt = imageLoad(fgHoriz, coord).r;
  if (centerInt <= 0) return;
  float center_depth = fx*baseline / (float(centerInt) / scale);
  
  int radius = int(expansion_radius * float(centerInt) / scale / baseline);

  float a = (float(coord.x) - cx)/fx;
  //float a_0 = (float(coord.x) - cx)/fx;
  float b_0 = (float(coord.y) - cy)/fy;


  float angle = float(centerInt & 0x3FF) / 1000. * PI - PI/2.;
  float Zc = center_depth + expansion_radius*cos(angle);
  float Xc = (float(coord.x) - cx)*center_depth/fx + expansion_radius*sin(angle);//(float(coord.x) - cx)*Zc/fx;
  float a_0 = Xc/Zc;
  //float Xc = a_0*Zc;
  //float Yc = b_0*Zc;

  float uc = fx*a_0 + cx;
  float vc = fy*b_0 + cy;
  float ru = fx*expansion_radius/Zc;
  float rv = fy*expansion_radius/Zc;

  int ymin = int(floor(vc - rv*sqrt(1. - ((float(coord.x) - uc)*(float(coord.x) - uc))/(ru*ru))));
  int ymax = int(ceil(vc + rv*sqrt(1. - ((float(coord.x) - uc)*(float(coord.x) - uc))/(ru*ru))));
  

  //for (int dy = -radius; dy <= radius; ++dy) {
  for (int y = ymin; y <= ymax; ++y) {
    //ivec2 p = coord + ivec2(0, dy);
    ivec2 p = ivec2(coord.x, y);
    if (p.y < 0 || p.y >= size.y) continue;

    float b = (float(p.y) - cy)/fy;
    float A = a*a + b*b + 1;
    float B = -2.*Zc*(a*a_0 + b*b_0 + 1);
    float C = Zc*Zc*(a_0*a_0 + b_0*b_0  + 1) - expansion_radius*expansion_radius;
    if((B*B - 4.*A*C) < 0.)
      continue;
    float Zp = (-B - sqrt(B*B - 4.*A*C))/(2.*A);
    int new_disp = int(scale*fx*baseline / Zp);
    //new_disp = (new_disp >> 10) << 10;

    //imageAtomicMax(fgFinal, p, (centerInt & 0x3FF)*int(scale));
    //imageAtomicMin(fgFinal, p, int((float(angle == angle) + 1.)*scale));
    imageAtomicMax(fgFinal, p, new_disp);
    //imageAtomicMax(fgFinal, p, int(Zc*scale));
    //imageAtomicMax(fgFinal, p, centerInt);
    //imageAtomicMin(bgFinal, p, centerInt);
  }
}
