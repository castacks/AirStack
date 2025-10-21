#version 450
layout(local_size_x = 16, local_size_y = 16) in;

// Input disparity (integer disparity * 10000)
layout(binding = 0, r32i) uniform readonly iimage2D disparityIn;

// Horizontal expansion outputs
layout(binding = 1, r32i) uniform iimage2D fgHoriz;
layout(binding = 2, r32i) uniform iimage2D bgHoriz;

uniform float baseline;
uniform float fx;
uniform float expansion_radius;
uniform float discontinuityThresh;

void main() {
  ivec2 coord = ivec2(gl_GlobalInvocationID.xy);
  ivec2 size = imageSize(disparityIn);
  if (coord.x >= size.x || coord.y >= size.y) return;

  int centerInt = imageLoad(disparityIn, coord).r;
  if (centerInt <= 0) return;
  
  int radius = int(expansion_radius * float(centerInt) / 10000.0 / baseline);

  for (int dx = -radius; dx <= radius; ++dx) {
    ivec2 p = coord + ivec2(dx, 0);
    if (p.x < 0 || p.x >= size.x) continue;
    //int neighInt = imageLoad(disparityIn, p).r;
    //if (neighInt > 0) {
    //  float neigh = float(neighInt) / 10000.0;
      //if (abs(neigh - center) > discontinuityThresh) continue;
    //}
    
    imageAtomicMax(fgHoriz, p, centerInt);
    imageAtomicMin(bgHoriz, p, centerInt);
  }
}
