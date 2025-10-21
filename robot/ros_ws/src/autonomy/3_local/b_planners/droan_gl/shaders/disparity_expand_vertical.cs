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
uniform float expansion_radius;
uniform float discontinuityThresh;

void main() {
  ivec2 coord = ivec2(gl_GlobalInvocationID.xy);
  ivec2 size = imageSize(fgHoriz);
  if (coord.x >= size.x || coord.y >= size.y) return;

  int centerInt = imageLoad(fgHoriz, coord).r;
  if (centerInt <= 0) return;

  int radius = int(expansion_radius * float(centerInt) / 10000.0 / baseline);

  for (int dy = -radius; dy <= radius; ++dy) {
    ivec2 p = coord + ivec2(0, dy);
    if (p.y < 0 || p.y >= size.y) continue;

    imageAtomicMax(fgFinal, p, centerInt);
    imageAtomicMin(bgFinal, p, centerInt);
  }
}
