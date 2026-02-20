// disparity_expand.comp
#version 450
layout(local_size_x = 16, local_size_y = 16) in;

layout(binding = 0, r32f) uniform readonly image2D disparityIn;
layout(binding = 1, r32f) uniform writeonly image2D disparityForeground;
layout(binding = 2, r32f) uniform writeonly image2D disparityBackground;

uniform float expansionRadius;     // in pixels
uniform float discontinuityThresh; // disparity difference threshold

void main() {
  ivec2 coord = ivec2(gl_GlobalInvocationID.xy);
  ivec2 size = imageSize(disparityIn);
  if (coord.x >= size.x || coord.y >= size.y) return;

  float center = imageLoad(disparityIn, coord).r;
  if (center <= 0.0) return; // no valid disparity

  float maxDisp = center;
  float minDisp = center;

  // local expansion
  for (int dy = -int(expansionRadius); dy <= int(expansionRadius); ++dy) {
    for (int dx = -int(expansionRadius); dx <= int(expansionRadius); ++dx) {
      ivec2 p = coord + ivec2(dx, dy);
      if (p.x < 0 || p.y < 0 || p.x >= size.x || p.y >= size.y) continue;
      float d = imageLoad(disparityIn, p).r;
      if (d <= 0.0) continue;
      // skip large depth discontinuities
      if (abs(d - center) > discontinuityThresh) continue;
      maxDisp = max(maxDisp, d);
      minDisp = min(minDisp, d);
    }
  }

  imageStore(disparityForeground, coord, vec4(maxDisp));
  imageStore(disparityBackground, coord, vec4(minDisp));
}
