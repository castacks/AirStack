# Packed-bitmask explored-area occupancy grid (1 bit per cell).
# Cell (i, j) is observed iff bit (j*width + i) of `data` is set (np.packbits order).
# Cell-center world metres: (origin_x + (i+0.5)*resolution, origin_y + (j+0.5)*resolution).
float32 resolution
float32 origin_x
float32 origin_y
uint32 width
uint32 height
uint8[] data
