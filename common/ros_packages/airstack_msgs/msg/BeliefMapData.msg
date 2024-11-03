# serialized BeliefMap data
# assumes same origin, same grid resolution

int16 x_start # cell number relative to the origin. max int16 is 32767, so this is a reasonable assumption
int16 y_start
int32 x_end # if we break things up in the future, could assume always square and remove this
int32 y_end # if we break things up in the future, could assume always square and remove this

uint16[] map_values # between 0 and 1, 65535 values, precision of 0.000015

