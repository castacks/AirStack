#ifndef NN_FILTER_H
#define NN_FILTER_H

#include <NatNetTypes.h>
#include <cmath>
#include <vector>
#include "natnet_ros2/object_data.hpp"

float distance(float &x1,float &y1,float &z1,float &x2,float &y2,float &z2);

int nn_filter(std::vector<object_data> &object_list, sMarker &data, float &E, float &E_x, float &E_y, float E_z, bool &individual_error, float &error_amp);

#endif
