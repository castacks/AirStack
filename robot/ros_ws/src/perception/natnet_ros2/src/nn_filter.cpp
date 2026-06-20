#include "natnet_ros2/nn_filter.hpp"

float distance(float &x1,float &y1,float &z1,float &x2,float &y2,float &z2)
{
    return pow((x2 - x1),2)+pow((y2 - y1),2)+pow((z2 - z1),2);
}

int nn_filter(std::vector<object_data> &object_list, sMarker &data, float &E, float &E_x, float &E_y, float E_z, bool &individual_error, float &error_amp)
{   int idx = -1;
    float d_min=10.0;
    for(int i=0; i<(int)object_list.size(); i++)
    {
        if(!individual_error)
        {   float dist = distance(object_list[i].x, object_list[i].y, object_list[i].z,
                                    data.x, data.y, data.z);
            if(dist<E*error_amp && dist<d_min)
                {   
                    d_min = dist;
                    idx = i;
                }
        }
        else
        {
            if(abs(object_list[i].x-data.x)<E_x*error_amp && abs(object_list[i].y-data.y)<E_y*error_amp && abs(object_list[i].z-data.z)<E_z*error_amp)
            {
                return i;
            }
        }
    }
    return idx;
}