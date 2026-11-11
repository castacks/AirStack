#ifndef OBJECT_DATA_HPP
#define OBJECT_DATA_HPP


#include <string>
#include <iostream>


struct object_data
    {
        std::string name;
        float x;
        float y;
        float z;
        bool detected;
        int marker_config;
    };

#endif
