#ifndef CMD_DEF_HPP_
#define CMD_DEF_HPP_
#include <vector>
#include "geometry/point.hpp"
#include <cstdint>

struct cmd_vel
{
    uint64_t stamp;
    
    double v;
    double steer;
};


#endif