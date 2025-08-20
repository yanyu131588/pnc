#ifndef DATA_DEF_HPP_
#define DATA_DEF_HPP_

#include <stdint.h>
#include <vector>
#include "geometry/point.hpp"

struct AckermannConfig {
    double wb;      
    double mst;      
    double msd;      
    double mac;      
    double vwd;  
    double vlt; 
    double ld_res; 
    double ld_mia;  
    double ld_ma;  
    double ld_mr;  
};

struct mapInfo
{
    uint64_t stamp;
    uint32_t width_;
    uint32_t height_;

    float resolution_;

    float originX_;
    float originY_;
    float originTheta_;

    uint8_t* pMapData_;

    mapInfo():width_(0),height_(0),resolution_(0.05f),originX_(0),originY_(0),pMapData_(nullptr)
    {

    }
};

struct LaserScanData {
    int stamp;
    std::vector<float> ranges;
    //std::vector<float> intensity;

    LaserScanData() : stamp(0), ranges(380, 0){}
};

struct SensorData
{
    std::vector<LaserScanData> lidardata;
    Point3d curPos;
    double cur_v;
    unsigned char ultrasonundData[8];
    int time;
};


#endif