#ifndef DATA_DEF_HPP_
#define DATA_DEF_HPP_

#include <stdint.h>
#include <vector>
#include "geometry/point.hpp"

/*
1. 基础车辆参数‌
    ‌wb (Wheelbase)‌
        默认值：0.756（单位：米）
        作用：前后轮轴距，决定车辆转弯半径的核心参数。
    ‌vwd (Vehicle Width)‌
        默认值：0.85（单位：米）
        作用：车辆宽度，用于计算最小转弯空间和碰撞检测。
    ‌vlt (Vehicle Length Total)‌
        默认值：1.44（单位：米）
        作用：车辆总长度（含前后悬），影响路径规划和泊车逻辑。
‌2. 运动限制参数‌
    ‌mst (Max Steering Angle)‌
        默认值：0.28（单位：弧度，约合16°）
        作用：前轮最大转向角，限制车辆转弯能力。
    ‌msd (Max Steering Delta)‌
        默认值：0.5（单位：弧度/秒）
        作用：转向角变化率上限，防止机械冲击。
    ‌mac (Max Acceleration)‌
        默认值：0.5（单位：m/s²）
        作用：最大加速度，用于速度规划。
‌3. 未初始化的高级参数（扩展功能）‌
    ‌ld_* 系列参数‌
        ld_res：转向分辨率（如舵机精度）
        ld_mia/ld_ma：最小/最大转向角（可能用于冗余校验）
        ld_mr：最大转向速率（硬件限制）
        注：这些字段未在示例中初始化，可能用于高级控制或硬件接口
‌典型应用场景‌
    ‌路径规划‌
    通过 wb 和 mst 计算最小转弯半径：
    Rmin=wbtan⁡(mst)
    Rmin​=tan(mst)wb​
    （示例中最小半径约 2.7 米）
    ‌运动控制‌
        使用 msd 平滑转向指令，避免突变
        通过 mac 限制加速度保证舒适性
*/
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