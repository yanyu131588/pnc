#ifndef OBSTACLE_DETECTOR_HPP_
#define OBSTACLE_DETECTOR_HPP_


#include "geometry/point.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include "common/dataDef.hpp"

#include <iostream>

struct Obstacle {
    float sAngle;
    float eAngle;
    float dis;
    float angle;
    struct BBox {
        float min_x, min_y, max_x, max_y;
    } bbox;
    float radius;
    std::vector<Point2d> points;
};

class ObstacleDetector {
    public:
        void processScan(const LaserScanData& scan, const Point3d & originalPoint);
        const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
        void clearObs() { obstacles_.clear(); current_id_ = 0; }

    private:
    /*
    athd (Angle Threshold)‌
        默认值：10.0f（单位：度或弧度，需结合上下文）
        作用：判定障碍物是否在前进方向上的角度阈值。例如，若障碍物与当前运动方向的夹角小于此值，则视为需要避开的障碍。
    ‌dthd (Distance Threshold)‌
        默认值：0.3f（单位：米）
        作用：障碍物的最小触发距离阈值。当障碍物距离小于此值时，系统会触发避障行为。
    mpts (Minimum Points)‌
        默认值：6
        作用：构成有效障碍物的最小点云数量（来自激光雷达或深度相机）。用于过滤噪声或孤立点。
    ‌mddst (Maximum Detection Distance)‌
        默认值：2.5f（单位：米）
        作用：障碍物检测的最大有效距离，超出此距离的物体可能被忽略。
    ‌middst (Minimum Inter-Distance)‌
        默认值：0.3f（单位：米）
        作用：同一障碍物内部点云之间的最小间距，用于聚类算法中的距离判定
    angle_min 和 angle_max‌
        默认值：-1.658 和 1.658（单位：弧度，约合 -95° 到 +95°）
        作用：定义传感器（如激光雷达）的水平扫描角度范围。
    ‌angle_increment‌
        默认值：0.0087（约合 0.5°）
        作用：每次扫描的角度步长，决定传感器分辨率。值越小，点云密度越高
    */
    struct Params {
        float athd = 10.0f;    
        float dthd = 0.3f;      
        int mpts = 6;               
        float mddst = 2.5f;    
        float middst = 0.3f;    

        float angle_min = -1.658;
        float angle_max = 1.658;
        float angle_increment = 0.0087;
    } params_;

    std::vector<Obstacle> obstacles_;
    int current_id_ = 0;

    inline bool isValidPoint(float range) const {
        return range > params_.middst && range < params_.mddst;
    }

    inline float toDegrees(float radians) const {
        return radians * 180.0f / M_PI;
    }

    void createObstacle(const std::vector<int>& indices, const LaserScanData& scan, const Point3d & originalPoint);
};
#endif