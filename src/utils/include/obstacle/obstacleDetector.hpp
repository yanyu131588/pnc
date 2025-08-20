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