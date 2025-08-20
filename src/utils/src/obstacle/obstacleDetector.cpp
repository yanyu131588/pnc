#include "obstacle/obstacleDetector.hpp"

void ObstacleDetector::processScan(const LaserScanData& scan , const Point3d & originalPoint) 
{
    obstacles_.clear();
    std::vector<int> vids;
    vids.reserve(scan.ranges.size());
    
    for (int i = 0; i < scan.ranges.size(); ++i) {
        if (isValidPoint(scan.ranges[i])) {
            vids.push_back(i);
        }
    }

    if (vids.empty()) return;

    std::vector<int> crucl;
    crucl.reserve(params_.mpts * 2);
    
    float lage = params_.angle_min + vids[0] * params_.angle_increment;
    float lrg = scan.ranges[vids[0]];
    //find continue point , and check compute obstacle box
    for (int idx : vids) {
        float cura = params_.angle_min + idx * params_.angle_increment;
        float curr = scan.ranges[idx];
        
        float adf = std::abs(toDegrees(cura - lage));
        float rdf = std::abs(curr - lrg);

        if (adf <= params_.athd && 
            rdf <= params_.dthd) {
            crucl.push_back(idx);
        } else {
            if (crucl.size() >= params_.mpts) {
                createObstacle(crucl, scan , originalPoint);
            }
            crucl.clear();
            crucl.push_back(idx);
        }

        lage = cura;
        lrg = curr;
    }

    if (crucl.size() >= params_.mpts) {
        createObstacle(crucl, scan , originalPoint);
    }
}

void transformOffset (const Point3d & originalPoint, const Point3d &offset,Point2d& transformedPoint)
{
    double x = originalPoint.x();
    double y = originalPoint.y();
    double theta = originalPoint.theta();

    double offsetX = offset.x();
    double offsetY = offset.y();

    transformedPoint.setX(offsetX * cos(theta) - offsetY * sin(theta) + x);
    transformedPoint.setY(offsetX * sin(theta) + offsetY * cos(theta) + y);
}

void ObstacleDetector::createObstacle(const std::vector<int>& indices, const LaserScanData& scan, const Point3d & originalPoint) {
    Obstacle obs;
    float sdst = 0.0f;
    obs.points.reserve(indices.size());

    float miang = std::numeric_limits<float>::max();
    float mang = std::numeric_limits<float>::lowest();
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    Point3d offset(0, 0, 0);
    for (int idx : indices) {
        float range = scan.ranges[idx];
        sdst += range;
        
        float angle = params_.angle_min + idx * params_.angle_increment;
        Point3d pt(range * std::cos(angle), range * std::sin(angle),angle);
        Point2d transformedPt;
        
        transformOffset(originalPoint, pt, transformedPt);
        
        obs.points.push_back(transformedPt);

        miang = std::min(miang, angle);
        mang = std::max(mang, angle);
        min_x = std::min(min_x, static_cast<float>(transformedPt.x()));
        min_y = std::min(min_y, static_cast<float>(transformedPt.y()));
        max_x = std::max(max_x, static_cast<float>(transformedPt.x()));
        max_y = std::max(max_y, static_cast<float>(transformedPt.y()));
    }

    float bbcx = (min_x + max_x) / 2.0f;
    float bbcy = (min_y + max_y) / 2.0f;
    
    float cag = std::atan2(bbcy, bbcx);

    obs.sAngle = toDegrees(miang);
    obs.eAngle = toDegrees(mang);
    obs.dis = sdst / indices.size();
    obs.angle = cag;
    obs.bbox = {min_x, min_y, max_x, max_y};
    obs.radius = std::hypot(max_x - min_x, max_y - min_y) / 2.0f;

    obstacles_.push_back(obs);
}