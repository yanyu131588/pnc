#include "footprint.hpp"
#include <limits>
#include "alogrithmHelper/alogrithmHelper.hpp"
#include <iostream>


std::pair<double, double> calculateMinAndMaxDistances(const std::vector<Point3d> & footprint)
{
    double min_dist = std::numeric_limits<double>::max();

    double max_dist = 0.0;

    if (footprint.size() <= 2) {
        return std::pair<double, double>(min_dist, max_dist);
    }

    for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
        double vertex_dist = distance(0.0, 0.0, footprint[i].x(), footprint[i].y());
        double edge_dist = distanceToLine(
        0.0, 0.0, footprint[i].x(), footprint[i].y(),
        footprint[i + 1].x(), footprint[i + 1].y());
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
        max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }

    double vertex_dist = distance(0.0, 0.0, footprint.back().x(), footprint.back().y());
    double edge_dist = distanceToLine(
      0.0, 0.0, footprint.back().x(), footprint.back().y(),
      footprint.front().x(), footprint.front().y());

    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  
    return std::pair<double, double>(min_dist, max_dist);
}

Polygon toPolygon(std::vector<Point3d> pts)
{
    Polygon polygon_;

    for (unsigned int i = 0; i < pts.size(); i++) {
        polygon_.points.push_back(pts[i]);
    }

    return polygon_;
}

std::vector<Point3d> toPointVector(Polygon polygon)
{
    std::vector<Point3d> pts;

    for (unsigned int i = 0; i < polygon.points.size(); i++) {
      pts.push_back(polygon.points[i]);
    }

    return pts;
}

void transformFootprint(
    double x, double y, double theta,
    const std::vector<Point3d> & footprint_spec,
    std::vector<Point3d> & oriented_footprint)
{
    oriented_footprint.resize(footprint_spec.size());

    double cos_th = cos(theta);
    double sin_th = sin(theta);

    for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
      double new_x = x + (footprint_spec[i].x() * cos_th - footprint_spec[i].y() * sin_th);
      double new_y = y + (footprint_spec[i].x() * sin_th + footprint_spec[i].y() * cos_th);

      Point3d & new_pt = oriented_footprint[i];
 
      new_pt.setX(new_x);
      new_pt.setY(new_y);
    }
}

void padFootprint(std::vector<Point3d> & footprint, double padding)
{
    for (unsigned int i = 0; i < footprint.size(); i++) {
      Point3d & pt = footprint[i];
      pt.setX(pt.x()+ sign0(pt.x()) * padding) ;
      pt.setY(pt.y() + sign0(pt.y()) * padding);
    }
}

std::vector<Point3d> makeFootprintFromRadius(double radius)
{
    std::vector<Point3d> points;

    int N = 16;
    Point3d pt;
    for (int i = 0; i < N; ++i) {
      double angle = i * 2 * M_PI / N;
      pt.setX(cos(angle) * radius);
      pt.setY(sin(angle) * radius);

      points.push_back(pt);
    }

    return points;
}

