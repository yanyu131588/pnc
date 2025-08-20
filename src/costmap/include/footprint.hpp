#ifndef FOOTPRINT_HPP_
#define FOOTPRINT_HPP_

#include "geometry/point.hpp"
#include <vector>
#include "geometry/polygon.hpp"
#include <string>



std::pair<double, double> calculateMinAndMaxDistances(const std::vector<Point3d> & footprint);

Polygon toPolygon(std::vector<Point3d> pts);
std::vector<Point3d> toPointVector(Polygon polygon);

void transformFootprint(
    double x, double y, double theta,
    const std::vector<Point3d> & footprint_spec,
    std::vector<Point3d> & oriented_footprint);

void padFootprint(std::vector<Point3d> & footprint, double padding);
std::vector<Point3d> makeFootprintFromRadius(double radius);


#endif