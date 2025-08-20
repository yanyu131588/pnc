#include "layeredCostmap.hpp"
#include "footprint.hpp"
#include <algorithm>
#include <limits>
#include <iostream>


LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown)
: primary_costmap_(), combined_costmap_(),
global_frame_(global_frame),
rolling_window_(rolling_window),
footprint_(std::make_shared<std::vector<Point3d>>())
{

    if (track_unknown) {
        primary_costmap_.setDefaultValue(255);
        combined_costmap_.setDefaultValue(255);
      } else {
        primary_costmap_.setDefaultValue(0);
        combined_costmap_.setDefaultValue(0);
      }

}

LayeredCostmap::~LayeredCostmap()
{
    while (plugins_.size() > 0) {
        plugins_.pop_back();
      }

}

void LayeredCostmap::addPlugin(std::shared_ptr<Layer> plugin)
{
    plugins_.push_back(plugin);
}

void LayeredCostmap::resizeMap(
    unsigned int size_x, unsigned int size_y, double resolution,
    double origin_x,
    double origin_y,
    bool size_locked)
{
    primary_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
    combined_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
    
    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
      plugin != plugins_.end(); ++plugin)
    {
      (*plugin)->matchSize();
    }
}

bool LayeredCostmap::isOutofBounds(double robot_x, double robot_y)
{
    int mx, my;
    return !combined_costmap_.worldToMap(robot_x, robot_y, mx, my);
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
    if (rolling_window_) {
        double new_origin_x = robot_x - combined_costmap_.getSizeInMetersX() / 2;
        double new_origin_y = robot_y - combined_costmap_.getSizeInMetersY() / 2;
        primary_costmap_.updateOrigin(new_origin_x, new_origin_y);
        combined_costmap_.updateOrigin(new_origin_x, new_origin_y);
      }
    if (plugins_.size() == 0) {
      return;
    }

    minx_ = miny_ = std::numeric_limits<double>::max();
    maxx_ = maxy_ = std::numeric_limits<double>::lowest();

    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
    {
      double prev_minx = minx_;
      double prev_miny = miny_;
      double prev_maxx = maxx_;
      double prev_maxy = maxy_;

      (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    }

    int x0, xn, y0, yn;
    combined_costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
    combined_costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

    x0 = std::max(0, x0);
    xn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsX()), xn + 1);
    y0 = std::max(0, y0);
    yn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsY()), yn + 1);

    if (xn < x0 || yn < y0) {
      return;
    }

      if (filters_.size() == 0) {
        combined_costmap_.resetMap(x0, y0, xn, yn);
        for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
          plugin != plugins_.end(); ++plugin)
        {
          (*plugin)->updateCosts(combined_costmap_, x0, y0, xn, yn);
        }
      }
}

void LayeredCostmap::setFootprint(const std::vector<Point3d> & footprint_spec)
{
    std::pair<double, double> inside_outside = calculateMinAndMaxDistances( 
        footprint_spec);
    std::atomic_store(
        &footprint_,
        std::make_shared<std::vector<Point3d>>(footprint_spec));
      inscribed_radius_.store(std::get<0>(inside_outside));
      circumscribed_radius_.store(std::get<1>(inside_outside));

    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
      plugin != plugins_.end();
      ++plugin)
    {
        (*plugin)->onFootprintChanged();
    }
}