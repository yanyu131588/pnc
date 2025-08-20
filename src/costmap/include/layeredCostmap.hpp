#ifndef LAYERED_COSTMAP_HPP_
#define LAYERED_COSTMAP_HPP_

#include "layer.hpp"

#include <string>
#include <vector>
#include "costmap2D.hpp"
#include "geometry/point.hpp"
#include <memory>
#include <atomic>


class Layer;

class LayeredCostmap
{
    public:
        LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);
        ~LayeredCostmap();
        void updateMap(double robot_x, double robot_y, double robot_yaw);

        std::string getGlobalFrameID() const
        {
          return global_frame_;
        }
    
        Costmap2D * getCostmap()
        {
          return &combined_costmap_;
        }

        bool isRolling()
        {
          return rolling_window_;
        }
    
        std::vector<std::shared_ptr<Layer>> * getPlugins()
        {
          return &plugins_;
        }

        void addPlugin(std::shared_ptr<Layer> plugin);

        void resizeMap(
        unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
        double origin_y,
        bool size_locked = false);

        void setFootprint(const std::vector<Point3d> & footprint_spec);
        const std::vector<Point3d> & getFootprint()
        {
            return *std::atomic_load(&footprint_);
        }

        bool isOutofBounds(double robot_x, double robot_y);

        double getCircumscribedRadius() {return circumscribed_radius_.load();}
        double getInscribedRadius() {return inscribed_radius_.load();}
    private:
        Costmap2D primary_costmap_, combined_costmap_;
        std::string global_frame_;
        bool rolling_window_;
        double minx_, miny_, maxx_, maxy_;

        std::vector<std::shared_ptr<Layer>> plugins_;
        std::vector<std::shared_ptr<Layer>> filters_;

        std::atomic<double> circumscribed_radius_, inscribed_radius_;
        std::shared_ptr<std::vector<Point3d>> footprint_;
};

#endif