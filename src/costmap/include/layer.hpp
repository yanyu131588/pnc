#ifndef LAYER_HPP_
#define LAYER_HPP_

#include <memory>
#include "costmap2D.hpp"
#include "costValues.hpp"
#include "layeredCostmap.hpp"
#include <string>
#include <vector>

#include "geometry/point.hpp"

class LayeredCostmap;
class Layer
{
    public:
        Layer();
        virtual ~Layer() {}

        void initialize(
            LayeredCostmap * parent,
            std::string name);

        virtual void reset() = 0;
        
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw, double * min_x,
            double * min_y,
            double * max_x,
            double * max_y) = 0;

        virtual void updateCosts(
            Costmap2D & master_grid,
            int min_i, int min_j, int max_i, int max_j) = 0;

        virtual void matchSize() {}

        virtual void onFootprintChanged() {}

        const std::vector<Point3d> & getFootprint() const;
    protected:
        virtual void onInitialize() {}
        bool enabled_;


        LayeredCostmap * layered_costmap_;
        std::string name_;

};


#endif