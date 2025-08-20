#include "layer.hpp"
#include <iostream>

Layer::Layer()
: layered_costmap_(nullptr),
name_(),
enabled_(false)
{
}

void Layer::initialize(
  LayeredCostmap * parent,
  std::string name)
{
    layered_costmap_ = parent;
    name_ = name;

    onInitialize();
}

const std::vector<Point3d> &Layer::getFootprint() const
{
    return layered_costmap_->getFootprint();
}