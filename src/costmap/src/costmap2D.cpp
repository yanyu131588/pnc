#include "costmap2D.hpp"

#include <cstring> 
#include <iostream>
#include <cmath>

Costmap2D::Costmap2D() : costmap_(nullptr)
{
}

void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
    delete[] costmap_;
    size_x_ = size_x;
    size_y_ = size_y;
    costmap_ = new unsigned char[size_x * size_y];
}

void Costmap2D::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution,
  double origin_x, double origin_y)
{
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;

    initMaps(size_x, size_y);
    resetMaps();
}

void Costmap2D::resetMaps()
{
    memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
    resetMapToValue(x0, y0, xn, yn, default_value_);
}

void Costmap2D::resetMapToValue(
  unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn, unsigned char value)
{
    unsigned int len = xn - x0;
    for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_) {
      memset(costmap_ + y, value, len * sizeof(unsigned char));
    }
}

Costmap2D & Costmap2D::operator=(const Costmap2D & map)
{
    if (this == &map) {
      return *this;
    }

    delete[] costmap_;
    costmap_ = NULL;

    size_x_ = map.size_x_;
    size_y_ = map.size_y_;
    resolution_ = map.resolution_;
    origin_x_ = map.origin_x_;
    origin_y_ = map.origin_y_;
    default_value_ = map.default_value_;

    initMaps(size_x_, size_y_);

    memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

    return *this;
}

Costmap2D::Costmap2D(const Costmap2D & map)
{
	*this = map;
}

Costmap2D::~Costmap2D() 
{
    delete[] costmap_;
    costmap_ = NULL;
}

unsigned int Costmap2D::cellDistance(double world_dist)
{
    double cells_dist = std::max(0.0, std::ceil(world_dist / resolution_));
    return (unsigned int)cells_dist;
}

unsigned char * Costmap2D::getCharMap() const
{
    return costmap_;
}

unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
    return costmap_[getIndex(mx, my)];
}

unsigned char Costmap2D::getCost(unsigned int undex) const
{
    return costmap_[undex];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
    costmap_[getIndex(mx, my)] = cost;
}

void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
    wx = origin_x_ + (mx + 0.5) * resolution_;  
    wy = origin_y_ + (size_y_ - my + 0.5) * resolution_;
}

bool Costmap2D::worldToMap(double wx, double wy, int & mx, int & my) const
{
    if (wx < origin_x_ || wy < origin_y_) {
      return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = size_y_ - static_cast<unsigned int>((wy - origin_y_) / resolution_);
    
    if (mx < size_x_ && my < size_y_) {
      return true;
    }

    return false;
}

void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int & mx, int & my) const
{
    if (wx < origin_x_) {
      mx = 0;
    } else if (wx > resolution_ * size_x_ + origin_x_) {
      mx = size_x_ - 1;
    } else {
      mx = static_cast<int>((wx - origin_x_) / resolution_);
    }

    if (wy < origin_y_) {
      my = 0;
    } else if (wy > resolution_ * size_y_ + origin_y_) {
      my = size_y_ - 1;
    } else {
      my = static_cast<int>((wy - origin_y_) / resolution_);
    }
}

void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
    int cell_ox, cell_oy;
    cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
    cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;

    int size_x = size_x_;
    int size_y = size_y_;

    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = std::min(std::max(cell_ox, 0), size_x);
    lower_left_y = std::min(std::max(cell_oy, 0), size_y);
    upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
    upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];

    copyMapRegion(
      costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x,
      cell_size_x,
      cell_size_y);

    resetMaps();

    origin_x_ = new_grid_ox;
    origin_y_ = new_grid_oy;

    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    copyMapRegion(
      local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x,
      cell_size_y);
      
    delete[] local_map;
}

unsigned int Costmap2D::getSizeInCellsX() const
{
    return size_x_;
}

unsigned int Costmap2D::getSizeInCellsY() const
{
    return size_y_;
}

double Costmap2D::getOriginX() const
{
    return origin_x_;
}

double Costmap2D::getOriginY() const
{
    return origin_y_;
}

double Costmap2D::getResolution() const
{
    return resolution_;
}

double Costmap2D::getSizeInMetersX() const
{
    return (size_x_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getSizeInMetersY() const
{
    return (size_y_ - 1 + 0.5) * resolution_;
}





