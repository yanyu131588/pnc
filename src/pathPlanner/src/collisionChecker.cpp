#include "collisionChecker.hpp"
#include <limits>
#include <cmath>
#include <iostream>
Costmap2D* CollisionChecker::getCostmap() const
{
  return costmap_;
}

bool CollisionChecker::inCollision(const unsigned int& i, const bool& traverse_unknown)
{
  double center_cost = getCost(i);
  if (std::isinf(center_cost))
  {
    return true;
  }

  if (center_cost == NO_INFORMATION && traverse_unknown)
  {
    return false;
  }

  return center_cost >= INSCRIBED_INFLATED_OBSTACLE * obstacle_factor_;
}

float CollisionChecker::getCost(const unsigned int& i)
{
  if (isInsideMap(i))
  {
        return costmap_->getCharMap()[i];
  }
  return std::numeric_limits<float>::infinity();
}

bool CollisionChecker::isInsideMap(const unsigned int& i)
{
  unsigned int size = static_cast<unsigned int>(costmap_->getSizeInCellsX() *
                                                costmap_->getSizeInCellsY());
  return ((i > 0) && (i < size));
}