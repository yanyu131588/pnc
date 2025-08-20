#include "pathPlanner.hpp"

PathPlanner::PathPlanner(std::shared_ptr<LayeredCostmap> masterGrid, double obstacle_factor)
  : obstacle_factor_(obstacle_factor)
  , nx_(static_cast<int>(masterGrid->getCostmap()->getSizeInCellsX()))
  , ny_(static_cast<int>(masterGrid->getCostmap()->getSizeInCellsY()))
  , map_size_(nx_ * ny_)
  , costmap_(masterGrid->getCostmap())
  ,collision_checker_(std::make_shared<CollisionChecker>(masterGrid, obstacle_factor))
{

}

void PathPlanner::index2Grid(int i, int& x, int& y)
{
  x = static_cast<int>(i % nx_);
  y = static_cast<int>(i / nx_);
}

Costmap2D* PathPlanner::getCostMap() const
{
  return costmap_;
}