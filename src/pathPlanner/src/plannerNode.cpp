#include "plannerNode.hpp"
#include <iostream>

#include "HybridStar.hpp"

PlannerNode& PlannerNode::getInstance()
{
    static PlannerNode instance;

    return instance;
}

bool PathPlannerFactory::createPlanner(std::shared_ptr<LayeredCostmap> masterGrid,
    PlannerProps& planner_props)
{
    double obstacle_factor= 0.01;
    
    planner_props.planner_ptr = std::make_shared<HybridStarPathPlanner>(masterGrid, obstacle_factor);

    return true;
}

void PlannerNode::init(std::shared_ptr<LayeredCostmap> masterGrid,
    Point3d start,Point3d goal)
{
    PathPlannerFactory::PlannerProps path_planner_props;
    
    if (PathPlannerFactory::createPlanner(masterGrid, path_planner_props))
    {
        planner_ = path_planner_props.planner_ptr;
        
        start_ = start;
        goal_ = goal;
    }
}

bool PlannerNode::update(Points3d& path)
{
    bool path_found = false;
    Points3d path_;
    path_found = planner_->plan(start_, goal_, path_);
    std::cout<< "found path:" << path_found << std::endl;
    path = path_;
   
    return path_found;
}