#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "pathPlanner.hpp"
#include <string>


class PlannerNode
{
    public:

        static PlannerNode& getInstance();
        void init(std::shared_ptr<LayeredCostmap> masterGrid,Point3d start,Point3d goal);

        bool update(Points3d& path);
        
    private:
        PlannerNode() = default;
        ~PlannerNode() = default;

        std::shared_ptr<PathPlanner> planner_;

        PlannerNode(const PlannerNode&) = delete;
        void operator=(const PlannerNode&) = delete;

        Point3d start_,goal_;  

};

class PathPlannerFactory
{
public:
  struct PlannerProps
  {
    std::shared_ptr<PathPlanner> planner_ptr;  
  };


  static bool createPlanner(std::shared_ptr<LayeredCostmap> masterGrid, PlannerProps& planner_props);
};


#endif