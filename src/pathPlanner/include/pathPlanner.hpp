#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <memory>
#include "layeredCostmap.hpp"
#include "costmap2D.hpp"
#include "geometry/point.hpp"
#include "collisionChecker.hpp"

#include <unordered_map>

#include "node.hpp"

class PathPlanner
{
    public:
        PathPlanner(std::shared_ptr<LayeredCostmap> masterGrid, double obstacle_factor = 1.0);
        virtual ~PathPlanner() = default;
        virtual bool plan(const Point3d& start, const Point3d& goal, Points3d& path) = 0;

        Costmap2D* getCostMap() const;
        void index2Grid(int i, int& x, int& y);

    protected:
        std::shared_ptr<CollisionChecker> collision_checker_;
 
    protected:

        float obstacle_factor_;
        int nx_, ny_, map_size_;
        Costmap2D* costmap_;

        template <typename Node>
        std::vector<Node> _convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start,
                                                   const Node& goal)
        {
          std::vector<Node> path;
          auto current = closed_list.find(goal.id());
          while (current->second != start)
          {
            path.emplace_back(current->second.x(), current->second.y());
            auto it = closed_list.find(current->second.pid());
            if (it != closed_list.end())
              current = it;
            else
              return {};
          }
          path.push_back(start);
          return path;
        }

        
};





#endif