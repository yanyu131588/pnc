#ifndef COLLISION_CHECKER_HPP_
#define COLLISION_CHECKER_HPP_
#include "layeredCostmap.hpp"
#include "costmap2D.hpp"
#include <cmath>


class CollisionChecker
{
    public:
        CollisionChecker(std::shared_ptr<LayeredCostmap> masterGrid, double obstacle_factor = 1.0)
            : costmap_(masterGrid->getCostmap()), obstacle_factor_(obstacle_factor){};
        ~CollisionChecker() = default;


        Costmap2D* getCostmap() const;
        bool inCollision(const unsigned int& i, const bool& traverse_unknown = true);
        float getCost(const unsigned int& i);
        bool isInsideMap(const unsigned int& i);

        template <typename Point, typename F_is_obs>
        static bool BresenhamCollisionDetection(const Point& pt1, const Point& pt2, F_is_obs func_is_obs)
        {
            int s_x = (pt1.x() - pt2.x() == 0) ? 0 : (pt1.x() - pt2.x()) / std::abs(pt1.x() - pt2.x());
            int s_y = (pt1.y() - pt2.y() == 0) ? 0 : (pt1.y() - pt2.y()) / std::abs(pt1.y() - pt2.y());
            int d_x = std::abs(pt1.x() - pt2.x());
            int d_y = std::abs(pt1.y() - pt2.y());

            if (d_x > d_y)
            {
                int tau = d_y - d_x;
                int x = pt2.x(), y = pt2.y();
                int e = 0;
                while (x != pt1.x())
                {
                    if (e * 2 > tau)
                    {
                        x += s_x;
                        e -= d_y;
                    }
                    else if (e * 2 < tau)
                    {
                        y += s_y;
                        e += d_x;
                    }
                    else
                    {
                    x += s_x;
                        y += s_y;
                        e += d_x - d_y;
                    }
                    if (func_is_obs(Point(x, y)))
                    return true;
                }
                }
                else
                {
                int tau = d_x - d_y;
                int x = pt2.x(), y = pt2.y();
                int e = 0;
                while (y != pt1.y())
                {
                    if (e * 2 > tau)
                    {
                    y += s_y;
                    e -= d_x;
                    }
                    else if (e * 2 < tau)
                    {
                    x += s_x;
                    e += d_y;
                    }
                    else
                    {
                    x += s_x;
                    y += s_y;
                    e += d_y - d_x;
                    }
                    if (func_is_obs(Point(x, y)))
                    return true;
                }
            }
            return false; 
        }

    private:
        Costmap2D* costmap_;  
        double obstacle_factor_;                 
};


#endif