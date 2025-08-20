#ifndef HYBRID_STAR_HPP_
#define HYBRID_STAR_HPP_

#include "pathPlanner.hpp"
#include <queue>

class HybridStarPathPlanner : public PathPlanner
{
public:
  HybridStarPathPlanner(std::shared_ptr<LayeredCostmap> masterGrid, double obstacle_factor = 1.0);
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path);

protected:
    double calTcst(const Node<int>& node, const Node<int>& parent, 
    const Node<int>& goal, const Node<int>& prev_node) const;
    double getEcst(unsigned char base_cost) const;
    double getScst(const Node<int>& node, const Node<int>& parent, 
    const Node<int>& prev_node) const;
    void sPath(Points3d& path);
    void uVet(const Node<int>& parent, Node<int>& child);

private:
  const std::vector<Node<int>> mpt = {
    { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
    { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
  };

  double ew_; 
  double sw_;
  double rs_; 

};

#endif