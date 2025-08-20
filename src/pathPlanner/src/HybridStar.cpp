#include "HybridStar.hpp"
#include <algorithm>
#include <iostream>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>


HybridStarPathPlanner::HybridStarPathPlanner(std::shared_ptr<LayeredCostmap> masterGrid, double obstacle_factor)
  : PathPlanner(masterGrid, obstacle_factor){

    ew_ = 2.5;
    sw_ = 1.5;
  };


bool HybridStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path)
{
  int mstx, msty, mgx, mgy;
  costmap_->worldToMap(start.x(),start.y(),mstx,msty);
  costmap_->worldToMap(goal.x(),goal.y(),mgx,mgy);

  Node<int> snod(mstx, msty);
  Node<int> gnod(mgx, mgy);

  snod.set_id(costmap_->getIndex(snod.x(), snod.y()));
  gnod.set_id(costmap_->getIndex(gnod.x(), gnod.y()));
  path.clear();

  std::priority_queue<Node<int>, std::vector<Node<int>>, Node<int>::compare_cost> olist;
  std::unordered_map<int, Node<int>> clist;

  olist.push(snod);

  while (!olist.empty())
  {
    auto curt = olist.top();
    olist.pop();

    if (clist.find(curt.id()) != clist.end())
      continue;

    clist.insert(std::make_pair(curt.id(), curt));
 
    if (curt == gnod)
    {     
      const auto& bta = _convertClosedListToPath<Node<int>>(clist, snod, gnod);
      for (auto iter = bta.rbegin(); iter != bta.rend(); ++iter)
      {
        double wx, wy;
        costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
        path.emplace_back(wx, wy);

      }
      sPath(path);
      return true;
    }

    for (const auto& m : mpt)
    {
      auto nnod = curt + m;  
      nnod.set_g(curt.g() + m.g());
      nnod.set_h(std::hypot(nnod.x() - gnod.x(), nnod.y() - gnod.y()));
      nnod.set_id(costmap_->getIndex(nnod.x(), nnod.y()));
      nnod.set_pid(curt.id());

      if (clist.find(nnod.id()) != clist.end())
        continue;
        
      if ((nnod.id() < 0) || (nnod.id() >= map_size_) ||
        (costmap_->getCharMap()[nnod.id()] >= LETHAL_OBSTACLE * obstacle_factor_)) {
        continue;
        }
            
        Node<int> pnot;
        if (clist.find(curt.pid()) != clist.end()) {
            pnot = clist[curt.pid()];
        }
        
        double tcst = calTcst(nnod, curt, gnod, pnot);
        nnod.set_g(curt.g() + m.g());
        nnod.set_h(tcst - nnod.g()); 

        Node<int> ppt;
        ppt.set_id(curt.pid());
        int tmp_x, tmp_y;
        index2Grid(ppt.id(), tmp_x, tmp_y);
        ppt.set_x(tmp_x);
        ppt.set_y(tmp_y);

        auto fppt = clist.find(ppt.id());
        if (fppt != clist.end())
        {
          ppt = fppt->second;
          uVet(ppt, nnod);
        }
        
        olist.push(nnod);
      }
    }

  return false;
}

void HybridStarPathPlanner::uVet(const Node<int>& ppt, Node<int>& child)
{
  auto icoio = [&](const Node<int>& node1, const Node<int>& node2) {
    return CollisionChecker::BresenhamCollisionDetection(node1, node2, [&](const Node<int>& node) {
      return costmap_->getCharMap()[costmap_->getIndex(node.x(), node.y())] >= LETHAL_OBSTACLE * obstacle_factor_;
    });
  };

  if (!icoio(ppt, child)) {
      double dist = std::hypot(ppt.x() - child.x(), ppt.y() - child.y());
      double new_g = ppt.g() + dist;
      
      if (new_g < child.g()) {
          child.set_g(new_g);
          child.set_pid(ppt.id());
      }
  }

}
//计算总代价
double HybridStarPathPlanner::calTcst(const Node<int>& node,
  const Node<int>& ppt,
  const Node<int>& goal,
  const Node<int>& pnot) const {

  unsigned char bst = costmap_->getCharMap()[node.id()];

  double est = getEcst(bst);

  double scst = 0.0;
  if (pnot.id() != -1) {
    scst = getScst(node, ppt, pnot);
  }

  double hstc = std::hypot(node.x() - goal.x(), node.y() - goal.y());

  return bst + ew_ * est + 
  sw_ * scst + hstc;
}
//根据节点的代价值（bst），计算估计代价（est）
double HybridStarPathPlanner::getEcst(unsigned char bst) const {
  if (bst > 0 && bst < LETHAL_OBSTACLE * obstacle_factor_) {
      double icst = 3.0; 
      double sig = 2.0;     
      double diff = bst - icst;
      return -std::exp(-(diff * diff) / (2 * sig * sig));//高斯函数
  }
  return 0.0;
}
//计算平滑代价
double HybridStarPathPlanner::getScst(const Node<int>& node,
  const Node<int>& ppt,
  const Node<int>& pnot) const {
//计算方向向量
  double dx1 = ppt.x() - pnot.x();
  double dy1 = ppt.y() - pnot.y();

  double dx2 = node.x() - ppt.x();
  double dy2 = node.y() - ppt.y();

  double mag1 = std::hypot(dx1, dy1);
  double mag2 = std::hypot(dx2, dy2);

  if (mag1 > 1e-3 && mag2 > 1e-3) {
  double cthe = (dx1 * dx2 + dy1 * dy2) / (mag1 * mag2);
  cthe = std::clamp(cthe, -1.0, 1.0);
  return (1.0 - cthe) / 2.0; 
  }
  return 0.0;
}
//使用B样条曲线对路径进行平滑处理
void HybridStarPathPlanner::sPath(Points3d& path ) {

  int dg = 2;int nps = 50;
  if (path.size() < 3) return;

  auto last = std::unique(path.begin(), path.end()); 
    path.erase(last, path.end());
    if (path.size() < 2) return;

  Eigen::MatrixXd points(2, path.size());
  for (size_t i = 0; i < path.size(); ++i) {
      points(0, i) = path[i].x();
      points(1, i) = path[i].y();
  }
//拟合
  Eigen::Spline<double, 2> spline = 
      Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(
          points, 
          std::min<int>(dg, path.size() - 1)
      );

  Points3d spth;
  for (int i = 0; i <= nps; ++i) {
      double u = i / static_cast<double>(nps);
      Eigen::Vector2d point = spline(u);
      spth.emplace_back(point.x(), point.y());
  }

  path = std::move(spth);
}