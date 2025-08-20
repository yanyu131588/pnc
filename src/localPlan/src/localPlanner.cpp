#include "localPlanner.hpp"
#include <limits>
#include <future>
#include <algorithm>
#include <iostream>
#include "math/angles.hpp"

LocalPlanner::LocalPlanner(const AckermannConfig& config) : config_(config) {
    lpps_.mst = config.mst;
    lpps_.mist = -config.mst;
    lpps_.msd = config.msd;
}
//根据当前状态和环境信息，计算避障控制命令，如果检测到前方有障碍物，调整局部目标点以绕过障碍物
std::pair<double, double> LocalPlanner::computeAvoidanceCommand(
    const Point3d& cup,
    const Points3d& gpat,
    double curv,
    double curst,
    std::vector<Obstacle> obss,
    UltrasonicObstacleState &ulState,
    unsigned char frontLeft, unsigned char frontRight) 
{
    if (gpat.empty()) {
        return {0.0, 0.0};
    }

    double lohd = std::max(3.0, curv * 2.0); 
    Point3d lga = flga(cup, gpat, lohd);//根据当前姿态和全局路径，计算一个局部目标点

    return calobPath(cup, lga, curv, curst,
                         obss, ulState);
}
//计算最优的速度和转向角命令。评估不同的速度和转向角组合，选择最优的控制命令   
std::pair<double, double> LocalPlanner::calobPath(
    const Point3d& cup,
    Point3d& oga,
    double curv,
    double curst,
    const std::vector<Obstacle>& obss,
    UltrasonicObstacleState &ulState)
{
    double modst = fmobd(cup, obss);

    Point3d lga = oga;
    //0808
    if (modst < 1.5) {//2

        lpps_.ocst = 3.2;//3.2 2.8
        lpps_.gcst = 0.9;
    }
    else
    {
        lpps_.ocst = 0.004;
        lpps_.gcst = 2.8;
    }

    double misd, msd;
    
    misd = std::max(curv - lpps_.mac * lpps_.dt, 
        lpps_.misd);
    msd = std::min(curv + lpps_.mac * lpps_.dt, 
        lpps_.msd);

    double mist,mst;
    mist = std::max(curst - lpps_.mdst * lpps_.dt,
        lpps_.mist);
    mst = std::min(curst + lpps_.mdst * lpps_.dt,
        lpps_.mst);

    if (modst > 3.0 && !(ulState.flObs || ulState.frObs)) {
        misd = std::max(0.0, misd);
    }
    //0808
    int spss = 4;//vec
    int stss = 12;//stear
    //0808
    if (modst < 2.0) {
    spss = 6;
    stss = 18;
        
    mist = std::max(lpps_.mist, 
                         curst - 1.2 * lpps_.mdst);
    mst = std::min(lpps_.mst,
                         curst + 1.2 * lpps_.mdst);
    } else if (curv > config_.msd * 0.7) {
        stss = 8;
    }

    double ode = obss.size() / 10.0; 
    if (ode > 1.0) {
        spss = std::max(6, spss - 2);
        stss = std::max(22, stss - 3);
    }

    std::vector<double> spcan;
    const double MESD = 0.15;  
    if (misd < -MESD && msd > MESD) {
        spcan = {misd, 
                        -MESD,
                        MESD,
                        msd};
    } 
    else {
        double step = (msd - misd) / (spss - 1);
        for (int i = 0; i < spss; ++i) {
            double speed = misd + i * step;
            if (std::abs(speed) < MESD) {
                speed = (speed > 0) ? MESD : -MESD;
            }
            spcan.push_back(speed);
        }
    }

    spcan.erase(
    std::remove_if(spcan.begin(), spcan.end(),
        [](double v) { return std::abs(v) < 0.14; }),
    spcan.end());

    std::vector<double> stcan;
    double cstr = (mist + mst) / 2;
    stcan.push_back(cstr);

    for (int i = 1; i <= stss/2; ++i) {
        double offset = i * (mst - mist) / stss;
        stcan.push_back(cstr + offset);
        stcan.push_back(cstr - offset);
    }

    std::vector<std::future<std::tuple<double, double, float>>> futures;
//生成轨迹并评估其代价
    for (double v : spcan) {
        for (double str : stcan) {
                futures.push_back(std::async([=]{
                auto trj = ptrj(cup, v, str);

                if (ccsin(trj, obss)) {
                    return std::make_tuple(v, str, -std::numeric_limits<float>::infinity());
                }
                    
                double gcst = cagcst(trj, lga);
                double ocst = caocst(trj, obss);

                double tcst = 
                lpps_.gcst * gcst   
                -lpps_.ocst * ocst;

                if (v > 0.1) tcst += 0.5;

                if (v < 0) {                  
                    tcst -= 5.0;  
                }

                return std::make_tuple(v, str, static_cast<float>(tcst));
            }));
        }
    }

    if(futures.empty())
    {
        return {std::max(curv - lpps_.mac * lpps_.dt, 0.0), 0.0};
    }

    double bv = 0.15;
    double bstr = 0.0;
    float bscr = -std::numeric_limits<float>::infinity();
//选择最优的控制命令
    for (auto& future : futures) {
        auto [v, str, scr] = future.get();
        if (scr > bscr) {
            bscr = scr;
            bv = v;
            bstr = str;
        }
    }

    if (bv < 0) {
        if (ulState.lObs && !ulState.rObs) {
            bstr -= 0.2;
        } else if (ulState.rObs && !ulState.lObs) {
            bstr += 0.2;          
        }

        if(ulState.flObs && !ulState.frObs && !ulState.lObs)
        {
            bstr -= 0.2;
        }
        else if(!ulState.flObs && ulState.frObs && !ulState.rObs)
        {
            bstr += 0.2;
        }

        if(ulState.bObs)
        {
            bv = -bv;
        }
    } 
    else {
        if (ulState.lObs && !ulState.rObs) {
            bstr += 0.2;
        } else if (ulState.rObs && !ulState.lObs) {
            bstr -= 0.2;
        }

        if(ulState.flObs && !ulState.frObs && !ulState.lObs)
        {
            bstr += 0.2;
        }
        else if(!ulState.flObs && ulState.frObs && !ulState.rObs)
        {
            bstr -=0.2;
        }
    }

    bv = std::clamp(bv, -config_.msd * 0.5, config_.msd);
    bstr = std::clamp(bstr, -config_.mst, config_.mst);

    return {bv, bstr};
}
//使用车辆的运动学模型，计算每个时间步的位置和方向
std::vector<Point3d> LocalPlanner::ptrj(const Point3d& pose, double v, double str) const{
    std::vector<Point3d> trj;
      trj.reserve(80);  
      Point3d cup = pose;
      const double dt = lpps_.dt;
      int tps = static_cast<int>(lpps_.pte / dt);
      
      const double efstr = (v >= 0) ? str : -str; 
      const double trds = (std::abs(efstr) > 0.001) 
      ? config_.wb / std::tan(efstr) 
      : std::numeric_limits<double>::max();
  
      for (int i = 0; i < tps; ++i) {
          
          if (std::abs(trds) > 1e5) {  
              cup.setX(cup.x() + v * std::cos(cup.theta()) * dt);
              cup.setY(cup.y() + v * std::sin(cup.theta()) * dt);
          } 
          else {
              const double yr = v / trds;
              const double deta = yr * dt;
              const double ccth = std::cos(cup.theta());
              const double ssth = std::sin(cup.theta());
              
              cup.setX(cup.x() + trds * (std::sin(cup.theta() + deta) - ssth));
              cup.setY(cup.y() - trds * (std::cos(cup.theta() + deta) - ccth));
              cup.setTheta(cup.theta() + deta);
          }
  
          cup.setTheta(std::fmod(cup.theta(), 2 * M_PI));

          if (cup.theta() > M_PI) cup.setTheta(cup.theta() - 2 * M_PI);
          if (cup.theta() < -M_PI) cup.setTheta(cup.theta() + 2 * M_PI);
  
          trj.push_back(cup);
      }
      
      return trj;
}
//计算轨迹到目标点的代价
double LocalPlanner::cagcst(const std::vector<Point3d>& trj, const Point3d& goal) const {
    if (trj.empty()) return 0.0;
  
      const auto& lps = trj.back();
      double dx = goal.x() - lps.x();
      double dy = goal.y() - lps.y();
      double dist = std::hypot(dx, dy);
  
      if (trj.size() > 1) {
          double mddst = std::hypot(trj.back().x() - trj.front().x(),
                                       trj.back().y() - trj.front().y());
          if (mddst < 0.01) {  
              return 0.0;  
          }
      }
  
      double tya = std::atan2(dy, dx);
      double ydf = std::abs(tya - lps.theta());
      ydf = std::min(ydf, 2*M_PI - ydf);
  
      double dfc = 1.0;
      double heading_score = 0.0;

      if (!trj.empty() && trj.size() > 1) {
          Point3d fst = trj.front();
          Point3d lst = trj.back();
          double mvd = std::atan2(lst.y() - fst.y(), lst.x() - fst.x());
          double gdr = std::atan2(goal.y() - fst.y(), goal.x() - fst.x());
          double ddf = std::abs(mvd - gdr);
          ddf = std::min(ddf, 2*M_PI - ddf);
          
          if (ddf > M_PI/2) {
              dfc = 0.7; 
          }
      }

      return dfc / (dist + 0.1 * ydf + 1e-6);
}
//计算轨迹与障碍物的代价
double LocalPlanner::caocst(const std::vector<Point3d>& trj, 
                                const std::vector<Obstacle>& obss) const
{
    if (trj.empty() || obss.empty()) 
    return 0.0;

    double mddt = std::numeric_limits<double>::max();

    rcte vrct;
    vrct.length = config_.vlt;
    vrct.width = config_.vwd;
    
    const Point3d& cup = trj.front();
   
    for (const auto& pose : trj) {
        vrct.center = Point2d(pose.x(), pose.y());
        vrct.yaw = pose.theta();
        
        for (const auto& obs : obss) {                    
            rcte obs_rect;
            obs_rect.center = Point2d(
                (obs.bbox.min_x + obs.bbox.max_x) / 2.0,
                (obs.bbox.min_y + obs.bbox.max_y) / 2.0);
            obs_rect.length = obs.bbox.max_x - obs.bbox.min_x;
            obs_rect.width = obs.bbox.max_y - obs.bbox.min_y;
            obs_rect.yaw = 0; 
           
            double dist = rctdce(vrct, obs_rect);
        
            if (dist < mddt) {
                mddt = dist;
            }
        }
    }
    
    const double sddt = config_.vlt + 1.5;
    if (mddt <= 0) return 100.0;  

    return 1.0 / (mddt + 0.3);
}

Point3d LocalPlanner::flga(const Point3d& cup, 
                              const Points3d& gpat, 
                              double lohd) const {
    if (gpat.empty()) return cup;
    
    size_t cidt = 0;
    double midt = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < gpat.size(); ++i) {
        double dx = gpat[i].x() - cup.x();
        double dy = gpat[i].y() - cup.y();
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < midt) {
            midt = dist;
            cidt = i;
        }
    }
    
    double acdt = 0.0;
    for (size_t i = cidt; i < gpat.size() - 1; ++i) {
        double dx = gpat[i+1].x() - gpat[i].x();
        double dy = gpat[i+1].y() - gpat[i].y();
        double sddt = std::sqrt(dx*dx + dy*dy);
        
        if (acdt + sddt >= lohd) {
            double rto = (lohd - acdt) / sddt;
            double x = gpat[i].x() + rto * dx;
            double y = gpat[i].y() + rto * dy;
            double theta = std::atan2(dy, dx);
            return Point3d(x, y, theta);
        }
        
        acdt += sddt;
    }
    
    return gpat.back();
}

double LocalPlanner::fmobd(const Point3d& pose, 
    const std::vector<Obstacle>& obss) const {
    if (obss.empty()) return std::numeric_limits<double>::max();

    double midt = std::numeric_limits<double>::infinity();


    for (const auto& obs : obss) {
        double dx_min = pose.x() - obs.bbox.min_x;
        double dx_max = pose.x() - obs.bbox.max_x;
        double dy_min = pose.y() - obs.bbox.min_y;
        double dy_max = pose.y() - obs.bbox.max_y;

        bool inx = (pose.x() >= obs.bbox.min_x && pose.x() <= obs.bbox.max_x);
        bool iny = (pose.y() >= obs.bbox.min_y && pose.y() <= obs.bbox.max_y);

        if (inx && iny) {
            return 0.0;
        }

        double disx = inx ? 0.0 : std::min(std::abs(dx_min), std::abs(dx_max));
        double disy = iny ? 0.0 : std::min(std::abs(dy_min), std::abs(dy_max));
        double dis = inx ? disy : (iny ? disx : std::hypot(disx, disy));

        if (dis < midt) {
            midt = dis;
        }
    }

    return midt;
}

bool LocalPlanner::ccsin(const std::vector<Point3d>& trj, 
    const std::vector<Obstacle>& obss){       
    for (const auto& pose : trj) {
        rcte vrct;
        vrct.center = Point2d(pose.x(), pose.y());
        vrct.length = config_.vlt;
        vrct.width = config_.vwd;
        vrct.yaw = pose.theta();

        for (const auto& obs : obss) {
            rcte obs_rect;
            obs_rect.center = Point2d(
                (obs.bbox.min_x + obs.bbox.max_x) / 2.0,
                (obs.bbox.min_y + obs.bbox.max_y) / 2.0);
            obs_rect.length = obs.bbox.max_x - obs.bbox.min_x;
            obs_rect.width = obs.bbox.max_y - obs.bbox.min_y;
            obs_rect.yaw = 0; 
            
            if (colsa(vrct, obs_rect)) {
                return true;
            }
        }
    }

    return false;
}

std::vector<Point2d> LocalPlanner::rcte::gcnrs() const {
    std::vector<Point2d> cnrs;
    double half_len = length / 2.0;
    double half_wid = width / 2.0;
    
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    
    cnrs.emplace_back(center.x() + half_len*cos_yaw - half_wid*sin_yaw,
    center.y() + half_len*sin_yaw + half_wid*cos_yaw);
    cnrs.emplace_back(center.x() + half_len*cos_yaw + half_wid*sin_yaw,
    center.y() + half_len*sin_yaw - half_wid*cos_yaw);
    cnrs.emplace_back(center.x() - half_len*cos_yaw + half_wid*sin_yaw,
    center.y() - half_len*sin_yaw - half_wid*cos_yaw);
    cnrs.emplace_back(center.x() - half_len*cos_yaw - half_wid*sin_yaw,
    center.y() - half_len*sin_yaw + half_wid*cos_yaw);
    
    return cnrs;
}

bool LocalPlanner::colsa(const rcte& rect1, const rcte& rect2) {
    auto cnrs1 = rect1.gcnrs();
    auto cnrs2 = rect2.gcnrs();
    
    std::vector<Point2d> axes;

    for (size_t i = 0; i < cnrs1.size(); ++i) {
        Point2d p1 = cnrs1[i];
        Point2d p2 = cnrs1[(i+1)%cnrs1.size()];
        Point2d ede(p2.x() - p1.x(), p2.y() - p1.y());
        Point2d normal(-ede.y(), ede.x());
        axes.push_back(normal);
    }
    
    for (size_t i = 0; i < cnrs2.size(); ++i) {
        Point2d p1 = cnrs2[i];
        Point2d p2 = cnrs2[(i+1)%cnrs2.size()];
        Point2d ede(p2.x() - p1.x(), p2.y() - p1.y());
        Point2d normal(-ede.y(), ede.x());
        axes.push_back(normal);
    }
    
    for (const auto& axis : axes) {
        if (axis.x() == 0 && axis.y() == 0) continue;
        
        double min1 = std::numeric_limits<double>::infinity();
        double max1 = -std::numeric_limits<double>::infinity();
        double min2 = std::numeric_limits<double>::infinity();
        double max2 = -std::numeric_limits<double>::infinity();
        
        for (const auto& p : cnrs1) {
            double proj = p.x() * axis.x() + p.y() * axis.y();
            min1 = std::min(min1, proj);
            max1 = std::max(max1, proj);
        }
        
        for (const auto& p : cnrs2) {
            double proj = p.x() * axis.x() + p.y() * axis.y();
            min2 = std::min(min2, proj);
            max2 = std::max(max2, proj);
        }
        
        if (max1 < min2 || max2 < min1) {
            return false; 
        }
    }
    
    return true; 
}

double LocalPlanner::rctdce(const rcte& rect1, const rcte& rect2) const {
    auto cnrs1 = rect1.gcnrs();
    auto cnrs2 = rect2.gcnrs();
    
    double midt = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < cnrs1.size(); ++i) {
        size_t next_i = (i + 1) % cnrs1.size();
        for (size_t j = 0; j < cnrs2.size(); ++j) {
            size_t next_j = (j + 1) % cnrs2.size();
            
            double dist = sstsd(
                cnrs1[i], cnrs1[next_i],
                cnrs2[j], cnrs2[next_j]
            );
            
            if (dist < midt) {
                midt = dist;
                if (midt <= 0.0) {
                    return 0.0;
                }
            }
        }
    }
    
    return midt;
}

double LocalPlanner::sstsd(const Point2d& p1, const Point2d& p2,
                                            const Point2d& p3, const Point2d& p4) const {
    if (ssist(p1, p2, p3, p4)) {
        return 0.0;
    }
    
    double d1 = ptsdc(p1, p3, p4);
    double d2 = ptsdc(p2, p3, p4);
    double d3 = ptsdc(p3, p1, p2);
    double d4 = ptsdc(p4, p1, p2);
    
    return std::min({d1, d2, d3, d4});
}

bool LocalPlanner::ssist(const Point2d& p1, const Point2d& p2,
                                   const Point2d& p3, const Point2d& p4) const {
    auto ccc = [](const Point2d& a, const Point2d& b, const Point2d& c) {
        return (b.x() - a.x())*(c.y() - a.y()) - (b.y() - a.y())*(c.x() - a.x());
    };
    
    bool isecc = 
        (ccc(p1, p2, p3) * ccc(p1, p2, p4) <= 0) &&
        (ccc(p3, p4, p1) * ccc(p3, p4, p2) <= 0);
    
    return isecc;
}

double LocalPlanner::ptsdc(const Point2d& point, 
    const Point2d& seg_start, 
    const Point2d& seg_end) const {
    double l2 = std::pow(seg_end.x() - seg_start.x(), 2) + 
    std::pow(seg_end.y() - seg_start.y(), 2);

    if (l2 == 0.0) return std::hypot(point.x() - seg_start.x(), 
        point.y() - seg_start.y());

        double t = std::max(0.0, std::min(1.0, 
        ((point.x() - seg_start.x()) * (seg_end.x() - seg_start.x()) + 
        (point.y() - seg_start.y()) * (seg_end.y() - seg_start.y())) / l2));

        Point2d projection(seg_start.x() + t * (seg_end.x() - seg_start.x()),
        seg_start.y() + t * (seg_end.y() - seg_start.y()));

    return std::hypot(point.x() - projection.x(), point.y() - projection.y());
}
