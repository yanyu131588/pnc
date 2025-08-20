#ifndef LOCAL_PLANNER_HPP_
#define LOCAL_PLANNER_HPP_

#include "common/dataDef.hpp"

#include "obstacle/obstacleDetector.hpp"
#include "obstacle/ulObsDetector.hpp"
#include <cmath>


class LocalPlanner {
    public:
        LocalPlanner(const AckermannConfig& config);
       
        std::pair<double, double> computeAvoidanceCommand(
            const Point3d& cup,
            const Points3d& gpat,
            double curv,
            double curst,
            std::vector<Obstacle> obss,UltrasonicObstacleState &ulState,unsigned char frontLeft,unsigned char frontRight
        );
        
        std::vector<Point3d> ptrj(const Point3d& pose, double v, double steer) const;

    private:
        AckermannConfig config_;
        
        struct Lpps {
            double msd = 0.5;
            double misd = -0.5; 
            double mst = 16* M_PI/180.0;
            double mist = -16* M_PI/180.0; 
            double mac = 6;  
            double mdst = 20; 
            double dt = 0.05;        
            double pte = 3;
            
            double gcst = 1.0;
            double ocst = 1.0;

        } lpps_;

        struct rcte {
            Point2d center;
            double length;
            double width;
            double yaw;  
            
            std::vector<Point2d> gcnrs() const ;
            bool istab(const rcte& other) const;
        };

        std::pair<double, double> calobPath(
            const Point3d& cup,
            Point3d& oga,
            double curv,
            double curst,
            const std::vector<Obstacle>& obss,
            UltrasonicObstacleState &ulState);

        double cagcst(const std::vector<Point3d>& traj, const Point3d& goal) const;

        double caocst(const std::vector<Point3d>& traj, const std::vector<Obstacle>& obss) const;

        Point3d flga(const Point3d& cup, const Points3d& gpat, double lookahead_dist) const;

        double fmobd(const Point3d& pose, const std::vector<Obstacle>& obss) const;

        bool ccsin(const std::vector<Point3d>& traj,const std::vector<Obstacle>& obss);

        bool colsa(const rcte& rect1, const rcte& rect2) ;     

        double rctdce(const rcte& rect1, const rcte& rect2) const;

        double ptsdc(const Point2d& point, 
                const Point2d& seg_start, 
                const Point2d& seg_end) const ;
        double sstsd(const Point2d& p1, const Point2d& p2,
                    const Point2d& p3, const Point2d& p4) const ;
        bool ssist(const Point2d& p1, const Point2d& p2,
                        const Point2d& p3, const Point2d& p4) const;    
                               
};

#endif