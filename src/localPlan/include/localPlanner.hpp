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
        /*
        LPPS结构体参数说明
        基础运动参数
        msd (Max Steering Delta): 最大转向增量，默认值0.5（单位：弧度或比例值）
        misd (Min Steering Delta): 最小转向增量，默认值-0.5
        mst (Max Steering Threshold): 最大转向阈值，默认16度（转换为弧度值：16×π/180）
        mist (Min Steering Threshold): 最小转向阈值，默认-16度
        控制参数
        mac (Max Acceleration): 最大加速度，默认值6（单位可能为m/s²）
        mdst (Max Distance): 最大距离阈值，默认值20（单位可能为米）
        dt (Delta Time): 时间步长，默认0.05秒（即50ms，常见于控制循环周期）
        性能参数
        pte (Path Tracking Error): 路径跟踪误差容限，默认值3（单位可能为米）
        代价函数权重
        gcst (Goal Cost Weight): 目标点代价权重，默认值1.0
        ocst (Obstacle Cost Weight): 障碍物代价权重，默认值1.0
        */
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