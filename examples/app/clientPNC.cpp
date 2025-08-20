#include "mapMgr/mapMgr.hpp"
#include "costmap.hpp"
#include "plannerNode.hpp"

#include "common/dataDef.hpp"
#include "common/cmdDef.hpp"
#include "loadJson.hpp"

#define cfgFile "data/maps/test.json"
#include <signal.h>
#include <atomic>
#include <thread>
#include <mutex>

#include "obstacle/obstacleDetector.hpp"
#include "obstacle/ulObsDetector.hpp"


#include "tcpClient.hpp"
#include "msgQueue.hpp"
#include <iostream>
#include <cstring>
#include <queue>
#include <condition_variable>
#include <fstream>
#include <chrono>
#include "localPlanner.hpp"

#include "animation/matplotlibcpp.h" 
namespace plt = matplotlibcpp;

#define SAFE_DIS 0.5

double arrival_threshold = 0.8; 
static double   targetVel = 0.5;

std::string g_mapName = "";

std::string g_parkingNum = "3519";

volatile std::atomic<bool> g_interrupt_flag(false);
volatile std::atomic<bool> g_mapping_flag(false); 
volatile std::atomic<bool> g_loadmap_flag(false);
volatile std::atomic<bool> g_plan_flag(false);
volatile std::atomic<bool> g_parseParkingNum_flag(false); 
volatile std::atomic<bool>  g_parse_control_status_flag(false);
volatile std::atomic<bool>  g_arrived_target(false);
volatile std::atomic<bool>  g_obs_stop(false);
volatile std::atomic<bool>  g_ulObs_stop(false);

volatile std::atomic<bool> stopStatus(false);
volatile std::atomic<bool> posStatus(true);

std::atomic<bool> isTimerStarted(false);
std::atomic<bool> isUltrasonicTimerStarted(false);
std::atomic<bool>  isStopTargetRunning(false);
std::atomic<int> eTime = 0;

std::chrono::steady_clock::time_point stopTarget_start_time;
std::chrono::steady_clock::time_point ultrasonicStopStartTime;

std::chrono::steady_clock::time_point obs_stop_start_time = std::chrono::steady_clock::now();

std::atomic<double> estTime(0);

std::mutex g_mapping_mutex;
std::mutex g_parking_mutex;
std::mutex g_control_mutex;

static cmd_vel vw;
AckermannConfig config;

struct laserScanData{
    std::vector<float> ranges;
    std::vector<float> intensity;

    laserScanData() : ranges(380, 0.0f),intensity(380, 0.0f) {}
};

class dataQueue
{
    private:
        std::queue<SensorData> queue;
        mutable std::mutex mtx;  
        std::condition_variable cv;
        const size_t max_size = 10;

    public:
        void push(const SensorData& data) {
            std::lock_guard<std::mutex> lock(mtx);
            if (queue.size() >= max_size) {
                queue.pop();
            }
            queue.push(data);
            cv.notify_one();
        }

        SensorData pop() {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [this]{ return !queue.empty(); });
            SensorData data = queue.front();
            queue.pop();
            return data;
        }

        SensorData get_latest() {
            std::lock_guard<std::mutex> lock(mtx);
            if (queue.empty()) {
                return SensorData{};
            }
            
            while (queue.size() > 1) {
                queue.pop();
            }
            return queue.front();
        }

        bool empty() const {
            std::lock_guard<std::mutex> lock(mtx);  
            return queue.empty();
        }
};
dataQueue sensorDataQueue;

ObstacleDetector obstacleDetector_;
double regularizeAngle(double angle)
{
  return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

float getMinObsDistance(const std::vector<Obstacle>& obstacles) {
    if (obstacles.empty()) {
        return std::numeric_limits<float>::infinity();
    }
    float minObsDistance = obstacles[0].dis;
    for (const auto& obstacle : obstacles) {
        if (obstacle.dis < minObsDistance) {
            minObsDistance = obstacle.dis;
        }
    }
    return minObsDistance;
}

std::vector<Point3d> setRobot(
    double length, double width)
  {
    std::vector<Point3d> polygon;
    polygon.emplace_back(width / 2, length / 2, 0.0);  
    polygon.emplace_back(-width / 2, length / 2, 0.0);  
    polygon.emplace_back(-width / 2, -length / 2, 0.0); 
    polygon.emplace_back(width / 2, -length / 2, 0.0);  
  
    return polygon;
}
  
void stop(int signal)
{
    if(signal == SIGINT)
    {
        printf("quit! \n");
        g_interrupt_flag=1;
    }
}

double estimateRuntime(const Points3d& path, double targetSpeed) {
    double estTime_ = 0.0;

    for (int i = 0; i < path.size() - 1; ++i) {
        const Point3d& current = path[i];
        const Point3d& next = path[i + 1];
        double dx = next.x() - current.x();
        double dy = next.y() - current.y();
        double distance = std::sqrt(dx * dx + dy * dy);
        estTime_ += distance / targetSpeed;
    }

    return estTime_;
}

std::string  parse_map_name(unsigned char *recebuf,int len)
{
    std::string input_str(reinterpret_cast<const char*>(recebuf), len);
    std::size_t first_comma = input_str.find(',');

    std::size_t stat_pos = input_str.find("stat", first_comma);
    if (stat_pos == std::string::npos) {
        std::cerr << "Error: 'stat' not found" << std::endl;
        return "unknown";
    }

    std::size_t status_pos = input_str.find(',', stat_pos) + 1;

    char status_char = input_str[status_pos];

    if (status_char == '1') {
        g_mapping_flag = true;
        std::cout << "mapping ok" << std::endl;
    } else {
        g_mapping_flag = false;
        std::cout << "mapping failed" << std::endl;
    }

    std::size_t map_pos = input_str.find(',', status_pos) + 1;
    if (map_pos >= input_str.size()) {
        std::cerr << "Error: No map name found" << std::endl;
        return "unknown";
    }

    std::string map_name = input_str.substr(map_pos);
 
    return map_name;
}

std::string parse_parking_num(unsigned char *recebuf,int len)
{
    std::string input_str(reinterpret_cast<const char*>(recebuf), len);

    std::size_t first_comma = input_str.find(',');

    std::size_t plan_pos = input_str.find("plan", first_comma + 1);

    std::size_t second_comma = input_str.find(',', plan_pos + 1);

    std::size_t pk_pos = input_str.find("PK", second_comma + 1);

    std::size_t third_comma = input_str.find(',', pk_pos + 1);

    std::size_t num_pos = third_comma + 1;

    std::string parking_num = input_str.substr(num_pos);

    return parking_num;
}

std::string parse_charging_station_num(const unsigned char *recebuf, int len) {
    std::string input_str(reinterpret_cast<const char*>(recebuf), len);

    std::size_t first_comma = input_str.find(',');

    std::size_t plan_pos = input_str.find("plan", first_comma + 1);

    std::size_t second_comma = input_str.find(',', plan_pos + 1);

    std::size_t pk_pos = input_str.find("CS", second_comma + 1);

    std::size_t third_comma = input_str.find(',', pk_pos + 1);

    std::size_t num_pos = third_comma + 1;

    std::string charingStation_num = input_str.substr(num_pos);

    std::cout << "charing station num: " << charingStation_num << std::endl;

    return charingStation_num;

}

bool parse_control_cmd(const unsigned char *recebuf, int len) {
    std::string input_str(reinterpret_cast<const char*>(recebuf), len);

    std::size_t first_comma = input_str.find(',');
    std::size_t ctrl_pos = input_str.find("ctrl", first_comma + 1);
    std::size_t second_comma = input_str.find(',', ctrl_pos + 1);

    std::size_t status_pos = second_comma + 1;
    char status_char = input_str[status_pos];

    std::cout << "control status: " << (status_char == '1' ? "can go" : "can not go") << std::endl;
    bool controlStatus = (status_char == '1');
    return controlStatus;
}

std::string parse_charging_station_low(const unsigned char *recebuf, int len) {
    std::string input_str(reinterpret_cast<const char*>(recebuf), len);
    std::size_t first_comma = input_str.find(',');
    std::size_t plan_pos = input_str.find("plan", first_comma + 1);
    std::size_t second_comma = input_str.find(',', plan_pos + 1);
    std::size_t cs_pos = input_str.find("CS", second_comma + 1);
    
    std::size_t third_comma = input_str.find(',', cs_pos + 1);
    std::size_t num_pos = third_comma + 1;
    std::size_t fourth_comma = input_str.find(',', num_pos + 1);
    
    std::size_t low_pos = input_str.find("LO", fourth_comma + 1);
    std::size_t fifth_comma = input_str.find(',', low_pos + 1);
    std::size_t status_pos = fifth_comma + 1;
    
    char status_char = input_str[status_pos];
    std::cout << "power status: " << (status_char == '1' ? "low" : "high") << std::endl;

    return std::string(1, status_char);
}

std::string parse_docking(const unsigned char *recebuf, int len) {
    std::string input_str(reinterpret_cast<const char*>(recebuf), len);

    std::size_t first_comma = input_str.find(',');
    std::size_t dock_pos = input_str.find("dock", first_comma + 1);
    std::size_t second_comma = input_str.find(',', dock_pos + 1);
    std::size_t status_pos = second_comma + 1;


    char status_char = input_str[status_pos];

    std::cout << "dock status: " << (status_char == '1' ? "docking" : "docked") << std::endl;

    return std::string(1, status_char);
}

uint8_t parse_stop(const unsigned char *recebuf, int len) {
    std::string input_str(reinterpret_cast<const char*>(recebuf), len);

    std::size_t first_comma = input_str.find(',');
    std::size_t stop_pos = input_str.find("stop", first_comma + 1);
    std::size_t second_comma = input_str.find(',', stop_pos + 1);

    std::size_t status_pos = second_comma + 1;

    char status_char = input_str[status_pos];

    std::cout << "stop status: ";
    if (status_char == '1') {
        std::cout << "normal stop" << std::endl;
    } else if (status_char == '2') {
        std::cout << "emergency stop" << std::endl;
    } else {
        std::cout << "unknown status" << std::endl;
    }
    return static_cast<uint8_t>(status_char);
}

bool parse_pos(const unsigned char *recebuf, int len) {
    std::string input_str(reinterpret_cast<const char*>(recebuf), len);

    std::size_t first_comma = input_str.find(',');
    std::size_t stop_pos = input_str.find("pos", first_comma + 1);
    std::size_t second_comma = input_str.find(',', stop_pos + 1);

    std::size_t status_pos = second_comma + 1;

    char status_char = input_str[status_pos];

    std::cout << "pos status: ";
    if (status_char == '0') {
        std::cout << "pos failed" << std::endl;
    } else if (status_char == '1') {
        std::cout << "pos normal" << std::endl;
    } else {
        std::cout << "unknown status" << std::endl;
    }
    return static_cast<bool>(status_char);
}

void parse_command(unsigned char *recebuf,int len)
{
	if(len==0)
	    return ;

	if(recebuf[0]=='#')
	{
		if(recebuf[1]=='1'&& recebuf[2]=='0' && recebuf[3]=='0')
		{
			g_mapName = parse_map_name(recebuf,len);
		}
		if(recebuf[1]=='1'&& recebuf[2]=='0' && recebuf[3]=='1')
		{
			std::cout<<"exit charging station"<<std::endl;
		}
		if(recebuf[1]=='1'&& recebuf[2]=='0' && recebuf[3]=='2')
		{
            g_parkingNum = parse_parking_num(recebuf,len);

            if(g_parkingNum != "0")
            {
                std::lock_guard<std::mutex> lock(g_parking_mutex);
                g_parseParkingNum_flag = true;
                g_arrived_target = false;
                std::cout<<"plan to parking:"<<g_parkingNum<<std::endl;
            }			
		}
        if(recebuf[1]=='1'&& recebuf[2]=='0' && recebuf[3]=='3')
		{
            parse_charging_station_low(recebuf,len);
            isStopTargetRunning = false;
			std::cout<<"plan to charing station"<<std::endl;
		}
        if(recebuf[1]=='1'&& recebuf[2]=='0' && recebuf[3]=='4')
		{
            g_parse_control_status_flag = parse_control_cmd(recebuf,len);
		}
        if(recebuf[1]=='1'&& recebuf[2]=='0' && recebuf[3]=='5')
		{
            parse_docking(recebuf,len);
		}

        if(recebuf[1]=='1'&& recebuf[2]=='0' && recebuf[3]=='6')
		{
            stopStatus = parse_stop(recebuf,len);
		}

        if(recebuf[1]=='1'&& recebuf[2]=='0' && recebuf[3]=='7')
		{
            posStatus = parse_pos(recebuf,len);
        }
	}
}

void tcp_client_fun()
{
    Qttcp_cli *tcpclient = new Qttcp_cli;
    tcpclient->client = (cli_info*)calloc(1,sizeof(cli_info));

    int ser_port=6000;  
    tcpclient->reconnect=true;  
    bool login_ok=false;

    char *ser_ip = "127.0.0.1";

    memset(tcpclient->client->recebuf,0,sizeof(tcpclient->client->recebuf));

    while(!g_interrupt_flag)
    {
        if(tcpclient->reconnect==true)
        {
            int ret = tcpclient->cli_reconnect(tcpclient->client,ser_ip,ser_port);

            if(ret>=0) 
            {
                login_ok=true;
                std::cout<<"pnc client connect ok "<<std::endl;
            }

            if(login_ok) tcpclient->reconnect=false;      
        }

        if(login_ok)
        {
            int len=0;

            memset(tcpclient->client->recebuf, 0, sizeof(tcpclient->client->recebuf));
	    	int ret = tcpclient->cli_recv(tcpclient->client);

            if(ret>0)
	    	{
                len=ret;
				printf("recv: %s \n",tcpclient->client->recebuf);

                parse_command(tcpclient->client->recebuf,len);
            }

            {
                std::lock_guard<std::mutex> lock(g_mapping_mutex);

                if(g_loadmap_flag)
                {
                    g_loadmap_flag = false;
                    
                    std::string loadMap_str ="#200,stat,1";
                    std::memcpy(tcpclient->client->sendbuf, loadMap_str.data(), loadMap_str.size());
                    ret = tcpclient->cli_send_len(tcpclient->client, loadMap_str.size());
                    if(ret>0) 
                    {
                        printf("send #200,stat,1 ok \n"); 
                    }
                }
            }
            if(g_plan_flag)
            {
                g_plan_flag = false;

                std::string planState_str ="#202,stat,1,time,"+ std::to_string(estTime);
                std::memcpy(tcpclient->client->sendbuf, planState_str.data(), planState_str.size());
                ret = tcpclient->cli_send_len(tcpclient->client, planState_str.size());
                if(ret>0) 
                {
                    printf("send 202,stat,1,time ok \n"); 
                }
            }

            {
                std::lock_guard<std::mutex> lock(g_control_mutex);
            
                if(g_arrived_target)
                {
                    g_arrived_target = false;
                    g_parse_control_status_flag = false;
                    std::string arrivedState_str ="#204,stat,1";
                    std::memcpy(tcpclient->client->sendbuf, arrivedState_str.data(), arrivedState_str.size());
                    ret = tcpclient->cli_send_len(tcpclient->client, arrivedState_str.size());
                    if(ret>0) 
                    {
                        printf("send #204,stat,1 ok \n"); 
                    }
                }
            }
        }

        usleep(1000000);
    }

    tcpclient->cli_close(tcpclient->client);
}

void mainThread_fun()
{
    mapInfo myMap;
    Costmap cm;
    
    auto costmap2DPtr = std::make_shared<Costmap2D>();
    auto& planner = PlannerNode::getInstance();
    auto layers = std::make_shared<LayeredCostmap>("frame", false, false);
    LocalPlanner local_planner(config);

    CarNode targetNode;
    Point3d start;
    Point3d goal;
    Points3d path;
    bool found = false;
    
    bool controlInit = false;
    Points3d wp;
    SensorData latest_data;
    std::vector<Obstacle> obstacles;
    UltrasonicObstacleDetector ultrasonicDetector;
    UltrasonicObstacleState ulObsState;

    cmd_vel vwCur;
    double current_steer = 0.0;

    while(!g_interrupt_flag)
    {     
        if (!sensorDataQueue.empty()) {
            latest_data = sensorDataQueue.get_latest();
        } 
        
        Point3d currentPos = latest_data.curPos;
        double velocity = latest_data.cur_v;

        {
            std::lock_guard<std::mutex> lock(g_mapping_mutex); 
        
            if(g_mapping_flag)
            {
                std::string name = "map";
                std::string path = g_mapName;

                if (loadMap(&myMap,name,path))
                {
                    g_mapping_flag  = false;
                    g_loadmap_flag = true;
                    auto polygon = setRobot(config.vlt ,config.vwd);
                                                            
                    cm.init(layers,&myMap,polygon);                  

                    *costmap2DPtr = *layers->getCostmap();   
                }                     
            }
        }
        
        {
            std::lock_guard<std::mutex> lock(g_parking_mutex);
        
            if(g_parseParkingNum_flag)
            {
                g_parseParkingNum_flag = false;
                controlInit = false;

                std::string targetId = g_parkingNum;
                std::vector<SpeedBumpNode> speedBumps;
                std::vector<RestructedZoneNode> restructedZones;
                std::vector<CarNode> carNodes;

                if (loadJsonParse(speedBumps, restructedZones, carNodes))
                {         
                    targetNode = getCarNodeById(carNodes, targetId);
                    if (!targetNode.id.empty()) {
                        std::cout << "\nCar Node with ID " << targetId << ":" << std::endl;
                        std::cout << "ID: " << targetNode.id << ", X: " << targetNode.x << ", Y: " << targetNode.y 
                                  << ", Angle: " << targetNode.angle << std::endl;
                    } 
                    else
                    {
                        std::cout << "\nCar Node with ID is empty!!" << std::endl;
                    }
                }

                {          
                    double startX = currentPos.x(); 
                    double startY = currentPos.y();
                    double startYaw = currentPos.theta();

                    std::cout << "start pos:" << "x   " << currentPos.x() << "  y"<< currentPos.y() << "  theta"<< currentPos.theta() << std::endl;
                    
                    double goalX = targetNode.x;
                    double goalY = targetNode.y;
                    double goalYaw = targetNode.angle * 3.14/180;
                    double gwx,gwy;

                    costmap2DPtr->mapToWorld(goalX,goalY,gwx,gwy);

                    start.setX(startX);
                    start.setY(startY);
                    start.setTheta(startYaw);
                    
                    goal.setX(gwx);
                    goal.setY(gwy);
                    goal.setTheta(goalYaw);

                    planner.init(layers, start,goal);
                    path.clear();
                    found = planner.update(path);

                    std::vector<std::vector<double>> path_points;
                    std::vector<std::vector<double>> lethal_obstacles, inscribed_obstacles, circumscribed_obstacles, free_space, unknown_space;
            
                    for (uint32_t y = 0; y < myMap.height_; ++y) {
                        for (uint32_t x = 0; x < myMap.width_; ++x) {
                            uint8_t cost = costmap2DPtr->getCost(x, y);
                            if (cost == LETHAL_OBSTACLE) {
                                lethal_obstacles.push_back({static_cast<double>(x), static_cast<double>(y)});
                            } else if (cost == INSCRIBED_INFLATED_OBSTACLE) {
                                inscribed_obstacles.push_back({static_cast<double>(x), static_cast<double>(y)});
                            } else if (cost == FREE_SPACE) {
                                free_space.push_back({static_cast<double>(x), static_cast<double>(y)});
                            } else {
                                unknown_space.push_back({static_cast<double>(x), static_cast<double>(y)});
                            }
                        }
                    }
                    
                    if (found) {
                        path_points.clear();
                        for (const auto& node : path) {
                            path_points.push_back({node.x(), node.y()});
                        }
                    }
        
                std::vector<double> x_vals, y_vals;
        
                for (const auto& point : inscribed_obstacles) {
                    x_vals.push_back(point[0]);
                    y_vals.push_back(point[1]);
                }
                //show 
                plt::ylim(500, 0);
                plt::plot(x_vals, y_vals, "r."); 
        
                std::vector<double> obsX, obsY;
        
                for (const auto& point : lethal_obstacles) { 
                    obsX.push_back(point[0]);
                    obsY.push_back(point[1]);
                }
        
                plt::plot(obsX, obsY, "b."); 
              
                std::vector<double> pathX, pathY;
                std::vector<std::vector<double>> waypoints;


                if (found) {
                    waypoints.clear();

                    for (const auto& point : path_points) {
                        int mxx,myy;
                        costmap2DPtr->worldToMap(point[0],point[1] ,mxx,myy);
                        pathX.push_back(  mxx);
                        pathY.push_back(  myy );
                        waypoints.push_back({point[0], point[1]});
                    }
        
                    plt::plot(pathX, pathY, "g-"); 

                }

                plt::grid(true);
                plt::show(true);
                }
                
                if(found)
                {
                    estTime = estimateRuntime(path,targetVel);
                    g_plan_flag = true;
                }
                else
                {
                    g_plan_flag = false;
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(g_control_mutex); 
        
            if(g_parse_control_status_flag)
            {         
                double final_waypoint_x ,final_waypoint_y;

                if(found)
                {                                   
                    if (!controlInit)
                    {
                        controlInit = true;
                        wp.clear();
                        for (const auto& waypoint : path) {                            
                            wp.push_back({waypoint[0],waypoint[1]});
                        }
                        
                        
                        final_waypoint_x = wp.back()[0];
                        final_waypoint_y = wp.back()[1];
                    }
                                
                    if(!g_arrived_target)
                    {
                        double dt = 0.2;
                                                   
                        {
                            bool ultrasonicObsDetected = ulObsState.flObs || ulObsState.rObs;
                        
                            if (ultrasonicObsDetected)
                            {
                                std::cout << "FrontLeft: " << (ulObsState.flObs ? "Obstacle" : "Clear") << std::endl;
                                std::cout << "FrontRight: " << (ulObsState.frObs ? "Obstacle" : "Clear") << std::endl;
        
                                if(2 < ultrasonicDetector.getFrontLeft() &&  ultrasonicDetector.getFrontLeft() < 30 || 2 < ultrasonicDetector.getFrontRight() && ultrasonicDetector.getFrontRight()< 30)
                                {
                                    std::cout << "Ultrasonic: Stop due to obstacle!  " << std::endl;
                                    g_ulObs_stop = true;  
                                }                                                            
                            }
                        }

                        double dist_to_goal = std::hypot(
                            currentPos.x() - final_waypoint_x,
                            currentPos.y() - final_waypoint_y);
                        
                        if (dist_to_goal < arrival_threshold) {
                            double eTheta = regularizeAngle(goal.theta() - currentPos.theta());
                            std::cout << "etheta: "<< eTheta << std::endl;
                            std::cout << "dist to goal: "<< dist_to_goal << std::endl;
                           
                            if(!isStopTargetRunning)
                            {                                
                                stopTarget_start_time = std::chrono::steady_clock::now();
                                isStopTargetRunning = true;                                

                                eTime = 6;
                            }
                            else
                            {
                                auto elapsed_time_target = std::chrono::duration_cast<std::chrono::seconds>(
                                    std::chrono::steady_clock::now() - stopTarget_start_time
                                ).count();
                                
                                if (elapsed_time_target >= eTime) {
                                    g_arrived_target = true;
                                    controlInit = false;
                                    
                                    vw.steer = 0;
                                    vw.v = 0;                               
                                }   
                            } 
                            
                            if (dist_to_goal < 0.2)
                            {
                                g_arrived_target = true;
                                controlInit = false;
                                vw.steer = 0; 
                                vw.v = 0;
                               std::cout << "Reached the final waypoint." << std::endl;
                            }                           
                        }

                        auto [speed, steer] = local_planner.computeAvoidanceCommand(
                            currentPos, wp, velocity, current_steer, obstacles,ulObsState,ultrasonicDetector.getFrontLeft(),ultrasonicDetector.getFrontRight());
                            current_steer = steer;
                            vw.steer = steer;
                            vw.v = speed;            
                    }
                }  
                
                if(!latest_data.lidardata.empty())
                {
                    obstacleDetector_.processScan(latest_data.lidardata.back(),currentPos);

                    obstacles = obstacleDetector_.getObstacles();

                    ultrasonicDetector.updateData(latest_data.ultrasonundData);
                
                    ulObsState = ultrasonicDetector.getObstacleState();

                    float currentMinDistance =  getMinObsDistance(obstacles);
                     
                    if (currentMinDistance < SAFE_DIS) {
                        if (!isTimerStarted) {
                            obs_stop_start_time = std::chrono::steady_clock::now();
                            isTimerStarted = true;
                            
                        }
                        g_obs_stop = true;   
                        std::cout << "obs:stop !!!"  << std::endl;            
                    } 

                    if (isTimerStarted) {
                        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::steady_clock::now() - obs_stop_start_time
                        ).count();
                
                        if (elapsed_time >= 5) {
                            isTimerStarted = false;
                            obstacleDetector_.clearObs();
                        }
                    } 
                    
                    if (!isTimerStarted && currentMinDistance >= SAFE_DIS) {
                        g_obs_stop = false;
                    }
                }
            }          
        }

        usleep(50000);
    }

    delete[] myMap.pMapData_;
}

void msgq_send_fun()
{
    struct _msg smsg_data; 
    std::shared_ptr msg_send = std::make_shared<Msgq>();
	msg_send->msg_create(&smsg_data,1236);
   
    int count = 0;

    while(!g_interrupt_flag)
    {
        usleep(50000);
        {
            cmd_vel current_vw;
            {
                std::lock_guard<std::mutex> lock(g_control_mutex);
                current_vw = vw;
            }
        
            if(stopStatus || !posStatus || g_obs_stop || g_ulObs_stop)
            {
                stopStatus = false;
                g_obs_stop = false;
                g_ulObs_stop = false;
                posStatus = true;
                isStopTargetRunning = false;
                smsg_data.data.type = 2;
                smsg_data.data.x = count;
                smsg_data.data.y = count;

                smsg_data.data.time = count;
                smsg_data.data.v = static_cast<int>(0);
                smsg_data.data.theta = static_cast<int>(0);
        
                int ret = msg_send->msg_send(&smsg_data);

                if(ret==-1)
                {
                    printf("msg queue send fail !\n");
                    msg_send->msg_create(&smsg_data,1236);
                }
            }     
            else if(g_parse_control_status_flag)
            {
                smsg_data.data.type = 2;
                smsg_data.data.x = 0;
                smsg_data.data.y = 0;
            
                smsg_data.data.time = 0;
                smsg_data.data.v = static_cast<int>(current_vw.v *1000);
                smsg_data.data.theta = static_cast<int>(current_vw.steer * 1000);
        
                int ret = msg_send->msg_send(&smsg_data);

                if(ret==-1)
                {
                    printf("msg queue send fail !\n");
                   msg_send->msg_create(&smsg_data,1236);
                }
                std::cout<<"v:  "<<current_vw.v<< "  theta:  " << current_vw.steer <<std::endl; 
            }     
        }
    }
    
    msg_send->msg_del(&smsg_data);
}

void transformOffset (const Point3d & originalPoint, const Point3d &offset,Point3d& transformedPoint)
{
    double x = originalPoint.x();
    double y = originalPoint.y();
    double theta = originalPoint.theta();

    double offsetX = offset.x();
    double offsetY = offset.y();
    double offsetTheta = offset.theta();

    transformedPoint.setX(offsetX * cos(theta) - offsetY * sin(theta) + x);
    transformedPoint.setY(offsetX * sin(theta) + offsetY * cos(theta) + y);
    transformedPoint.setTheta(theta + offsetTheta);


    while (transformedPoint.theta() > M_PI) {
        transformedPoint.setTheta(transformedPoint.theta() -2.0 * M_PI);
    }
    while (transformedPoint.theta() <= -M_PI) {
        transformedPoint.setTheta(transformedPoint.theta()+ 2.0 * M_PI);
    }
}

void msgq_recv_fun()
{
    struct _msg msg_data_rec;
    std::shared_ptr msg_rec = std::make_shared<Msgq>();
    msg_rec->msg_create(&msg_data_rec,1235);

    SensorData data;

    while(!g_interrupt_flag)
    {
        usleep(20000);
        int ret = msg_rec->msg_recv_nowait(&msg_data_rec);

        if(ret==-1)
        {
            usleep(100000);
            continue;
        }
        
        {
            Point3d originalPoint = Point3d(static_cast<double>(msg_data_rec.data.x )/ 1000 , static_cast<double>(msg_data_rec.data.y )/ 1000, static_cast<double>(msg_data_rec.data.theta )/ 1000);
            
            Point3d offset(-0.6,0,0);
            Point3d transformedPoint;
            
            transformOffset(originalPoint,offset,transformedPoint);
            data.curPos = transformedPoint;

            data.cur_v =  static_cast<double>(msg_data_rec.data.v )/ 1000;     
        }

        {         
            LaserScanData newScan;
            data.lidardata.clear();

            std::reverse_copy(msg_data_rec.data.laserscan,
            msg_data_rec.data.laserscan + 380,
            newScan.ranges.begin());

            newScan.stamp =  msg_data_rec.data.time;
            data.lidardata.push_back(newScan);
            memcpy(data.ultrasonundData,msg_data_rec.data.ultrasound,sizeof(data.ultrasonundData));
        }

        sensorDataQueue.push(data);
               
        msg_data_rec.data.type = 1;
        msg_data_rec.data.x = 0;
        msg_data_rec.data.y  = 0;
        msg_data_rec.data.yaw = 0;
        msg_data_rec.data.v = 0;
        msg_data_rec.data.theta = 0;
    }
    
}

int main()
{
    signal(SIGINT, stop);

    config.wb = 0.756;   
    config.mst = 0.28;      
    config.msd = 0.5;       
    config.mac = 0.5;      
    config.vwd = 0.85;   
    config.vlt = 1.44;  

    std::string cfgFile_= cfgFile;

    loaderJsonInit(cfgFile_);

    std::thread t(tcp_client_fun);
    std::thread mainThread(mainThread_fun);
    std::thread msg_t(msgq_send_fun);
    std::thread msg_r(msgq_recv_fun);

    while (!g_interrupt_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (g_interrupt_flag) {
            break;
        }
    }

    t.join();
    mainThread.join();
    msg_t.join();
    msg_r.join();

    return 0;
}
