#ifndef LOAD_JSON_HPP_
#define LOAD_JSON_HPP_
#include <string>
#include <vector>

struct SpeedBumpNode {
    std::string id;
    double x;
    double y;
    int x1, x2, x3, x4;
    int y1, y2, y3, y4;
};

struct RestructedZoneNode {
    std::string id;
    double x;
    double y;
    int x1, x2, x3, x4;
    int y1, y2, y3, y4;
};

struct CarNode {
    std::string id;
    double x;
    double y;
    int x1, x2, x3, x4;
    int y1, y2, y3, y4;
    double angle;
};

void loaderJsonInit(const std::string &cfgFile);
bool loadJsonParse(std::vector<SpeedBumpNode>& speedBumps, 
    std::vector<RestructedZoneNode>& restructedZones, 
    std::vector<CarNode>& carNodes);

    CarNode getCarNodeById(const std::vector<CarNode>& carNodes, const std::string& id) ;
    SpeedBumpNode getSpeedBumpNodeById(const std::vector<SpeedBumpNode>& carNodes, const std::string& id) ;    
    RestructedZoneNode getRestructedZoneNodeById(const std::vector<RestructedZoneNode>& carNodes, const std::string& id) ;
#endif

