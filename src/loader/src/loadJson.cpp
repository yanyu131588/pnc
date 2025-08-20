#include "loadJson.hpp"
#include <fcntl.h>
#include <unistd.h>

#include "json/include/cJSON.h"

#include<fstream>
#include <iostream>


std::string cfgFile_;

void loaderJsonInit(const std::string &cfgFile)
{
    cfgFile_ = cfgFile;
}

void parseSpeedBumpNode(cJSON* nodeObj, std::vector<SpeedBumpNode>& speedBumps) {
    if (!nodeObj || !cJSON_IsObject(nodeObj)) return;

    cJSON* idValue = cJSON_GetObjectItem(nodeObj, "id");
    cJSON* xValue = cJSON_GetObjectItem(nodeObj, "x");
    cJSON* yValue = cJSON_GetObjectItem(nodeObj, "y");
    cJSON* x1Value = cJSON_GetObjectItem(nodeObj, "x1");
    cJSON* x2Value = cJSON_GetObjectItem(nodeObj, "x2");
    cJSON* x3Value = cJSON_GetObjectItem(nodeObj, "x3");
    cJSON* x4Value = cJSON_GetObjectItem(nodeObj, "x4");
    cJSON* y1Value = cJSON_GetObjectItem(nodeObj, "y1");
    cJSON* y2Value = cJSON_GetObjectItem(nodeObj, "y2");
    cJSON* y3Value = cJSON_GetObjectItem(nodeObj, "y3");
    cJSON* y4Value = cJSON_GetObjectItem(nodeObj, "y4");

    if (!idValue || !cJSON_IsString(idValue) ||
        !xValue || !cJSON_IsNumber(xValue) ||
        !yValue || !cJSON_IsNumber(yValue)) return;

    SpeedBumpNode node;
    node.id = cJSON_GetStringValue(idValue);
    node.x = xValue->valuedouble;
    node.y = yValue->valuedouble;
    node.x1 = x1Value ? x1Value->valueint : 0;
    node.x2 = x2Value ? x2Value->valueint : 0;
    node.x3 = x3Value ? x3Value->valueint : 0;
    node.x4 = x4Value ? x4Value->valueint : 0;
    node.y1 = y1Value ? y1Value->valueint : 0;
    node.y2 = y2Value ? y2Value->valueint : 0;
    node.y3 = y3Value ? y3Value->valueint : 0;
    node.y4 = y4Value ? y4Value->valueint : 0;

    speedBumps.push_back(node);
}

void parseRestructedZoneNode(cJSON* nodeObj, std::vector<RestructedZoneNode>& restructedZones) {
    if (!nodeObj || !cJSON_IsObject(nodeObj)) return;

    cJSON* idValue = cJSON_GetObjectItem(nodeObj, "id");
    cJSON* xValue = cJSON_GetObjectItem(nodeObj, "x");
    cJSON* yValue = cJSON_GetObjectItem(nodeObj, "y");
    cJSON* x1Value = cJSON_GetObjectItem(nodeObj, "x1");
    cJSON* x2Value = cJSON_GetObjectItem(nodeObj, "x2");
    cJSON* x3Value = cJSON_GetObjectItem(nodeObj, "x3");
    cJSON* x4Value = cJSON_GetObjectItem(nodeObj, "x4");
    cJSON* y1Value = cJSON_GetObjectItem(nodeObj, "y1");
    cJSON* y2Value = cJSON_GetObjectItem(nodeObj, "y2");
    cJSON* y3Value = cJSON_GetObjectItem(nodeObj, "y3");
    cJSON* y4Value = cJSON_GetObjectItem(nodeObj, "y4");

    if (!idValue || !cJSON_IsString(idValue) ||
        !xValue || !cJSON_IsNumber(xValue) ||
        !yValue || !cJSON_IsNumber(yValue)) return;

    RestructedZoneNode node;
    node.id = cJSON_GetStringValue(idValue);
    node.x = xValue->valuedouble;
    node.y = yValue->valuedouble;
    node.x1 = x1Value ? x1Value->valueint : 0;
    node.x2 = x2Value ? x2Value->valueint : 0;
    node.x3 = x3Value ? x3Value->valueint : 0;
    node.x4 = x4Value ? x4Value->valueint : 0;
    node.y1 = y1Value ? y1Value->valueint : 0;
    node.y2 = y2Value ? y2Value->valueint : 0;
    node.y3 = y3Value ? y3Value->valueint : 0;
    node.y4 = y4Value ? y4Value->valueint : 0;

    restructedZones.push_back(node);
}

void parseCarNode(cJSON* nodeObj, std::vector<CarNode>& carNodes) {
    if (!nodeObj || !cJSON_IsObject(nodeObj)) return;

    cJSON* idValue = cJSON_GetObjectItem(nodeObj, "id");
    cJSON* xValue = cJSON_GetObjectItem(nodeObj, "x");
    cJSON* yValue = cJSON_GetObjectItem(nodeObj, "y");
    cJSON* x1Value = cJSON_GetObjectItem(nodeObj, "x1");
    cJSON* x2Value = cJSON_GetObjectItem(nodeObj, "x2");
    cJSON* x3Value = cJSON_GetObjectItem(nodeObj, "x3");
    cJSON* x4Value = cJSON_GetObjectItem(nodeObj, "x4");
    cJSON* y1Value = cJSON_GetObjectItem(nodeObj, "y1");
    cJSON* y2Value = cJSON_GetObjectItem(nodeObj, "y2");
    cJSON* y3Value = cJSON_GetObjectItem(nodeObj, "y3");
    cJSON* y4Value = cJSON_GetObjectItem(nodeObj, "y4");
    cJSON* angleValue = cJSON_GetObjectItem(nodeObj, "angle");

    if (!idValue || !cJSON_IsString(idValue) ||
        !xValue || !cJSON_IsNumber(xValue) ||
        !yValue || !cJSON_IsNumber(yValue) ||
        !angleValue || !cJSON_IsNumber(angleValue)) return;

    CarNode node;
    node.id = cJSON_GetStringValue(idValue);
    node.x = xValue->valuedouble;
    node.y = yValue->valuedouble;
    node.x1 = x1Value ? x1Value->valueint : 0;
    node.x2 = x2Value ? x2Value->valueint : 0;
    node.x3 = x3Value ? x3Value->valueint : 0;
    node.x4 = x4Value ? x4Value->valueint : 0;
    node.y1 = y1Value ? y1Value->valueint : 0;
    node.y2 = y2Value ? y2Value->valueint : 0;
    node.y3 = y3Value ? y3Value->valueint : 0;
    node.y4 = y4Value ? y4Value->valueint : 0;
    node.angle = angleValue->valuedouble;

    carNodes.push_back(node);
}

bool loadJsonParse(std::vector<SpeedBumpNode>& speedBumps, 
    std::vector<RestructedZoneNode>& restructedZones, 
    std::vector<CarNode>& carNodes)
{
    const char* jsonFilePath = cfgFile_.c_str();

    std::ifstream file(jsonFilePath);
    if (!file) {
        std::cerr << "Error: Could not open JSON file " << jsonFilePath << std::endl;
        return false;
    }

    std::string jsonContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (jsonContent.empty()) {
        std::cerr << "Error: JSON file is empty" << std::endl;
        return false;
    }

    cJSON* rootNode = cJSON_ParseWithOpts(jsonContent.c_str(), nullptr, true);
    if (!rootNode) {
        const char* error_ptr = cJSON_GetErrorPtr();
        if (error_ptr) {
            std::cerr << "Error before: " << error_ptr << std::endl;
        } else {
            std::cerr << "Error: JSON Parse failed" << std::endl;
        }
        return false;
    }

    cJSON* speedBumpArray = cJSON_GetObjectItem(rootNode, "speedBump");
    if (cJSON_IsArray(speedBumpArray)) {
        for (int i = 0; i < cJSON_GetArraySize(speedBumpArray); ++i) {
            cJSON* nodeObj = cJSON_GetArrayItem(speedBumpArray, i);
            parseSpeedBumpNode(nodeObj, speedBumps);
        }
    }

    cJSON* restructedZoneArray = cJSON_GetObjectItem(rootNode, "restructedZone");
    if (cJSON_IsArray(restructedZoneArray)) {
        for (int i = 0; i < cJSON_GetArraySize(restructedZoneArray); ++i) {
            cJSON* nodeObj = cJSON_GetArrayItem(restructedZoneArray, i);
            parseRestructedZoneNode(nodeObj, restructedZones);
        }
    }

    cJSON* carNodesArray = cJSON_GetObjectItem(rootNode, "carNodes");
    if (cJSON_IsArray(carNodesArray)) {
        for (int i = 0; i < cJSON_GetArraySize(carNodesArray); ++i) {
            cJSON* nodeObj = cJSON_GetArrayItem(carNodesArray, i);
            parseCarNode(nodeObj, carNodes);
        }
    }

    cJSON_Delete(rootNode);

    return true;
}

CarNode getCarNodeById(const std::vector<CarNode>& carNodes, const std::string& id) {
    for (const auto& node : carNodes) {
        if (node.id == id) {
            return node;
        }
    }

    return CarNode();
}

SpeedBumpNode getSpeedBumpNodeById(const std::vector<SpeedBumpNode>& carNodes, const std::string& id) {
    for (const auto& node : carNodes) {
        if (node.id == id) {
            return node;
        }
    }

    return SpeedBumpNode();
}

RestructedZoneNode getRestructedZoneNodeById(const std::vector<RestructedZoneNode>& carNodes, const std::string& id) {
    for (const auto& node : carNodes) {
        if (node.id == id) {
            return node;
        }
    }

    return RestructedZoneNode();
}