#include "pgmMgr/yamlMgr.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <iostream>

bool readMapYAML(std::string name,float &resolution,std::vector<float> &origin)
{
    YAML::Node yamlNode;

    try {    
        std::ifstream yamlFile(name);
        if (!yamlFile.is_open()) {
            std::cerr << "Failed to open YAML file: " << name << std::endl;
            return false;
        }
        yamlNode = YAML::Load(yamlFile);
    } catch (const YAML::Exception& e) {
        printf("Error parsing YAML file: %s", e.what());
        return false;
    }

    if (!yamlNode["image"]) {
            throw std::invalid_argument("Missing required field: image");
        }
    std::string image_filename = yamlNode["image"].as<std::string>();

    if (!yamlNode["mode"]) {
    }

    if (!yamlNode["resolution"]) {
        throw std::invalid_argument("Missing required field: resolution");
    }
    resolution = yamlNode["resolution"].as<double>();

    if (!yamlNode["origin"]) {
        throw std::invalid_argument("Missing required field: origin");
    }

    YAML::Node origin_node = yamlNode["origin"];
    if (origin_node.IsSequence() && origin_node.size() == 3) {
        for (const auto& item : origin_node) {
            origin.push_back(item.as<double>());
        }
    } else {
        throw std::invalid_argument("Invalid origin field: expected a sequence of 3 elements");
    }

    if (!yamlNode["negate"]) {
            throw std::invalid_argument("Missing required field: negate");
        }
    int negate = yamlNode["negate"].as<int>();

    if (!yamlNode["occupied_thresh"]) {
        throw std::invalid_argument("Missing required field: occupied_thresh");
    }
    double occupied_thresh = yamlNode["occupied_thresh"].as<double>();

    if (!yamlNode["free_thresh"]) {
        throw std::invalid_argument("Missing required field: free_thresh");
    }
    double free_thresh = yamlNode["free_thresh"].as<double>();

    return 1;
}