#include "mapMgr/mapMgr.hpp"

#include <string>
#include <filesystem>
#include <optional>

#include "pgmMgr/pgmMgr.hpp"
#include "pgmMgr/yamlMgr.hpp"
#include <cstring>

#include <iostream>

namespace fs = std::filesystem;

std::optional<std::string> getPath(std::string name,std::string path)
{
    fs::path targetDirPath = path;
    fs::path targetFilePath = targetDirPath / (name);

    if (fs::exists(targetFilePath) && fs::is_regular_file(targetFilePath)) {

        return targetFilePath.string();
    } else {
        return std::nullopt;
    }
}

bool loadMap(mapInfo *pMapInfo,std::string name,std::string path)
{
    auto mapFileName = getPath(name+".pgm",path);

    if (!mapFileName) {
        std::cerr << "Error: Map file does not exist." << std::endl;
        return false;
    }

    pgmDef image;

    if(!readPGM(*mapFileName,image))
    {
        std::cerr << "Failed to read PGM file." << std::endl;
        return false;
    }

    float resolution;
    std::vector<float> origin;
    auto yamlFilename = getPath(name+".yaml",path);

    readMapYAML(*yamlFilename,resolution,origin);

    pMapInfo->width_ = image.width_;
    pMapInfo->height_= image.height_;
    uint32_t dataSize = pMapInfo->width_ * pMapInfo->height_;

    pMapInfo->pMapData_ = new uint8_t[dataSize](); 
 
    std::memcpy(pMapInfo->pMapData_, image.mapData_.data(), dataSize);

    pMapInfo->originX_ = origin[0];
    pMapInfo->originY_ = origin[1];
    pMapInfo->resolution_ = resolution;

    return true;
}
