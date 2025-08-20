#include "costmap.hpp"

#include <cstring>
#include "costValues.hpp"

#include <iostream>
#include <map>

Costmap::Costmap():
lethal_threshold_(10),
ilayers_(nullptr)
{
	
}

Costmap::~Costmap()
{
	if (mapInfo_.pMapData_ != nullptr) {
		delete[] mapInfo_.pMapData_;
		mapInfo_.pMapData_ = nullptr;
	}
}

void addInflationLayer(
LayeredCostmap & layers,
std::shared_ptr<InflationLayer> & ilayer)
{
    ilayer = std::make_shared<InflationLayer>();
    ilayer->initialize(&layers, "inflation");
    std::shared_ptr<Layer> ipointer(ilayer);
    layers.addPlugin(ipointer);
}

void Costmap::init(std::shared_ptr<LayeredCostmap> layers,mapInfo* pMapInfo,std::vector<Point3d> polygon)
{  
    std::vector<std::vector<unsigned char> > originMapData;

    if (readmapInfo(originMapData, pMapInfo))
    {
        layers_ = layers;
        layers_->resizeMap(mapInfo_.width_, mapInfo_.height_, mapInfo_.resolution_, mapInfo_.originX_, mapInfo_.originY_);
		addInflationLayer(*layers_,ilayers_);

        layers_->setFootprint(polygon);
        
        layers_->updateMap(0, 0, 0);
        set_static_map(originMapData);

        ilayers_->updateCosts(*layers_->getCostmap(), 0, 0, mapInfo_.width_, mapInfo_.height_);        
    }
}

void Costmap::set_static_map(std::vector<std::vector<unsigned char> >& originMapInfo)
{
	uint32_t width, height;
	uint32_t freeSpaceNum = 0;

	width = originMapInfo[0].size();
	height = originMapInfo.size();

    for (uint32_t i = 0; i < width; i++)
    {
        for (uint32_t j = 0; j < height; j++)
        {
			if (originMapInfo[j][i] == 0)
			{
				layers_->getCostmap()->setCost(i, j, LETHAL_OBSTACLE);
			}
			else if (originMapInfo[j][i] >= 250)
			{
				layers_->getCostmap()->setCost(i, j, FREE_SPACE);
			}
			else if (originMapInfo[j][i] == 127)
			{
				layers_->getCostmap()->setCost(i, j, NO_INFORMATION);
			}
			else 
			{
				double scale = static_cast<double>(originMapInfo[j][i] / lethal_threshold_);
				layers_->getCostmap()->setCost(i, j, static_cast<uint8_t>(scale * LETHAL_OBSTACLE));
			}
        }
    }
}

bool Costmap::readmapInfo(std::vector<std::vector<unsigned char> >& originmapInfo, mapInfo* pMapInfo)
{
	if (mapInfo_.pMapData_ != nullptr) {
		delete[] mapInfo_.pMapData_;
		mapInfo_.pMapData_ = nullptr;
	  }

	mapInfo_.width_ = pMapInfo->width_;
	mapInfo_.height_ = pMapInfo->height_;

	if (mapInfo_.width_ == 0 || mapInfo_.height_ == 0)
	{
		return false;
	}

	mapInfo_.resolution_ =  pMapInfo->resolution_;
	mapInfo_.originX_ = pMapInfo->originX_;
	mapInfo_.originY_ = pMapInfo->originY_;

	uint32_t dataSize = mapInfo_.width_ * mapInfo_.height_;
    mapInfo_.pMapData_ = new unsigned char[dataSize];

    memcpy(mapInfo_.pMapData_,pMapInfo->pMapData_,dataSize*sizeof(unsigned char));

	originmapInfo.resize(mapInfo_.height_);
	for (uint32_t i = 0; i < mapInfo_.height_; ++i) {
		originmapInfo[i].resize(mapInfo_.width_);
		for (uint32_t j = 0; j < mapInfo_.width_; ++j) {
			originmapInfo[i][j] = mapInfo_.pMapData_[i * mapInfo_.width_ + j];
		}
	}

	return true;
}
