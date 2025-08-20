#ifndef COSTMAP_HPP_
#define COSTMAP_HPP_


#include <memory>
#include <vector>
#include "common/dataDef.hpp"
#include "layeredCostmap.hpp"
#include "inflationLayer.hpp"

class Costmap
{
	public:
		Costmap();
		~Costmap();

		void init(std::shared_ptr<LayeredCostmap> layers,mapInfo* pMapInfo,std::vector<Point3d> polygon);

    private:
        mapInfo mapInfo_;
        std::shared_ptr<LayeredCostmap>  layers_;

        int lethal_threshold_;
        std::shared_ptr<InflationLayer> ilayers_;

    private:
        void set_static_map(std::vector<std::vector<unsigned char> >& originMapInfo);
        bool readmapInfo(std::vector<std::vector<unsigned char> >& originMapInfo, mapInfo* pMapInfo);
};


#endif