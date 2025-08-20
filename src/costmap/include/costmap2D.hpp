#ifndef COSTMAP2D_HPP_
#define COSTMAP2D_HPP_

#include <cstdint>
#include "common/dataDef.hpp"
#include "costValues.hpp"
#include <string.h>


class Costmap2D
{
    public:
        Costmap2D();

        ~Costmap2D();
        explicit Costmap2D(const Costmap2D & map);

        Costmap2D & operator=(const Costmap2D & map);

        unsigned char getCost(unsigned int mx, unsigned int my) const;
        unsigned char getCost(unsigned int index) const;
        void setCost(unsigned int mx, unsigned int my, unsigned char cost);

        void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;
        bool worldToMap(double wx, double wy, int & mx, int & my) const;
        void worldToMapEnforceBounds(double wx, double wy, int & mx, int & my) const;
        
        unsigned int getIndex(unsigned int mx, unsigned int my) const
        {
            return my * size_x_ + mx;
        }

        void indexToCells(unsigned int index, unsigned int & mx, unsigned int & my) const
        {
            my = index / size_x_;
            mx = index - (my * size_x_);
        }

        unsigned char * getCharMap() const;

        unsigned int getSizeInCellsX() const;
        unsigned int getSizeInCellsY() const;
        double getSizeInMetersX() const;
        double getSizeInMetersY() const;
        double getOriginX() const;
        double getOriginY() const;
        double getResolution() const;

        void setDefaultValue(unsigned char c)
        {
            default_value_ = c;
        }

        unsigned char getDefaultValue()
        {
            return default_value_;
        }

        virtual void updateOrigin(double new_origin_x, double new_origin_y);

        void resizeMap(
        unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
        double origin_y);
        void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);
        void resetMapToValue(
        unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn, unsigned char value);

        unsigned int cellDistance(double world_dist);

    protected:
        unsigned int size_x_;
        unsigned int size_y_;
        double resolution_;
        double origin_x_;
        double origin_y_;

        unsigned char * costmap_;
        unsigned char default_value_;

        virtual void resetMaps();
        virtual void initMaps(unsigned int size_x, unsigned int size_y);

        template<typename data_type>
        void copyMapRegion(
        data_type * source_map, unsigned int sm_lower_left_x,
        unsigned int sm_lower_left_y,
        unsigned int sm_size_x, data_type * dest_map, unsigned int dm_lower_left_x,
        unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
        unsigned int region_size_y)
        {
          data_type * sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
          data_type * dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

          for (unsigned int i = 0; i < region_size_y; ++i) {
            memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
            sm_index += sm_size_x;
            dm_index += dm_size_x;
          }
        }
};





#endif