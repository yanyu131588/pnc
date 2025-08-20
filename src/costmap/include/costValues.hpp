#ifndef COST_VALUES_HPP_
#define COST_VALUES_HPP_

#include <cstdint>

typedef struct
{
	double inflation_radius_;
	double inscribed_radius_;
	double cost_scaling_factor_;
}costmapParams;

typedef enum mapCostInfo {
	FREE_SPACE = 0,            

	MAP_CIRCUMSCRIBED = 127,   
	INSCRIBED_INFLATED_OBSTACLE = 90,  
	LETHAL_OBSTACLE = 254,     
	NO_INFORMATION = 255         
};

#define INFLATION_RADIUS 1.8 
#define INSCRIBED_RADIUS 1.4
#define COST_SCALING_FACTOR 3.8 //0808 small -> far , 
#endif