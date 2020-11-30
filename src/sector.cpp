/***************************************************************************************************************************
 * Function: obstacle avoid strategy  mainly for 2D laser
 * 
 * Node name: the cpp file of the sector
 *
 * Author: Ma Chenxiang
 *
 * Update Time: 2020.11.29
 *
 * Email: mcx1243858461@gmail.com
 * 
 * Note: In this simulation,we use hokuyo_lidar which min and max valid distace respectly are 0.8 and 10m and samples are 640. 
 * 
 * The frame of the laser.
 *           
 *           |
 *           |      Y
 *     ——————|——————>
 *           |
 *           |
 *           ▽
 * 
***************************************************************************************************************************/
#include"include/sector.h"



bool SectorMap::is_front_safe(std::vector<float> v){

}

float SectorMap::change_direction(std::vector<float>v){

}

void SectorMap::down_sample(std::vector<float>r){
    float dist[int(360/sector_value)] = { 0 };
	map_cv.clear();
    std::cout<<r.size()<<std::endl;
	for (size_t i = 0; i < r.size(); i++)
	{
		//A non-zero value (true) if x is a NaN value; and zero (false) otherwise.
		//isinf A non-zero value (true) if x is an infinity; and zero (false) otherwise.
		//判断数据有效性
		if (!std::isnan(r[i]) && !std::isinf(r[i]))
		{
			float scan_distance = r[i];
			//judge the scan range in which sector
			//floor take an  integer down
			//判断这束激光属于12个30°扇区中的哪一个
			int sector_index = std::floor((i*angle_resolution) / sector_value);
			if (scan_distance >= scan_distance_max || scan_distance < scan_distance_min)
				scan_distance = 0;
			else
				scan_distance = scan_distance_max - scan_distance;
			//divided into 24  mesh,and put the into the mesh
			dist[sector_index] += scan_distance;
		}

	}
	for (int j = 0; j < (int)(360 / sector_value); j++)
	{
		map_cv.push_back(dist[j]);
	}
}