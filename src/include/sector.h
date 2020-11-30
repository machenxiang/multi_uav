/***************************************************************************************************************************
 * Function: obstacle avoid strategy  mainly for 2D laser
 * 
 * Node name: the .h file of sector
 *
 * Author: Ma Chenxiang
 *
 * Update Time: 2020.11.29
 *
 * Email: mcx1243858461@gmail.com
 * 
 * Note: In this simulation,we use hokuyo_lidar which min and max valid distace respectly are 0.08 and 10m and samples are 640. 
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

#ifndef SECTORMAP_H
#define SECTORMAP_H
#include<iostream>
#include <vector>
#include<cmath>


class SectorMap
{
public:
    float scan_distance_max;
	float scan_distance_min;
    int sector_value;
	bool safety_flag;
	float angle_resolution;
	int begin_index;
	int end_index;
	//【扇区确信值，越小越安全】
	std::vector<float>map_cv;

	SectorMap()
	{
		scan_distance_max = 2.1;
		scan_distance_min = 0.1;
		sector_value = 15;
		begin_index=209;
		end_index=509;
		//【This value come from laser's sdf】
		angle_resolution=0.5625;
        //safe is ture
		safety_flag=true;
	}
	virtual ~SectorMap()
	{

	}
	//【判断前方是否安全】
    bool is_front_safe(std::vector<float> v);
    //【计算出前进角度】
	float change_direction(std::vector<float>v);
	//【将640组数据映射成24组，并计算确信值】
	void down_sample(std::vector<float>r);

};

#endif // SPHEREMAP_H