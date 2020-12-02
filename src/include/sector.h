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
 *                                  △ X
 *           |                      |
 *           |      Y          Y    |
 *     ——————|——————>        <——————|———————
 *           |      *               |
 *           |                      |
 *           ▽
 * 
***************************************************************************************************************************/

#ifndef SECTORMAP_H
#define SECTORMAP_H
#include<iostream>
#include <vector>
#include<cmath>
#include<map>
#include"multi_uav/uav_state.h"
#include <eigen_conversions/eigen_msg.h>
#define PI 3.141592653589793238327950

struct SECTOR
{
	bool safe;
	float cv;
	float radian_value;
};


class SectorMap
{
public:
    float scan_distance_max;
	float scan_distance_min;
    int sector_value;
	float angle_resolution;
	int begin_index;
	int end_index;
	int front_index;
	float target_direction;
	//【用于存放安全标志】
	bool safety_flag;
	//【扇区确信值，越小越安全】
	std::vector<float>map_cv;
	//【用于存放飞机前向150°，5个扇区确信值】
	std::vector<SECTOR>sector_cv;

	SectorMap()
	{
		safety_flag=true;
		scan_distance_max = 2.1;
		scan_distance_min = 0.1;
		sector_value = 15;
		begin_index=209;
		end_index=509;
		//【This value come from laser's sdf】
		angle_resolution=0.5625;
		front_index=2;
	}
	virtual ~SectorMap()
	{

	}
	//【判断前方是否安全】
    bool is_front_safe();
    //【计算出前进角度】
	float change_direction(float target_dir);
	//【将640组数据映射成24组，并计算确信值,然后返回正前方五个扇区确信值】
	void down_sample(std::vector<float>r);
	//【计算目方向】
	float calculate_target_direction(multi_uav::uav_state leader,multi_uav::uav_state follower);
	float calculate_target_direction1(Eigen::Vector3d target,Eigen::Vector3d follower);

};

#endif // SPHEREMAP_H