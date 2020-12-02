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
 * The frame of the laser. The frame of uav in GAZEBO。
 *                                  △ X
 *           |                      |
 *           |      Y          Y    |
 *     ——————|——————>        <——————|———————
 *           |      *               |
 *           |                      |
 *           ▽
 * 
***************************************************************************************************************************/
#include"include/sector.h"
#define MID 2


bool SectorMap::is_front_safe(){
    float temp_cv;
    for(int i=0;i<sector_cv.size();i++){
        if(i==2){
            temp_cv=sector_cv[i].cv;
        }
    }
    if(temp_cv<0.1){
        safety_flag=true;
    }
    else
    {
        safety_flag=false;
    }
    
    return safety_flag;
}

float SectorMap::change_direction(float direction){
  
    std::cout<<"sector_cv_size "<<sector_cv.size()<<std::endl;
    if(safety_flag==true){
        std::cout<<"safe "<<std::endl;
        return 0;
    }
    // else{
    //     std::cout<<"danger "<<std::endl;
    //     for(int i=0;i<sector_cv.size();i++){
    //         if(sector_cv[i].safe==true){
                
    //             temp.push_back(sector_cv[i]);
    //         }
    //     }
    //     std::cout<<"temp_size1"<<temp.size()<<std::endl;
    //     int index=0;float diff=abs(temp[0].radian_value-target_direction);
    //     for(int i=0;i<temp.size();i++ ){
    //         if(diff>abs(temp[i].radian_value-target_direction)){
    //             index=i;
    //         }
    //     }
    //     std::cout<<"temp_size"<<temp.size()<<" "<<temp[index].radian_value<<std::endl;
    //     if(temp.size()==0){
    //         return PI;
    //     }
    //     return temp[index].radian_value;
    // }
}

void SectorMap::down_sample(std::vector<float>r){
    float radian=0.0;
    float dist[int(360/sector_value)] = { 0 };
	map_cv.clear();
    sector_cv.clear();
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
    //【将扇区值存放到map_cv中】
	for (int j = 0; j < (int)(360 / sector_value); j++)
	{
        //std::cout<<"the i"<<j<<"th is "<<dist[j]<<std::endl;
		map_cv.push_back(dist[j]);
	}
    for(int i=7;i<=16;i=i+2){
        float sum_cv=0;
        radian=radian+0.5236;
        for(int j=i;j<=i+1;j++){
            sum_cv=sum_cv+map_cv[j];
        }
        SECTOR temp;
        if(sum_cv<0.1){
            temp.safe=true;
        }
        else{
            temp.safe=false;
        }
        temp.cv=sum_cv;
        temp.radian_value=radian;
        sector_cv.push_back(temp);

    }    
    // for(int i=0;i<sector_cv.size();i++){
    //     std::cout<<"cv "<<sector_cv[i].cv<<" radian "<<sector_cv[i].radian_value<<std::endl;
    // }
}

float SectorMap::calculate_target_direction(multi_uav::uav_state leader,multi_uav::uav_state follower){
    target_direction=atan2((leader.pose.position.y-follower.pose.position.y),(leader.pose.position.x-follower.pose.position.x));
    return target_direction;
}

float SectorMap::calculate_target_direction1(Eigen::Vector3d target,Eigen::Vector3d follower){
    std::cout<<"position 2"<<std::endl;
    target_direction=atan2((target[1]-follower[1]),(target[0]-follower[0]));
    return target_direction;
}