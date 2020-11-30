/***************************************************************************************************************************
 * Function: follower_uav track the leader uav and avoid the obstacle 
 * 
 * Node name: follower_node
 *
 * Author: Ma Chenxiang
 *
 * Update Time: 2020.11.29
 *
 * Email: mcx1243858461@gmail.com
***************************************************************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <eigen_conversions/eigen_msg.h>
#include"multi_uav/uav_state.h"
#include<sensor_msgs/LaserScan.h>
#include<string>
#include"include/sector.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>宏 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define PI 3.141592653589793238327950
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量 声 明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Eigen::Vector3d follower_currrent_pos;
multi_uav::uav_state leader_state;
multi_uav::uav_state follower_state;
SectorMap* sector;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 声 明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void follower_currrent_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void leader_state_cb(const multi_uav::uav_state::ConstPtr& msg);
void follower_laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc,char **argv){

    sector=new SectorMap;

    ros::init(argc,argv,"follower_node");

    ros::NodeHandle nh;
    //【follower自身位置信息订阅】
    ros::Subscriber follower_currrent_pos_sub=nh.subscribe<geometry_msgs::PoseStamped>("iris_1/mavros/vision_pose/pose",100,follower_currrent_pos_cb);
    //【leader机状态订阅】
    ros::Subscriber leader_state_sub=nh.subscribe<multi_uav::uav_state>("leader_state",100,leader_state_cb);
    //【follower机状态信息发布】
    ros::Publisher follower_state_pub=nh.advertise<multi_uav::uav_state>("follower_state",100);
    //【follower速度信息发布】
    ros::Publisher follower_vel_pub=nh.advertise<mavros_msgs::PositionTarget>("/iris_1/mavros/setpoint_raw/local", 100);
    //【follower激光雷达数据订阅】
    ros::Subscriber follower_laser_sub=nh.subscribe<sensor_msgs::LaserScan>("iris_1/scan",100,follower_laser_cb);

    while (ros::ok())
    {
        ros::spinOnce();
    }
    
    delete sector;
    return 0;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void follower_currrent_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    follower_currrent_pos={msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
    //std::cout<<follower_currrent_pos[0]<<follower_currrent_pos[1]<<follower_currrent_pos[2]<<std::endl;
}
void leader_state_cb(const multi_uav::uav_state::ConstPtr& msg){
    leader_state=*msg;
    //std::cout<<leader_state.state<<std::endl;
}
void follower_laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
    sector->down_sample(msg->ranges);
    //std::cout<<laser.ranges.size()<<std::endl;
}