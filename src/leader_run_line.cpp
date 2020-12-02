/***************************************************************************************************************************
 * Function: leader_uav run line
 *
 * Node name: run_line_node
 * 
 * Author: Ma Chenxiang
 *
 * Update Time: 2020.11.28
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
#include <vector>
#include "multi_uav/uav_state.h"
#include <string>
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>宏 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define PI 3.141592653589793238327950
#define TARGET 50

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量 声 明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Eigen::Vector3d leader_current_pos;
geometry_msgs::PoseStamped target_pos;
mavros_msgs::PositionTarget leader_vel;
multi_uav::uav_state leader_state;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 声 明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void leader_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void time_cb(const ros::TimerEvent& e);
void set_leader_state_msg(const std::string& str,Eigen::Vector3d pos,multi_uav::uav_state& leader_state);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc,char **argv){
    target_pos.pose.position.x=TARGET;
    target_pos.pose.position.y=0;
    target_pos.pose.position.z=2.5;

    ros::init(argc, argv, "leader_node");
	ros::NodeHandle nh;
    //建议控制频率10~20Hz,控制频率取决于控制形式，若控制方式为速度或者加速度应适当提高频率
    ros::Rate rate(30);
    //【leader机位置订阅】
    ros::Subscriber leader_state_sub=nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/vision_pose/pose",100,leader_pos_cb);
    //【leader机速度发布】
    ros::Publisher  leader_vel_pub=nh.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 100);
    //【定时器用于状态发布】
    ros::Timer timer = nh.createTimer(ros::Duration(5.0),time_cb);
    //【leader自身状态发布】
    ros::Publisher leader_state_pub=nh.advertise<multi_uav::uav_state>("leader_state",100);
    

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 循 环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


    while(ros::ok()){
        ros::spinOnce();
        set_leader_state_msg("MOVE",leader_current_pos,leader_state);
        if(abs(leader_current_pos[0]-target_pos.pose.position.x)<1&&abs(leader_current_pos[1]-target_pos.pose.position.y)<1){
            ROS_INFO("arrive at the target");
            break;
        }

        leader_vel.coordinate_frame=1;
		leader_vel.type_mask = 1 + 2 + 4 + /*8 + 16+ 32 */ + 64 + 128 + 256 + 512 + /*1024*/ + 2048;
		leader_vel.velocity.x=cos(0.7854);
		leader_vel.velocity.y=sin(0.7854);
        leader_vel.yaw=atan2(leader_vel.velocity.y,leader_vel.velocity.x);
        leader_vel_pub.publish(leader_vel);
        leader_state_pub.publish(leader_state);

    }
    

    ROS_INFO("change mode to hover");
    while (ros::ok())
    {
        ros::spinOnce();
        set_leader_state_msg("HOVER",leader_current_pos,leader_state);
        leader_vel.coordinate_frame=1;
		leader_vel.type_mask = 1 + 2 + 4/* + 8 + 16 + 32 */+ 64 + 128 + 256 + 512 + 1024 + 2048;
		leader_vel.velocity.x=0;
		leader_vel.velocity.y=0;
        leader_vel_pub.publish(leader_vel);
        leader_state_pub.publish(leader_state);
    }
    
    return 0;

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void leader_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    leader_current_pos={msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    //std::cout<<"x:"<<leader_pos[0]<<" y:"<<leader_pos[1]<<" z"<<leader_pos[2]<<std::endl;

}

void time_cb(const ros::TimerEvent& e)
{
    ROS_INFO("leader state [%s],leader pose[%f,%f,%f]",leader_state.state.c_str(),leader_state.pose.position.x,leader_state.pose.position.y,leader_state.pose.position.z);
}

void set_leader_state_msg(const std::string& str,Eigen::Vector3d pos,multi_uav::uav_state& leader_state){
    leader_state.state=str;
    leader_state.pose.position.x=pos[0];
    leader_state.pose.position.y=pos[1];
    leader_state.pose.position.z=pos[2];
}