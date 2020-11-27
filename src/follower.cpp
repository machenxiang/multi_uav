/***************************************************************************************************************************
 * Function: follower_uav track the leader uav and avoid the obstacle 
 * 
 * Node name: follower_node
 *
 * Author: Ma Chenxiang
 *
 * Update Time: 2020.11.25
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

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>宏 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define PI 3.141592653589793238327950
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量 声 明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Eigen::Vector3d leader_current_pos;
Eigen::Vector3d follower_current_pos;
//std::queue<geometry_msgs::PoseStamped>target_pos;
geometry_msgs::PoseStamped target_pos;

mavros_msgs::PositionTarget leader_vel;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 声 明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void leader_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void follower_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
float compute_angel(const geometry_msgs::PoseStamped& taget,const Eigen::Vector3d local);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc,char **argv){

    target_pos.pose.position.x=6;
    target_pos.pose.position.y=6;
    target_pos.pose.position.z=2.5;

    ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;
    //建议控制频率10~20Hz,控制频率取决于控制形式，若控制方式为速度或者加速度应适当提高频率
    ros::Rate rate(20);

    ros::Subscriber leader_state_sub=nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/vision_pose/pose",100,leader_pos_cb);
    ros::Subscriber follower_state_sub=nh.subscribe<geometry_msgs::PoseStamped>("/iris_1/mavros/vision_pose/pose",100,follower_pos_cb);
    ros::Publisher  leader_vel_pub=nh.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 100);
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 循 环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok()){
        ros::spinOnce();
        if(abs(target_pos.pose.position.x-leader_current_pos[0])<1&&abs(target_pos.pose.position.y-leader_current_pos[1])<1){
            break;
        }
        leader_vel.coordinate_frame=1;
		leader_vel.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + /*1024*/ + 2048;
		leader_vel.velocity.x = cos(compute_angel(target_pos,leader_current_pos));
		leader_vel.velocity.y = sin(compute_angel(target_pos,leader_current_pos));
        leader_vel.yaw=atan2(leader_vel.velocity.y ,leader_vel.velocity.x);
        leader_vel_pub.publish(leader_vel);
    }
    while(ros::ok()){
        ros::spinOnce();
        if(abs(target_pos.pose.position.x-leader_current_pos[0])<1&&abs(target_pos.pose.position.y-leader_current_pos[1])<1){
            break;
        }
        leader_vel.coordinate_frame=1;
		leader_vel.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + /*1024*/ + 2048;
		leader_vel.velocity.x = cos(compute_angel(target_pos,leader_current_pos));
		leader_vel.velocity.y = sin(compute_angel(target_pos,leader_current_pos));
        leader_vel.yaw=atan2(leader_vel.velocity.y ,leader_vel.velocity.x);
        leader_vel_pub.publish(leader_vel);
    }

    return 0;

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void leader_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    leader_current_pos={msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    //std::cout<<"x:"<<leader_pos[0]<<" y:"<<leader_pos[1]<<" z"<<leader_pos[2]<<std::endl;

}

void follower_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    follower_current_pos={msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    //std::cout<<"x:"<<follower_pos[0]<<" y:"<<follower_pos[1]<<" z"<<follower_pos[2]<<std::endl;
}

float compute_angel(const geometry_msgs::PoseStamped& target,const Eigen::Vector3d local){
    float angel=atan2((target.pose.position.y-local[1]),(target.pose.position.x-local[0]));
    return angel;
}