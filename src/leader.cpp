/***************************************************************************************************************************
 * Function: leader_uav run a square
 *
 * Node name: leader_node
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
#include<vector>
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>宏 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define PI 3.141592653589793238327950
#define TARGET_NUM 5
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量 声 明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Eigen::Vector3d leader_current_pos;
std::vector<geometry_msgs::PoseStamped>target_pos;
mavros_msgs::PositionTarget leader_vel;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 声 明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void leader_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
float compute_angel(const geometry_msgs::PoseStamped& taget,const Eigen::Vector3d local);
void init_path();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc,char **argv){
    init_path();
    ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;
    //建议控制频率10~20Hz,控制频率取决于控制形式，若控制方式为速度或者加速度应适当提高频率
    ros::Rate rate(30);

    ros::Subscriber leader_state_sub=nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/vision_pose/pose",100,leader_pos_cb);

    ros::Publisher  leader_vel_pub=nh.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 100);
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 循 环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int i=0;
HOME:
        ROS_INFO("go to target[%d]",i);
        std::cout<<"the x axis difference between loacal pose and path"<<i<<" "<<abs(target_pos[i].pose.position.x-leader_current_pos[0])<<std::endl;
        std::cout<<"the y axis difference between loacal pose and path"<<i<<" "<<abs(target_pos[i].pose.position.y-leader_current_pos[1])<<std::endl;
        while(ros::ok()){
            ros::spinOnce();
            if(abs(target_pos[i].pose.position.x-leader_current_pos[0])<1 && abs(target_pos[i].pose.position.y-leader_current_pos[1])<1){
                
                i++;
                ROS_INFO("arrive at point[%d]",i);
                goto HOME;
                }

            leader_vel.coordinate_frame=1;
		    leader_vel.type_mask = 1 + 2 + 4 + /*8 + 16+ 32 */ + 64 + 128 + 256 + 512 + /*1024*/ + 2048;
		    leader_vel.velocity.x=cos(compute_angel(target_pos[i],leader_current_pos));
		    leader_vel.velocity.y=sin(compute_angel(target_pos[i],leader_current_pos));
            leader_vel.yaw=atan2(leader_vel.velocity.y,leader_vel.velocity.x);
            leader_vel_pub.publish(leader_vel);

        }
    

    ROS_INFO("change mode to hover");
    while (ros::ok())
    {
        leader_vel.coordinate_frame=1;
		leader_vel.type_mask = 1 + 2 + 4/* + 8 + 16*/ + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
		leader_vel.velocity.x=0;
		leader_vel.velocity.y=0;
        leader_vel.yaw=0;
        leader_vel_pub.publish(leader_vel);
    }
    
    return 0;

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void leader_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    leader_current_pos={msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    //std::cout<<"x:"<<leader_pos[0]<<" y:"<<leader_pos[1]<<" z"<<leader_pos[2]<<std::endl;

}
//矩形航点发布，先飞到[0,0,2.5]然后顺时针飞一个边长10的矩形然后悬停在[0,0,2.5],leader不考虑yaw角，只需要跑航点
void init_path(){
    geometry_msgs::PoseStamped *target0,*target1,*target2,*target3,*target4;

    target0=new geometry_msgs::PoseStamped;
    target1=new geometry_msgs::PoseStamped;
    target2=new geometry_msgs::PoseStamped;
    target3=new geometry_msgs::PoseStamped;
    target4=new geometry_msgs::PoseStamped;

    (*target0).pose.position.x=0;
    (*target0).pose.position.y=0;
    (*target0).pose.position.z=2.5;
    target_pos.push_back(*target0);

    (*target1).pose.position.x=0;
    (*target1).pose.position.y=10;
    (*target1).pose.position.z=2.5;
    target_pos.push_back(*target1);

    (*target2).pose.position.x=10;
    (*target2).pose.position.y=10;
    (*target2).pose.position.z=2.5;
    target_pos.push_back(*target2);

    (*target3).pose.position.x=10;
    (*target3).pose.position.y=0;
    (*target3).pose.position.z=2.5;
    target_pos.push_back(*target3);

    (*target4).pose.position.x=0;
    (*target4).pose.position.y=0;
    (*target4).pose.position.z=2.5;
    target_pos.push_back(*target4);

    delete target0,target1,target2,target3,target4;

}

float compute_angel(const geometry_msgs::PoseStamped& target,const Eigen::Vector3d local){
    float angel=atan2((target.pose.position.y-local[1]),(target.pose.position.x-local[0]));
    return angel;
}
