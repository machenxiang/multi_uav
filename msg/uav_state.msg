#***************************************************************************************************************************
# * Function: uav_state msg 
# *
# * Three tpye state:   UAV_DANGER = DANGER   UAV_HOVER  = HOVER UAV_SAFE   = SAFE UAV_MOVE   = MOVE
# *
# * Node name: run_line_node
# * 
# * Author: Ma Chenxiang
# *
# * Update Time: 2020.11.28
# *
# * Email: mcx1243858461@gmail.com
#***************************************************************************************************************************/
std_msgs/Header header

string state
geometry_msgs/Pose pose

