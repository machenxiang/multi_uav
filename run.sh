######################################
# Project: Multi_uav

#  Author:Ma Chenxiang,Wang Yuan

#    Date:2020.11.24
######################################
gnome-terminal --window -e 'bash -c;exec bash' \
--tab -e 'bash -c "sleep 2; killall gzserver;roslaunch px4 multi_vehicle.launch;exec bash"' \
--tab -e 'bash -c "sleep 20; echo "get_local_pose"  ;cd ~/XTDrone/sensing/pose_ground_truth;python get_local_pose.py iris 2;exec bash"' \
--tab -e 'bash -c "sleep 20; cd ~/XTDrone/communication;bash multi_vehicle_communication.sh;exec bash"' \
--tab -e 'bash -c "sleep 20; cd ~;./QGroundControl.AppImage ;exec bash"' \

