每个命令行都得开新的窗口
docker exec -it 65abafec5dc5 bash

catkin_make
source devel/setup.bash

docker exec 65abafec5dc5 bash -c "pkill -9 -f ros; sleep 2; echo '进程已清理'"

docker exec -it 65abafec5dc5 bash -c "cd /home/developer/ros_ws/ddo-topo-mppi && source devel/setup.bash && roslaunch /home/developer/ros_ws/ddo-topo-mppi/src/planner/plan_manage/launch/topo_mppi_fastplanner_map.launch"

docker exec -e DISPLAY=$DISPLAY 65abafec5dc5 bash -c "source /home/developer/ros_ws/ddo-topo-mppi/devel/setup.bash && rviz -d /home/developer/ros_ws/ddo-topo-mppi/src/planner/plan_manage/launch/fastplanner_test.rviz"
