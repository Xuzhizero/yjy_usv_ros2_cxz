main分支存储的是包含动力学模型的分支，usvSim-pureAlgorithm存储的是不包含动力学模型的分支

运行顺序：
1. 在matlab里面打开UE5
2. 运行ROS2中算法
3. 点击maltab中的run
4. 点击UE5中的play
以上顺序不能颠倒，完成以上步骤以后正式启动。

运行ROS2算法的流程：
1. 进入工作空间ros2_usv_ws
2. source ./install/setup.bash
3. ros2 launch control_planner launch_usvSim.launch.py 
4. ros2 run control_planner keyboard_interaction 

结束顺序：
1. 在matlab里面点击stop
2. 关闭UE5
3. 关闭ROS2中的节点
上面的顺序也不能乱。