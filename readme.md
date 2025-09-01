main分支存储的是包含动力学模型的分支，usvSim-pureAlgorithm存储的是不包含动力学模型的分支

运行顺序：
0. 在maltab的命令行输入setenv('ROS_DOMAIN_ID','5')，建立和Ubuntu ROS2的连接
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


代码功能基本介绍：
usvSimV3.py is the version for rudder-thrust model



urdf....
my_ball_with_rudderV2.urdf is the version for rudder-thrust model.

dynamics_model...
usvDynamicsV2.py: used for rudder-thrust model

launch_usvSimV2.launch.py: a launch file used for launching rudder-thrust model relative files.

keyboard_interactionV2.py: 模拟方向盘，可以根据按键时长实现持续输入

publish_joint_angleV2.py: 可以订阅来自keyboard_interactionV2.py的消息，更新自己的joint_position, 然后发布话题/joint_states, 负责在Rviz中呈现角度的变化
