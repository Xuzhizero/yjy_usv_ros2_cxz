#!/usr/bin/env python3
# Counter part for waterballControlBasisV9_2.py
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import time
from math import sqrt
from scipy import optimize

# 你的自定义包内模块按标准Python包结构导入
from control_planner import usvParam as P
from control_planner.usvDynamics import usvDynamics, usvKinematics #V2 is for rudder model
from control_planner.signalGenerator import signalGenerator
from control_planner import thrustAllocation as thrustAllocation
from control_planner.windGenerator import windGenerator
from control_planner.MFACControl import MFAC_Controller
from control_planner.waterballControlBasis import Waterball_50


from geometry_msgs.msg import PoseWithCovarianceStamped

class USVSimNode(Node):
    def __init__(self):
        super().__init__('usv_sim')
        self.get_logger().info("USVSimNode starting...")

        # Basic class init
        self.Ball = Waterball_50(self)
        self.Ball.Sub.get_lon0lat0(mode="Sim")
        self.usv_dyna = usvDynamics(self.Ball.tf_buffer,self.Ball)
        self.usv_kine = usvKinematics()
        self.wind = windGenerator()

        self.psi_reference = signalGenerator(amplitude=30 * np.pi / 180.0, frequency=0.05)
        self.force_reference = signalGenerator(amplitude=50, frequency=0.05)
        self.ref_x_reference = signalGenerator(amplitude=10, frequency=0.05)
        self.ref_y_reference = signalGenerator(amplitude=10, frequency=0.05)

        self.disturbance = signalGenerator(amplitude=5)
        self.noise = signalGenerator(amplitude=5)
        self.fix_noise = 0

        self.u, self.u_queue, self.phi_queue, self.phi_real = 0, [], [], 0
        self.psi_real_data = []
        self.time_data = []
        self.omega_data = []
        self.y_data = []
        self.y_d_data = []
        self.phi_est_data = []

        self.loop_counter = 0
        self.time_clock = 0
        self.w_c = 0

        self.rate = self.create_rate(50)
        self.running = True

    def run_main_loop(self):
        self.get_logger().info("now within main_loop=====")
        self.Ball.model = "Thrust"
        try:
            while rclpy.ok() and self.running:
                
                t = self.time_clock * P.Ts

                wind_u = self.wind.update(t)
                wind_u = [0, 0]
                
                n_noise = 0  # simulate sensor noise

                self.Ball.Sub.state_update(mode='Sim')
                self.Ball.Mode.mode_judge()
                self.Ball.Basis.publish_v_w_force(mode='Sim') 
                self.Ball.Basis.visualize_expected_path(self.Ball.e_list, self.Ball.n_list)

                start_time = self.get_clock().now().nanoseconds / 1e9
                self.Ball.Basis.visualize_actual_path(self.Ball.n, self.Ball.e)
                end_time = self.get_clock().now().nanoseconds / 1e9
                
                
                self.Ball.Basis.baselink_NED_transform(self.Ball.n, self.Ball.e, self.Ball.psi)
                
                self.Ball.Basis.state_Pub()  # topic name:/topic_state
                # self.Ball.Basis.publish_joint_state() # topic name:/joint_states
                

                self.loop_counter += 1
                self.time_clock += 1

                if self.Ball.model == "Rudder":
                    thrust_F = thrustAllocation.rpm2thrust(self.Ball.thrust_n)
                    
                    self.usv_dyna.thrust_F = thrust_F
                    self.usv_dyna.rudder2tau()  # 计算无人船基于舵角和线速度的偏航力矩
                    

                    
                else:
                    pwm_l, pwm_r = thrustAllocation.thrust_alloc(self.Ball.w_force, self.Ball.v_force, mode=2)
                    T_l, self.Ball.omega_l = thrustAllocation.thrust_cal(pwm_l, self.Ball.omega_l)
                    T_r, self.Ball.omega_r = thrustAllocation.thrust_cal(pwm_r, self.Ball.omega_r)
                    
                    d_noise_l = 0
                    d_noise_r = 0
                    self.usv_dyna.T_list = [T_l + d_noise_l + self.fix_noise, T_r + d_noise_r - self.fix_noise]                    


                self.usv_dyna.update(model=self.Ball.model)                                  
                self.Ball.e, self.Ball.n = self.usv_kine.update(self.Ball.psi, self.Ball.vx, self.Ball.vy)
                

                if self.loop_counter > 1:
                    # print("\n vx_d:%f, angular_v_d:%f,\n psi_d:%f, psi:%f" % (self.Ball.vx_d, self.Ball.angular_v_d, self.Ball.psi_d, self.Ball.psi))
                    # print(f"angular_v:{self.Ball.angular_v}")
                    # print("e:%f, n:%f, e1:%f,  n1:%f" % (self.Ball.e, self.Ball.n, self.Ball.e1, self.Ball.n1))
                    # print("mode:", self.Ball.mode)
                    # print("model:",self.Ball.model)
                    # print(f"steady_state:{self.Ball.steady_state}")
                    # print(f"wind_psi:{wind_u[0]}, wind_force:{wind_u[1]}")
                    # print(f"w_force:{self.Ball.w_force}")
                    print(f"vx:{self.Ball.vx},vy:{self.Ball.vy}")
                    # print(f"goal_pose:{self.Ball.goal_pose}")
                    # # print(f"distance:{self.Ball.distance}")
                    # # print(f"mb_status:{self.Ball.mb_status}")
                    # print(f"self.test:{self.Ball.test}")
                    # print(f"x_local:{self.Ball.x_local},y_local:{self.Ball.y_local}")
                    # print(f"x_global_NED, y_global_NED:{self.Ball.x_global_NED},{self.Ball.y_global_NED}")
                    # print(f"x_global_map, y_global_map:{self.Ball.x_global_map},{self.Ball.y_global_map}")
                    # # print(f"object_angle:{self.Ball.object_angle*180/3.14},object_distance:{self.Ball.object_distance}")
                    # print(f"rudder_angle:{self.Ball.rudder_angle},thrust_rpm:{self.Ball.thrust_n}")
                    # print(f"thrust_F:{self.usv_dyna.thrust_F},tau:{self.usv_dyna.tau}")
                    self.w_c = 0
                    self.loop_counter = 0

                work_time = end_time - start_time
                self.w_c += work_time

                self.y_data.append(self.Ball.psi)
                self.omega_data.append(self.Ball.angular_v)
                
                # rclpy.spin_once(self, timeout_sec=0.02)
                
                self.rate.sleep()
                
        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt, exiting.")
        finally:
            self.running = False
            self.get_logger().info("Exiting main loop.")

    def safe_shutdown(self):
        self.get_logger().info("Node shutting down, sending stop signal...")
        for _ in range(10):
            self.Ball.w_force = 0.0
            self.Ball.v_force = 0.0
            self.Ball.Basis.twist_msg.linear.y = float(self.Ball.v_force)
            self.Ball.Basis.twist_msg.angular.z = float(self.Ball.w_force)
            self.Ball.Basis.pub_v_w_force.publish(self.Ball.Basis.twist_msg)
            time.sleep(0.1)
        print("Close waterball object")

        # 保存数据
        from control_planner.generate_curve import save_data
        save_data(self.y_data, self.omega_data)

def main(args=None):
    rclpy.init(args=args)
    node = USVSimNode()
    try:
        node.get_logger().info("USVSimNode started successfully.")
        node.run_main_loop()
    finally:
        node.safe_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
