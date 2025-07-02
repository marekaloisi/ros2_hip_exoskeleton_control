import os
import time
import rclpy
import subprocess
from rclpy.node import Node
from std_srvs.srv import Trigger
import numpy as np
from canopen_interfaces.srv import COWrite, COTargetDouble,CORead
from epos4_interfaces.msg import GaitFlag, GaitParams, GaitDone






class EPOS4Controller(Node):
    def __init__(self):
        super().__init__('epos4_controller_left')
        self.swing_left_v_arr  = [8,12,8]
        self.stance_left_v_arr  = [10,10]


        self.swing__left_theta_arr = [0,0,0]
        self.stance_left_theta_arr = [0,0]
        self.execute = False 
        self.angle_index = 0
        self.ActPos = 0.0
        self.TargetPos = 0.0
        self.statusword = 0
        self.v_arr = []
        self.p_arr = []
        self.phase_in_progress = False



        self.swing_phase_flag = 0 # -1 left foot swings ; 1 right foot swings
        self.prev_gait_flag = 1
        self.done = False
        self.step_count = 0

        self.DF_Ratio = 0.0
        self.DB_Ratio = 0.0


        self.SD_Left = 0 #mmm  
        self.SD_Right = 0 #mmm

        self.RSH = 0 #MM
        self.RSV = 0 #RPM

        self.LSH = 0 #MM
        self.LSV = 0 #RPM

        
        self.Height = 0 #MM
        self.Shank_Length = 0.0
        self.Thigh_Length = 0.0
        self.LL = 0.0

        
        # Init clients
        self.start_nmt_client = self.create_client(Trigger, '/joint_hip_left/nmt_start_node')
        self.init_client = self.create_client(Trigger, '/joint_hip_left/init')
        self.ppm_client = self.create_client(Trigger, '/joint_hip_left/position_mode')
        self.sdo_write_client = self.create_client(COWrite, '/joint_hip_left/sdo_write')
        self.sdo_read_client = self.create_client(CORead, '/joint_hip_left/sdo_read')
        self.target_client = self.create_client(COTargetDouble, '/joint_hip_left/target')
        self.reset_nmt_client = self.create_client(Trigger, '/joint_hip_left/nmt_reset_node')

        self.create_subscription(GaitFlag, '/gait_flag', self.gait_flag_cb, 10)
        self.create_subscription(GaitParams, '/gait_params', self.gait_params_cb, 10)
        self.phase_done_pub = self.create_publisher(GaitDone, '/joint_hip_left/gait_done', 10) 

        #self.get_logger().info('Starting controller: running start.sh...')
        #subprocess.run(['bash', './start_left.sh'], check=True)

        self.call_service_sync(self.start_nmt_client, Trigger.Request(), "NMT Start Node")
        self.call_service_sync(self.init_client, Trigger.Request(), "Init")
        self.call_service_sync(self.ppm_client, Trigger.Request(), "Position Mode")

        
        # pos = float(input("Enter target angle [inc]: "))
        # vel = int(input("Enter target velocity [rpm]: "))
        # self.set_velocity_sdo(vel)
        # self.set_angle(pos)
        
    
        self.create_timer(0.05, self.command_loop)  # every 100ms


    def gait_flag_cb(self, msg: GaitFlag):
        if msg.gait_flag != self.swing_phase_flag:
            self.swing_phase_flag = msg.gait_flag
            self.done = False
            self.execute = True
            self.phase_in_progress = False  # prepare for new phase
            self.get_logger().info(f"New gait flag received: {self.swing_phase_flag}, starting motion.")
        else:
            self.get_logger().info(f"Ignored duplicate gait flag: {msg.gait_flag}")




    def gait_params_cb(self, msg: GaitParams):
       
    # Store values if needed
        self.SD_Left = msg.sdleft
        self.LSH = msg.lsh
        self.LSV = msg.lsv

        self.SD_Right = msg.sdright
        self.RSH = msg.rsh
        self.RSV = msg.rsv

        self.DF_Ratio = msg.dfratio
        self.DB_Ratio = msg.dbratio

        self.Height = msg.userheight #MM
        self.Shank_Length = self.Height * 0.22 #MM
        self.Thigh_Length = self.Height * 0.24 #MM
        self.LL = self.Shank_Length + self.Thigh_Length #MM


    def call_service_sync(self, client, request, service_name):
        while not client.wait_for_service(timeout_sec=5):
            self.get_logger().info(f'{service_name} service not available, waiting...')

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'{service_name} OK')
        else:
            self.get_logger().error(f'{service_name} FAILED: {future.result().message}')

    def get_theta_HL(self):
        if self.swing_phase_flag == -1:
            self.swing__left_theta_arr[0] = -np.degrees(np.arcsin((self.SD_Right * self.DB_Ratio)/ self.LL))
            self.swing__left_theta_arr[1] = np.degrees(np.arccos((self.Thigh_Length**2 + (self.LL-self.LSH)**2 - self.Shank_Length**2 )/ (2 * self.Thigh_Length * (self.LL-self.LSH))))
            self.swing__left_theta_arr[2] = np.degrees(np.arcsin((self.SD_Left * self.DF_Ratio)/ self.LL))
        if self.swing_phase_flag == 1:
            self.stance_left_theta_arr[0] = np.degrees(np.arcsin((self.SD_Left * self.DF_Ratio)/ self.LL))
            self.stance_left_theta_arr[1] = -np.degrees(np.arcsin((self.SD_Right * self.DB_Ratio)/ self.LL))
             

    def set_velocity_sdo(self, velocity): 
        req = COWrite.Request()
        req.index = 0x6081
        req.subindex = 0x00
        req.data = velocity
        self.sdo_write_client.call_async(req)
        self.get_logger().info(f"Velocity command sent: {velocity}")
       
    def send_controlword_sdo(self, state): 
        req = COWrite.Request()
        req.index = 0x6040
        req.subindex = 0x00
        req.data = state
        self.sdo_write_client.call_async(req)
        self.get_logger().info(f"Controlword command sent: {hex(state)}")

    def set_angle(self, angle):
        req = COTargetDouble.Request()
        req.target = angle * (-120/114.4)
        self.target_client.call_async(req)
        self.get_logger().info(f"Angle command sent: {angle}")


    def handle_actual_pos(self, future):
        if not future.result().success:
            self.get_logger().error("Failed to read actual position.")
            return

        self.ActPos = future.result().data

        req_target = CORead.Request()
        req_target.index = 0x607A
        req_target.subindex = 0x00
        future_target = self.sdo_read_client.call_async(req_target)
        future_target.add_done_callback(self.handle_target_pos)


    def handle_target_pos(self, future):
        if not future.result().success:
            self.get_logger().error("Failed to read target position.")
            return

        self.TargetPos = future.result().data
        diff = abs(self.TargetPos - self.ActPos)

        if diff < 500.0 and self.angle_index < len(self.p_arr):
            self.set_velocity_sdo(self.v_arr[self.angle_index])
            self.set_angle(self.p_arr[self.angle_index])
            self.get_logger().info(f"Sent angle {self.p_arr[self.angle_index]} at index {self.angle_index}")
            self.angle_index += 1

    def handle_statusword_check(self, future):
        if not future.result().success:
            self.get_logger().error("Failed to read statusword.")
            return

        status = future.result().data
        if (status & 0x0400) and abs(self.TargetPos - self.ActPos) < 500.0:
            self.get_logger().info("Target reached.")
            self.angle_index = 0
            self.step_count += 1
            self.done = True
            self.phase_in_progress = False
            self.execute = False

            msg = GaitDone()
            msg.done = True
            self.phase_done_pub.publish(msg)


    def command_loop(self):
        if not self.execute:
            return

        if not self.phase_in_progress:
            self.get_logger().info("Starting new motion phase")
            self.get_theta_HL()

            if self.swing_phase_flag == -1:
                self.v_arr = self.swing_left_v_arr
                self.p_arr = self.swing__left_theta_arr
            elif self.swing_phase_flag == 1:
                self.v_arr = self.stance_left_v_arr
                self.p_arr = self.stance_left_theta_arr
            else:
                self.get_logger().error("Invalid gait flag")
                return

            self.angle_index = 0
            self.phase_in_progress = True

            self.set_velocity_sdo(self.v_arr[self.angle_index])
            self.set_angle(self.p_arr[self.angle_index])
            self.get_logger().info(f"Sent angle {self.p_arr[self.angle_index]} at index {self.angle_index}")
            self.angle_index += 1
            return

        # Done sending all targets
        if self.angle_index >= len(self.p_arr):
            req = CORead.Request()
            req.index = 0x6041
            req.subindex = 0x00
            future = self.sdo_read_client.call_async(req)
            future.add_done_callback(self.handle_statusword_check)
            return

        # Still mid-phase, check position before sending next angle
        req_act = CORead.Request()
        req_act.index = 0x6064
        req_act.subindex = 0x00
        future_act = self.sdo_read_client.call_async(req_act)
        future_act.add_done_callback(self.handle_actual_pos)

                
def main(args=None):
    rclpy.init(args=args)
    node = EPOS4Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
