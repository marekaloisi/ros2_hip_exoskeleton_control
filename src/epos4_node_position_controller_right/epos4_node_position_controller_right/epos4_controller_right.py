import os
import time
import rclpy
import subprocess
from rclpy.node import Node
from std_srvs.srv import Trigger
import numpy as np
from canopen_interfaces.srv import COWrite, COTargetDouble,CORead
from canopen_interfaces.msg import COData

from epos4_interfaces.msg import GaitFlag, GaitParams, GaitDone




class EPOS4Controller(Node):
    def __init__(self):
        super().__init__('epos4_controller_right')
        self.swing_right_v_arr  = [8,12,8]
        self.stance_right_v_arr  = [10,10]


        self.swing_right_theta_arr = [20,0,-20]
        self.stance_right_theta_arr = [0,0]
        self.execute = False 
        self.angle_index = 0
        self.ActPos = 0.0
        self.TargetPos = 0.0
        self.statusword = 0
        self.v_arr = []
        self.p_arr = []
        self.phase_in_progress = False

        self.swing_phase_flag = 0#-1 left foot swings ; 1 right foot swings
        self.step_count = 0

        self.done = False

        self.DF_Ratio = 0.553
        self.DB_Ratio = 0.447


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

        self.params_initialized = False
            
        self.param_sequence = [
    # Step 1 – RIGHT STANCE (keep previous RSH/RSV or init with left swing values)
    {
        "SD_Left": 559.47,
        "SD_Right": 559.47,
        "RSH": 269.60,
        "RSV": 2562.51,
        "DF_Ratio": 0.553,
        "DB_Ratio": 0.447,
        "Height": 1750.0
    },
    # Step 2 – RIGHT SWING
    {
        "SD_Left": 466.00,
        "SD_Right": 466.00,
        "RSH": 246.19,
        "RSV": 2199.92,
        "DF_Ratio": 0.553,
        "DB_Ratio": 0.447,
        "Height": 1750.0
    },
    # Step 3 – RIGHT STANCE (keep previous RSH/RSV)
    {
        "SD_Left": 508.56,
        "SD_Right": 508.56,
        "RSH": 246.19,
        "RSV": 2199.92,
        "DF_Ratio": 0.553,
        "DB_Ratio": 0.447,
        "Height": 1750.0
    },
    # Step 4 – RIGHT SWING
    {
        "SD_Left": 440.08,
        "SD_Right": 440.08,
        "RSH": 215.29,
        "RSV": 2037.16,
        "DF_Ratio": 0.553,
        "DB_Ratio": 0.447,
        "Height": 1750.0
    },
]




               
        self.param_step_index = 0

        
        # Init clients
        self.start_nmt_client = self.create_client(Trigger, '/joint_hip_right/nmt_start_node')
        self.init_client = self.create_client(Trigger, '/joint_hip_right/init')
        self.ppm_client = self.create_client(Trigger, '/joint_hip_right/position_mode')
        self.sdo_write_client = self.create_client(COWrite, '/joint_hip_right/sdo_write')
        self.sdo_read_client = self.create_client(CORead, '/joint_hip_right/sdo_read')
        self.target_client = self.create_client(COTargetDouble, '/joint_hip_right/target')
        self.reset_nmt_client = self.create_client(Trigger, '/joint_hip_right/nmt_reset_node')
        self.create_subscription(COData, '/joint_hip_right/rpdo', self.rpdo_cb, 10)
        self.log_file = open('/home/marek/right_rpdo_actual_position.csv', 'a')

        #self.get_logger().info('Starting controller: running start.sh...')
        #subprocess.run(['bash', './start_left.sh'], check=True)
        #self.create_subscription(GaitFlag, '/gait_flag', self.gait_flag_cb, 10)
        #self.create_subscription(GaitParams, '/gait_params', self.gait_params_cb, 10)
        self.phase_done_pub = self.create_publisher(GaitDone, '/joint_hip_right/gait_done', 10) 

        self.call_service_sync(self.start_nmt_client, Trigger.Request(), "NMT Start Node")
        self.call_service_sync(self.init_client, Trigger.Request(), "Init")
        self.call_service_sync(self.ppm_client, Trigger.Request(), "Position Mode")

        
        # pos = float(input("Enter target angle [inc]: "))
        # vel = int(input("Enter target velocity [rpm]: "))
        # self.set_velocity_sdo(vel)
        # self.set_angle(pos)
        
       
        self.create_timer(0.1, self.command_loop)  # every 100ms

    def gait_flag_cb(self, msg: GaitFlag):
        if msg.gait_flag != self.swing_phase_flag:
            self.swing_phase_flag = msg.gait_flag
            self.done = False
            self.execute = True
            self.phase_in_progress = False  # prepare for new phase
            self.get_logger().info(f"New gait flag received: {self.swing_phase_flag}, starting motion.")
        else:
            self.get_logger().info(f"Ignored duplicate gait flag: {msg.gait_flag}") 
            self.done = False  # Reset done flag to allow re-execution of the same phase

    def rpdo_cb(self, msg: COData):
        if msg.index == 0x6064 and msg.subindex == 0x00:
            try:
                # Handle both int and bytes formats
                if isinstance(msg.data, bytes):
                    val = int.from_bytes(msg.data, byteorder='little', signed=True)
                else:
                    val = int(msg.data)
                    if val >= 2**31:
                        val -= 2**32  # convert unsigned to signed

                actual_pos = val / 1000.0  # motor increments to encoder counts
                angle = actual_pos / (120 / 114.4)  # encoder counts to degrees

                timestamp = self.get_clock().now().to_msg()
                if hasattr(self, 'log_file') and self.log_file:
                    self.log_file.write(f"{timestamp.sec}.{timestamp.nanosec},{angle}\n")

            except Exception as e:
                self.get_logger().warn(f"Failed to decode RPDO data: {e}")

    def destroy_node(self):
        if hasattr(self, 'log_file') and self.log_file:
            self.log_file.close()
        super().destroy_node()


    # def gait_params_cb(self, msg: GaitParams):
       
    #  # Store values if needed
    #     self.SD_Left = msg.sdleft
    #     self.LSH = msg.lsh
    #     self.LSV = msg.lsv

    #     self.SD_Right = msg.sdright
    #     self.RSH = msg.rsh
    #     self.RSV = msg.rsv

    #     self.DF_Ratio = msg.dfratio
    #     self.DB_Ratio = msg.dbratio

    #     self.Height = msg.userheight #MM
    #     self.Shank_Length = self.Height * 0.22 #MM
    #     self.Thigh_Length = self.Height * 0.24 #MM
    #     self.LL = self.Shank_Length + self.Thigh_Length #MM

    #     self.params_initialized = True



       

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
        params = self.param_sequence[self.param_step_index % len(self.param_sequence)]

        self.SD_Left = params["SD_Left"]
        self.SD_Right = params["SD_Right"]
        self.RSH = params["RSH"]
        self.RSV = params["RSV"]
        self.DF_Ratio = params["DF_Ratio"]
        self.DB_Ratio = params["DB_Ratio"]
        self.Height = params["Height"]

        self.Thigh_Length = self.Height * 0.24
        self.Shank_Length = self.Height * 0.22
        self.LL = self.Thigh_Length + self.Shank_Length

        self.params_initialized = True
        if self.LL == 0.0 or not self.params_initialized:
            self.get_logger().warn("LL is zero – skipping angle computation.")
            return

        if self.swing_phase_flag == 1:
        
            val = (self.Thigh_Length**2 + (self.LL - self.RSH)**2 - self.Shank_Length**2) / (2 * self.Thigh_Length * (self.LL - self.RSH))
            #val = np.clip(val, -1.0, 1.0)

            theta0 = -np.degrees(np.arcsin((self.SD_Left * self.DB_Ratio) / self.LL))
            theta1 = np.degrees(np.arccos(val))
            theta2 = np.degrees(np.arcsin((self.SD_Right * self.DF_Ratio) / self.LL))
            self.swing_right_theta_arr = [theta0, theta1, theta2]

            v_calc = int((self.RSV) / ((self.LL + 100) * (2 * np.pi)) * 60)
            v_safe = min(v_calc, 10)
            self.swing_right_v_arr = [10,10,10]

        if self.swing_phase_flag == -1:
            theta0 = np.degrees(np.arcsin((self.SD_Right * self.DF_Ratio) / self.LL))
            theta1 = -np.degrees(np.arcsin((self.SD_Left * self.DB_Ratio) / self.LL))

            self.stance_right_theta_arr = [theta0, theta1]
            self.stance_right_v_arr = [10,10,10]

    def get_velocity_MD(self):
        if self.swing_phase_flag == 1:
            self.swing_right_v_arr[1] = int((self.RSV)/((self.Shank_Length + self.Thigh_Length + 100) * (2*np.pi)) * 60)      

    def set_velocity_sdo(self, velocity): 
        req = COWrite.Request()
        req.index = 0x6081
        req.subindex = 0x00
        req.data = velocity
        future = self.sdo_write_client.call_async(req)
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
        req.target = angle * (120/114.4)
        future = self.target_client.call_async(req)
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
            self.param_step_index += 1
           

    def command_loop(self):
        if not self.execute:
            if self.param_step_index < len(self.param_sequence):
                self.swing_phase_flag = -1 if self.param_step_index % 2 == 0 else 1
                self.done = False
                self.execute = True
                self.phase_in_progress = False
                self.get_logger().info(f"[AUTO] Starting phase {self.param_step_index}, flag: {self.swing_phase_flag}")
            else:
                return  # Finished all predefined steps

        if not self.phase_in_progress:
            self.get_logger().info("Starting new motion phase")
            self.get_theta_HL()

            if self.swing_phase_flag == -1:
                self.v_arr = self.stance_right_v_arr
                self.p_arr = self.stance_right_theta_arr
            elif self.swing_phase_flag == 1:
                self.v_arr = self.swing_right_v_arr
                self.p_arr = self.swing_right_theta_arr
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
