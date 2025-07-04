import rclpy
from rclpy.node import Node
from epos4_interfaces.msg import GaitParams, GaitFlag, GaitDone


sdleft_arr = [586, 550, 490,600]
sdright_arr = [608, 570, 510,620]

class GaitPrediction(Node):
    def __init__(self):
        super().__init__('gait_prediction')

        # Publishers

        self.param_pub = self.create_publisher(GaitParams, '/gait_params', 10)
        #self.flag_pub = self.create_publisher(GaitFlag, '/gait_flag', 10)

        self.create_subscription(GaitDone, '/joint_hip_left/gait_done', self.left_done_cb, 10)
        self.create_subscription(GaitDone, '/joint_hip_right/gait_done', self.right_done_cb, 10)


        self.index = 0

        self.flag = 1  # initial gait_flag

        self.left_done = True
        self.right_done = True
        self.create_timer(0.5, self.check_and_publish)


    def left_done_cb(self, msg: GaitDone):
        if msg.done:
            self.left_done = True
            self.get_logger().info("Left leg completed phase.")

    def right_done_cb(self, msg: GaitDone):
        if msg.done:
            self.right_done = True
            self.get_logger().info("Right leg completed phase.")

    def check_and_publish(self):
        if self.left_done and self.right_done:
            self.publish_predicted_params()
            self.left_done = False
            self.right_done = False


    def publish_predicted_params(self):
        #Publish gait_flag
        #self.flag *= -1
        #flag_msg = GaitFlag()
        #flag_msg.gait_flag = self.flag

        params = GaitParams()

        if self.index >= len(sdleft_arr):
            self.index = 0
            self.get_logger().info("Resetting index to 0")
           
        if self.index < len(sdleft_arr):
            params.sdleft = sdleft_arr[self.index]
            params.sdright = sdright_arr[self.index]
            self.index += 1 

        params.lsh = 177
        params.lsv = 3491
        params.sdright = 608
        params.rsh = 179
        params.rsv = 3510
        params.userheight = 1800
        params.dfratio = 0.553
        params.dbratio = 0.447            
        self.param_pub.publish(params)

        #self.flag_pub.publish(flag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaitPrediction()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
