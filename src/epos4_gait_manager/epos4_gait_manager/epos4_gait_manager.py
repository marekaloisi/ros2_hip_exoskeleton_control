import rclpy
from rclpy.node import Node
from epos4_interfaces.msg import GaitParams, GaitFlag, GaitDone



class GaitManager(Node):
    def __init__(self):
        super().__init__('gait_manager')

        # Publishers

        #self.param_pub = self.create_publisher(GaitParams, '/gait_params', 10)
        self.flag_pub = self.create_publisher(GaitFlag, '/gait_flag', 10)

        self.create_subscription(GaitDone, '/joint_hip_left/gait_done', self.left_done_cb, 10)
        self.create_subscription(GaitDone, '/joint_hip_right/gait_done', self.right_done_cb, 10)




        self.flag = 1  # initial gait_flag

        self.left_done = True
        self.right_done = True
        self.create_timer(0.01, self.check_and_publish)


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
            self.publish_flag()
            self.left_done = False
            self.right_done = False


    def publish_flag(self):
        # Publish gait_flag
        self.flag *= -1
        flag_msg = GaitFlag()
        flag_msg.gait_flag = self.flag
        
        self.flag_pub.publish(flag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaitManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
