import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import ReliabilityPolicy, QoSProfile


class Main_exec(Node):

    def __init__(self):
        # Here we have the class constructor
        # call super() in the constructor in order to initialize the Node object
        # the parameter we pass is the node name
        super().__init__('simple_publisher')
        # create the publisher object
        # in this case the publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # define the timer period for 0.5 seconds
        self.subscriber= self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) #is the most used to read LaserScan data and some sensor data too.
        # prevent unused variable warning
        self.subscriber
        self.linear_x = 1.0
        self.angular_z = 1.0
        # define the variable to save the received info
        # create a timer sending two parameters:
        # - the duration between 2 callbacks (0.5 seeconds)
        # - the timer function (timer_callback)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # Here we have the callback method
        # create a Twist message
        msg = Twist()
        # define the linear x-axis velocity of /cmd_vel topic paramater to 0.5
        msg.linear.x = self.linear_x
        # define the angular z-axis velocity of /cmd_vel topic paramater to 0.5
        msg.angular.z = self.angular_z
        # Publish the message to the topic
        self.publisher_.publish(msg)
            
    def listener_callback(self, msg):   
        # save the received data
        if(not math.isinf(msg.ranges[0]) and msg.ranges[0]<5):
            self.angular_z = 0.0
            self.linear_x = 0.2
            if(msg.ranges[0]<0.5):
                self.linear_x = 0.0
        
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    main_exec = Main_exec()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(main_exec)
    # Explicity destroy the node
    main_exec.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()