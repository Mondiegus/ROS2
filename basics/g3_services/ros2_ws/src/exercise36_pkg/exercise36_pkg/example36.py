import rclpy
# import the ROS2 python dependencies
from rclpy.node import Node
# import the Twist module from geometry_msgs dependencies
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs dependencies
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interfaces.msg import Age

class Example36(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('example36')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.move_turtlebot, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)
        self.age = Age()

    def move_turtlebot(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359]       

    def motion(self):
        self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
        # Read the first distance until fin a wall
        if self.laser_forward > 5:
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.5
        # Wall founded, it's time go go straight
        elif self.laser_forward <5 and self.laser_forward>=0.5:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
         
        # be careful you have to stop.    
            
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

        self.publisher_.publish(self.cmd)

    def on_shutdown(self):
        self.age.years = 2021.0
        self.age.months = 5.0
        self.age.days = 21.0
        self.get_logger().info('date this program was made : %s' % self.age.days + '/%s' % self.age.months +'/%s' % self.age.years)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    example36 = Example36()
        
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.get_default_context().on_shutdown(example36.on_shutdown)
    try:
        rclpy.spin(example36)
    except KeyboardInterrupt:
        pass
    # Explicity destroy the node
   
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()