# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the MyCustomServiceMessages module from custom_interfaces_service interface
from custom_interfaces.srv import MyCustomServiceMessages
# import the ROS2 python client libraries
import rclpy
from rclpy.node import Node

class Service(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('movement_server')
        # create the service server object
        # defines the type, name and callback function
        self.srv = self.create_service(MyCustomServiceMessages, 'movement', self.CustomService_callback)
        # create the publisher object
        # in this case the publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def CustomService_callback(self, request, response):
        # The callback function recives the self class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as response

        # create a Twist message
        msg = Twist()
        
        if request.move == "Turn Right":
            # define the linear x-axis velocity of /cmd_vel topic paramater to 0.1
            msg.linear.x = 0.1
            # define the angular z-axis velocity of /cmd_vel topic paramater to -0.5 to turn right
            msg.angular.z = -0.5
            # Publish the message to the topic
            self.publisher_.publish(msg)
            # print a pretty message
            self.get_logger().info('Turning to right direction!!')
            # response state
            response.success = True
        elif request.move == "Turn Left":
            # define the linear x-axis velocity of /cmd_vel topic paramater to 0.1
            msg.linear.x = 0.1
            # define the angular z-axis velocity of /cmd_vel topic paramater to 0.5 to turn left
            msg.angular.z = 0.5
            # Publish the message to the topic
            self.publisher_.publish(msg)
            # print a pretty message
            self.get_logger().info('Turning to left direction!!')
            # response state
            response.success = True
        elif request.move == "Stop":
            # define the linear x-axis velocity of /cmd_vel topic paramater to 0
            msg.linear.x = 0.0
            # define the angular z-axis velocity of /cmd_vel topic paramater to 0
            msg.angular.z = 0.0
            # Publish the message to the topic
            self.publisher_.publish(msg)
            # print a pretty message
            self.get_logger().info('Stop there!!')
            # response state
            response.success = True
        else:
            # response state
            response.success = False
        
        # return the response parameter
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()