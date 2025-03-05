import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from mavros_msgs.msg import State
from interfaces.srv import SetFloat
import time
import threading

class DroneMovement(Node):
    def __init__(self):
        super().__init__('drone_movement')

        # Subscribe to pot topic
        self.debug_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.hor_vel_sub = self.create_subscription(Twist, '/right/cmd_vel_hor', self.hor_vel_callback, 10)
        self.yaw_vel_sub = self.create_subscription(Twist, '/left/cmd_vel_hor', self.yaw_vel_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        # self.vert_vel_sub = self.create_subscription(Float32, '/left/cmd_vel_vert', self.vert_vel_callback, 10)
        self.vert_srv = self.create_service(SetFloat, '/vert_vel', self.vert_vel_srv_callback)
        self.lock_srv = self.create_service(SetBool, '/lock_axis', self.lock_axis_callback)
        self.lock_zero_srv = self.create_service(SetBool, '/lock_zero', self.lock_zero_callback)
        self.land_srv = self.create_service(SetBool, '/landing', self.landing_callback)
        self.linear_x = 0
        self.linear_y = 0
        self.linear_z = 0
        self.angular_z = 0
        self.zero_lock = False
        self.axis_lock = False
        self.drone_state = False
        self.drone_landing = False

    def publish_vel(self):
        msg = Twist()
        if self.zero_lock:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("zero lock")
        else:
            msg.linear.x = float(self.linear_x)
            msg.linear.y = float(self.linear_y)
            msg.linear.z = float(self.linear_z)
            msg.angular.z = float(self.angular_z)
        self.debug_pub.publish(msg)
        if not self.drone_state or self.drone_landing:
            # self.get_logger().info("Drone not activated")
            return
        
        self.pub.publish(msg)
        return
    def state_callback(self, msg):
        if msg.system_status != 4:
            self.drone_state = False
            self.drone_landing = False
        else:
            self.drone_state = True
    
    def landing_callback(self, request, response):
        self.drone_landing = request.data
        response = SetBool.Response()
        response.success = True
        return response

    def hor_vel_callback(self, data):
        if self.axis_lock:
            if abs(data.linear.x) > abs(data.linear.y):
                self.linear_y = -data.linear.x
                self.linear_x = 0
            else:
                self.linear_x = data.linear.y
                self.linear_y = 0
            self.angular_z = 0
        else:
            self.linear_x = data.linear.y
            self.linear_y = -data.linear.x
        self.publish_vel()


    def yaw_vel_callback(self, data):
        if self.axis_lock:
            self.angular_z = 0
        else:
            self.angular_z = data.linear.y
        self.publish_vel()

    def vert_vel_srv_callback(self, request, response):
        self.linear_z = request.data
        response.success = True
        self.publish_vel()
        return response
    
    def lock_zero_callback(self, request, response):
        self.zero_lock = request.data
        response.success = True
        return response
    
    def lock_axis_callback(self, request, response):
        self.axis_lock = request.data
        response.success = True
        return response



def main(args=None):
    rclpy.init(args=args)
    drone_movement = DroneMovement()
    rclpy.spin(drone_movement)
    drone_movement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
