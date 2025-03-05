import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import SetBool
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from controller.madgwick_py import madgwickahrs, quaternion
from controller.helpers.joystick import Joystick
from std_msgs.msg import Float32MultiArray
import time

# maximum velocity in m/s when at full throttle 
MAX_VELOCITY = 0.1

class ImuConverter(Node):
    def __init__(self):
        super().__init__('imu_converter')

        self.declare_parameter('hand', 'right')  # Declare 'hand' parameter

    # Declare parameters with default values
        self.declare_parameter('beta', 0.2)
        self.declare_parameter('zeta', 0.3)
        self.declare_parameter('zero', 0)
        self.declare_parameter('max_vel', 1.0)
        self.declare_parameter('max_imu_pitch', 45)
        self.declare_parameter('min_imu_pitch', -20)
        self.declare_parameter('zero_imu_pitch', None)
        self.declare_parameter('deadzone_imu_pitch', 15)

        self.declare_parameter('max_imu_roll', 30)
        self.declare_parameter('min_imu_roll', -30)
        self.declare_parameter('zero_imu_roll', None)
        self.declare_parameter('deadzone_imu_roll', 15)

        self.hand = self.get_parameter('hand').value

        # Validate 'hand' parameter
        if self.hand not in ["right", "left"]:
            self.get_logger().warn("Invalid 'hand' parameter! Must be 'right' or 'left'. Defaulting to 'right'.")
            self.hand = "right"

        # Initialize parameters
        self.beta = self.get_parameter('beta').value
        self.zeta = self.get_parameter('zeta').value
        self.max_vel = self.get_parameter('max_vel').value
        self.max_imu_pitch = self.get_parameter('max_imu_pitch').value
        self.min_imu_pitch = self.get_parameter('min_imu_pitch').value
        self.zero_imu_pitch = self.get_parameter('zero_imu_pitch').value
        if self.zero_imu_pitch is None or self.zero_imu_pitch == 0:
            self.zero_imu_pitch = (self.max_imu_pitch + self.min_imu_pitch) / 2
        self.pitch_deadzone = self.get_parameter('deadzone_imu_pitch').value
        self.pitch_joystick = Joystick(self.zero_imu_pitch, self.pitch_deadzone, self.max_imu_pitch, self.min_imu_pitch, self.max_vel)

        self.max_imu_roll = self.get_parameter('max_imu_roll').value
        self.min_imu_roll = self.get_parameter('min_imu_roll').value
        self.zero_imu_roll = self.get_parameter('zero_imu_roll').value
        if self.zero_imu_roll is None or self.zero_imu_roll == 0:
            self.zero_imu_roll = (self.max_imu_roll + self.min_imu_roll) / 2
        self.roll_deadzone = self.get_parameter('deadzone_imu_roll').value
        self.roll_joystick = Joystick(self.zero_imu_roll, self.roll_deadzone, self.max_imu_roll, self.min_imu_roll, self.max_vel)

        base_quat = quaternion.Quaternion([1, 0, 0, 0])

    # Initialize Madgwick filter
        self.madgwick = madgwickahrs.MadgwickAHRS(quaternion=base_quat, beta=self.beta, zeta=self.zeta)

        self.current_time = time.time()

    # Subscriber to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/' + self.hand + '/data',  # Topic name
            self.imu_callback,
            10  # Queue size
        )

    # Publisher for velocity command
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/' + self.hand + '/cmd_vel_hor',  # Topic name for velocity command
            10  # Queue size
        )

    # Publisher for Euler angles
        self.euler_publisher = self.create_publisher(
            Float32MultiArray,
            '/imu/' + self.hand  + '/euler',  # Topic name for Euler angles
            10  # Queue size
        )

    # Parameter event callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info('IMU Subscriber and Velocity Publisher node has been started.')

    # Service to recenter imu
        self.center_srv = self.create_service(SetBool, self.hand + '/center_imu', self.center_imu)


    def parameter_callback(self, params):
        """
        Callback to update internal parameters when they're changed dynamically.
        """
        for param in params:
            if param.name == 'beta':
                self.beta = param.value
                self.madgwick.beta = self.beta
            elif param.name == 'zeta':
                self.zeta = param.value
                self.madgwick.zeta = self.zeta
            elif param.name == 'max_vel':
                self.max_vel = param.value
                self.pitch_joystick.max_output = self.max_vel
                self.roll_joystick.max_output = self.max_vel
            elif param.name == 'max_imu_pitch':
                self.max_imu_pitch = param.value
                if self.zero_imu_pitch == 0 or self.zero_imu_pitch is None:
                    self.zero_imu_pitch = (self.max_imu_pitch + self.min_imu_pitch) / 2
                self.pitch_joystick.max_input = self.max_imu_pitch
            elif param.name == 'min_imu_pitch':
                self.min_imu_pitch = param.value
                if self.zero_imu_pitch == 0 or self.zero_imu_pitch is None:
                    self.zero_imu_pitch = (self.max_imu_pitch + self.min_imu_pitch) / 2
                self.pitch_joystick.min_input = self.min_imu_pitch
            elif param.name == 'zero_imu_pitch':
                self.zero_imu_pitch = param.value
                if self.zero_imu_pitch == 0 or self.zero_imu_pitch is None:
                    self.zero_imu_pitch = (self.max_imu_pitch + self.min_imu_pitch) / 2
                self.pitch_joystick.zero = self.zero_imu_pitch
            elif param.name == 'deadzone_imu_pitch':
                self.pitch_deadzone = param.value
                self.pitch_joystick.dead_zone = self.pitch_deadzone
            elif param.name == 'max_imu_roll':
                self.max_imu_roll = param.value
                if self.zero_imu_roll == 0 or self.zero_imu_roll is None:
                    self.zero_imu_roll = (self.max_imu_roll + self.min_imu_roll) / 2
                self.roll_joystick.max_input = self.max_imu_roll
            elif param.name == 'min_imu_roll':
                self.min_imu_roll = param.value
                if self.zero_imu_roll == 0 or self.zero_imu_roll is None:
                    self.zero_imu_roll = (self.max_imu_roll + self.min_imu_roll) / 2
                self.roll_joystick.min_input = self.min_imu_roll
            elif param.name == 'zero_imu_roll':
                self.zero_imu_roll = param.value
                if self.zero_imu_roll == 0 or self.zero_imu_roll is None:
                    self.zero_imu_roll = (self.max_imu_roll + self.min_imu_roll) / 2
                self.roll_joystick.zero = self.zero_imu_roll
            elif param.name == 'deadzone_imu_roll':
                self.roll_deadzone = param.value
                self.roll_joystick.dead_zone = self.roll_deadzone

        return SetParametersResult(successful=True)

    def imu_callback(self, msg):
        # Process IMU data (you can modify the logic based on your use case)
        linear_acceleration = msg.linear_acceleration
        angular_velocity = msg.angular_velocity

        # Convert data to float
        accel_data =[float(linear_acceleration.x), float(linear_acceleration.y), float(linear_acceleration.z)]

        gyro_data = [
            float(angular_velocity.x) * (3.141592653589793 / 180), 
            float(angular_velocity.y) * (3.141592653589793 / 180), 
            float(angular_velocity.z) * (3.141592653589793 / 180)
        ]


        interval = time.time() - self.current_time
        self.current_time = time.time()
        self.madgwick.samplePeriod = interval
        # Update Madgwick filter
        self.madgwick.update_imu(accelerometer=accel_data, gyroscope=gyro_data)

        # Get Euler angles
        pitch, yaw, roll = self.madgwick.quaternion.to_euler_angles()

        # Publish Euler angles
        euler_msg = Float32MultiArray()
        euler_msg.data = [pitch, roll, yaw]
        self.euler_publisher.publish(euler_msg)

        # Example: Using IMU data to create simple velocity commands
        # Here, we use linear acceleration along the x-axis to control forward speed
        # and angular velocity along the z-axis for yaw control.
        
        forward_output = self.pitch_joystick.get_output(pitch)
        side_output = self.roll_joystick.get_output(roll)

        # cast as as Python's float object as some instances appear as a numpy.float64
        forward_vel = float(forward_output * MAX_VELOCITY)
        side_vel = float(side_output * MAX_VELOCITY)

        velocity_linear = Vector3(x=forward_vel, y=side_vel, z=0.0)
        velocity_angular = Vector3()
        velocity_msg = Twist(linear=velocity_linear, angular=velocity_angular)
        self.get_logger().info(f"forward_vel type: {type(forward_vel)}")
        self.get_logger().info(f"side_vel type: {type(side_vel)}")
        # Publish velocity command
        self.velocity_publisher.publish(velocity_msg)

        # Log the velocity command and Euler angles for debugging
        # self.get_logger().info(f"linear.x={velocity_msg.linear.x}, linear.y={velocity_msg.linear.y}")
        # self.get_logger().info(f"pitch={pitch}, roll={roll}, yaw={yaw}")

    def center_imu(self, request, response):
        pitch, yaw, roll = self.madgwick.quaternion.to_euler_angles()
        pitch_max = self.max_imu_pitch - self.zero_imu_pitch + pitch
        pitch_min = pitch - (self.zero_imu_pitch - self.min_imu_pitch)
        self.pitch_joystick.recenter(center=pitch, min_input=pitch_min, max_input=pitch_max)
        roll_max = self.max_imu_roll - self.zero_imu_roll + roll
        roll_min = roll - (self.zero_imu_roll - self.min_imu_roll)
        self.roll_joystick.recenter(center=roll, min_input=roll_min, max_input=roll_max)
        response.success = True
        self.get_logger().info(f"IMU recentered to pitch {pitch}, roll {roll}")
        return response

def main(args=None):
    rclpy.init(args=args)
    imu_converter = ImuConverter()
    rclpy.spin(imu_converter)
    imu_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
