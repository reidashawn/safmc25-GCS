import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32  # Import Int32 message type
from controller.helpers.serial_helper import SerialHelper

VALID_MODES = [
    'GUIDED',
    'LAND',
    'STABILIZE',
    'LOITER'
]



class ControllerPubNode(Node):
    def __init__(self):
        super().__init__('controller_pub')
        
        # Declare parameters for serial port and baud rate with default values
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('hand', 'right')  # Declare 'hand' parameter


        # Get the parameters set at runtime (or use the defaults)
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.hand = self.get_parameter('hand').get_parameter_value().string_value

        # Validate 'hand' parameter
        if self.hand not in ["right", "left"]:
            self.get_logger().warn("Invalid 'hand' parameter! Must be 'right' or 'left'. Defaulting to 'right'.")
            self.hand = "right"

        # Initialize the SerialHelper with the parameters
        self.ser = SerialHelper(serial_port, baud_rate)

        
        # Create publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu/' + self.hand + '/data', 10)
        self.pot_publisher = self.create_publisher(Int32, 'controller/' + self.hand + '/pot', 10)
        self.but1_publisher = self.create_publisher(Int32, 'controller/' + self.hand + '/but1', 10)
        self.but2_publisher = self.create_publisher(Int32, 'controller/' + self.hand + '/but2', 10)
        self.but3_publisher = self.create_publisher(Int32, 'controller/' + self.hand + '/but3', 10)
        self.but4_publisher = self.create_publisher(Int32, 'controller/' + self.hand + '/but4', 10)
        

        # TODO: Subscribe to get state of drone for deconfliction

        # TODO: Subscribe to get state of drone for deconfliction

        # Start a timer to update IMU data every 50ms (20 Hz)
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        try:
            # Read and process serial data
            line = self.ser.read_from_serial()
            # print(line)
            # TODO: Send data from ESP in dict format and convert it from its string representation
            if line:
                # print(line)
                split = line.split(',')
                if len(split) != 11:
                    return
                
                # Convert data to float (accelerometer and gyroscope data)
                accel_data = list(map(float, split[:3]))  # accelx, accely, accelz
                gyro_data = list(map(lambda x: float(x) * (3.141592653589793 / 180), split[3:6]))  # Convert to radians/sec
                pot_value = int(split[6])  # Potentiometer value (7th number)
                but1_value = int(split[7])  # Button state (8th number)
                but2_value = int(split[8])
                but3_value = int(split[9])
                but4_value = int(split[10])

                # Create and populate IMU message
                imu_msg = Imu()
                imu_msg.linear_acceleration.x = accel_data[0]
                imu_msg.linear_acceleration.y = accel_data[1]
                imu_msg.linear_acceleration.z = accel_data[2]
                imu_msg.angular_velocity.x = gyro_data[0]
                imu_msg.angular_velocity.y = gyro_data[1]
                imu_msg.angular_velocity.z = gyro_data[2]
                self.imu_publisher.publish(imu_msg)

                # Create and publish Potentiometer message
                pot_msg = Int32()
                pot_msg.data = pot_value
                self.pot_publisher.publish(pot_msg)
                
                # Create and publish Button message
                self.but1_publisher.publish(Int32(data=but1_value))
                self.but2_publisher.publish(Int32(data=but2_value))
                self.but3_publisher.publish(Int32(data=but3_value))
                self.but4_publisher.publish(Int32(data=but4_value))


        except Exception as e:
            self.get_logger().error(f"Error in update: {e}")

    



def main(args=None):
    rclpy.init(args=args)
    controller_pub = ControllerPubNode()
    rclpy.spin(controller_pub)
    controller_pub.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
