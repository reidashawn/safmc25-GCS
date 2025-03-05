import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import SetBool
from controller.helpers.button import Button
from time import sleep
from interfaces.srv import ToggleStepper, SetFloat



VALID_MODES = [
    'GUIDED',
    'LAND',
    'STABILIZE',
    'LOITER'
]

TAKEOFF_HEIGHT = 1.0
MAV_CMD_NAV_TAKEOFF = 22 # From Mavlink MAV_CMD: https://mavlink.io/en/messages/common.html#mav_commands)
MAX_VELOCITY = 0.10


class ButtonManagerNode(Node):
    VALID_MODES = [
        'GUIDED',
        'LAND',
        'STABILIZE',
        'LOITER'
    ]

    TAKEOFF_HEIGHT = 1.0
    MAV_CMD_NAV_TAKEOFF = 22 # From Mavlink MAV_CMD: https://mavlink.io/en/messages/common.html#mav_commands)
    MAX_VELOCITY = 0.10

    def __init__(self):
        super().__init__('button_manager')

        self.declare_parameter('hand', 'right')
    
        self.hand = self.get_parameter('hand').get_parameter_value().string_value

        print(self.hand)

        if self.hand not in ['right', 'left']:
            self.get_logger().error("Invalid hand parameter passed")
            return
        self.state_subscriber = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.drone_state = False
        self.drone_arm = True
        self.drone_mode = None
        
        if self.hand == 'right':
            self.button1 = Button(topic='controller/right/but1', short_callback=self.guided_arm_takeoff)
            self.button2 = Button(topic='controller/right/but2', short_callback=self.land_callback)
            self.button3 = Button(topic='controller/right/but3', short_callback=self.recenter, long_callback=self.lock_zero, release_callback=self.unlock_zero)
            self.button4 = Button(topic='controller/right/but4', short_callback=self.lock_axis, release_callback=self.unlock_axis)
            
            self.get_logger().info('Right hand initialized')

            # RIGHT CONTROLLER BUTTONS
            self.but1_subscriber = self.create_subscription(Int32, 'controller/right/but1', self.button1.data_callback, 10)
            self.but2_subscriber = self.create_subscription(Int32, 'controller/right/but2', self.button2.data_callback, 10)
            self.but3_subscriber = self.create_subscription(Int32, 'controller/right/but3', self.button3.data_callback, 10)
            self.but4_subscriber = self.create_subscription(Int32, 'controller/right/but4', self.button4.data_callback, 10)

        else:
            self.button1 = Button(topic='controller/left/but1', press_callback=self.vertical_up_movement_callback,release_callback=self.vertical_zero_movement_callback)
            self.button2 = Button(topic='controller/left/but2', press_callback=self.vertical_down_movement_callback, release_callback=self.vertical_zero_movement_callback)
            self.button3 = Button(topic='controller/left/but3', short_callback=self.bag_in_callback, long_callback=self.bag_in_hold_callback, release_callback=self.bag_stop_callback)
            self.button4 = Button(topic='controller/left/but4', short_callback=self.bag_out_callback, long_callback=self.bag_out_hold_callback, release_callback=self.bag_stop_callback)

            # LEFT CONTROLLER BUTTONS
            self.but1_subscriber = self.create_subscription(Int32, 'controller/left/but1', self.button1.data_callback, 10)
            self.but2_subscriber = self.create_subscription(Int32, 'controller/left/but2', self.button2.data_callback, 10)
            self.but3_subscriber = self.create_subscription(Int32, 'controller/left/but3', self.button3.data_callback, 10)
            self.but4_subscriber = self.create_subscription(Int32, 'controller/left/but4', self.button4.data_callback, 10)

            self.get_logger().info('Left hand initialized')


        

        
    
        self.mavros_clients = {
            'mode': self.create_client(SetMode, '/mavros/set_mode'),
            'arm': self.create_client(CommandBool, '/mavros/cmd/arming'),
            'takeoff': self.create_client(CommandLong, '/mavros/cmd/command'),
            'bag': self.create_client(ToggleStepper, '/rotate_stepper'),
            'bag_hold': self.create_client(ToggleStepper, '/toggle_stepper'),
            'move_vert': self.create_client(SetFloat, '/vert_vel'),
            'lock_axis': self.create_client(SetBool, '/lock_axis'),
            'lock_zero': self.create_client(SetBool, '/lock_zero'),
            'landing': self.create_client(SetBool, '/landing')
        }

        # for service_name, client in self.mavros_clients.items():
        #     while not client.wait_for_service(timeout_sec=1.0):
        #         self.get_logger().warn(f'Waiting for {service_name} service')

        self.get_logger().info('All services online')

        self.centering_clients = {
            'pot_right': self.create_client(SetBool, '/right/center_pot'),
            'pot_left': self.create_client(SetBool, '/left/center_pot'),
            'imu_right': self.create_client(SetBool, '/right/center_imu'),
            'imu_left': self.create_client(SetBool, '/left/center_imu')
        }

    def state_callback(self, msg):
        self.drone_arm = msg.armed
        self.drone_mode = msg.mode
        if msg.system_status == 4:
            self.drone_state = True
        else:
            self.drone_state = False
        pass

    def arm_takeoff_callback(self):
        self.get_logger().info("arm and takeoff")
        self.arm_drone()
            
    def land_callback(self):
        self._change_mode('LAND')
        request = SetBool.Request()
        request.data = True
        self.mavros_clients['landing'].call_async(request)
    
    def set_guided_callback(self):
        self.get_logger().info("PRESSED")
        self._change_mode('GUIDED')

    def guided_arm_takeoff(self):
        if self.drone_mode != 'GUIDED':
            self._change_mode('GUIDED')
            self.get_logger().info("Setting mode to guided")
        elif not self.drone_arm:
            self.arm_drone()
            self.get_logger().info("Arming Drone")
        elif not self.drone_state:
            self.takeoff()
            self.get_logger().info("Taking off")

    def arm_callback(self):
        self.arm_drone()

    def takeoff_callback(self):
        self.takeoff()

    def vertical_up_movement_callback(self):
        float_data_req = SetFloat.Request()
        float_data_req.data = MAX_VELOCITY
        future = self.mavros_clients['move_vert'].call_async(float_data_req)
        self.get_logger().info("Positive vertical movement")
    
    def vertical_zero_movement_callback(self):        
        float_data_req = SetFloat.Request()
        float_data_req.data = float(0)
        future = self.mavros_clients['move_vert'].call_async(float_data_req)
        self.get_logger().info("Zero vertical movement")
    
    def vertical_down_movement_callback(self):        
        float_data_req = SetFloat.Request()
        float_data_req.data = -MAX_VELOCITY
        future = self.mavros_clients['move_vert'].call_async(float_data_req)
        self.get_logger().info("Negative vertical movement")


    def bag_in_callback(self):
        bag_in_req = ToggleStepper.Request()
        bag_in_req.stepper_id = 1
        bag_in_req.speed = float(1)
        self.get_logger().info("sending bag in")
        future = self.mavros_clients['bag'].call_async(bag_in_req)
        try:
            self.get_logger().info(f"bag_in result: {future.result()}")
        except Exception as e:
            self.get_logger().error('bag_in: Exception {e} occured')

    def bag_in_hold_callback(self):
        bag_in_req = ToggleStepper.Request()
        bag_in_req.stepper_id = 1
        bag_in_req.speed = float(1)
        self.get_logger().info("sending bag in")
        future = self.mavros_clients['bag_hold'].call_async(bag_in_req)
        try:
            self.get_logger().info(f"bag result: {future.result()}")
        except Exception as e:
            self.get_logger().error('bag: Exception {e} occured')

    def bag_out_callback(self):
        bag_out_req = ToggleStepper.Request()
        bag_out_req.stepper_id = 1
        bag_out_req.speed = float(-1)
        future = self.mavros_clients['bag'].call_async(bag_out_req)
        try:
            self.get_logger().info(f"bag_in result: {future.result()}")
        except Exception as e:
            self.get_logger().error('bag_in: Exception {e} occured')
    
    def bag_out_hold_callback(self):
        bag_out_req = ToggleStepper.Request()
        bag_out_req.stepper_id = 1
        bag_out_req.speed = float(-1)
        future = self.mavros_clients['bag_hold'].call_async(bag_out_req)
        try:
            self.get_logger().info(f"bag result: {future.result()}")
        except Exception as e:
            self.get_logger().error('bag: Exception {e} occured')

    def bag_stop_callback(self):
        bag_out_req = ToggleStepper.Request()
        bag_out_req.stepper_id = 1
        bag_out_req.speed = float(0)
        future = self.mavros_clients['bag_hold'].call_async(bag_out_req)
        try:
            self.get_logger().info(f"bag result: {future.result()}")
        except Exception as e:
            self.get_logger().error('bag: Exception {e} occured')        

    def _change_mode(self, mode: str):
        mode_upper = mode.upper()
        self.get_logger().info("{mode}, {mode_upper}")
        if mode_upper not in VALID_MODES:
            self.get_logger().error(f"{mode.upper()} is not a valid mode")
            return
        mode_req = SetMode.Request(custom_mode=mode_upper)
        future = self.mavros_clients['mode'].call_async(mode_req)
        self.get_logger().info(f"Change mode to {mode_upper} result: {future.result()}")

    def arm_drone(self):
        arm_req = CommandBool.Request(value=True)
        future = self.mavros_clients['arm'].call_async(arm_req)
        self.get_logger().info(f"Arm drone result: {future.result()}")

        # # rclpy.spin_until_future_complete(self, future)
        # if future.exception():
        #     print('exception raised')
        # elif future.result() is None:
        #     print('result returned none')
        # else:
        #     print(f"{future.result()}")

        # self.get_logger().info(f"{future.done()}")
        # return future.done()

    def takeoff(self):
        takeoff_req = CommandLong.Request(
            broadcast=False, 
            command=MAV_CMD_NAV_TAKEOFF, 
            confirmation=0, 
            param1=0.0, 
            param2=0.0, 
            param3=0.0, 
            param4=0.0, 
            param5=0.0, 
            param6=0.0, 
            param7=TAKEOFF_HEIGHT
            )
        future = self.mavros_clients['takeoff'].call_async(takeoff_req)
        self.get_logger().info(f"Takeoff command result: {future.result()}")

    def recenter(self):
        request = SetBool.Request()
        request.data = True
        self.get_logger().info(f"Recentering")
        for service_name, client in self.centering_clients.items():
            client.call_async(request)

    def lock_axis(self):
        request = SetBool.Request()
        request.data = True
        self.mavros_clients['lock_axis'].call_async(request)
    
    def unlock_axis(self):
        request = SetBool.Request()
        request.data = False
        self.mavros_clients['lock_axis'].call_async(request)

    def lock_zero(self):
        request = SetBool.Request()
        request.data = True
        self.get_logger().info(f"Lock zero")
        self.mavros_clients['lock_zero'].call_async(request)
    
    def unlock_zero(self):
        request = SetBool.Request()
        request.data = False
        self.get_logger().info(f"Unlock zero")
        self.mavros_clients['lock_zero'].call_async(request)

    

def main(args=None):
    rclpy.init(args=args)
    button_manager = ButtonManagerNode()
    rclpy.spin(button_manager)
    button_manager.destroy_node()

if __name__ == '__main__':
    main()

        
