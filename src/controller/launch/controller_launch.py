import os
import subprocess
import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def check_and_launch_drone_movement(context, *args, **kwargs):
    """Check if 'drone_movement' is running and launch it if not found."""
    try:
        # ✅ Run `ros2 node list` to check if 'drone_movement' is already running
        result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)
        running_nodes = result.stdout.splitlines()

        if '/drone_movement' in running_nodes:
            print("✅ 'drone_movement' is already running, skipping launch.")
            return []  # Don't launch another instance
        else:
            print("🚀 'drone_movement' is not running, launching now.")
            return [
                launch_ros.actions.Node(
                    package='controller',
                    executable='drone_movement',
                    name='drone_movement',
                    output='screen',
                    parameters=[{
                        'some_param': 'value'  # Replace with actual parameters if needed
                    }]
                )
            ]

    except Exception as e:
        print(f"⚠️ Error checking running nodes: {e}")
        return []  # Fallback to not launching if an error occurs

def generate_launch_description():
    # ✅ Declare launch arguments correctly
    serial_port_right_arg = DeclareLaunchArgument(
        'serial_port_right',
        default_value='/dev/ttyUSB1',
        description='Serial port for the controller'
    )
    
    serial_port_left_arg = DeclareLaunchArgument(
        'serial_port_left',
        default_value='/dev/ttyUSB0',
        description='Serial port for the controller'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )


    return launch.LaunchDescription([
        serial_port_right_arg,
        serial_port_left_arg,
        baud_rate_arg,

        # ✅ Controller Publisher Node
        launch_ros.actions.Node(
            package='controller',
            executable='controller_pub',
            name='controller_pub_right',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port_right'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'hand': 'right'
            }],
            output='screen'
        ),

        # ✅ IMU Converter Node
        launch_ros.actions.Node(
            package='controller',
            executable='imu_converter',
            name='imu_converter_right',
            parameters=[{
                'hand': 'right'
            }],
            output='screen'
        ),

        # ✅ Potentiometer Converter Node
        launch_ros.actions.Node(
            package='controller',
            executable='pot_converter',
            name='pot_converter_right',
            parameters=[{
                'hand': 'right'
            }],
            output='screen'
        ),

        launch_ros.actions.Node(
            package='controller',
            executable='button_manager',
            name='button_manager_right',
            parameters=[{
                'hand': 'right'
            }],
            output='screen'
        ),

                # ✅ Controller Publisher Node
        launch_ros.actions.Node(
            package='controller',
            executable='controller_pub',
            name='controller_pub_left',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port_left'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'hand': 'left'
            }],
            output='screen'
        ),

        # ✅ IMU Converter Node
        launch_ros.actions.Node(
            package='controller',
            executable='imu_converter',
            name='imu_converter_left',
            parameters=[{
                'hand': 'left'
            }],
            output='screen'
        ),

        # ✅ Potentiometer Converter Node
        launch_ros.actions.Node(
            package='controller',
            executable='pot_converter',
            name='pot_converter_left',
            parameters=[{
                'hand': 'left'
            }],
            output='screen'
        ),

        launch_ros.actions.Node(
            package='controller',
            executable='button_manager',
            name='button_manager_left',
            parameters=[{
                'hand': 'left'
            }],
            output='screen'
        ),

        # ✅ Check and launch drone_movement only if it's not running
        OpaqueFunction(function=check_and_launch_drone_movement)
    ])
