import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel,
                             QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QGraphicsRectItem, 
                             QSpacerItem, QSizePolicy, QScrollArea)
from PyQt5.QtGui import QIcon, QFont, QPixmap, QPalette, QColor, QPainter, QImage
from PyQt5.QtCore import Qt, QRectF, QRect, QThread, pyqtSignal, QTimer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # For PWM sensor data (adjust if necessary)
from cv_bridge import CvBridge  # For converting ROS2 Image messages to OpenCV format
import cv2  # For OpenCV image processing
from sensor_msgs.msg import CompressedImage

from mavros_msgs.msg import State, OpticalFlow, StatusText
from sensor_msgs.msg import Imu, BatteryState, Range
from std_msgs.msg import Int32, Float64, Float32MultiArray

import transforms3d
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import time

qos_reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

qos_best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Dimensions
margin = 10
camera_width, camera_height = 900, 600
label_height = 30
info_width, info_height = 300, 450

# Colours
text_colour = "#F0F1F1"
background_colour = "#353535"
window_colour = "#242424"

messages_max = 6

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage, 
            "/camera/camera/color/image_raw/compressed",
            self.camera_sub_callback, 10
        )
        self.signal = pyqtSignal(object)

    def camera_sub_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image {e}")
            return

        self.signal.emit(cv_image)

class MavrosSubscriberNode(Node):
    def __init__(self):
        super().__init__('mavros_subscriber')
        self.create_subscription(State, '/mavros/state', self.state_callback, qos_reliable)
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos_best_effort)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos_best_effort)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_callback, qos_best_effort)
        self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self.rangefinder_callback, qos_reliable)
        self.create_subscription(OpticalFlow, '/mavros/optical_flow/raw/optical_flow', self.optflow_callback, qos_reliable)
        self.create_subscription(StatusText, '/mavros/statustext/recv', self.error_callback, qos_best_effort)
        
        self.data = {
            "armed": "Unknown",
            "mode": "Unknown",
            "battery": "Unknown",
            "roll": "Unknown",
            "pitch": "Unknown",
            "yaw": "Unknown",
            "altitude": "Unknown",
            "rangefinder": "Unknown",
            "optflow": "Unknown",
            "error": ""
        }
        self.signal = pyqtSignal(dict)
    
    def state_callback(self, msg):
        self.data["armed"] = "Armed" if msg.armed else "Disarmed"
        self.data["mode"] = msg.mode
        self.signal.emit(self.data)

    def battery_callback(self, msg):
        self.data["battery"] = f"{msg.percentage * 100:.1f}%"
        self.signal.emit(self.data)

    def imu_callback(self, msg):
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        roll, pitch, yaw = transforms3d.euler.quat2euler(quaternion)
        self.data["roll"] = f"{roll:.2f}"
        self.data["pitch"] = f"{pitch:.2f}"
        self.data["yaw"] = f"{yaw*100:.2f}"
        self.signal.emit(self.data)

    def altitude_callback(self, msg):
        self.data["altitude"] = f"{msg.data:.2f} m"
        self.signal.emit(self.data)
    
    def rangefinder_callback(self, msg):
        self.data["rangefinder"] = f"{msg.range:.2f} m"
        self.signal.emit(self.data)
    
    def optflow_callback(self, msg):
        self.data["optflow"] = f"Qual:{msg.quality:.2f}, Opt_x:{msg.flow_rate.x:.2f}, Opt_y:{msg.flow_rate.y:.2f}, "
        self.signal.emit(self.data)

    def error_callback(self, msg):
        self.data["error"] = msg.text
        self.signal.emit(self.data)

class ControllerSubscriberNode(Node):
    def __init__(self):
        super().__init__('mavros_subscriber')
        self.create_subscription(Int32, '/controller/right/but1', self.right_but1_callback, qos_reliable)
        self.create_subscription(Int32, '/controller/right/but2', self.right_but2_callback, qos_reliable)
        self.create_subscription(Int32, '/controller/right/but3', self.right_but3_callback, qos_reliable)
        self.create_subscription(Int32, '/controller/right/but4', self.right_but4_callback, qos_reliable)
        self.create_subscription(Int32, '/controller/right/pot', self.right_pot_callback, qos_reliable)
        self.create_subscription(Int32, '/controller/left/but1', self.left_but1_callback, qos_reliable)
        self.create_subscription(Int32, '/controller/left/but2', self.left_but2_callback, qos_reliable)
        self.create_subscription(Int32, '/controller/left/but3', self.left_but3_callback, qos_reliable)
        self.create_subscription(Int32, '/controller/left/but4', self.left_but4_callback, qos_reliable)
        self.create_subscription(Int32, '/controller/left/pot', self.left_pot_callback, qos_reliable)
        self.create_subscription(Float32MultiArray, '/imu/right/euler', self.right_imu_callback, qos_reliable)

        self.data = {
            "right_but1": 0,
            "right_but2": 0,
            "right_but3": 0,
            "right_but4": 0,
            "right_pot": 2048,
            "right_imu": [0, 0],
            "left_but1": 0,
            "left_but2": 0,
            "left_but3": 0,
            "left_but4": 0,
            "left_pot": 2048
        }
        self.signal = pyqtSignal(dict)
    
    def right_but1_callback(self, msg):
        self.data["right_but1"] = msg.data
        self.signal.emit(self.data)
    
    def right_but2_callback(self, msg):
        self.data["right_but2"] = msg.data
        self.signal.emit(self.data)

    def right_but3_callback(self, msg):
        self.data["right_but3"] = msg.data
        self.signal.emit(self.data)
    
    def right_but4_callback(self, msg):
        self.data["right_but4"] = msg.data
        self.signal.emit(self.data)

    def right_pot_callback(self, msg):
        self.data["right_pot"] = msg.data
        self.signal.emit(self.data)
    
    def right_imu_callback(self, msg):
        self.data["right_imu"][0] = msg.data[0]
        self.data["right_imu"][1] = msg.data[1]
        self.signal.emit(self.data)

    def left_but1_callback(self, msg):
        self.data["left_but1"] = msg.data
        self.signal.emit(self.data)
    
    def left_but2_callback(self, msg):
        self.data["left_but2"] = msg.data
        self.signal.emit(self.data)

    def left_but3_callback(self, msg):
        self.data["left_but3"] = msg.data
        self.signal.emit(self.data)
    
    def left_but4_callback(self, msg):
        self.data["left_but4"] = msg.data
        self.signal.emit(self.data)

    def left_pot_callback(self, msg):
        self.data["left_pot"] = msg.data
        self.signal.emit(self.data)

class Button(QLabel):
    """A label that acts as an indicator, toggling between two colors when its key is pressed."""

    def __init__(self, key, color_on="#F0F1F1", color_off="#353535", parent=None):
        super().__init__("", parent)

        self.key = key  # The key that toggles this indicator
        self.trigger = False  # Initial state
        self.color_on = color_on  # Color when "on"
        self.color_off = color_off  # Color when "off"

        self.setFixedSize(50, 50)  # Set indicator size
        self.update_button(False)

    def toggle(self):
        """Toggles the indicator color."""
        self.trigger = not self.trigger
        self.update_button(self.trigger)

    def update_button(self, is_on):
        """Updates the indicator based on whether the key is pressed."""
        color = self.color_on if is_on else self.color_off
        self.setStyleSheet(f"background-color: {color};")

class RosThread(QThread):
    fwd_cam_received = pyqtSignal(object)
    telem_received = pyqtSignal(dict)
    controller_received = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.executor = rclpy.executors.MultiThreadedExecutor()

        self.cam_node = CameraSubscriberNode()
        self.cam_node.signal = self.fwd_cam_received
        self.executor.add_node(self.cam_node)

        self.telem_node = MavrosSubscriberNode()
        self.telem_node.signal = self.telem_received
        self.executor.add_node(self.telem_node)

        self.controller_node = ControllerSubscriberNode()
        self.controller_node.signal = self.controller_received
        self.executor.add_node(self.controller_node)

        self.running = True

    def run(self):
        while self.running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)

    def stop(self):
        self.running = False
        self.executor.shutdown()
        rclpy.shutdown()
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):

        super().__init__()
        self.setWindowTitle("SAFMC GUI")
        self.setGeometry(QApplication.primaryScreen().geometry())

        self.ros_thread = RosThread()

        self.createStatus()
        self.createOverview()
        self.createButtons()
        self.createMessages()
        self.createCamera()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_messages)
        self.timer.start(500)  # Update every 500ms

        # General
        self.setStyleSheet("background-color: #353535;")

        # start background ROS thread
        self.ros_thread.start()

        self.initUI()
    
    def createCamera(self):

        self.label_fwd_cam = QLabel("    Camera", self)
        self.label_fwd_cam.setFixedHeight(label_height)
        self.label_fwd_cam.setStyleSheet("color: #F0F1F1;"
                                         "background-color: #242424;")
        self.label_fwd_cam.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        self.pic_fwd_cam = QLabel(self)
        self.pic_fwd_cam.setStyleSheet("background-color: #242424")
        self.pic_fwd_cam.setAlignment(Qt.AlignCenter)
        self.pic_fwd_cam.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.pixmap_cam = QPixmap("dwd_cam_fake.jpg")
        self.ros_thread.fwd_cam_received.connect(self.updateCam)

    def createStatus(self):

        self.label_status = QLabel("    Status", self)
        self.label_status.setFixedHeight(label_height)
        self.label_status.setStyleSheet("color: #F0F1F1;"
                                        "background-color: #242424;")
        self.label_status.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        self.layout_status = QGridLayout()
        self.layout_status.setSpacing(0)
        self.statuses = {
            "armed": QLabel("Armed: Unknown"),
            "mode": QLabel("Mode: Unknown"),
            "battery": QLabel("Battery: Unknown"),
            "roll": QLabel("Roll: Unknown"),
            "pitch": QLabel("Pitch: Unknown"),
            "yaw": QLabel("Yaw: Unknown"),
            "altitude": QLabel("Altitude: Unknown"),
            "rangefinder": QLabel("Rangefinder: Unknown"),
            "optflow": QLabel("Optflow: Unknown")
        }

        row, column = 0, 0
        for label in self.statuses.values():
            label.setStyleSheet("background-color: #242424;"
                                "color: #F0F1F1;")
            label.setAlignment(Qt.AlignCenter)
            self.layout_status.addWidget(label, row, column)
            column += 1
            if column == 3:
                row += 1
                column = 0

        self.ros_thread.telem_received.connect(self.updateStatus)

    def createOverview(self):
        self.label_overview = QLabel("    Overview", self)
        self.label_overview.setFixedHeight(label_height)
        self.label_overview.setStyleSheet("color: #F0F1F1;"
                                         "background-color: #242424;")
        self.label_overview.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        self.drone_states = {
            0: QPixmap("drone_00.jpg"),
            1: QPixmap("drone_01.jpg"),
            2: QPixmap("drone_02.jpg"),
            3: QPixmap("drone_00.jpg"),
            4: QPixmap("drone_00.jpg"),
            5: QPixmap("drone_00.jpg")
        }

        self.pic_drone = QLabel(self)
        self.pic_drone.setStyleSheet("background-color: #242424")
        self.pic_drone.setAlignment(Qt.AlignCenter)
        self.pic_drone.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.pixmap_drone = self.drone_states[0]

    def createButtons(self):
        self.L1_label = QLabel("Increase Altitude", self)
        self.L1_label.setAlignment(Qt.AlignCenter)
        self.L2_label = QLabel("Decrease Altitude", self)
        self.L2_label.setAlignment(Qt.AlignCenter)
        self.L3_label = QLabel("Receive L Payload", self)
        self.L3_label.setAlignment(Qt.AlignCenter)
        self.L4_label = QLabel("Dropoff L Payload", self)
        self.L4_label.setAlignment(Qt.AlignCenter)
        self.R1_label = QLabel("Arm / Guided / Takeoff", self)
        self.R1_label.setAlignment(Qt.AlignCenter)
        self.R2_label = QLabel("Land", self)
        self.R2_label.setAlignment(Qt.AlignCenter)
        self.R3_label = QLabel("Lock Axis", self)
        self.R3_label.setAlignment(Qt.AlignCenter)
        self.R4_label = QLabel("Set Zero", self)
        self.R4_label.setAlignment(Qt.AlignCenter)

        self.L1_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.L2_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.L3_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.L4_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.R1_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.R2_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.R3_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.R4_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")

        self.buttons = {
            Qt.Key_Q: Button(Qt.Key_Q, "#F0F1F1", "#464646", self),
            Qt.Key_W: Button(Qt.Key_W, "#F0F1F1", "#464646", self),
            Qt.Key_E: Button(Qt.Key_E, "#F0F1F1", "#464646", self),
            Qt.Key_R: Button(Qt.Key_R, "#F0F1F1", "#464646", self),
            Qt.Key_U: Button(Qt.Key_U, "#F0F1F1", "#464646", self),
            Qt.Key_I: Button(Qt.Key_I, "#F0F1F1", "#464646", self),
            Qt.Key_O: Button(Qt.Key_O, "#F0F1F1", "#464646", self),
            Qt.Key_P: Button(Qt.Key_P, "#F0F1F1", "#464646", self),
        }

        self.ros_thread.controller_received.connect(self.updateButtons)

    def createMessages(self):

        self.label_messages = QLabel("    Messages", self)
        self.label_messages.setFixedHeight(label_height)
        self.label_messages.setStyleSheet("color: #F0F1F1;"
                                         "background-color: #242424;")
        self.label_messages.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        self.message_scroll_area = QScrollArea(self)
        self.message_scroll_area.setStyleSheet("background-color: #242424;"
                                                "color: #F0F1F1;")
        self.message_scroll_area.setWidgetResizable(True)

        self.message_container_widget = QWidget()
        self.message_layout = QVBoxLayout(self.message_container_widget)
        self.message_layout.setAlignment(Qt.AlignTop)

        self.message_scroll_area.setWidget(self.message_container_widget)

        self.messages = [""]
        # self.displays = []
        # for i in range(messages_max):
        #     label = QLabel("")
        #     label.setStyleSheet("background-color: #242424;"
        #                         "color: #F0F1F1;")
        #     self.displays.append(label)

        # self.message_layout = QVBoxLayout()

        # for message in self.displays:
        #     self.message_layout.addWidget(message)

    def updateCam(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = channel * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.pixmap_cam = QPixmap.fromImage(q_image)
        self.pic_fwd_cam.setPixmap(self.pixmap_cam)

    def updateStatus(self, data):
        for key, value in data.items():
            if key != "error":
                self.statuses[key].setText(f"{key.capitalize()}: {value}")
                self.statuses[key].setStyleSheet("color: #F0F1F1;"
                                            "background-color: #242424;")

        # self.messages.append(data["error"])
        # while len(self.messages) >= messages_max:
        #     self.messages.pop(0)
        # for i in range(len(self.messages)):
        #     j = len(self.messages) - 1
        #     self.displays[j - i].setText(self.messages[i])
        
        text = data["error"]
        
        # Only store unique messages (avoiding duplicates)
        if text:
            if text != self.messages[0]:
                self.messages.insert(0, text)  # Insert new message at the top
            elif time.time() - last_update > 5:
                self.messages.insert(0, text)  # Insert new message at the top
            last_update = time.time()
        
            # Keep only the last 50 messages to prevent memory overflow
            if len(self.messages) > 50:
                self.messages.pop()

    
    def update_messages(self):
        """ Update the GUI with the latest error messages. """
        # Clear old widgets
        while self.message_layout.count():
            item = self.message_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

        # Re-add messages with the newest on top
        for msg in self.messages:
            label = QLabel(msg)
            label.setWordWrap(True)
            self.message_layout.addWidget(label)

        # Adjust scrolling to show the newest messages
        self.message_scroll_area.verticalScrollBar().setValue(0)
                    
    def updateButtons(self, data):
        self.buttons[Qt.Key_U].update_button(data["right_but1"])
        self.buttons[Qt.Key_I].update_button(data["right_but2"])
        self.buttons[Qt.Key_O].update_button(data["right_but3"])
        self.buttons[Qt.Key_P].update_button(data["right_but4"])
        self.buttons[Qt.Key_R].update_button(data["left_but1"])
        self.buttons[Qt.Key_E].update_button(data["left_but2"])
        self.buttons[Qt.Key_W].update_button(data["left_but3"])
        self.buttons[Qt.Key_Q].update_button(data["left_but4"])

        if data["left_but3"]:
            self.pixmap_drone = self.drone_states[1]
        elif data["left_but4"]:
            self.pixmap_drone = self.drone_states[1]

        self.updatePixmap()

    def buttonUI(self):
        self.layout_buttons = QGridLayout()
        self.layout_buttons.setSpacing(5)

        self.layout_buttons.addWidget(self.L1_label, 0, 0)
        self.layout_buttons.addWidget(self.L2_label, 1, 0)
        self.layout_buttons.addWidget(self.L3_label, 2, 0)
        self.layout_buttons.addWidget(self.L4_label, 3, 0)

        self.layout_buttons.addWidget(self.buttons[Qt.Key_R], 0, 1)
        self.layout_buttons.addWidget(self.buttons[Qt.Key_E], 1, 1)
        self.layout_buttons.addWidget(self.buttons[Qt.Key_W], 2, 1)
        self.layout_buttons.addWidget(self.buttons[Qt.Key_Q], 3, 1)

        self.layout_buttons.addWidget(self.buttons[Qt.Key_U], 0, 2)
        self.layout_buttons.addWidget(self.buttons[Qt.Key_I], 1, 2)
        self.layout_buttons.addWidget(self.buttons[Qt.Key_O], 2, 2)
        self.layout_buttons.addWidget(self.buttons[Qt.Key_P], 3, 2)

        self.layout_buttons.addWidget(self.R1_label, 0, 3)
        self.layout_buttons.addWidget(self.R2_label, 1, 3)
        self.layout_buttons.addWidget(self.R3_label, 2, 3)
        self.layout_buttons.addWidget(self.R4_label, 3, 3)

        self.layout_buttons.setRowStretch(0, 1)
        self.layout_buttons.setRowStretch(1, 1)
        self.layout_buttons.setRowStretch(2, 1)
        self.layout_buttons.setRowStretch(3, 1)

        self.layout_buttons.setColumnStretch(0, 1)
        self.layout_buttons.setColumnStretch(1, 2)
        self.layout_buttons.setColumnStretch(2, 2)
        self.layout_buttons.setColumnStretch(3, 1)

    def initUI(self):
            self.buttonUI()

            central_widget = QWidget()
            self.setCentralWidget(central_widget)

            camera_container = QVBoxLayout()
            camera_container.setSpacing(0)
            camera_container.addWidget(self.label_fwd_cam)
            camera_container.addWidget(self.pic_fwd_cam)

            overview_container = QVBoxLayout()
            overview_container.setSpacing(0)
            overview_container.addWidget(self.label_overview)
            overview_container.addWidget(self.pic_drone)
            overview_container.addLayout(self.layout_buttons)

            status_container = QVBoxLayout()
            status_container.setSpacing(0)
            status_container.addWidget(self.label_status)
            status_container.addWidget(self.pic_fwd_cam)

            message_container = QVBoxLayout()
            message_container.setSpacing(0)
            message_container.addWidget(self.label_messages)
            message_container.addWidget(self.message_scroll_area)
            
            grid = QGridLayout()
            grid.setSpacing(5)

            grid.addLayout(camera_container, 0, 0)
            grid.addLayout(overview_container, 0, 1)
            grid.addLayout(self.layout_status, 1, 0)
            grid.addLayout(message_container, 1, 1)

            grid.setRowStretch(0, 5)
            grid.setRowStretch(1, 2)
            grid.setColumnStretch(0, 22)
            grid.setColumnStretch(1, 10)

            central_widget.setLayout(grid)

    def updatePixmap(self):
        """Scale the image to fit within its QLabel while maintaining aspect ratio."""
        if not self.pixmap_cam.isNull():
            scaled_pixmap_cam = self.pixmap_cam.scaled(
                self.pic_fwd_cam.size(), 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            self.pic_fwd_cam.setPixmap(scaled_pixmap_cam)
        if not self.pixmap_drone.isNull():
            self.pic_drone.setAutoFillBackground
            scaled_pixmap_drone = self.pixmap_drone.scaled(
                min(self.pic_drone.width(),self.pic_drone.height()), min(self.pic_drone.width(),self.pic_drone.height()), 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            self.pic_drone.setPixmap(scaled_pixmap_drone)

    def resizeEvent(self, event):
        self.updatePixmap()

    def closeEvent(self, event):
        self.ros_thread.stop()
        event.accept()

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()