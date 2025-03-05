import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel,
                             QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QGraphicsRectItem, 
                             QSpacerItem, QSizePolicy)
from PyQt5.QtGui import QIcon, QFont, QPixmap, QPalette, QColor, QPainter, QImage
from PyQt5.QtCore import Qt, QRectF, QRect, QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # For PWM sensor data (adjust if necessary)
from cv_bridge import CvBridge  # For converting ROS2 Image messages to OpenCV format
import cv2  # For OpenCV image processing
from sensor_msgs.msg import CompressedImage

from mavros_msgs.msg import State, OpticalFlow, StatusText
from sensor_msgs.msg import Imu, BatteryState, Range
from std_msgs.msg import Float64
# import tf_transformations
# from controller.madgwick_py import quarternion

import transforms3d
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
            "error": "Unknown"
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
        print(msg)
        self.data["error"] = msg.text
        self.signal.emit(self.data)

class RosThread(QThread):
    fwd_cam_received = pyqtSignal(object)
    telem_received = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.cam_node = CameraSubscriberNode()
        self.cam_node.signal = self.fwd_cam_received

        self.telem_node = MavrosSubscriberNode()
        self.telem_node.signal = self.telem_received

    def run(self):
        # rclpy.spin(self.cam_node)
        rclpy.spin(self.telem_node)

    def stop(self):
        rclpy.shutdown()
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):

        super().__init__()
        self.setWindowTitle("SAFMC GUI")
        self.setGeometry(QApplication.primaryScreen().geometry())

        self.ros_thread = RosThread()

        # Forward cam (SHAWN)
        self.label_fwd_cam, self.pic_fwd_cam = self.createCam("    Forward Cam")
        self.ros_thread.fwd_cam_received.connect(self.updateCam)
        # self.updateCam_fake(self.pic_fwd_cam)

        # Downward cam
        self.label_dwd_cam, self.pic_dwd_cam = self.createCam("    Downward Cam")
        # self.updateCam_fake(self.pic_dwd_cam)

        # UAV Info
        self.label_UAV, self.info_UAV = self.createUAVInfo()
        self.ros_thread.telem_received.connect(self.updateUAVInfo)

        # Payload
        self.label_payload = QLabel("    Payload", self)
        self.label_payload.setGeometry(2 * margin + info_width, 2 * margin + label_height + camera_height, info_height // 6 * 10, label_height)
        self.label_payload.setFixedSize(info_height // 6 * 10, label_height)
        self.label_payload.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        self.label_payload.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        self.pic_drone = QLabel(self)
        self.pic_drone.setGeometry(margin, margin + label_height, info_height // 6 * 10, info_height)
        pixmap_drone = QPixmap("drone.jpg")        
        self.pic_drone.setPixmap(pixmap_drone.scaled(self.pic_drone.size(), aspectRatioMode=1))
        self.pic_drone.setFixedSize(info_height // 6 * 10, info_height)
        self.pic_drone.setAlignment(Qt.AlignHCenter | Qt.AlignTop)

        # Controllers
        self.label_ctrl = QLabel("    Controllers", self)
        self.label_ctrl.setGeometry(3 * margin + info_width // 6 * 16, 2 * margin + label_height + camera_height, info_height // 6 * 10, label_height)
        self.label_ctrl.setFixedSize(info_height // 6 * 10, label_height)
        self.label_ctrl.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        self.label_payload.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        # Controllers
        self.pic_ctrl = QLabel(self)
        self.pic_ctrl.setGeometry(margin, margin + label_height, info_height // 6 * 10, info_height)
        pixmap_ctrl = QPixmap("controllers.jpg")        
        self.pic_ctrl.setPixmap(pixmap_ctrl.scaled(self.pic_ctrl.size(), aspectRatioMode=1))
        self.pic_ctrl.setFixedSize(info_height // 6 * 10, info_height)
        self.pic_ctrl.setAlignment(Qt.AlignHCenter | Qt.AlignTop)

        # Misc
        self.label_spacer = QLabel(self)
        self.label_spacer.setGeometry(margin, margin + label_height + camera_height, info_width, label_height)
        self.label_spacer.setFixedHeight(margin)

        # General
        self.setStyleSheet("background-color: #353535;")

        # start background ROS thread
        self.ros_thread.start()
        self.initUI()
    
    def createCam(self, label):
        label_cam = QLabel(label, self)
        label_cam.setGeometry(margin, margin, camera_width, label_height)
        label_cam.setFixedHeight(label_height)
        label_cam.setStyleSheet("color: #F0F1F1;"
                                         "background-color: #242424;")
        label_cam.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        pic_cam = QLabel(self)
        pic_cam.setGeometry(margin, margin + label_height, camera_width, camera_height)
        
        # TODO: fix this
        pic_cam.setFixedSize(camera_width, camera_height)
        pic_cam.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        pic_cam.setStyleSheet("""
            border: 20px solid #242424;
        """)

        return label_cam, pic_cam
    
    def createUAVInfo(self):
        label_UAV = QLabel("    UAV Info", self)
        label_UAV.setGeometry(margin, 2 * margin + label_height + camera_height, info_width, label_height)
        label_UAV.setFixedSize(info_width, label_height)
        label_UAV.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        label_UAV.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        self.info_UAV = QVBoxLayout()
        self.labels = {
            "armed": QLabel("Armed: Unknown"),
            "mode": QLabel("Mode: Unknown"),
            "battery": QLabel("Battery: Unknown"),
            "roll": QLabel("Roll: Unknown"),
            "pitch": QLabel("Pitch: Unknown"),
            "yaw": QLabel("Yaw: Unknown"),
            "altitude": QLabel("Altitude: Unknown"),
            "rangefinder": QLabel("Rangefinder: Unknown"),
            "optflow": QLabel("Optflow: Unknown"),
            "error": QLabel("Error: Unknown")
        }
        for label in self.labels.values():
            self.info_UAV.addWidget(label)
        
        return label_UAV, self.info_UAV

    def updateCam(self, cv_image):  # (SHAWN)
        height, width, channel = cv_image.shape
        bytes_per_line = channel * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.pixmap_cam = QPixmap.fromImage(q_image)
        self.pic_fwd_cam.setPixmap(self.pixmap_cam)

    def updateUAVInfo(self, data):
        for key, value in data.items():
            self.labels[key].setText(f"{key.capitalize()}: {value}")
            self.labels[key].setStyleSheet("color: #F0F1F1;"
                                           "background-color: #242424;")

    def buttonUI(self):
        self.buttons = QGridLayout()
        self.buttons.setSpacing(5)

        self.buttons.addWidget(self.L1_label, 0, 0)
        self.buttons.addWidget(self.L2_label, 1, 0)
        self.buttons.addWidget(self.L3_label, 2, 0)
        self.buttons.addWidget(self.L4_label, 3, 0)

        self.buttons.addWidget(self.holdIndicators[Qt.Key_R], 0, 1)
        self.buttons.addWidget(self.holdIndicators[Qt.Key_E], 1, 1)
        self.buttons.addWidget(self.holdIndicators[Qt.Key_W], 2, 1)
        self.buttons.addWidget(self.holdIndicators[Qt.Key_Q], 3, 1)

        self.buttons.addWidget(self.holdIndicators[Qt.Key_U], 0, 2)
        self.buttons.addWidget(self.holdIndicators[Qt.Key_I], 1, 2)
        self.buttons.addWidget(self.holdIndicators[Qt.Key_O], 2, 2)
        self.buttons.addWidget(self.holdIndicators[Qt.Key_P], 3, 2)

        self.buttons.addWidget(self.R1_label, 0, 3)
        self.buttons.addWidget(self.R2_label, 1, 3)
        self.buttons.addWidget(self.R3_label, 2, 3)
        self.buttons.addWidget(self.R4_label, 3, 3)

        self.buttons.setRowStretch(0, 1)
        self.buttons.setRowStretch(1, 1)
        self.buttons.setRowStretch(2, 1)
        self.buttons.setRowStretch(3, 1)

        self.buttons.setColumnStretch(0, 1)
        self.buttons.setColumnStretch(1, 2)
        self.buttons.setColumnStretch(2, 2)
        self.buttons.setColumnStretch(3, 1)

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
            overview_container.addLayout(self.buttons)

            grid = QGridLayout()
            grid.setSpacing(5)

            grid.addLayout(camera_container, 0, 0)
            grid.addLayout(overview_container, 0, 1)
            grid.addLayout(self.info_UAV, 1, 0)

            grid.setRowStretch(0, 3)
            grid.setRowStretch(1, 1)
            grid.setColumnStretch(0, 22)
            grid.setColumnStretch(1, 10)

            central_widget.setLayout(grid)

    def resizeEvent(self, event):
        self.updatePixmap()
    
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