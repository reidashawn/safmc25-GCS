import os
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/home/shawnchan/.local/lib/python3.10/site-packages/PyQt5/Qt/plugins"
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QThread, pyqtSignal
from sensor_msgs.msg import CompressedImage
from PyQt5.QtGui import QPixmap, QImage
from cv_bridge import CvBridge

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

class RosThread(QThread):
    data_received = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.node = CameraSubscriberNode()
        self.node.signal = self.data_received

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        rclpy.shutdown()
        self.wait()

class GuiWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.label = QLabel("waiting for data")
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)
        self.setGeometry(100, 100, 1000, 900)

        self.ros_thread = RosThread()
        self.ros_thread.data_received.connect(self.update_image)
        self.ros_thread.start()
    
    def update_image(self, cv_image):
        height, width, channels = cv_image.shape
        bytes_per_line = channels * width
        qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)

        # Update the QLabel with the QImage
        self.label.setPixmap(QPixmap.fromImage(qt_image))
    
    def closeEvent(self, event):
        self.ros_thread.stop()
        event.accept()

if __name__ == "__main__":
    rclpy.init()
    app = QApplication(sys.argv)
    window = GuiWindow()
    window.show()
    sys.exit(app.exec_())