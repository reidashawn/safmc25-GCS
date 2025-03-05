import sys
import time
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QHBoxLayout, QGridLayout, QSizePolicy, QLabel
from PyQt5.QtCore import Qt, QTimer, QPointF
from PyQt5.QtGui import QPainter, QBrush, QPen, QPixmap


from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel,
                             QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QGraphicsRectItem, 
                             QSpacerItem, QSizePolicy)
from PyQt5.QtGui import QIcon, QFont, QPixmap, QPalette, QColor, QPainter, QImage
from PyQt5.QtCore import Qt, QRectF, QRect, QThread, pyqtSignal


import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32MultiArray
from sensor_msgs.msg import Imu

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MIN_ROLL_PWM, MAX_ROLL_PWM = -90, 90
MIN_PITCH_PWM, MAX_PITCH_PWM = -90, 90
MIN_YAW_PWM, MAX_YAW_PWM = 1, 4096
MIN_ALT_PWM, MAX_ALT_PWM = 1, 4096

qos_reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

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

class RosThread(QThread):

    controller_received = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.controller_node = ControllerSubscriberNode()
        self.controller_node.signal = self.controller_received

    def run(self):
        rclpy.spin(self.controller_node)

    def stop(self):
        rclpy.shutdown()
        self.wait()

# class VirtualJoystick(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setFixedSize(200, 200)
#         self.joystick_radius = 80
#         self.knob_radius = 20
#         self.center = QPointF(self.width() / 2, self.height() / 2)
#         self.knob_pos = self.center
#         self.active = False

#     def paintEvent(self, event):
#         painter = QPainter(self)
#         painter.setRenderHint(QPainter.Antialiasing)

#         # Draw joystick background
#         painter.setBrush(QBrush(Qt.lightGray))
#         painter.setPen(QPen(Qt.black, 2))
#         painter.drawEllipse(self.center, self.joystick_radius, self.joystick_radius)

#         # Draw joystick knob
#         painter.setBrush(QBrush(Qt.red))
#         painter.setPen(QPen(Qt.black, 2))
#         painter.drawEllipse(self.knob_pos, self.knob_radius, self.knob_radius)

#         painter.end()

#     def mousePressEvent(self, event):
#         if (event.pos() - self.knob_pos).manhattanLength() < self.knob_radius:
#             self.active = True

#     def mouseMoveEvent(self, event):
#         if self.active:
#             delta = event.pos() - self.center
#             if delta.manhattanLength() > self.joystick_radius:
#                 delta *= self.joystick_radius / delta.manhattanLength()
#             self.knob_pos = self.center + delta
#             self.update()

#     def mouseReleaseEvent(self, event):
#         self.active = False
#         self.knob_pos = self.center
#         self.update()

#     def get_values(self):
#         """ Returns joystick x, y values normalized between -1 and 1 """
#         x = (self.knob_pos.x() - self.center.x()) / self.joystick_radius
#         y = -(self.knob_pos.y() - self.center.y()) / self.joystick_radius
#         return round(x, 2), round(y, 2)

# class JoystickWindow(QMainWindow):
#     """ Separate window for the virtual joystick """
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Virtual Joystick")
#         self.setGeometry(100, 100, 250, 250)
#         self.joystick = VirtualJoystick()
#         self.setCentralWidget(self.joystick)

class Button(QLabel):
    """A label that acts as an indicator, toggling between two colors when its key is pressed."""

    def __init__(self, key, color_on="#F0F1F1", color_off="#353535", parent=None):
        super().__init__("", parent)

        self.key = key  # The key that toggles this indicator
        self.trigger = False  # Initial state
        self.color_on = color_on  # Color when "on"
        self.color_off = color_off  # Color when "off"

        self.setFixedSize(100, 100)  # Set indicator size
        self.update_indicator(False)

    def toggle(self):
        """Toggles the indicator color."""
        self.trigger = not self.trigger
        self.update_indicator(self.trigger)

    def update_indicator(self, is_on):
        """Updates the indicator based on whether the key is pressed."""
        color = self.color_on if is_on else self.color_off
        self.setStyleSheet(f"background-color: {color};")

class JoystickGraph(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time Graphs")
        self.setGeometry(QApplication.primaryScreen().geometry())
        self.ros_thread = RosThread()

        # ROLL GRAPH
        self.x_plot = pg.PlotWidget(title="Roll")
        self.x_curve = self.x_plot.plot(pen='r')
        self.x_plot.setXRange(MIN_ROLL_PWM, MAX_ROLL_PWM)  # Fix Y-axis range
        self.x_plot.showGrid(x=True, y=True)

        # PITCH GRAPH
        self.y_plot = pg.PlotWidget(title="Pitch")
        self.y_curve = self.y_plot.plot(pen='b')
        self.y_plot.setYRange(MIN_PITCH_PWM, MAX_PITCH_PWM)  # Fix Y-axis range
        self.y_plot.showGrid(x=True, y=True)

        # ROLL-PITCH GRAPH
        self.xy_plot = pg.PlotWidget(title="Roll-Pitch")
        self.xy_curve = self.xy_plot.plot(pen='g', symbol='o')
        self.xy_plot.setXRange(MIN_ROLL_PWM, MAX_ROLL_PWM)
        self.xy_plot.setYRange(MIN_PITCH_PWM, MAX_PITCH_PWM)
        self.xy_plot.setLabel('left', 'Pitch')
        self.xy_plot.setLabel('bottom', 'Roll')
        self.xy_plot.showGrid(x=True, y=True)

            # Data storage
        self.time_window = 3  # Time window for X and Y graphs
        self.trail_window = 1  # Time window for 2D joystick trail
        self.x_data = []
        self.y_data = []
        self.time_data = []
        self.time_data2 = []

        # Store the joystick instance
        # self.joystick = joystick
        # self.joystick2 = joystick2

        # YAW GRAPH
        self.yaw_plot = pg.PlotWidget(title="YAW PWM")
        self.yaw_curve = self.yaw_plot.plot(pen='g')
        self.yaw_plot.setXRange(MIN_YAW_PWM, MAX_YAW_PWM)  # Fix Y-axis range
        self.yaw_plot.showGrid(x=True, y=True)
        self.yaw_data = []
        
        # ALT GRAPH
        self.z_plot = pg.PlotWidget(title="ALT PWM")
        self.z_curve = self.z_plot.plot(pen='g')
        self.z_plot.setYRange(MIN_ALT_PWM, MAX_ALT_PWM)  # Fix Y-axis range
        self.z_plot.showGrid(x=True, y=True)
        self.z_data = []

        # Timer for updates
        self.start_time = time.time()
        self.timer = QTimer()
        # self.timer.timeout.connect(self.update_data)
        self.timer.start(50)

        # Buttons
        self.L1_label = QLabel("Toggle Axis", self)
        self.L1_label.setAlignment(Qt.AlignCenter)
        self.L2_label = QLabel("Toggle Camera", self)
        self.L2_label.setAlignment(Qt.AlignCenter)
        self.L3_label = QLabel("L3", self)
        self.L3_label.setAlignment(Qt.AlignCenter)
        self.L4_label = QLabel("Takeoff / Land", self)
        self.L4_label.setAlignment(Qt.AlignCenter)
        self.R1_label = QLabel("Toggle L/R Screw", self)
        self.R1_label.setAlignment(Qt.AlignCenter)
        self.R2_label = QLabel("Receive Payload", self)
        self.R2_label.setAlignment(Qt.AlignCenter)
        self.R3_label = QLabel("Dropoff Payload", self)
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

        self.holdIndicators = {
            Qt.Key_Q: Button(Qt.Key_Q, "#F0F1F1", "#353535", self),
            Qt.Key_W: Button(Qt.Key_W, "#F0F1F1", "#353535", self),
            Qt.Key_E: Button(Qt.Key_E, "#F0F1F1", "#353535", self),
            Qt.Key_R: Button(Qt.Key_R, "#F0F1F1", "#353535", self),
            Qt.Key_U: Button(Qt.Key_U, "#F0F1F1", "#353535", self),
            Qt.Key_I: Button(Qt.Key_I, "#F0F1F1", "#353535", self),
            Qt.Key_O: Button(Qt.Key_O, "#F0F1F1", "#353535", self),
            Qt.Key_P: Button(Qt.Key_P, "#F0F1F1", "#353535", self),
        }

        self.ros_thread.controller_received.connect(self.updateController)

        self.setStyleSheet("background-color: #242424;")

        self.ros_thread.start()
        self.initUI()
    
    def initUI(self):
        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        # self.main_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QGridLayout()
        self.main_widget.setLayout(layout)

        layout.addWidget(self.xy_plot, 0, 0)

        layout.addWidget(self.x_plot, 1, 0)
        layout.addWidget(self.y_plot, 0, 1)
        layout.addWidget(self.yaw_plot, 2, 0)
        layout.addWidget(self.z_plot, 0, 2)

        buttons_top = QGridLayout()
        buttons_bottom = QGridLayout()
        self.setLayout(layout)

        # Instruction Label
        buttons_top.addWidget(self.L1_label, 0, 0)
        buttons_top.addWidget(self.L2_label, 1, 0)
        buttons_bottom.addWidget(self.L3_label, 0, 0)
        buttons_bottom.addWidget(self.L4_label, 1, 0)

        buttons_top.addWidget(self.holdIndicators[Qt.Key_R], 0, 1)
        buttons_top.addWidget(self.holdIndicators[Qt.Key_E], 1, 1)
        buttons_bottom.addWidget(self.holdIndicators[Qt.Key_W], 0, 1)
        buttons_bottom.addWidget(self.holdIndicators[Qt.Key_Q], 1, 1)

        buttons_top.addWidget(self.holdIndicators[Qt.Key_U], 0, 2)
        buttons_top.addWidget(self.holdIndicators[Qt.Key_I], 1, 2)
        buttons_bottom.addWidget(self.holdIndicators[Qt.Key_O], 0, 2)
        buttons_bottom.addWidget(self.holdIndicators[Qt.Key_P], 1, 2)

        buttons_top.addWidget(self.R1_label, 0, 3)
        buttons_top.addWidget(self.R2_label, 1, 3)
        buttons_bottom.addWidget(self.R3_label, 0, 3)
        buttons_bottom.addWidget(self.R4_label, 1, 3)

        layout.addLayout(buttons_top, 1, 1)
        layout.addLayout(buttons_bottom, 2, 1)


        layout.setRowStretch(0, 3)  # Row 0 is twice the height of Row 1
        layout.setRowStretch(1, 1)  # Row 1 is smaller
        layout.setRowStretch(2, 1)  # Row 2 is smaller

        layout.setColumnStretch(0, 3)  # Column 0 is twice as wide as Column 1
        layout.setColumnStretch(1, 1)  # Column 1 is smaller
        layout.setColumnStretch(2, 1)  # Column 2 is smaller

    def updateController(self, data):
        self.updateButtons(data)
        self.updatePot(data)

    def updateButtons(self, data):
        self.holdIndicators[Qt.Key_U].update_indicator(data["right_but1"])
        self.holdIndicators[Qt.Key_I].update_indicator(data["right_but2"])
        self.holdIndicators[Qt.Key_O].update_indicator(data["right_but3"])
        self.holdIndicators[Qt.Key_P].update_indicator(data["right_but4"])
        self.holdIndicators[Qt.Key_R].update_indicator(data["left_but1"])
        self.holdIndicators[Qt.Key_E].update_indicator(data["left_but2"])
        self.holdIndicators[Qt.Key_W].update_indicator(data["left_but3"])
        self.holdIndicators[Qt.Key_Q].update_indicator(data["left_but4"])          

    def updatePot(self, data):
        x_value = data["right_imu"][1]
        y_value = data["right_imu"][0]
        yaw_value = data["right_pot"]
        z_value = data["left_pot"]
        
        current_time = time.time() - self.start_time
        self.x_data.append(-x_value)
        self.y_data.append(y_value)
        self.yaw_data.append(yaw_value)
        self.z_data.append(z_value)
        self.time_data.append(current_time)

        # Trim data to last `time_window` seconds
        while self.time_data and (current_time - self.time_data[0] > self.time_window):
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.yaw_data.pop(0)
            self.z_data.pop(0)
            self.time_data.pop(0)

        x_trail = []
        y_trail = []
        for i in range(len(self.time_data)):
            if current_time - self.time_data[i] <= self.trail_window:
                x_trail.append(self.x_data[i])
                y_trail.append(self.y_data[i])

        self.x_curve.setData(self.x_data, self.time_data)  # Swap x and y values
        self.x_plot.setYRange(max(0, current_time - self.time_window), current_time)  # Set the time as the Y-axis range

        self.y_curve.setData(self.time_data, self.y_data)
        self.y_plot.setXRange(max(0, current_time - self.time_window), current_time)

        self.xy_curve.setData(x_trail, y_trail)

        self.yaw_curve.setData(self.yaw_data, self.time_data)  # Swap x and y values
        self.yaw_plot.setYRange(max(0, current_time - self.time_window), current_time)  # Set the time as the Y-axis range

        self.z_curve.setData(self.time_data, self.z_data)  # Swap x and y values
        self.z_plot.setXRange(max(0, current_time - self.time_window), current_time)  # Set the time as the Y-axis range


    def keyPressEvent(self, event):
        """Handles key presses and toggles corresponding indicators."""
        # if event.key() in self.toggleIndicators:
        #     self.toggleIndicators[event.key()].toggle()
        if event.key() in self.holdIndicators:
            self.holdIndicators[event.key()].update_indicator(True)  # Turn on while pressed

    def keyReleaseEvent(self, event):
        """Turns off the hold indicator when key is released."""
        if event.key() in self.holdIndicators:
            self.holdIndicators[event.key()].update_indicator(False)  # Turn on while pressed

    # def update_data(self):
    #     x_value, y_value = self.joystick.get_values()
    #     _, z_value = self.joystick2.get_values()
    #     current_time = time.time() - self.start_time

    #     self.x_data.append(x_value)
    #     self.y_data.append(y_value)
    #     # self.yaw_data.append(yaw_value)
    #     self.z_data.append(z_value)
    #     self.time_data.append(current_time)

    #     # Trim data to last `time_window` seconds
    #     while self.time_data and (current_time - self.time_data[0] > self.time_window):
    #         self.x_data.pop(0)
    #         self.y_data.pop(0)
    #         # self.yaw_data.pop(0)
    #         self.z_data.pop(0)
    #         self.time_data.pop(0)

    #     # Trim 2D joystick trail to last `trail_window` seconds
    #     x_trail = []
    #     y_trail = []
    #     for i in range(len(self.time_data)):
    #         if current_time - self.time_data[i] <= self.trail_window:
    #             x_trail.append(self.x_data[i])
    #             y_trail.append(self.y_data[i])

    #     # Update X Graph (rotated 90 degrees anticlockwise)
    #     self.x_curve.setData(self.x_data, self.time_data)  # Swap x and y values
    #     self.x_plot.setYRange(max(0, current_time - self.time_window), current_time)  # Set the time as the Y-axis range

    #     # Update Y Graph
    #     self.y_curve.setData(self.time_data, self.y_data)
    #     self.y_plot.setXRange(max(0, current_time - self.time_window), current_time)

    #     # Update 2D Joystick Position Graph with shorter trail
    #     self.xy_curve.setData(x_trail, y_trail)

    #     # Update Yaw Graph (rotated 90 degrees anticlockwise)
    #     # self.yaw_curve.setData(self.yaw_data, self.time_data)  # Swap x and y values
    #     # self.yaw_plot.setYRange(max(0, current_time - self.time_window), current_time)  # Set the time as the Y-axis range

    #     # Update Z Graph
    #     self.z_curve.setData(self.time_data, self.z_data)
    #     self.z_plot.setXRange(max(0, current_time - self.time_window), current_time)

if __name__ == "__main__":
    rclpy.init()
    app = QApplication(sys.argv)
    
    # Create joystick window
    # joystick_window = JoystickWindow()
    # joystick_window.show()
    
    # joystick2_window = JoystickWindow()
    # joystick2_window.show()

    # Create graph window and pass the joystick instance
    graph_window = JoystickGraph()
    graph_window.show()

    sys.exit(app.exec_())