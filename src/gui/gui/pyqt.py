import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel,
                             QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QGraphicsRectItem, 
                             QSpacerItem, QSizePolicy)
from PyQt5.QtGui import QIcon, QFont, QPixmap, QPalette, QColor, QPainter, QImage
from PyQt5.QtCore import Qt, QRectF, QRect, QThread, pyqtSignal


margin = 10
camera_width, camera_height = 900, 600
label_height = 30
info_width, info_height = 300, 450

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SAFMC GUI")
        # self.setGeometry(0, 0, 3 * margin + 2 * camera_width, 1150)

        #Labels
        self.label_fwd_cam = QLabel("    Forward Cam", self)
        self.label_dwd_cam = QLabel("    Downward Cam", self)

        self.label_fwd_cam.setGeometry(margin, margin, camera_width, label_height)
        self.label_dwd_cam.setGeometry(2 * margin + camera_width, margin, camera_width, label_height)

        self.label_fwd_cam.setFixedHeight(label_height)
        self.label_dwd_cam.setFixedHeight(label_height)

        self.label_fwd_cam.setStyleSheet("color: #F0F1F1;"
                                         "background-color: #242424;")
        self.label_dwd_cam.setStyleSheet("color: #F0F1F1;"
                                         "background-color: #242424;")
        
        self.label_fwd_cam.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        self.label_dwd_cam.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        self.label_spacer = QLabel(self)
        self.label_spacer.setGeometry(margin, margin + label_height + camera_height, info_width, label_height)
        self.label_spacer.setFixedHeight(margin)

        self.label_UAV = QLabel("    UAV Info", self)
        self.label_UAV.setGeometry(margin, 2 * margin + label_height + camera_height, info_width, label_height)
        self.label_UAV.setFixedSize(info_width, label_height)
        self.label_UAV.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        self.label_UAV.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        self.label_payload = QLabel("    Payload", self)
        self.label_payload.setGeometry(2 * margin + info_width, 2 * margin + label_height + camera_height, info_height // 6 * 10, label_height)
        self.label_payload.setFixedSize(info_height // 6 * 10, label_height)
        self.label_payload.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        self.label_payload.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        self.label_ctrl = QLabel("    Controllers", self)
        self.label_ctrl.setGeometry(3 * margin + info_width // 6 * 16, 2 * margin + label_height + camera_height, info_height // 6 * 10, label_height)
        self.label_ctrl.setFixedSize(info_height // 6 * 10, label_height)
        self.label_ctrl.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        self.label_payload.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        #Cameras
        self.pic_fwd_cam = QLabel(self)
        self.pic_dwd_cam = QLabel(self)
        
        self.pic_fwd_cam.setGeometry(margin, margin + label_height, camera_width, camera_height)
        self.pic_dwd_cam.setGeometry(2 * margin + camera_width, margin + label_height, camera_width, camera_height)

        pixmap_fwd_cam = QPixmap("fwd_cam_fake.jpg")        
        pixmap_dwd_cam = QPixmap("dwd_cam_fake.jpg")

        self.pic_fwd_cam.setPixmap(pixmap_fwd_cam.scaled(self.pic_fwd_cam.size(), aspectRatioMode=1))
        self.pic_dwd_cam.setPixmap(pixmap_dwd_cam.scaled(self.pic_dwd_cam.size(), aspectRatioMode=1))

        self.pic_fwd_cam.setFixedSize(camera_width, camera_height)
        self.pic_dwd_cam.setFixedSize(camera_width, camera_height)

        self.pic_fwd_cam.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self.pic_dwd_cam.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        
        self.pic_fwd_cam.setStyleSheet("""
            border: 20px solid #242424;
        """)

        self.pic_dwd_cam.setStyleSheet("""
            border: 20px solid #242424;
        """)

        # UAV Info Widget
        self.info_UAV = QLabel("    Armed : ARMED\n    Battery : 96%\n    Flight Mode : GUIDED\n\n    Pitch : 0.6\n    Roll : -0.3\n    Yaw : 359\n    Altitude : 1.6m\n", self)
        self.info_UAV.setGeometry(margin, 2 * margin + 2 * label_height + camera_height, info_width, info_height)
        self.info_UAV.setFont(QFont("Arial", 10))
        self.info_UAV.setFixedSize(info_width, info_height)
        self.info_UAV.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        self.info_UAV.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)


        # UAV
        self.pic_drone = QLabel(self)
        self.pic_drone.setGeometry(margin, margin + label_height, info_height // 6 * 10, info_height)
        pixmap_drone = QPixmap("drone.jpg")        
        self.pic_drone.setPixmap(pixmap_drone.scaled(self.pic_drone.size(), aspectRatioMode=1))
        self.pic_drone.setFixedSize(info_height // 6 * 10, info_height)
        self.pic_drone.setAlignment(Qt.AlignHCenter | Qt.AlignTop)

        # Controllers
        self.pic_ctrl = QLabel(self)
        self.pic_ctrl.setGeometry(margin, margin + label_height, info_height // 6 * 10, info_height)
        pixmap_ctrl = QPixmap("controllers.jpg")        
        self.pic_ctrl.setPixmap(pixmap_ctrl.scaled(self.pic_ctrl.size(), aspectRatioMode=1))
        self.pic_ctrl.setFixedSize(info_height // 6 * 10, info_height)
        self.pic_ctrl.setAlignment(Qt.AlignHCenter | Qt.AlignTop)

        # General
        self.setStyleSheet("background-color: #353535;")
        self.initUI()

        # ROS2 node
        self.ros2_sub = ROS2
    
    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        grid1 = QGridLayout()
        grid1.setSpacing(0)

        grid2 = QGridLayout()
        grid2.setSpacing(0)

        vbox = QVBoxLayout()

        
        spacer = QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding)

        # create label objects here
        grid1.addWidget(self.label_fwd_cam, 0, 0)
        grid1.addItem(spacer, 0, 1)
        grid1.addWidget(self.label_dwd_cam, 0, 2)

        grid1.addWidget(self.pic_fwd_cam, 1, 0)
        grid1.addItem(spacer, 1, 1)
        grid1.addWidget(self.pic_dwd_cam, 1, 2)

        # grid.addWidget(self.label_spacer, 2, 0)  # Span across columns 0-2

        grid2.addWidget(self.label_UAV, 0, 0)
        grid2.addWidget(self.info_UAV, 1, 0)

        grid1.addItem(spacer, 0, 1)
        grid1.addItem(spacer, 1, 1)

        grid2.addWidget(self.label_payload, 0, 2)
        grid2.addWidget(self.pic_drone, 1, 2)

        grid1.addItem(spacer, 0, 3)
        grid1.addItem(spacer, 1, 3)

        grid2.addWidget(self.label_ctrl, 0, 4)
        grid2.addWidget(self.pic_ctrl, 1, 4)

        vbox.addLayout(grid1)
        vbox.addLayout(grid2)

        central_widget.setLayout(vbox)
    
    def update_image(self, cv_image):
        # Convert OpenCV image to QImage
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_image)
        self.pic_fwd_cam(pixmap)

