import cv2
import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QWidget, QLabel

from api import sim


class VideoFeedWidget(QWidget):
    def __init__(self, client_id, camera_handle):
        super().__init__()
        self.client_id = client_id
        self.camera_handle = camera_handle
        self.initUI()
        # Start streaming camera data
        error, res, im = sim.simxGetVisionSensorImage(self.client_id, self.camera_handle, 0, sim.simx_opmode_streaming)

    def initUI(self):
        self.image_label = QLabel(self)
        self.image_label.resize(800, 500)  # Set this to your desired dimensions

        # Setup a timer to update the image_label with the camera feed
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(100)  # Refresh every 100 ms

    def update_image(self):
        # Fetch the image from CoppeliaSim and update the image_label
        errorCode, resolution, image = sim.simxGetVisionSensorImage(self.client_id, self.camera_handle, 0, sim.simx_opmode_buffer)
        if errorCode == sim.simx_return_ok:
            image = np.array(image, dtype=np.uint8)
            image.resize([resolution[1], resolution[0], 3])  # Note the order of resolution indices
            image = np.flip(image, 0)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            qt_image = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(pixmap)
        elif errorCode == sim.simx_return_novalue_flag:
            # No data yet, ignore or add a retry mechanism
            pass
        else:
            print("Error fetching image data: ", errorCode)
