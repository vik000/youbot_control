import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget,
                             QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QSlider, QRadioButton,
                             QLabel, QGroupBox)

from api import sim
from api.youbot import YouBot, BotSpeed, AngularSpeed
from gui.video_frame import VideoFeedWidget

youbot = YouBot()
SPEED = BotSpeed.SLOW.value
DEFAULT_ANGULAR_SPEED = AngularSpeed.SLOW.value
DURATION = 1


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)

        # Video feed placeholder
        error, camera_handle = sim.simxGetObjectHandle(youbot.client_id, f"/{youbot.name}/botcam", sim.simx_opmode_oneshot_wait)
        if error:
            raise TypeError(str(error))

        video_widget = VideoFeedWidget(youbot.client_id, camera_handle)
        video_widget.setMinimumSize(512, 512)
        layout.addWidget(video_widget)

        # Side panel
        side_panel_layout = QVBoxLayout()

        # Control group box (for radio buttons and sliders)
        control_group = QGroupBox('Controls')
        control_group_layout = QVBoxLayout(control_group)

        # Radio buttons
        radio_button_1 = QRadioButton('Option 1')
        radio_button_2 = QRadioButton('Option 2')
        control_group_layout.addWidget(radio_button_1)
        control_group_layout.addWidget(radio_button_2)

        # Sliders
        slider_1 = QSlider()
        slider_2 = QSlider()
        slider_3 = QSlider()
        control_group_layout.addWidget(slider_1)
        control_group_layout.addWidget(slider_2)
        control_group_layout.addWidget(slider_3)

        side_panel_layout.addWidget(control_group)

        # Button grid
        button_grid = QGridLayout()
        self.left_button = QPushButton('Left')
        self.right_button = QPushButton('Right')
        self.forward_button = QPushButton('Forward')
        self.backward_button = QPushButton('Backward')
        button_grid.addWidget(self.left_button, 1, 0)
        button_grid.addWidget(self.forward_button, 0, 1)
        button_grid.addWidget(self.right_button, 1, 2)
        button_grid.addWidget(self.backward_button, 1, 1)
        self.setup_controls()

        # Add button grid to the side panel
        side_panel_layout.addLayout(button_grid)

        # Add side panel to the main layout
        layout.addLayout(side_panel_layout)

        # Set the window title and show the main window
        self.setWindowTitle('Robot Control Dashboard')
        self.show()

    def setup_controls(self):
        # Setup control buttons with event handlers
        self.left_button.clicked.connect(self.move_left)
        self.forward_button.clicked.connect(self.move_forward)
        self.right_button.clicked.connect(self.move_right)
        self.backward_button.clicked.connect(self.move_backward)

    def move_left(self):
        print("Moving left")
        youbot.set_rotation_in_place(-15, DEFAULT_ANGULAR_SPEED)

    def move_forward(self):
        print("Moving forward")
        youbot.move(SPEED, DURATION)

    def move_right(self):
        print("Moving right")
        youbot.set_rotation_in_place(15, DEFAULT_ANGULAR_SPEED)

    def move_backward(self):
        print("Moving backward")
        youbot.move(-SPEED, DURATION)

    def stop(self):
        print("Stopping")
        # Code to stop the robot
        youbot.stop()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    sys.exit(app.exec_())
