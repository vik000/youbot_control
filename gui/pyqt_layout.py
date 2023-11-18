import sys

import cv2
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget,
                             QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QSlider, QRadioButton,
                             QLabel, QGroupBox, QFileDialog, QLineEdit)
from PyQt5.QtCore import Qt

from api import sim
from api.youbot import YouBot, BotSpeed, AngularSpeed, Revolute, Grip
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

        self.video_widget = VideoFeedWidget(youbot.client_id, camera_handle)
        self.video_widget.setMinimumSize(512, 512)
        layout.addWidget(self.video_widget)

        # Side panel
        side_panel_layout = QVBoxLayout()

        # Control group box (for radio buttons and sliders)
        control_group = QGroupBox('Arm Controls')
        control_group_layout = QVBoxLayout(control_group)

        # Add a save button
        self.save_button = QPushButton('Save Image')
        self.save_button.clicked.connect(self.save_image)
        side_panel_layout.addWidget(self.save_button)

        # Arm controls
        self.arm_sliders = []
        SCALE_FACTOR = 100
        for joint in youbot.arm.joints:
            if isinstance(joint, Revolute):
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(int(joint.range[0] * SCALE_FACTOR))
                slider.setMaximum(int(joint.range[1] * SCALE_FACTOR))
                slider.setValue(int(joint.current_position * SCALE_FACTOR))

                # Create labels for min, max, and current value
                min_label = QLabel(f"Min: {joint.range[0]} deg")
                max_label = QLabel(f"Max: {joint.range[1]} deg")
                current_label = QLabel(f"Current: {joint.current_position} deg")

                slider.valueChanged.connect(lambda value, j=joint: j.move(value / SCALE_FACTOR))
                slider.valueChanged.connect(lambda value, lbl=current_label: lbl.setText(f"Current: {value} deg"))

                # add widgets to sidebar section:
                control_group_layout.addWidget(min_label)
                control_group_layout.addWidget(slider)
                control_group_layout.addWidget(max_label)
                control_group_layout.addWidget(current_label)

                self.arm_sliders.append(slider)
            else:
                assert isinstance(joint, Grip)
                self.grip = joint
                # Grip Control Section
                grip_layout = QVBoxLayout()
                self.grip_input = QLineEdit()
                self.grip_input.setPlaceholderText("Enter grip distance")
                self.grip_button = QPushButton("Activate Grip")
                self.grip_button.clicked.connect(self.__on_grip_button_clicked)
                self.grip_label_0 = QLabel("Grip0: 0")
                self.grip_label_1 = QLabel("Grip1: 0")

                # Setup grip layout
                grip_layout.addWidget(QLabel("Grip Size:"))
                grip_layout.addWidget(self.grip_input)
                grip_layout.addWidget(self.grip_button)
                grip_layout.addWidget(self.grip_label_0)
                grip_layout.addWidget(self.grip_label_1)
                control_group_layout.addLayout(grip_layout)

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

    def save_image(self):
        # Open file dialog to select save path
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(self, "Save Image", "", "Images (*.png *.jpg *.jpeg)",
                                                   options=options)
        if file_path:
            # Assuming self.video_widget has a method to get the current frame
            frame = self.video_widget.get_current_frame()
            if frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite(file_path, frame)

    def __on_grip_button_clicked(self):
        try:
            distance = float(self.grip_input.text())
            self.grip.grip(distance)  # Replace with your grip method call
        except ValueError:
            print("Please enter a valid number for the grip distance")

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


def run():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    sys.exit(app.exec_())


if __name__ == '__main__':
    run()
