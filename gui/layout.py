import flet as ft
from flet import Column, Container, Row, border_radius

from gui.constants import BACKGROUND_COLOUR
from api.youbot import YouBot, BotSpeed, AngularSpeed


youbot = YouBot()
SPEED = BotSpeed.SLOW.value
DEFAULT_ANGULAR_SPEED = AngularSpeed.SLOW.value
DURATION = 1


def move_forward():
    print("Moving forward")
    # Here, you would add the code to make the robot move forward
    youbot.move(SPEED, DURATION)


def move_backward():
    print("Moving backward")
    # Code for moving the robot backward
    youbot.move(-SPEED, DURATION)


def turn_left():
    print("Turning left")
    # Code for turning the robot left
    youbot.set_rotation_in_place(-15, DEFAULT_ANGULAR_SPEED)


def turn_right():
    print("Turning right")
    # Code for turning the robot right
    youbot.set_rotation_in_place(15, DEFAULT_ANGULAR_SPEED)


def stop():
    print("Stopping")
    # Code to stop the robot
    youbot.stop()


def build_page(page: ft.Page):
    page.title = "YoutBot control"

    # Create icon buttons for each direction
    forward_btn = ft.IconButton(icon=ft.icons.ARROW_UPWARD, on_click=lambda _: move_forward(), tooltip="Move Forward")
    backward_btn = ft.IconButton(icon=ft.icons.ARROW_DOWNWARD, on_click=lambda _: move_backward(), tooltip="Move "
                                                                                                           "Backward")
    left_btn = ft.IconButton(icon=ft.icons.ARROW_BACK, on_click=lambda _: turn_left(), tooltip="Turn Left")
    right_btn = ft.IconButton(icon=ft.icons.ARROW_FORWARD, on_click=lambda _: turn_right(), tooltip="Turn Right")

    # layout structure
    # -viewport:
    viewport = Column(expand=5, controls=[Container(expand=1)])

    # -sidebar:
    sd_top_row = Row([Container(expand=1)], expand=4)
    sd_md_row = Row([Container(expand=1)], expand=1)
    sd_bot_row = Row([
                        Column(controls=[Container(expand=1), left_btn], expand=1),
                        Column(controls=[forward_btn, backward_btn], expand=1),
                        Column(controls=[Container(expand=1), right_btn], expand=1),
                    ], expand=1)
    sidebar = Column(expand=1, controls=[sd_top_row, sd_md_row, sd_bot_row])

    # Create layout
    page.add(
        Container(
            width=1200,
            height=600,
            bgcolor=BACKGROUND_COLOUR,
            border_radius=border_radius.all(0),
            padding=20,
            content=Row([viewport, sidebar]),
        )
    )


def run():
    ft.app(target=build_page)


# Run the Flet application
if __name__ == "__main__":
    run()
