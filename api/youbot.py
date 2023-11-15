class Youbot:
    def __init__(self) -> None:
        self.name = "youBot"
        self.left_wheel = f"/{self.name}/rollingJoint_rl"
        self.right_wheel = f"/{self.name}/rollingJoint_rr"
        self.back_axis = f"/{self.name}/rollingJoint_rr"
        self.arm = f"/{self.name}/{self.name}ArmJoint0"

YOUBOT = Youbot()