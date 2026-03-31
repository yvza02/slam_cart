import tkinter as tk
from robot_serial_lib import RobotSerial


class ManualController:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Manual Control")
        self.root.geometry("420x280")
        self.root.resizable(False, False)

        self.speed = 60
        self.min_speed = 10
        self.max_speed = 200
        self.step = 10

        self.keys_pressed = set()
        self.robot = None

        self.status_var = tk.StringVar(value="Connecting...")
        self.speed_var = tk.StringVar(value=f"Speed: {self.speed}")
        self.motion_var = tk.StringVar(value="Motion: STOP")

        self.build_ui()
        self.connect_robot()

        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.root.focus_force()
        self.loop()

    def build_ui(self):
        tk.Label(self.root, text="Manual Robot Control", font=("Arial", 16, "bold")).pack(pady=10)
        tk.Label(self.root, textvariable=self.status_var, font=("Arial", 11)).pack(pady=4)
        tk.Label(self.root, textvariable=self.speed_var, font=("Arial", 13, "bold")).pack(pady=4)
        tk.Label(self.root, textvariable=self.motion_var, font=("Arial", 12, "bold")).pack(pady=6)

        help_text = (
            "W = Forward\n"
            "S = Backward\n"
            "A = Left\n"
            "D = Right\n"
            "W+A = Forward Left\n"
            "W+D = Forward Right\n"
            "S+A = Backward Left\n"
            "S+D = Backward Right\n"
            "Q = Speed Down\n"
            "E = Speed Up\n"
            "SPACE = Stop\n"
            "X = Exit"
        )
        tk.Label(self.root, text=help_text, justify="left", font=("Arial", 10)).pack(pady=8)

    def connect_robot(self):
        try:
            self.robot = RobotSerial(port="/dev/ttyUSB0", baud=115200)
            self.status_var.set("Connected to /dev/ttyUSB0")
        except Exception as e:
            self.status_var.set(f"Connect failed: {e}")
            self.robot = None

    def on_key_press(self, event):
        key = event.keysym.lower()

        if key in ["w", "a", "s", "d"]:
            self.keys_pressed.add(key)

        elif key == "q":
            self.speed = max(self.min_speed, self.speed - self.step)
            self.speed_var.set(f"Speed: {self.speed}")

        elif key == "e":
            self.speed = min(self.max_speed, self.speed + self.step)
            self.speed_var.set(f"Speed: {self.speed}")

        elif key == "space":
            self.keys_pressed.clear()
            self.send_stop()

        elif key == "x":
            self.on_close()

    def on_key_release(self, event):
        key = event.keysym.lower()
        self.keys_pressed.discard(key)

    def loop(self):
        self.apply_motion()
        self.root.after(100, self.loop)

    def apply_motion(self):
        if self.robot is None:
            self.motion_var.set("Motion: NO CONNECTION")
            return

        w = "w" in self.keys_pressed
        a = "a" in self.keys_pressed
        s = "s" in self.keys_pressed
        d = "d" in self.keys_pressed

        try:
            if w and a:
                self.robot.forward_left(self.speed)
                self.motion_var.set(f"Motion: FORWARD-LEFT @ {self.speed}")
            elif w and d:
                self.robot.forward_right(self.speed)
                self.motion_var.set(f"Motion: FORWARD-RIGHT @ {self.speed}")
            elif s and a:
                self.robot.backward_left(self.speed)
                self.motion_var.set(f"Motion: BACKWARD-LEFT @ {self.speed}")
            elif s and d:
                self.robot.backward_right(self.speed)
                self.motion_var.set(f"Motion: BACKWARD-RIGHT @ {self.speed}")
            elif w:
                self.robot.forward(self.speed)
                self.motion_var.set(f"Motion: FORWARD @ {self.speed}")
            elif s:
                self.robot.backward(self.speed)
                self.motion_var.set(f"Motion: BACKWARD @ {self.speed}")
            elif a:
                self.robot.left(self.speed)
                self.motion_var.set(f"Motion: LEFT @ {self.speed}")
            elif d:
                self.robot.right(self.speed)
                self.motion_var.set(f"Motion: RIGHT @ {self.speed}")
            else:
                self.robot.stop()
                self.motion_var.set("Motion: STOP")
        except Exception as e:
            self.motion_var.set(f"Motion Error: {e}")

    def send_stop(self):
        if self.robot is not None:
            try:
                self.robot.stop()
                self.motion_var.set("Motion: STOP")
            except Exception as e:
                self.motion_var.set(f"Motion Error: {e}")

    def on_close(self):
        try:
            if self.robot is not None:
                self.robot.stop()
                self.robot.close()
        except Exception:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ManualController(root)
    root.mainloop()