import tkinter as tk
import threading
import time

from robot_serial_lib import RobotSerial
import gyro_lib
import tag_mission_lib


# =========================
# ROBOT SETUP
# =========================
robot = RobotSerial(port="/dev/ttyUSB1")


def fresh_gyro_start():
    try:
        gyro_lib.stop()
    except Exception:
        pass

    time.sleep(0.1)
    gyro_lib.init()
    gyro_lib.calibrate()
    gyro_lib.reset()
    gyro_lib.start()


class MissionGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("MISSION PANEL")
        self.root.geometry("700x500")
        self.root.resizable(False, False)

        self.running = False

        self.status_var = tk.StringVar(value="READY")
        self.info_var = tk.StringVar(value="SELECT MISSION")

        title = tk.Label(
            root,
            text="ROBOT MISSION PANEL",
            font=("Arial", 22, "bold")
        )
        title.pack(pady=20)

        status_label = tk.Label(
            root,
            textvariable=self.status_var,
            font=("Arial", 18, "bold"),
            fg="blue"
        )
        status_label.pack(pady=8)

        info_label = tk.Label(
            root,
            textvariable=self.info_var,
            font=("Arial", 14),
            fg="green"
        )
        info_label.pack(pady=8)

        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=40)

        btn_style = {
            "width": 14,
            "height": 4,
            "font": ("Arial", 22, "bold"),
            "bd": 5
        }

        self.btn_a = tk.Button(
            btn_frame,
            text="A",
            bg="#4CAF50",
            fg="white",
            command=lambda: self.start_mission("A"),
            **btn_style
        )
        self.btn_a.grid(row=0, column=0, padx=20, pady=20)

        self.btn_ab = tk.Button(
            btn_frame,
            text="A|B",
            bg="#2196F3",
            fg="white",
            command=lambda: self.start_mission("A|B"),
            **btn_style
        )
        self.btn_ab.grid(row=0, column=1, padx=20, pady=20)

        self.btn_b = tk.Button(
            btn_frame,
            text="B",
            bg="#FF9800",
            fg="white",
            command=lambda: self.start_mission("B"),
            **btn_style
        )
        self.btn_b.grid(row=1, column=0, columnspan=2, padx=20, pady=20)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def set_buttons_state(self, state):
        self.btn_a.config(state=state)
        self.btn_ab.config(state=state)
        self.btn_b.config(state=state)

    def start_mission(self, mode):
        if self.running:
            return

        self.running = True
        self.set_buttons_state("disabled")
        self.status_var.set("RUNNING")

        th = threading.Thread(target=self.run_sequence, args=(mode,), daemon=True)
        th.start()

    def run_sequence(self, mode):
        try:
            if mode == "A":
                # TARGET 1 -> wait 30 sec -> TARGET 0
                sequence = [1, 0]
                delays = [30]

            elif mode == "A|B":
                # TARGET 1 -> wait 30 sec -> TARGET 2 -> wait 30 sec -> TARGET 0
                sequence = [1, 2, 0]
                delays = [30, 30]

            elif mode == "B":
                # TARGET 2 -> wait 30 sec -> TARGET 0
                sequence = [2, 0]
                delays = [30]

            else:
                sequence = []
                delays = []

            for i, target_id in enumerate(sequence):
                self.update_info(f"PREPARING GYRO FOR TARGET ID #{target_id}")
                fresh_gyro_start()

                self.update_info(f"MOVING TO TARGET ID #{target_id}")
                tag_mission_lib.run_tag_mission(robot, target_id=target_id)

                if i < len(delays):
                    self.countdown(delays[i])

            self.update_status("DONE")
            self.update_info("MISSION FINISHED")

        except Exception as e:
            self.update_status("ERROR")
            self.update_info(str(e))
            print("Mission error:", e)

        finally:
            try:
                robot.stop()
            except Exception:
                pass

            self.running = False
            self.root.after(0, lambda: self.set_buttons_state("normal"))

    def countdown(self, seconds):
        for remain in range(seconds, 0, -1):
            self.update_info(f"WAITING {remain} SECONDS")
            time.sleep(1)

    def update_status(self, text):
        self.root.after(0, lambda: self.status_var.set(text))

    def update_info(self, text):
        self.root.after(0, lambda: self.info_var.set(text))

    def on_close(self):
        try:
            robot.stop()
        except Exception:
            pass

        try:
            gyro_lib.stop()
        except Exception:
            pass

        try:
            robot.close()
        except Exception:
            pass

        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = MissionGUI(root)
    root.mainloop()