import xarm
import tkinter as tk
from InverseKinematics import XArmIK  # Importing the class with forward kinematics

class GUI:
    def __init__(self):
        # Connect to robot
        self.robot = self.connect_to_robot()
        if self.robot is None:
            print("Exiting Program")
            exit()
        print(self.robot)

        # Initialize FK/IK solver
        self.ik_solver = XArmIK()

        # Create the main window
        self.mw = tk.Tk()
        self.mw.geometry('500x500')

        # Labels for joint angles
        self.label_gripper = tk.Label(self.mw, text="Gripper")
        self.label_link2 = tk.Label(self.mw, text="Link 2")
        self.label_link3 = tk.Label(self.mw, text="Link 3")
        self.label_link4 = tk.Label(self.mw, text="Link 4")
        self.label_link5 = tk.Label(self.mw, text="Link 5")
        self.label_link6 = tk.Label(self.mw, text="Link 6")

        # Sliders for joint angles
        self.scale_gripper = tk.Scale(self.mw, from_=-125, to=125, orient=tk.HORIZONTAL, command=self.send_val)
        self.scale_link2 = tk.Scale(self.mw, from_=-125, to=125, orient=tk.HORIZONTAL, command=self.send_val)
        self.scale_link3 = tk.Scale(self.mw, from_=-125, to=125, orient=tk.HORIZONTAL, command=self.send_val)
        self.scale_link4 = tk.Scale(self.mw, from_=-125, to=125, orient=tk.HORIZONTAL, command=self.send_val)
        self.scale_link5 = tk.Scale(self.mw, from_=-125, to=125, orient=tk.HORIZONTAL, command=self.send_val)
        self.scale_link6 = tk.Scale(self.mw, from_=-125, to=125, orient=tk.HORIZONTAL, command=self.send_val)

        # Setting gripper to be vertical in start position
        self.scale_gripper.set(float(0))
        self.scale_link2.set(float(0))
        self.scale_link3.set(float(0))
        self.scale_link4.set(float(0))
        self.scale_link5.set(float(0))
        self.scale_link6.set(float(0))

        # Place labels, sliders, and buttons
        self.place_widgets()

        # Main loop
        tk.mainloop()

    def place_widgets(self):
        # Place labels and sliders
        self.label_gripper.place(x=10, y=10)
        self.label_link2.place(x=10, y=70)
        self.label_link3.place(x=10, y=130)
        self.label_link4.place(x=10, y=190)
        self.label_link5.place(x=10, y=250)
        self.label_link6.place(x=10, y=310)

        self.scale_gripper.place(x=100, y=0)
        self.scale_link2.place(x=100, y=60)
        self.scale_link3.place(x=100, y=120)
        self.scale_link4.place(x=100, y=180)
        self.scale_link5.place(x=100, y=240)
        self.scale_link6.place(x=100, y=300)

        # Add Increment and Decrement Buttons for each joint
        joints = [
            ("Gripper", self.scale_gripper, 10),
            ("Link 2", self.scale_link2, 70),
            ("Link 3", self.scale_link3, 130),
            ("Link 4", self.scale_link4, 190),
            ("Link 5", self.scale_link5, 250),
            ("Link 6", self.scale_link6, 310),
        ]

        for joint_name, slider, y_position in joints:
            tk.Button(self.mw, text=f"+1 {joint_name}", command=lambda s=slider: self.increment_slider(s, 1)).place(x=300, y=y_position)
            tk.Button(self.mw, text=f"-1 {joint_name}", command=lambda s=slider: self.increment_slider(s, -1)).place(x=400, y=y_position)

        # Add Reset and Print Buttons
        tk.Button(self.mw, text="Print Angles", command=self.print_position).place(x=150, y=400)
        tk.Button(self.mw, text="Reset to Neutral", command=self.reset_to_neutral).place(x=250, y=400)

    def increment_slider(self, slider, increment):
        # Increment or decrement the slider value
        new_value = slider.get() + increment
        if -125 <= new_value <= 125:  # Ensure the value stays within bounds
            slider.set(new_value)
            self.send_val(None)  # Update the robot with the new value

    def send_val(self, evt):
        # Get values from sliders and send to the robot
        gripper_val = self.scale_gripper.get()
        link2_val = self.scale_link2.get()
        link3_val = self.scale_link3.get()
        link4_val = self.scale_link4.get()
        link5_val = self.scale_link5.get()
        link6_val = self.scale_link6.get()

        self.robot.setPosition(1, float(gripper_val), wait=False)
        self.robot.setPosition(2, float(link2_val), wait=False)
        self.robot.setPosition(3, float(link3_val), wait=False)
        self.robot.setPosition(4, float(link4_val), wait=False)
        self.robot.setPosition(5, float(link5_val), wait=False)
        self.robot.setPosition(6, float(link6_val), wait=False)

    def print_position(self):
        # Get current angles from sliders
        joint_angles = [
            self.scale_gripper.get(),
            self.scale_link2.get(),
            self.scale_link3.get(),
            self.scale_link4.get(),
            self.scale_link5.get(),
            self.scale_link6.get(),
        ]

        # Print the joint angles
        print(f"Joint Angles: [ {joint_angles[0]}.0 , {joint_angles[1]}.0 , {joint_angles[2]}.0, "
              f"{joint_angles[3]}.0, {joint_angles[4]}.0, {joint_angles[5]}.0 ]")

    def reset_to_neutral(self):
        # Reset all sliders to 0
        self.scale_gripper.set(0)
        self.scale_link2.set(0)
        self.scale_link3.set(0)
        self.scale_link4.set(0)
        self.scale_link5.set(0)
        self.scale_link6.set(0)

        # Set the robot joints to 0 degrees
        self.robot.setPosition(1, float(0), wait=False)
        self.robot.setPosition(2, float(0), wait=False)
        self.robot.setPosition(3, float(0), wait=False)
        self.robot.setPosition(4, float(0), wait=False)
        self.robot.setPosition(5, float(0), wait=False)
        self.robot.setPosition(6, float(0), wait=False)
        print("Robot reset to neutral position.")

    def connect_to_robot(self):
        try:
            arm = xarm.Controller('USB')
            return arm
        except Exception as e:
            print(f"Problem Connecting to the arm: {e}")
            return None


if __name__ == '__main__':
    gui = GUI()
