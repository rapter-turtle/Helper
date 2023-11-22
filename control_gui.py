import tkinter as tk
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Int32, Float32  # Import Float32 message type
import math

class CompassGUI:
    def __init__(self, root, canvas_width=400, canvas_height=400, font_size=12):
        self.root = root
        self.root.title("Compass GUI")

        # Initialize ROS node
        rospy.init_node('compass_gui', anonymous=True)

        # Create frame for left side (USV Flag and Ship ID)
        left_frame = tk.Frame(root)
        left_frame.pack(side="left", padx=10, pady=10)

        # Create label and variable for /usv_flag
        self.usv_flag_label = tk.Label(left_frame, text="USV Flag:")
        self.usv_flag_label.pack(side="top", anchor="w")  # Place label at the top left
        self.usv_flag_var = tk.StringVar()
        self.usv_flag_var.set("N/A")
        self.usv_flag_value = tk.Label(left_frame, textvariable=self.usv_flag_var, anchor="w")
        self.usv_flag_value.pack(side="top", fill="x", expand=True, anchor="w")  # Place value to the right, filling the width

        # Create label and variable for Ship_ID
        self.ship_id_label = tk.Label(left_frame, text="Ship ID:")
        self.ship_id_label.pack(side="top", anchor="w")  # Place label below USV Flag
        self.ship_id_var = tk.StringVar()
        self.ship_id_var.set("N/A")
        self.ship_id_value = tk.Label(left_frame, textvariable=self.ship_id_var, anchor="w")
        self.ship_id_value.pack(side="top", anchor="w")  # Place value below Ship ID label

        # Create frame for right side (Compass)
        right_frame = tk.Frame(root)
        right_frame.pack(side="right", padx=10, pady=10, fill="both", expand=True)

        # Create canvas for compass display
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.canvas = tk.Canvas(right_frame, width=self.canvas_width, height=self.canvas_height)
        self.canvas.pack(side="top", fill="both", expand=True)  # Fill and expand the right side

        # Set font size for labels
        self.font_size = font_size
        self.usv_flag_label.config(font=("Helvetica", self.font_size))
        self.usv_flag_value.config(font=("Helvetica", self.font_size))
        self.ship_id_label.config(font=("Helvetica", self.font_size))
        self.ship_id_value.config(font=("Helvetica", self.font_size))

        # Subscribe to ROS topics
        rospy.Subscriber("/imu/data", Imu, self.update_own_heading)
        rospy.Subscriber("/desired_heading", String, self.update_desired_heading)
        rospy.Subscriber("/usv_flag", Int32, self.update_usv_flag)  # Subscribe to /usv_flag
        rospy.Subscriber("/wp_id", Float32, self.update_ship_id)  # Subscribe to /wp_id

        # Initialize headings and Ship_ID
        self.own_heading = 0.0
        self.desired_heading = "North"  # Initialize with a cardinal direction
        self.usv_flag = None
        self.ship_id = None

        # Define angle mappings for cardinal directions
        self.angle_map = {
            "North": 0.0,
            "East": math.pi / 2,
            "South": math.pi,
            "West": 3 * math.pi / 2
        }

        # Update the compass display and monitors
        self.update_compass()
        self.update_usv_flag(None)  # Initialize with None
        self.update_ship_id(None)  # Initialize with None

    def update_own_heading(self, msg):
        # Extract orientation data from the Imu message
        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Calculate heading from the quaternion orientation
        roll, pitch, yaw = self.quaternion_to_euler(quaternion)

        # Use yaw (in radians) as own heading
        self.own_heading = yaw

        self.update_compass()

    def update_desired_heading(self, msg):
        # Use the received cardinal direction to update the desired heading
        self.desired_heading = msg.data

        self.update_compass()

    def update_usv_flag(self, msg):
        # Update the /usv_flag monitor with the received value
        if msg is not None:  # Check for None to handle the initial None value
            self.usv_flag = msg.data
            self.usv_flag_var.set(str(self.usv_flag))

    def update_ship_id(self, msg):
        # Update the Ship_ID monitor with the received value
        if msg is not None:  # Check for None to handle the initial None value
            self.ship_id = msg.data
            self.ship_id_var.set(str(self.ship_id))

    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z

    def update_compass(self):
        # Clear the compass canvas
        self.canvas.delete("all")

        # Calculate the rotation angle to align the own heading with the top direction
        rotation_angle = -self.own_heading

        # Calculate the coordinates after rotation
        cx = self.canvas_width / 2  # Center X
        cy = self.canvas_height / 2  # Center Y
        own_heading_x = cx + 40 * math.cos(0.5*math.pi)
        own_heading_y = cy - 40 * math.sin(0.5*math.pi)

        # Draw the compass circle
        self.canvas.create_oval(50, 50, self.canvas_width - 50, self.canvas_height - 50)

        # Draw the rotated own heading arrow
        self.canvas.create_line(cx, cy, own_heading_x, own_heading_y, fill="blue")

        # Calculate the angle difference between desired and own headings
        desired_heading_angle = self.angle_map.get(self.desired_heading, 0.0)
        angle_difference = desired_heading_angle - self.own_heading

        # Draw the desired heading arrow based on the angle difference
        desired_heading_x = cx + 40 * math.cos(0.5*math.pi + angle_difference)
        desired_heading_y = cy - 40 * math.sin(0.5*math.pi + angle_difference)
        self.canvas.create_line(cx, cy, desired_heading_x, desired_heading_y, fill="red")

if __name__ == "__main__":
    root = tk.Tk()
    app = CompassGUI(root, canvas_width=400, canvas_height=400, font_size=16)
    root.mainloop()
