import tkinter as tk
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Int32, Float32, Float32MultiArray  # Import Float32 message type
import math

class CompassGUI:
    def __init__(self, root, canvas_width=800, canvas_height=800, font_size=20):
        self.root = root
        self.root.title("Compass GUI")

        # Initialize ROS node
        rospy.init_node('compass_gui', anonymous=True)

        # Create frame for left side (USV Flag and Ship ID)
        left_frame = tk.Frame(root)
        left_frame.pack(side="left", padx=20, pady=40)

        self.line_length = 350
        self.ship_distance = 0.0
        self.rel_x = 0.0
        self.rel_y = 0.0
        self.ship_id = 0.0
        self.arrow_thickness = 8.0
        self.usv_flag = 0

        # Create label for /usv_flag and variable for its value
        self.usv_flag_label = tk.Label(left_frame, text="USV Flag : ")
        self.usv_flag_label.grid(row=0, column=0, sticky="w")  # Place label at the top left
        self.usv_flag_var = tk.StringVar()
        self.usv_flag_var.set("N/A")
        self.usv_flag_value = tk.Label(left_frame, textvariable=self.usv_flag_var, anchor="w")
        self.usv_flag_value.grid(row=0, column=1, sticky="w")  # Place value to the right

        # Create label for Ship ID and variable for its value
        self.ship_id_label = tk.Label(left_frame, text="  Ship ID   : ")
        self.ship_id_label.grid(row=1, column=0, sticky="w")  # Place label below USV Flag
        self.ship_id_var = tk.StringVar()
        self.ship_id_var.set("N/A")
        self.ship_id_value = tk.Label(left_frame, textvariable=self.ship_id_var, anchor="w")
        self.ship_id_value.grid(row=1, column=1, sticky="w")  # Place value below Ship ID label

        # Create label for Ship distance and variable for its value
        self.ship_distance_label = tk.Label(left_frame, text="Ship Dis   : ")
        self.ship_distance_label.grid(row=2, column=0, sticky="w")  # Place label below USV Flag
        self.ship_distance_var = tk.StringVar()
        self.ship_distance_var.set("N/A")
        self.ship_distance_value = tk.Label(left_frame, textvariable=self.ship_distance_var, anchor="w")
        self.ship_distance_value.grid(row=2, column=1, sticky="w")  # Place value below Ship ID label

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
        self.ship_distance_label.config(font=("Helvetica", self.font_size))
        self.ship_distance_value.config(font=("Helvetica", self.font_size))

        # Subscribe to ROS topics
        rospy.Subscriber("/imu/data", Imu, self.update_own_heading)
        rospy.Subscriber("/desired_heading", String, self.update_desired_heading)
        rospy.Subscriber("/usv_flag", Int32, self.update_usv_flag)  # Subscribe to /usv_flag
        rospy.Subscriber("/wp_id", Float32, self.update_ship_id)  # Subscribe to /wp_id
        rospy.Subscriber('/lidar_track', Float32MultiArray, self.sub_local_xy)
        rospy.Subscriber('/global_track', Float32MultiArray, self.sub_global_xy)

        # Initialize headings and Ship_ID
        self.own_heading = 0.0
        self.desired_heading = "North"  # Initialize with a cardinal direction
        self.usv_flag = None
        self.ship_id = None
        self.ship_heading = 0.0
        self.ship_direction = 0.0
        self.usv_x = 0.0
        self.usv_y = 0.0
        self.target_x = 0.0
        self.target_y =0.0

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

    def sub_local_xy(self,msg):
        if msg is not None:
            if len(msg.data) > 3:

                N = int(msg.data[0])
                for i in range(N):
                    if self.ship_id == msg.data[1+8*i]:
                        self.rel_x = msg.data[1+8*i+1]
                        self.rel_y = msg.data[1+8*i+2]
                        self.ship_heading = msg.data[1+8*i+5]

                        if 120 > math.sqrt(self.rel_x*self.rel_x + self.rel_y*self.rel_y):
                            self.ship_distance = math.sqrt(self.rel_x*self.rel_x + self.rel_y*self.rel_y)
                            self.ship_distance_var.set(str(round(self.ship_distance,1)))
                        
                        self.update_compass()

    def sub_global_xy(self,msg):
        if msg is not None:
            if len(msg.data) > 4:

                N = int(msg.data[2])
                for i in range(N):
                    if self.ship_id == msg.data[3+5*i]:
                        self.target_x = msg.data[3+5*i+1]
                        self.target_y = msg.data[3+5*i+2]
                        self.usv_x = msg.data[0]
                        self.usv_y = msg.data[1]
                        self.ship_direction = math.atan2(self.target_y - self.usv_y, self.target_x - self.usv_x)
                     
                        if 120 < math.sqrt((self.target_x - self.usv_x)**2 + (self.target_y - self.usv_y)**2):
                            self.ship_distance = math.sqrt((self.target_x - self.usv_x)**2 + (self.target_y - self.usv_y)**2)
                            self.ship_distance_var.set(str(round(self.ship_distance,1)))

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
        own_heading_x = cx + self.line_length * math.cos(0.5*math.pi)
        own_heading_y = cy - self.line_length * math.sin(0.5*math.pi)

        # Draw the compass circle
        self.canvas.create_oval(50, 50, self.canvas_width - 50, self.canvas_height - 50)

        for angle in range(12):
            off_x = cx + self.line_length * math.cos(0.5*math.pi + (math.pi/6)*angle)
            off_y = cy - self.line_length * math.sin(0.5*math.pi + (math.pi/6)*angle)
            self.canvas.create_line(cx, cy, off_x, off_y,fill="#888888")

        # Calculate the angle difference between desired and own headings
        desired_heading_angle = self.angle_map.get(self.desired_heading, 0.0)
        ship_heading_angle = self.angle_map.get(self.ship_heading, 0.0)
        ship_direction_angle = self.angle_map.get(self.ship_direction, 0.0)
        angle_difference = desired_heading_angle - self.own_heading

        # Draw the rotated own heading arrow
        self.canvas.create_line(cx, cy, own_heading_x, own_heading_y, fill="blue", width=self.arrow_thickness)

        if self.usv_flag is not None:
            if self.usv_flag == 3 or self.usv_flag == 6:
                ship_direction_x = cx + self.line_length * math.cos(0.5*math.pi + self.ship_direction + rotation_angle)
                ship_direction_y = cy - self.line_length * math.sin(0.5*math.pi + self.ship_direction + rotation_angle)
                self.canvas.create_line(cx, cy, ship_direction_x, ship_direction_y, fill="green",width=self.arrow_thickness)

        # Draw the ship heading
        if self.usv_flag is not None:
            if self.usv_flag == 4 or self.usv_flag == 7 or self.usv_flag == 5 :
                ship_heading_x = cx + self.line_length * math.cos(0.5*math.pi + self.ship_heading+rotation_angle)
                ship_heading_y = cy - self.line_length * math.sin(0.5*math.pi + self.ship_heading+rotation_angle)
                self.canvas.create_line(cx, cy, ship_heading_x, ship_heading_y, fill="green",width=self.arrow_thickness)
                ship_heading_x = cx - self.line_length * math.cos(0.5*math.pi + self.ship_heading+rotation_angle)
                ship_heading_y = cy + self.line_length * math.sin(0.5*math.pi + self.ship_heading+rotation_angle)
                self.canvas.create_line(cx, cy, ship_heading_x, ship_heading_y, fill="green",width=self.arrow_thickness)

if __name__ == "__main__":
    root = tk.Tk()
    app = CompassGUI(root, canvas_width=800, canvas_height=800, font_size=20)
    root.mainloop()
