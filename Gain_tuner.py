#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Int32
import tkinter as tk
import tkinter.font as tkFont
import threading

class GainTuningGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Gain Tuning GUI")
        self.gains = {
            "/Y_Kp": 0.0,
            "/Y_Kd": 0.0,
            "/X_Kp": 0.0,
            "/X_Kd": 0.0,
            "/N_Kp": 0.0,
            "/N_Kd": 0.0,
            "/wpt": 0.0,
            "/U_con": 0.0
        }  # Dictionary to store added topics and gain values
        self.publishers = {}  # Dictionary to store publishers for each topic
        self.running = {}  # Dictionary to track whether each topic is running
        self.lock = threading.Lock()  # Lock for synchronization

        # Set the dimensions of the GUI
        self.root.geometry("400x450")

        custom_font = tkFont.Font(size=16)

        self.topic_label = tk.Label(root, text="Enter Topic Name:", font=custom_font)
        self.topic_label.pack()
        self.topic_entry = tk.Entry(root, font=custom_font)
        self.topic_entry.pack()

        self.message_type_label = tk.Label(root, text="Select Message Type:", font=custom_font)
        self.message_type_label.pack()
        self.message_type_var = tk.StringVar(root)
        self.message_type_var.set("Float64")  # Default message type
        self.message_type_menu = tk.OptionMenu(root, self.message_type_var, "Float64", "Int32")
        self.message_type_menu.pack()

        self.add_topic_button = tk.Button(root, text="Add Topic", command=self.add_topic, font=custom_font)
        self.add_topic_button.pack()

        self.topic_listbox = tk.Listbox(root, selectmode=tk.SINGLE, font=custom_font)
        self.topic_listbox.pack()

        # Create widgets for default topics
        self.default_topics = {
            "/Y_Kp": "Float64",
            "/Y_Kd": "Float64",
            "/X_Kp": "Float64",
            "/X_Kd": "Float64",
            "/N_Kp": "Float64",
            "/N_Kd": "Float64",
            "/wpt": "Float64",
            "/U_con": "Float64"
        }

        for default_topic, message_type in self.default_topics.items():
            self.create_default_gain_widgets(default_topic, message_type)

        self.rate = rospy.Rate(10)  # Set the publication rate to 10 Hz

        # Create a thread for publishing all gains continuously
        self.publish_thread = threading.Thread(target=self.publish_all_gains)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def add_topic(self):
        topic_name = self.topic_entry.get()
        message_type = self.message_type_var.get()
        try:
            if topic_name not in self.gains:
                if message_type == "Float64":
                    self.gains[topic_name] = 0.0
                else:
                    self.gains[topic_name] = 0  # Set to 0 for Int32
                self.topic_listbox.insert(tk.END, topic_name)
                rospy.loginfo("Added Topic: %s (Message Type: %s)", topic_name, message_type)
                self.create_publisher(topic_name, message_type)
                self.create_gain_widgets(topic_name)
            else:
                rospy.loginfo("Topic already exists: %s", topic_name)
        except ValueError:
            rospy.logerr("Invalid Topic Name")

    def create_gain_widgets(self, topic_name):
        frame = tk.Frame(self.root)
        frame.pack()

        custom_font = tkFont.Font(size=16)

        gain_label = tk.Label(frame, text=f"Gain for {topic_name}:", font=custom_font)
        gain_label.pack(side=tk.LEFT)
        gain_entry = tk.Entry(frame, font=custom_font)
        gain_entry.pack(side=tk.LEFT)
        enter_button = tk.Button(frame, text="Enter", command=lambda t=topic_name: self.enter_adjustment(t, gain_entry), font=custom_font)
        enter_button.pack(side=tk.LEFT)

        start_button = tk.Button(frame, text="Start", command=lambda t=topic_name: self.start_topic(t), font=custom_font)
        start_button.pack(side=tk.LEFT)

        stop_button = tk.Button(frame, text="Stop", command=lambda t=topic_name: self.stop_topic(t), font=custom_font)
        stop_button.pack(side=tk.LEFT)

        self.running[topic_name] = False

    def create_default_gain_widgets(self, topic_name, message_type):
        frame = tk.Frame(self.root)
        frame.pack()

        custom_font = tkFont.Font(size=16)

        gain_label = tk.Label(frame, text=f"Gain for {topic_name}:", font=custom_font)
        gain_label.pack(side=tk.LEFT)
        gain_entry = tk.Entry(frame, font=custom_font)
        gain_entry.pack(side=tk.LEFT, padx=10)
        enter_button = tk.Button(frame, text="Enter", command=lambda t=topic_name: self.enter_adjustment(t, gain_entry), font=custom_font)
        enter_button.pack(side=tk.LEFT)

        start_button = tk.Button(frame, text="Start", command=lambda t=topic_name: self.start_topic(t), font=custom_font)
        start_button.pack(side=tk.LEFT)

        stop_button = tk.Button(frame, text="Stop", command=lambda t=topic_name: self.stop_topic(t), font=custom_font)
        stop_button.pack(side=tk.LEFT)

        self.running[topic_name] = False

    def enter_adjustment(self, topic_name, gain_entry):
        try:
            new_gain = float(gain_entry.get())
            with self.lock:
                self.gains[topic_name] = new_gain
                rospy.loginfo("Updated Gain for %s: %f", topic_name, new_gain)
        except ValueError:
            rospy.logerr("Invalid Gain Value")

    def create_publisher(self, topic_name, message_type):
        if message_type == "Float64":
            self.publishers[topic_name] = rospy.Publisher(topic_name, Float64, queue_size=10)
        else:
            self.publishers[topic_name] = rospy.Publisher(topic_name, Int32, queue_size=10)

    def publish_gain(self, topic_name):
        if topic_name in self.publishers:
            with self.lock:
                gain = self.gains[topic_name]
                message_type = self.publishers[topic_name].data_class  # Get the message type
            if message_type == Float64:
                self.publishers[topic_name].publish(gain)
            elif message_type == Int32:
                self.publishers[topic_name].publish(int(gain))
            rospy.loginfo("Published Gain for %s: %s", topic_name, gain)
        else:
            rospy.logerr("Topic '%s' not found", topic_name)

    def start_topic(self, topic_name):
        if topic_name in self.running:
            self.running[topic_name] = True
            rospy.loginfo("Started Topic: %s", topic_name)
        else:
            rospy.logerr("Topic '%s' not found", topic_name)

    def stop_topic(self, topic_name):
        if topic_name in self.running:
            self.running[topic_name] = False
            rospy.loginfo("Stopped Topic: %s", topic_name)
        else:
            rospy.logerr("Topic '%s' not found", topic_name)

    def publish_all_gains(self):
        while not rospy.is_shutdown():
            for topic_name in self.gains.keys():
                if self.running.get(topic_name, False):
                    self.publish_gain(topic_name)
            self.rate.sleep()

def main():
    rospy.init_node('gain_tuning_gui', anonymous=True)
    root = tk.Tk()
    app = GainTuningGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()

