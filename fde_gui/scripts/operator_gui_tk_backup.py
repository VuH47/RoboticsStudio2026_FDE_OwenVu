#!/usr/bin/env python3

import tkinter as tk
import customtkinter as ctk

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


# ---------------------------------------------------------
# ROS INTERFACE
# ---------------------------------------------------------

class OperatorRosNode(Node):

    def __init__(self):
        super().__init__("fde_operator_gui")

        self.mission_state = "IDLE"
        self.latest_detection = None

        # GUI -> ROS
        self.command_pub = self.create_publisher(
            String,
            "/fde/mission_command",
            10
        )

        # ROS -> GUI
        self.create_subscription(
            String,
            "/fde/mission_state",
            self.mission_state_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            "/fde/detection",
            self.detection_callback,
            10
        )

        self.get_logger().info("Operator GUI ROS interface started.")

    def mission_state_callback(self, msg):
        self.mission_state = msg.data

    def detection_callback(self, msg):
        self.latest_detection = msg

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        self.get_logger().info(
            f"Mission command sent: {command}"
        )


# ---------------------------------------------------------
# GUI
# ---------------------------------------------------------

class OperatorGUI(ctk.CTk):

    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node

        # -------------------------
        # Window
        # -------------------------

        self.title("FDE Alpine Hazard Inspection")
        self.geometry("1400x760")
        self.minsize(1200, 700)

        ctk.set_appearance_mode("dark")

        self.configure(fg_color="#3c4c63")

        # Main layout:
        # LEFT = Map
        # RIGHT = Operations / hazards

        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        self.grid_rowconfigure(0, weight=1)

        self.create_left_panel()
        self.create_right_panel()

        # Start polling ROS
        self.after(20, self.process_ros)

        self.protocol("WM_DELETE_WINDOW", self.close_app)

    # =====================================================
    # LEFT SIDE
    # =====================================================

    def create_left_panel(self):

        self.left_frame = ctk.CTkFrame(
            self,
            corner_radius=0,
            fg_color="#3c4c63"
        )

        self.left_frame.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(8, 4),
            pady=8
        )

        self.left_frame.grid_rowconfigure(1, weight=1)
        self.left_frame.grid_columnconfigure(0, weight=1)

        # -------------------------------------
        # Header / mission stages
        # -------------------------------------

        header = ctk.CTkFrame(
            self.left_frame,
            fg_color="transparent"
        )

        header.grid(
            row=0,
            column=0,
            sticky="ew"
        )

        title = ctk.CTkLabel(
            header,
            text="Mission Interface\nMap",
            font=ctk.CTkFont(size=24),
            justify="left"
        )

        title.pack(
            side="left",
            padx=10,
            pady=5
        )

        mission_state_frame = ctk.CTkFrame(
            header,
            fg_color="transparent"
        )

        mission_state_frame.pack(
            side="right",
            padx=10
        )

        stages = [
            "User\nWaypoints",
            "Drone\nScanning",
            "Manual\nInspection",
            "Husky\nIntervention",
            "Mission\nComplete"
        ]

        for stage in stages:

            ctk.CTkLabel(
                mission_state_frame,
                text=stage,
                width=70,
                font=ctk.CTkFont(size=11)
            ).pack(
                side="left",
                padx=3
            )

        # -------------------------------------
        # Map
        # -------------------------------------

        self.map_canvas = tk.Canvas(
            self.left_frame,
            bg="white",
            highlightthickness=2,
            highlightbackground="#111111"
        )

        self.map_canvas.grid(
            row=1,
            column=0,
            sticky="nsew",
            padx=8,
            pady=4
        )

        # Temporary text until we put your map in
        self.map_canvas.create_text(
            300,
            200,
            text="MISSION MAP",
            fill="black",
            font=("Arial", 24)
        )

        # -------------------------------------
        # Robot map selector
        # -------------------------------------

        self.map_robot_selector = ctk.CTkSegmentedButton(
            self.left_frame,
            values=[
                "Drone",
                "Husky",
                "Both"
            ]
        )

        self.map_robot_selector.set("Drone")

        self.map_robot_selector.grid(
            row=2,
            column=0,
            sticky="ew",
            padx=8,
            pady=4
        )

        # -------------------------------------
        # Mission outcomes/objectives
        # -------------------------------------

        lower = ctk.CTkFrame(
            self.left_frame,
            fg_color="transparent"
        )

        lower.grid(
            row=3,
            column=0,
            sticky="ew",
            padx=8
        )

        lower.grid_columnconfigure(0, weight=1)
        lower.grid_columnconfigure(1, weight=1)

        outcomes = ctk.CTkFrame(lower)

        outcomes.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(0, 5)
        )

        ctk.CTkLabel(
            outcomes,
            text="Mission Outcomes",
            font=ctk.CTkFont(size=19)
        ).pack(anchor="w", padx=10)

        self.outcomes_label = ctk.CTkLabel(
            outcomes,
            justify="left",
            text=(
                "Detected Hazards:        0\n"
                "Awaiting Husky:          0\n"
                "Resolved / Safe:         0\n"
                "Confirmed Hazards:       0"
            )
        )

        self.outcomes_label.pack(
            anchor="w",
            padx=10,
            pady=5
        )

        objectives = ctk.CTkFrame(lower)

        objectives.grid(
            row=0,
            column=1,
            sticky="nsew"
        )

        ctk.CTkLabel(
            objectives,
            text="Mission Objectives",
            font=ctk.CTkFont(size=19)
        ).pack(anchor="w", padx=10)

        self.objective_label = ctk.CTkLabel(
            objectives,
            text="Waypoints reached: 0/10"
        )

        self.objective_label.pack(
            anchor="w",
            padx=10,
            pady=5
        )

        self.progress_bar = ctk.CTkProgressBar(
            objectives
        )

        self.progress_bar.pack(
            fill="x",
            padx=10,
            pady=5
        )

        self.progress_bar.set(0)

        # -------------------------------------
        # Mission buttons
        # -------------------------------------

        buttons = ctk.CTkFrame(
            self.left_frame,
            fg_color="transparent"
        )

        buttons.grid(
            row=4,
            column=0,
            sticky="ew",
            padx=8,
            pady=5
        )

        buttons.grid_columnconfigure(0, weight=1)
        buttons.grid_columnconfigure(1, weight=1)

        self.start_button = ctk.CTkButton(
            buttons,
            text="Start Mission",
            fg_color="#3eaa27",
            height=40,
            font=ctk.CTkFont(size=20),
            command=lambda:
                self.ros_node.send_command("start")
        )

        self.start_button.grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 5)
        )

        self.abort_button = ctk.CTkButton(
            buttons,
            text="Return Home / Abort",
            fg_color="red",
            hover_color="#aa0000",
            height=40,
            font=ctk.CTkFont(size=20),
            command=lambda:
                self.ros_node.send_command("abort")
        )

        self.abort_button.grid(
            row=0,
            column=1,
            sticky="ew"
        )

    # =====================================================
    # RIGHT SIDE
    # =====================================================

    def create_right_panel(self):

        self.right_frame = ctk.CTkFrame(
            self,
            corner_radius=0,
            fg_color="#273b55"
        )

        self.right_frame.grid(
            row=0,
            column=1,
            sticky="nsew",
            padx=(4, 8),
            pady=8
        )

        self.right_frame.grid_rowconfigure(1, weight=1)
        self.right_frame.grid_columnconfigure(0, weight=1)

        # -------------------------------------
        # Main tabs
        # -------------------------------------

        self.main_tabs = ctk.CTkSegmentedButton(
            self.right_frame,
            values=[
                "Live Operations",
                "Documented Hazards"
            ],
            command=self.switch_main_view,
            height=42,
            font=ctk.CTkFont(size=20)
        )

        self.main_tabs.set("Live Operations")

        self.main_tabs.grid(
            row=0,
            column=0,
            sticky="ew",
            padx=8,
            pady=6
        )

        # Container for both screens
        self.content = ctk.CTkFrame(
            self.right_frame,
            fg_color="transparent"
        )

        self.content.grid(
            row=1,
            column=0,
            sticky="nsew"
        )

        self.content.grid_rowconfigure(0, weight=1)
        self.content.grid_columnconfigure(0, weight=1)

        self.create_live_operations()
        self.create_documented_hazards()

        self.show_live_operations()

    # =====================================================
    # LIVE OPERATIONS
    # =====================================================

    def create_live_operations(self):

        self.live_frame = ctk.CTkFrame(
            self.content,
            fg_color="transparent"
        )

        self.live_frame.grid_rowconfigure(1, weight=3)
        self.live_frame.grid_rowconfigure(2, weight=1)
        self.live_frame.grid_columnconfigure(0, weight=1)

        self.camera_selector = ctk.CTkSegmentedButton(
            self.live_frame,
            values=[
                "Drone",
                "Husky",
                "Both"
            ]
        )

        self.camera_selector.set("Drone")

        self.camera_selector.grid(
            row=0,
            column=0,
            sticky="ew",
            padx=8,
            pady=5
        )

        # Camera
        self.camera_frame = ctk.CTkFrame(
            self.live_frame,
            fg_color="white"
        )

        self.camera_frame.grid(
            row=1,
            column=0,
            sticky="nsew",
            padx=8,
            pady=5
        )

        self.camera_frame.grid_rowconfigure(0, weight=1)
        self.camera_frame.grid_columnconfigure(0, weight=1)

        self.camera_label = ctk.CTkLabel(
            self.camera_frame,
            text="Robot POV",
            text_color="black",
            font=ctk.CTkFont(size=24)
        )

        self.camera_label.grid(
            row=0,
            column=0
        )

        # Action Log
        log_frame = ctk.CTkFrame(
            self.live_frame
        )

        log_frame.grid(
            row=2,
            column=0,
            sticky="nsew",
            padx=8,
            pady=5
        )

        ctk.CTkLabel(
            log_frame,
            text="Robot Action Log",
            font=ctk.CTkFont(size=19)
        ).pack(anchor="w", padx=10)

        self.action_log = ctk.CTkTextbox(
            log_frame,
            height=120
        )

        self.action_log.pack(
            fill="both",
            expand=True,
            padx=10,
            pady=5
        )

        self.action_log.insert(
            "end",
            "Waiting for mission...\n"
        )

        self.manual_button = ctk.CTkButton(
            self.live_frame,
            text="Manual Control",
            height=36
        )

        self.manual_button.grid(
            row=3,
            column=0,
            sticky="ew",
            padx=8,
            pady=5
        )

    # =====================================================
    # DOCUMENTED HAZARDS
    # =====================================================

    def create_documented_hazards(self):

        self.hazards_frame = ctk.CTkFrame(
            self.content,
            fg_color="transparent"
        )

        self.hazards_frame.grid_columnconfigure(
            0,
            weight=1
        )

        self.hazards_frame.grid_columnconfigure(
            1,
            weight=2
        )

        self.hazards_frame.grid_rowconfigure(
            0,
            weight=1
        )

        # -------------------------------------
        # Hazard List
        # -------------------------------------

        hazard_list = ctk.CTkFrame(
            self.hazards_frame
        )

        hazard_list.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(8, 4),
            pady=5
        )

        ctk.CTkLabel(
            hazard_list,
            text="Hazard List",
            font=ctk.CTkFont(size=20)
        ).pack(
            anchor="w",
            padx=10,
            pady=5
        )

        self.status_filter = ctk.CTkComboBox(
            hazard_list,
            values=[
                "All Statuses",
                "Detected",
                "Husky Required",
                "In Progress",
                "Resolved / Safe",
                "Confirmed Hazard"
            ]
        )

        self.status_filter.set("All Statuses")

        self.status_filter.pack(
            fill="x",
            padx=10,
            pady=4
        )

        self.type_filter = ctk.CTkComboBox(
            hazard_list,
            values=[
                "All Types",
                "Slippery Ice",
                "No / Low Snow",
                "Wood Debris",
                "Stone Debris"
            ]
        )

        self.type_filter.set("All Types")

        self.type_filter.pack(
            fill="x",
            padx=10,
            pady=4
        )

        self.hazard_list_container = (
            ctk.CTkScrollableFrame(
                hazard_list
            )
        )

        self.hazard_list_container.pack(
            fill="both",
            expand=True,
            padx=10,
            pady=10
        )

        # Dummy hazard
        self.add_dummy_hazard()

        # -------------------------------------
        # Hazard Details
        # -------------------------------------

        self.hazard_details = ctk.CTkFrame(
            self.hazards_frame
        )

        self.hazard_details.grid(
            row=0,
            column=1,
            sticky="nsew",
            padx=(4, 8),
            pady=5
        )

        self.hazard_title = ctk.CTkLabel(
            self.hazard_details,
            text="Select a hazard",
            font=ctk.CTkFont(
                size=22,
                weight="bold"
            )
        )

        self.hazard_title.pack(
            anchor="w",
            padx=12,
            pady=10
        )

        self.hazard_info = ctk.CTkLabel(
            self.hazard_details,
            text=(
                "Detected by: —\n"
                "Time Detected: —\n"
                "Husky Assignment: —\n"
                "Coordinates: —"
            ),
            justify="left"
        )

        self.hazard_info.pack(
            anchor="w",
            padx=12,
            pady=10
        )

        self.notes = ctk.CTkTextbox(
            self.hazard_details,
            height=120
        )

        self.notes.pack(
            fill="x",
            padx=12,
            pady=10
        )

    def add_dummy_hazard(self):

        button = ctk.CTkButton(
            self.hazard_list_container,
            text=(
                "H-02\n"
                "Slippery Ice\n"
                "Husky Required"
            ),
            height=80,
            command=self.select_dummy_hazard
        )

        button.pack(
            fill="x",
            pady=5
        )

    def select_dummy_hazard(self):

        self.hazard_title.configure(
            text="H-02    Slippery Ice"
        )

        self.hazard_info.configure(
            text=(
                "Status: Husky Required\n\n"
                "Detected by: Drone\n"
                "Time Detected: 12 Sep 2026 14:32\n"
                "Husky Assignment: Queued\n"
                "Coordinates: X 30 Y 50 Z 20"
            )
        )

    # =====================================================
    # PAGE SWITCHING
    # =====================================================

    def switch_main_view(self, value):

        if value == "Live Operations":
            self.show_live_operations()

        else:
            self.show_documented_hazards()

    def show_live_operations(self):

        self.hazards_frame.grid_forget()

        self.live_frame.grid(
            row=0,
            column=0,
            sticky="nsew"
        )

    def show_documented_hazards(self):

        self.live_frame.grid_forget()

        self.hazards_frame.grid(
            row=0,
            column=0,
            sticky="nsew"
        )

    # =====================================================
    # ROS PROCESSING
    # =====================================================

    def process_ros(self):

        # Process pending ROS callbacks without blocking Tk
        rclpy.spin_once(
            self.ros_node,
            timeout_sec=0.0
        )

        # Example: update title based on mission state
        state = self.ros_node.mission_state

        self.title(
            f"FDE Alpine Hazard Inspection — {state}"
        )

        # Check again in 20 ms
        self.after(
            20,
            self.process_ros
        )

    # =====================================================
    # CLOSE
    # =====================================================

    def close_app(self):

        self.ros_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        self.destroy()


# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    ros_node = OperatorRosNode()

    app = OperatorGUI(ros_node)

    app.mainloop()


if __name__ == "__main__":
    main()