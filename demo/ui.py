"""
Robot control UI

Tkinter GUI for robot control
"""

# -*- coding: utf-8 -*-
import sys
import os
import json

# Add parent dir so dobot_sdk imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from threading import Thread
import time
from tkinter import *
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from dobot_sdk import load_robot_config
from dobot_sdk import CoordinateType
from dobot_sdk.api.error_controller import (
  get_error,
  format_error_messages_from_http,
  SUPPORTED_LANGUAGES
)
from hrc.perception.skins import SKIN_PARTS
from hrc.robot import CS3ASRobot

LABEL_JOINT = [["J1-", "J2-", "J3-", "J4-", "J5-", "J6-"],
        ["J1:", "J2:", "J3:", "J4:", "J5:", "J6:"],
        ["J1+", "J2+", "J3+", "J4+", "J5+", "J6+"]]

LABEL_COORD = [["X-", "Y-", "Z-", "Rx-", "Ry-", "Rz-"],
        ["X:", "Y:", "Z:", "Rx:", "Ry:", "Rz:"],
        ["X+", "Y+", "Z+", "Rx+", "Ry+", "Rz+"]]

LABEL_ROBOT_MODE = {
  0: "ROBOT_MODE_UNKNOWN",
  1: "ROBOT_MODE_INIT",
  2: "ROBOT_MODE_BRAKE_OPEN",
  3: "ROBOT_MODE_POWEROFF",
  4: "ROBOT_MODE_DISABLED",
  5: "ROBOT_MODE_ENABLE",
  6: "ROBOT_MODE_BACKDRIVE",
  7: "ROBOT_MODE_RUNNING",
  8: "ROBOT_MODE_SINGLE_MOVE",
  9: "ROBOT_MODE_ERROR",
  10: "ROBOT_MODE_PAUSE",
  11: "ROBOT_MODE_JOG"
}


class RobotUI(object):

  def __init__(self, robot_ip=None, di_count=24, do_count=24):
    self.root = Tk()
    self.root.title("CS3AS HRC — arm, SafeSkin, gripper")
    self.root.geometry("920x1040")
    self.robot_ip = robot_ip or load_robot_config().ip
    self.di_count = di_count
    self.do_count = do_count
    # Current language
    self.current_lang = 'en'

    # Error-text cache to avoid redundant refresh
    self.last_error_ids = None
    self.last_error_message = None

    # global state dict
    self.global_state = {}

    # all button
    self.button_list = []

    # all entry
    self.entry_dict = {}

    # Robot Connect
    self.frame_robot = LabelFrame(self.root, text="Robot Connect",
                   labelanchor="nw", bg="#FFFFFF", width=870, height=100, border=2)

    # First row (vertically centered)
    row_y = 0.35
    
    # IP Address (left)
    self.label_ip = Label(self.frame_robot, text="IP Address:")
    self.label_ip.place(rely=row_y, x=15, anchor=W)
    ip_port = StringVar(self.root, value="192.168.5.1")
    self.entry_ip = Entry(self.frame_robot, width=16)
    self.entry_ip.place(rely=row_y, x=95, anchor=W)
    self.entry_ip.insert(0, self.robot_ip)

    # Dashboard port
    self.label_dash = Label(self.frame_robot, text="Dashboard:")
    self.label_dash.place(rely=row_y, x=220, anchor=W)
    dash_port = IntVar(self.root, value=29999)
    self.entry_dash = Entry(self.frame_robot, width=6, textvariable=dash_port)
    self.entry_dash.place(rely=row_y, x=300, anchor=W)

    # Feedback port
    self.label_fb = Label(self.frame_robot, text="Feedback:")
    self.label_fb.place(rely=row_y, x=380, anchor=W)
    self.combo_feedback = ttk.Combobox(self.frame_robot, width=12)
    self.combo_feedback["value"] = ["30004 (8ms)", "30005 (200ms)", "30006 (1000ms)"]
    self.combo_feedback.current(0)
    self.combo_feedback["state"] = "readonly"
    self.combo_feedback.place(rely=row_y, x=455, anchor=W)

    # Language (right-center)
    self.label_lang = Label(self.frame_robot, text="Lang:")
    self.label_lang.place(rely=row_y, x=590, anchor=W)
    self.combo_lang = ttk.Combobox(self.frame_robot, width=12)
    self.combo_lang["value"] = [lang[0] for lang in SUPPORTED_LANGUAGES]
    self.combo_lang.current(0)
    self.combo_lang["state"] = "readonly"
    self.combo_lang.place(rely=row_y, x=640, anchor=W)
    self.combo_lang.bind("<<ComboboxSelected>>", self.on_lang_change)

    # Connect / Disconnect (right)
    self.button_connect = self.set_button(master=self.frame_robot,
                       text="Connect", rely=row_y, x=780, anchor=W, command=self.connect_port)
    self.button_connect["width"] = 8
    self.global_state["connect"] = False

    # Dashboard controls, centered
    self.frame_dashboard = LabelFrame(self.root, text="Dashboard Function",
                     labelanchor="nw", bg="#FFFFFF", pady=10, width=870, height=80, border=2)

    # vertically centered
    row_y = 0.5

    # Enable/Disable
    self.button_enable = self.set_button(master=self.frame_dashboard,
                       text="Enable", rely=row_y, x=150, anchor=CENTER, command=self.enable)
    self.button_enable["width"] = 7
    self.global_state["enable"] = False

    self.set_button(master=self.frame_dashboard,
            text="ClearError", rely=row_y, x=280, anchor=CENTER, command=self.clear_error)

    # Speed Ratio
    self.label_speed = Label(self.frame_dashboard, text="Speed Ratio:")
    self.label_speed.place(rely=row_y, x=410, anchor=CENTER)

    s_value = StringVar(self.root, value="50")
    self.entry_speed = Entry(self.frame_dashboard,
                 width=6, textvariable=s_value)
    self.entry_speed.place(rely=row_y, x=500, anchor=CENTER)
    self.label_cent = Label(self.frame_dashboard, text="%")
    self.label_cent.place(rely=row_y, x=535, anchor=CENTER)

    self.set_button(master=self.frame_dashboard,
            text="Confirm", rely=row_y, x=590, anchor=CENTER, command=self.confirm_speed)

    self.set_button(master=self.frame_dashboard,
            text="TCP", rely=row_y, x=680, anchor=CENTER, command=self.request_tcp)

    self.set_button(master=self.frame_dashboard,
            text="ResetSafety", rely=row_y, x=790, anchor=CENTER, command=self.reset_safety)

    # Move Function
    self.frame_move = LabelFrame(self.root, text="Move Function", labelanchor="nw",
                   bg="#FFFFFF", width=870, pady=10, height=130, border=2)

    self.set_move(text="X:", label_value=10,
           default_value="600", entry_value=40, rely=0.1, master=self.frame_move)
    self.set_move(text="Y:", label_value=110,
           default_value="-260", entry_value=140, rely=0.1, master=self.frame_move)
    self.set_move(text="Z:", label_value=210,
           default_value="380", entry_value=240, rely=0.1, master=self.frame_move)
    self.set_move(text="Rx:", label_value=310,
           default_value="170", entry_value=340, rely=0.1, master=self.frame_move)
    self.set_move(text="Ry:", label_value=410,
           default_value="12", entry_value=440, rely=0.1, master=self.frame_move)
    self.set_move(text="Rz:", label_value=510,
           default_value="140", entry_value=540, rely=0.1, master=self.frame_move)

    self.set_button(master=self.frame_move, text="MovJ",
            rely=0.05, x=610, command=self.movj)
    self.set_button(master=self.frame_move, text="MovL",
            rely=0.05, x=700, command=self.movl)

    self.set_move(text="J1:", label_value=10,
           default_value="0", entry_value=40, rely=0.5, master=self.frame_move)
    self.set_move(text="J2:", label_value=110,
           default_value="-20", entry_value=140, rely=0.5, master=self.frame_move)
    self.set_move(text="J3:", label_value=210,
           default_value="-80", entry_value=240, rely=0.5, master=self.frame_move)
    self.set_move(text="J4:", label_value=310,
           default_value="30", entry_value=340, rely=0.5, master=self.frame_move)
    self.set_move(text="J5:", label_value=410,
           default_value="90", entry_value=440, rely=0.5, master=self.frame_move)
    self.set_move(text="J6:", label_value=510,
           default_value="120", entry_value=540, rely=0.5, master=self.frame_move)

    self.set_button(master=self.frame_move,
            text="MovJ", rely=0.45, x=610, command=self.joint_movj)

    # Digital IO: three equal rows
    self.frame_io = LabelFrame(self.root, text="Digital IO",
                  labelanchor="nw", bg="#FFFFFF", width=870, height=150, border=2)

    # Row 1: digital-output controls
    row1_y = 1/6 # approx 0.1667
    
    self.label_do = Label(self.frame_io, text="Digital Outputs:")
    self.label_do.place(rely=row1_y, x=15, anchor=W)

    self.label_do_index = Label(self.frame_io, text="Index:")
    self.label_do_index.place(rely=row1_y, x=130, anchor=W)
    i_value = IntVar(self.root, value="1")
    self.entry_index = Entry(self.frame_io, width=5, textvariable=i_value)
    self.entry_index.place(rely=row1_y, x=180, anchor=W)

    self.label_do_status = Label(self.frame_io, text="Status:")
    self.label_do_status.place(rely=row1_y, x=230, anchor=W)
    self.combo_status = ttk.Combobox(self.frame_io, width=5)
    self.combo_status["value"] = ("On", "Off")
    self.combo_status.current(0)
    self.combo_status["state"] = "readonly"
    self.combo_status.place(rely=row1_y, x=285, anchor=W)

    self.button_confirm_do = self.set_button(master=self.frame_io,
                         text="Confirm", rely=row1_y, x=350, anchor=W, command=self.confirm_do)

    # Row 2: 24 DI lamps, aligned with DO
    row2_y = 3/6 # 0.5, center
    
    self.label_di = Label(self.frame_io, text="Digital Inputs:")
    self.label_di.place(rely=row2_y, x=15, anchor=W)
    self.di_indicators = []
    # Single row, aligned start
    for i in range(self.di_count):
      canvas = Canvas(self.frame_io, width=18, height=18, bg="white", highlightthickness=1)
      canvas.place(rely=row2_y, x=130 + i*20, anchor=CENTER)
      canvas.create_oval(2, 2, 16, 16, fill="gray", outline="black")
      canvas.create_text(9, 9, text=str(i+1), fill="white", font=("Arial", 8))
      self.di_indicators.append(canvas)

    # Row 3: 24 DO lamps, aligned with DI
    row3_y = 5/6 # approx 0.8333, bottom
    
    self.label_do_display = Label(self.frame_io, text="Digital Outputs:")
    self.label_do_display.place(rely=row3_y, x=15, anchor=W)
    self.do_indicators = []
    # Single row, aligned with DI
    for i in range(self.do_count):
      canvas = Canvas(self.frame_io, width=18, height=18, bg="white", highlightthickness=1)
      canvas.place(rely=row3_y, x=130 + i*20, anchor=CENTER)
      canvas.create_oval(2, 2, 16, 16, fill="gray", outline="black")
      canvas.create_text(9, 9, text=str(i+1), fill="white", font=("Arial", 8))
      self.do_indicators.append(canvas)

    self.frame_hrc = LabelFrame(
      self.root, text="HRC Cell — SafeSkin + Gripper",
      labelanchor="nw", bg="#FFFFFF", width=870, height=175, border=2)

    self.label_skin_collision = Label(self.frame_hrc, text="Collision: --", fg="#444444")
    self.label_skin_collision.place(rely=0.12, x=15, anchor=W)
    self.label_skin_mode = Label(self.frame_hrc, text="TCP: --", fg="#444444")
    self.label_skin_mode.place(rely=0.12, x=200, anchor=W)

    self.skin_bars = {}
    for i, name in enumerate(SKIN_PARTS):
      x0 = 15 + i * 175
      Label(self.frame_hrc, text=name).place(rely=0.38, x=x0, anchor=W)
      canvas = Canvas(self.frame_hrc, width=90, height=14, bg="#EEEEEE", highlightthickness=1)
      canvas.place(rely=0.38, x=x0 + 62, anchor=W)
      bar = canvas.create_rectangle(0, 0, 0, 14, fill="#4CAF50", outline="")
      value = Label(self.frame_hrc, text="0.00", width=5)
      value.place(rely=0.38, x=x0 + 158, anchor=W)
      self.skin_bars[name] = {"canvas": canvas, "bar": bar, "label": value}

    self.button_skin_on = self.set_button(
      master=self.frame_hrc, text="Skin On", rely=0.68, x=70, anchor=CENTER,
      command=self.skins_enable)
    self.button_skin_off = self.set_button(
      master=self.frame_hrc, text="Skin Off", rely=0.68, x=160, anchor=CENTER,
      command=self.skins_disable)
    self.button_skin_reset = self.set_button(
      master=self.frame_hrc, text="Skin Reset", rely=0.68, x=260, anchor=CENTER,
      command=self.skins_reset)

    self.skin_safety_var = IntVar(self.root, value=0)
    self.check_skin_safety = Checkbutton(
      self.frame_hrc, text="Firmware safety stop", variable=self.skin_safety_var,
      bg="#FFFFFF", command=self.skins_toggle_safety)
    self.check_skin_safety.place(rely=0.68, x=430, anchor=CENTER)
    self.button_list.append(self.check_skin_safety)

    self.label_grip = Label(self.frame_hrc, text="Gripper: disconnected")
    self.label_grip.place(rely=0.90, x=15, anchor=W)

    self.button_grip_connect = self.set_button(
      master=self.frame_hrc, text="Gripper", rely=0.90, x=280, anchor=CENTER,
      command=self.gripper_connect)
    self.button_grip_open = self.set_button(
      master=self.frame_hrc, text="Open", rely=0.90, x=360, anchor=CENTER,
      command=self.gripper_open)
    self.button_grip_close = self.set_button(
      master=self.frame_hrc, text="Close", rely=0.90, x=430, anchor=CENTER,
      command=self.gripper_close)

    self.grip_scale = Scale(
      self.frame_hrc, from_=0, to=1000, orient=HORIZONTAL, length=220,
      showvalue=True, command=None)
    self.grip_scale.set(1000)
    self.grip_scale.place(rely=0.88, x=470, anchor=W)
    self.grip_scale.bind("<ButtonRelease-1>", self.gripper_set_pos)
    self.button_list.append(self.grip_scale)

    self._last_grip_poll = 0.0
    self._tcp_owned = False
    self._pending_safety_reset = False

    self.frame_feed_log = Frame(
      self.root, bg="#FFFFFF", width=870, pady=10, height=430, border=2)

    # Feedback (left)
    self.frame_feed = LabelFrame(self.frame_feed_log, text="Feedback", labelanchor="nw",
                   bg="#FFFFFF")
    self.frame_feed.place(relx=0, rely=0, relheight=1, relwidth=0.6)

    # Speed ratio and robot mode on one row
    self.set_label(self.frame_feed,
            text="Current Speed Ratio:", rely=0.02, x=10)
    self.label_feed_speed = self.set_label(
      self.frame_feed, "", rely=0.02, x=145)
    self.set_label(self.frame_feed, text="%", rely=0.02, x=175)

    self.set_label(self.frame_feed, text="Robot Mode:", rely=0.02, x=220)
    self.label_robot_mode = self.set_label(
      self.frame_feed, "", rely=0.02, x=310)

    # Jog / coordinates: Joint and Coord split equally
    self.label_feed_dict = {}
    
    # Joint left, Coord right
    # Joint panel
    self.set_label(self.frame_feed, text="Joint:", rely=0.10, x=95)
    for i in range(6):
      row_y = 0.2 + i * 0.12
      self.set_button_bind(self.frame_feed, LABEL_JOINT[0][i], rely=row_y, x=10)
      self.set_label(self.frame_feed, LABEL_JOINT[1][i], rely=row_y + 0.01, x=55)
      self.label_feed_dict[LABEL_JOINT[1][i]] = self.set_label(self.frame_feed, " ", rely=row_y + 0.01, x=95)
      self.set_button_bind(self.frame_feed, LABEL_JOINT[2][i], rely=row_y, x=170)

    # Coord panel (right half)
    self.set_label(self.frame_feed, text="Coord:", rely=0.10, x=335)
    self.combo_coord_type = ttk.Combobox(self.frame_feed, width=10)
    self.combo_coord_type["value"] = ["User frame", "Tool frame"]
    self.combo_coord_type.current(0)
    self.combo_coord_type["state"] = "readonly"
    self.combo_coord_type.place(rely=0.10, x=390, anchor=W)
    for i in range(6):
      row_y = 0.2 + i * 0.12
      self.set_button_bind(self.frame_feed, LABEL_COORD[0][i], rely=row_y, x=250)
      self.set_label(self.frame_feed, LABEL_COORD[1][i], rely=row_y + 0.01, x=295)
      self.label_feed_dict[LABEL_COORD[1][i]] = self.set_label(self.frame_feed, " ", rely=row_y + 0.01, x=335)
      self.set_button_bind(self.frame_feed, LABEL_COORD[2][i], rely=row_y, x=400)

    # Error info (upper right)
    self.frame_err = LabelFrame(self.frame_feed_log, text="Error Info", labelanchor="nw",
                  bg="#FFFFFF")
    self.frame_err.place(relx=0.62, rely=0, relheight=0.48, relwidth=0.36)

    self.text_err = ScrolledText(
      self.frame_err, width=170, height=50, relief="flat")
    self.text_err.place(rely=0, relx=0, relheight=0.85, relwidth=1)

    self.set_button(self.frame_err, "Clear", rely=0.88,
            x=100, command=self.clear_error_info)

    # Log (lower right)
    self.frame_log = LabelFrame(self.frame_feed_log, text="Log", labelanchor="nw",
                  bg="#FFFFFF")
    self.frame_log.place(relx=0.62, rely=0.52, relheight=0.46, relwidth=0.36)

    self.text_log = ScrolledText(
      self.frame_log, width=270, height=140, relief="flat")
    self.text_log.place(rely=0, relx=0, relheight=1, relwidth=1)

    # Init robot client
    self.robot = None
    
    # Store current robot mode
    self.current_robot_mode = 0

  def on_lang_change(self, event):
    """Language change handler"""
    idx = self.combo_lang.current()
    self.current_lang = SUPPORTED_LANGUAGES[idx][1]
    self.add_log(f"Language switched to: {SUPPORTED_LANGUAGES[idx][0]}")
    # If connected, sync robot language
    if self.robot and self.global_state["connect"]:
      self.robot.error.set_language(self.current_lang)

  def add_log(self, message):
    """Append a log line"""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    self.text_log.insert(END, f"[{timestamp}] {message}\n")
    self.text_log.see(END)

  def mainloop(self):
    self.root.mainloop()

  def pack(self):
    self.frame_robot.pack()
    self.frame_dashboard.pack()
    self.frame_move.pack()
    self.frame_io.pack()
    self.frame_hrc.pack()
    self.frame_feed_log.pack()

  def set_move(self, text, label_value, default_value, entry_value, rely, master):
    self.label = Label(master, text=text)
    self.label.place(rely=rely, x=label_value)
    value = StringVar(self.root, value=default_value)
    self.entry_temp = Entry(master, width=6, textvariable=value)
    self.entry_temp.place(rely=rely, x=entry_value)
    self.entry_dict[text] = self.entry_temp

  def move_jog(self, text):
    """Jog"""
    if self.global_state["connect"] and self.robot:
      if self.current_robot_mode == 9:
        self.add_log("Robot in error; jog blocked")
        return
      if text and text[0] in ("J", "j"):
        coord_type = CoordinateType.JOINT
      else:
        idx = self.combo_coord_type.current()
        if idx == 1:
          coord_type = CoordinateType.TOOL
        else:
          coord_type = CoordinateType.CARTESIAN
      self.robot.motion.MoveJog(text, coord_type=coord_type)

  def move_stop(self, event):
    """Stop jog"""
    if self.global_state["connect"] and self.robot:
      self.robot.motion.MoveJog("")

  def set_button(self, master, text, rely, x, **kargs):
    self.button = Button(master, text=text, padx=5,
               command=kargs["command"])
    anchor = kargs.get("anchor", NW)
    self.button.place(rely=rely, x=x, anchor=anchor)

    if text != "Connect":
      self.button["state"] = "disable"
      self.button_list.append(self.button)
    return self.button

  def set_button_bind(self, master, text, rely, x, **kargs):
    self.button = Button(master, text=text, padx=5)
    self.button.bind("<ButtonPress-1>",
             lambda event: self.move_jog(text=text))
    self.button.bind("<ButtonRelease-1>", self.move_stop)
    self.button.place(rely=rely, x=x)

    if text != "Connect":
      self.button["state"] = "disable"
      self.button_list.append(self.button)
    return self.button

  def set_label(self, master, text, rely, x):
    self.label = Label(master, text=text)
    self.label.place(rely=rely, x=x)
    return self.label

  def connect_port(self):
    """Connect / disconnect"""
    if self.global_state["connect"]:
      # Disconnect
      try:
        if self.robot:
          try:
            self.robot.shutdown(
              disable=bool(self.global_state.get("enable")),
              skins_off=True,
            )
          except Exception:
            pass
          self.robot.Disconnect()
          self.robot = None
        self.add_log("Disconnected")
      except Exception as e:
        self.add_log(f"Disconnect failed: {e}")

      for i in self.button_list:
        i["state"] = "disable"
      self.button_connect["text"] = "Connect"
      self.global_state["enable"] = False
      self.button_enable["text"] = "Enable"
      self.label_grip["text"] = "Gripper: disconnected"
      self.label_skin_collision["text"] = "Collision: --"
      self.label_skin_mode["text"] = "TCP: --"
      self._tcp_owned = False
    else:
      # Connect
      try:
        ip = self.entry_ip.get()
        dash_port = int(self.entry_dash.get())
        
        # Feedback port from the dropdown
        feedback_selection = self.combo_feedback.get()
        feedback_port = int(feedback_selection.split()[0])
        
        self.add_log(f"Connecting to robot: {ip}")
        self.add_log(f" Dashboard: {dash_port}, Feedback: {feedback_port}")
        
        self.robot = CS3ASRobot.from_config(
          ip=ip,
          dashboard_port=dash_port,
          feedback_port=feedback_port,
        )
        self.robot.Connect()
        
        # Set robot language
        self.robot.error.set_language(self.current_lang)
        
        self.robot.prepare(enable=False, connect_gripper=False)
        if self.robot.cfg.skins.enabled:
          self.robot.skins.enable(firmware_safety=bool(self.skin_safety_var.get()))
        self._tcp_owned = True
        self.root.after(400, self._poll_gripper)
        self.add_log(
          f"Connected  TCP={self.robot.in_tcp_mode()}  "
          f"mode={self.robot.mode_name()}  skins={self.robot.skins.active}"
        )
      except Exception as e:
        messagebox.showerror("Attention!", f"Connection Error:{e}")
        self.add_log(f"Connection failed: {e}")
        return

      for i in self.button_list:
        i["state"] = "normal"
      self.button_connect["text"] = "Disconnect"
    
    self.global_state["connect"] = not self.global_state["connect"]
    self.set_feed_back()

  def set_feed_back(self):
    """Start feedback thread"""
    if self.global_state["connect"]:
      thread = Thread(target=self.feed_back)
      thread.start()

  def enable(self):
    """Enable / disable"""
    if not self.robot:
      return
    
    # Block actions while in error mode
    if self.current_robot_mode == 9:
      self.add_log("Robot in error; clear errors first")
      return
    
    if self.global_state["enable"]:
      try:
        response = self.robot.robot_control.DisableRobot()
        # Check the response for failure
        if response and ("Failed" in str(response) or "Error" in str(response)):
          self.add_log(f"Disable failed: {response}")
          return
        self.button_enable["text"] = "Enable"
        self.add_log("Robot disabled")
        self.global_state["enable"] = False
      except Exception as e:
        self.add_log(f"Disable failed: {e}")
    else:
      try:
        response = self.robot.robot_control.EnableRobot()
        # Check the response for failure
        if response and ("Failed" in str(response) or "Error" in str(response)):
          self.add_log(f"Enable failed: {response}")
          return
        self.button_enable["text"] = "Disable"
        self.add_log("Robot enabled")
        self.global_state["enable"] = True
      except Exception as e:
        self.add_log(f"Enable failed: {e}")

  def clear_error(self):
    """Clear errors"""
    if self.robot:
      try:
        response = self.robot.robot_control.ClearError()
        if response and not str(response).startswith("0,"):
          self.add_log(f"ClearError failed: {response}")
          return
        self.add_log("Errors cleared")
        self.clear_error_info()
      except Exception as e:
        self.add_log(f"ClearError failed: {e}")

  def confirm_speed(self):
    """Set speed scale"""
    if not self.robot:
      return
    
    # Check robot mode
    if self.current_robot_mode == 9:
      self.add_log("Robot in error; cannot set speed")
      return
    
    try:
      speed = int(self.entry_speed.get())
      response = self.robot.robot_control.SpeedFactor(speed)
      if response and ("Failed" in str(response) or "Error" in str(response)):
        self.add_log(f"Set speed failed: {response}")
        return
      self.add_log(f"Speed scale set to: {speed}%")
    except Exception as e:
      self.add_log(f"Set speed failed: {e}")

  def movj(self):
    """Cartesian MovJ"""
    if not self.robot:
      return
    
    # Check robot mode
    if self.current_robot_mode == 9:
      self.add_log("Robot in error; motion blocked")
      return
    
    if not self.global_state["enable"]:
      self.add_log("Robot not enabled; motion blocked")
      return
      
    try:
      x = float(self.entry_dict["X:"].get())
      y = float(self.entry_dict["Y:"].get())
      z = float(self.entry_dict["Z:"].get())
      rx = float(self.entry_dict["Rx:"].get())
      ry = float(self.entry_dict["Ry:"].get())
      rz = float(self.entry_dict["Rz:"].get())
      
      response = self.robot.motion.MovJ([x, y, z, rx, ry, rz], CoordinateType.CARTESIAN)
      if response and ("Failed" in str(response) or "Error" in str(response)):
        self.add_log(f"MovJ failed: {response}")
        return
      self.add_log(f"MovJ: ({x}, {y}, {z}, {rx}, {ry}, {rz})")
    except Exception as e:
      self.add_log(f"MovJ failed: {e}")

  def movl(self):
    """Cartesian MovL"""
    if not self.robot:
      return
    
    # Check robot mode
    if self.current_robot_mode == 9:
      self.add_log("Robot in error; motion blocked")
      return
    
    if not self.global_state["enable"]:
      self.add_log("Robot not enabled; motion blocked")
      return
      
    try:
      x = float(self.entry_dict["X:"].get())
      y = float(self.entry_dict["Y:"].get())
      z = float(self.entry_dict["Z:"].get())
      rx = float(self.entry_dict["Rx:"].get())
      ry = float(self.entry_dict["Ry:"].get())
      rz = float(self.entry_dict["Rz:"].get())
      
      response = self.robot.motion.MovL([x, y, z, rx, ry, rz], CoordinateType.CARTESIAN)
      if response and ("Failed" in str(response) or "Error" in str(response)):
        self.add_log(f"MovL failed: {response}")
        return
      self.add_log(f"MovL: ({x}, {y}, {z}, {rx}, {ry}, {rz})")
    except Exception as e:
      self.add_log(f"MovL failed: {e}")

  def joint_movj(self):
    """Joint MovJ"""
    if not self.robot:
      return
    
    # Check robot mode
    if self.current_robot_mode == 9:
      self.add_log("Robot in error; motion blocked")
      return
    
    if not self.global_state["enable"]:
      self.add_log("Robot not enabled; motion blocked")
      return
      
    try:
      j1 = float(self.entry_dict["J1:"].get())
      j2 = float(self.entry_dict["J2:"].get())
      j3 = float(self.entry_dict["J3:"].get())
      j4 = float(self.entry_dict["J4:"].get())
      j5 = float(self.entry_dict["J5:"].get())
      j6 = float(self.entry_dict["J6:"].get())
      
      response = self.robot.motion.MovJ([j1, j2, j3, j4, j5, j6], CoordinateType.JOINT)
      if response and ("Failed" in str(response) or "Error" in str(response)):
        self.add_log(f"Joint MovJ failed: {response}")
        return
      self.add_log(f"MovJ(Joint): ({j1}, {j2}, {j3}, {j4}, {j5}, {j6})")
    except Exception as e:
      self.add_log(f"Joint MovJ failed: {e}")

  def confirm_do(self):
    """Set digital output"""
    if not self.robot:
      return
    
    # Check robot mode
    if self.current_robot_mode == 9:
      self.add_log("Robot in error; cannot set DO")
      return
      
    try:
      index = int(self.entry_index.get())
      status = 1 if self.combo_status.get() == "On" else 0
      
      if status == 1:
        response = self.robot.io.DO(index, 1)
        if response and ("Failed" in str(response) or "Error" in str(response)):
          self.add_log(f"Set DO{index}failed: {response}")
          return
        self.add_log(f"DO{index} set HIGH")
      else:
        response = self.robot.io.DO(index, 0)
        if response and ("Failed" in str(response) or "Error" in str(response)):
          self.add_log(f"Set DO{index}failed: {response}")
          return
        self.add_log(f"DO{index} set LOW")
    except Exception as e:
      self.add_log(f"Set DOfailed: {e}")

  def set_feed(self, text_list, x1, x2, x3, x4):
    """Build feedback widgets"""
    for i in range(6):
      self.set_button_bind(self.frame_feed, text_list[0][i], rely=0.2 + i*0.1, x=x1)
      self.set_label(self.frame_feed, text_list[1][i], rely=0.21 + i*0.1, x=x2)
      self.label_feed_dict[text_list[1][i]] = self.set_label(self.frame_feed, " ", rely=0.21 + i*0.1, x=x3)
      self.set_button_bind(self.frame_feed, text_list[2][i], rely=0.2 + i*0.1, x=x4)

  def feed_back(self):
    """Feedback loop"""
    while True:
      if not self.global_state["connect"] or not self.robot:
        break

      try:
        status = self.robot.GetStatus()
        if status:
          # Update speed scale
          self.label_feed_speed["text"] = str(status.speed_scaling)
          
          # Update robot mode
          mode_value = status.robot_mode.value
          self.current_robot_mode = mode_value
          self.label_robot_mode["text"] = LABEL_ROBOT_MODE.get(mode_value, f"Unknown({mode_value})")
          
          # Update DI/DO lamps
          di_value = status.digital_inputs
          do_value = status.digital_outputs
          
          for i, indicator in enumerate(self.di_indicators):
            if (di_value >> i) & 1:
              indicator.itemconfig(1, fill="green")
            else:
              indicator.itemconfig(1, fill="gray")
          
          for i, indicator in enumerate(self.do_indicators):
            if (do_value >> i) & 1:
              indicator.itemconfig(1, fill="green")
            else:
              indicator.itemconfig(1, fill="gray")
          
          # Update joint labels
          if hasattr(status, 'joint_state') and status.joint_state:
            q_actual = status.joint_state.q_actual
            if q_actual:
              for i, label in enumerate(["J1:", "J2:", "J3:", "J4:", "J5:", "J6:"]):
                if i < len(q_actual):
                  self.label_feed_dict[label]["text"] = f"{q_actual[i]:.2f}"
          
          # Update Cartesian labels
          if hasattr(status, 'tool_vector_actual') and status.tool_vector_actual:
            tool_vector = status.tool_vector_actual.to_list()
            for i, label in enumerate(["X:", "Y:", "Z:", "Rx:", "Ry:", "Rz:"]):
              if i < len(tool_vector):
                self.label_feed_dict[label]["text"] = f"{tool_vector[i]:.2f}"

          # Show errors whenever present
          if mode_value == 9:
            self.display_error_info()
          else:
            # Also poll leftover errors outside ERROR mode
            self.display_error_info()

          self._update_hrc_panel(status)
      except Exception as e:
        self.add_log(f"Feedback error: {e}")
      
      time.sleep(0.1)

  def display_error_info(self):
    """Show detailed errors via HTTP"""
    if not self.robot:
      return
      
    try:
      # Fetch errors over HTTP
      error_info = get_error(self.robot.ip, self.current_lang)
      
      # Error message list
      err_msg_list = error_info.get("errMsg", [])
      
      # True if there is an error
      if not err_msg_list:
        # Clear the panel when errors are gone
        if self.last_error_message:
          self.text_err.delete("1.0", "end")
          self.last_error_message = None
        return
      
      # Format error text
      error_message = format_error_messages_from_http(error_info)
      
      # Skip refresh if text is unchanged
      if error_message == self.last_error_message:
        return
      
      # Update display
      self.text_err.delete("1.0", "end")
      self.text_err.insert(END, error_message)
      self.text_err.see(END)
      self.last_error_message = error_message
      
      # Log
      error_ids = [err.get("id") for err in err_msg_list]
      self.add_log(f"Error info updated: {error_ids}")
      
    except Exception as e:
      self.add_log(f"Failed to fetch errors: {e}")

  def clear_error_info(self):
    """Clear the error panel"""
    self.text_err.delete("1.0", "end")
    # Reset cache so the next fetch can refresh
    self.last_error_message = None
    self.last_error_ids = None

  def request_tcp(self):
    if not self.robot:
      return
    try:
      mode = self.robot.take_tcp_control()
      self._tcp_owned = True
      self.add_log(f"TCP control: mode={self.robot.mode_name(mode)}")
    except Exception as e:
      self.add_log(f"TCP control failed: {e}")

  def reset_safety(self):
    if not self.robot:
      return
    try:
      mode = self.robot.reset_safety()
      self.add_log(f"Safety reset: mode={self.robot.mode_name(mode)}")
    except Exception as e:
      self.add_log(f"Safety reset failed: {e}")

  def skins_enable(self):
    if not self.robot:
      return
    try:
      self.robot.skins.enable(firmware_safety=bool(self.skin_safety_var.get()))
      self.add_log(
        f"SafeSkin on (firmware_safety={bool(self.skin_safety_var.get())})"
      )
    except Exception as e:
      self.add_log(f"SafeSkin enable failed: {e}")

  def skins_disable(self):
    if not self.robot:
      return
    try:
      self.robot.skins.disable()
      self.robot.reset_safety()
      self.add_log("SafeSkin off")
    except Exception as e:
      self.add_log(f"SafeSkin disable failed: {e}")

  def skins_reset(self):
    if not self.robot:
      return
    try:
      self.robot.skins.reset()
      self.add_log("SafeSkin reset (collision-protect cleared)")
    except Exception as e:
      self.add_log(f"SafeSkin reset failed: {e}")

  def skins_toggle_safety(self):
    if not self.robot or not getattr(self.robot, "skins", None):
      return
    try:
      self.robot.skins.firmware_safety = bool(self.skin_safety_var.get())
      self.robot.skins.auto_clear = not self.robot.skins.firmware_safety
      self.add_log(
        f"SafeSkin firmware_safety={self.robot.skins.firmware_safety}"
      )
    except Exception as e:
      self.add_log(f"SafeSkin safety toggle failed: {e}")

  def gripper_connect(self):
    if not self.robot:
      return
    grip = getattr(self.robot, "gripper", None)
    if grip is None:
      return
    try:
      if grip.connected:
        grip.disconnect()
        self.button_grip_connect["text"] = "Gripper"
        self.label_grip["text"] = "Gripper: disconnected"
        self.add_log("Gripper disconnected")
      else:
        grip.connect()
        self.button_grip_connect["text"] = "GripOff"
        self.add_log("Gripper connected")
    except Exception as e:
      self.add_log(f"Gripper connect failed: {e}")

  def gripper_open(self):
    if not self.robot or not getattr(self.robot.gripper, "connected", False):
      self.add_log("Gripper not connected")
      return
    try:
      self.robot.gripper.open()
      self.grip_scale.set(int(self.robot.cfg.gripper.open_position))
      self.add_log("Gripper open")
    except Exception as e:
      self.add_log(f"Gripper open failed: {e}")

  def gripper_close(self):
    if not self.robot or not getattr(self.robot.gripper, "connected", False):
      self.add_log("Gripper not connected")
      return
    try:
      self.robot.gripper.close()
      self.grip_scale.set(int(self.robot.cfg.gripper.close_position))
      self.add_log("Gripper close")
    except Exception as e:
      self.add_log(f"Gripper close failed: {e}")

  def gripper_set_pos(self, event=None):
    if not self.robot or not getattr(self.robot.gripper, "connected", False):
      return
    try:
      pos = int(self.grip_scale.get())
      self.robot.gripper.set_position(pos, wait=False)
      self.add_log(f"Gripper position {pos}")
    except Exception as e:
      self.add_log(f"Gripper position failed: {e}")

  def _update_hrc_panel(self, status):
    if not self.robot:
      return
    collision = int(getattr(status, "collision_state", 0) or 0)
    paused = self.current_robot_mode in (9, 10)
    self.label_skin_collision["text"] = (
      f"Collision: {'YES' if collision else 'no'}"
    )
    self.label_skin_collision["fg"] = "#C62828" if collision else "#2E7D32"
    self.label_skin_mode["text"] = (
      f"TCP: {'yes' if self._tcp_owned else 'NO'}  "
      f"{LABEL_ROBOT_MODE.get(self.current_robot_mode, '')}"
    )

    reading = None
    try:
      reading = self.robot.skins.read(status, clear=False)
    except Exception:
      reading = None
    if reading is not None:
      for i, name in enumerate(SKIN_PARTS):
        widgets = self.skin_bars.get(name)
        if not widgets:
          continue
        prox = float(reading.proximity[i]) if i < reading.proximity.size else 0.0
        approach = float(reading.approach[i]) if i < reading.approach.size else 0.0
        width = max(1, int(90 * min(1.0, max(0.0, prox))))
        color = "#C62828" if abs(approach) > 0 or prox >= 0.85 else "#FB8C00" if prox >= 0.35 else "#43A047"
        widgets["canvas"].coords(widgets["bar"], 0, 0, width, 14)
        widgets["canvas"].itemconfig(widgets["bar"], fill=color)
        widgets["label"]["text"] = f"{prox:.2f}"

    skins = getattr(self.robot, "skins", None)
    if (
      skins is not None
      and skins.auto_clear
      and not skins.firmware_safety
      and (collision or paused)
      and not self._pending_safety_reset
    ):
      self._pending_safety_reset = True
      self.root.after(0, self._auto_reset_safety)

  def _poll_gripper(self):
    if not self.global_state.get("connect") or not self.robot:
      return
    grip = getattr(self.robot, "gripper", None)
    if grip is None:
      return
    if not grip.connected:
      self.label_grip["text"] = "Gripper: disconnected"
      self.button_grip_connect["text"] = "Gripper"
      self.root.after(500, self._poll_gripper)
      return
    try:
      info = grip.as_dict()
      pos = info.get("position")
      opening = info.get("opening_mm")
      state = info.get("grip_state_name")
      pos_txt = "--" if pos is None else f"{int(pos)}"
      open_txt = "--" if opening is None else f"{opening:.1f}mm"
      self.label_grip["text"] = f"Gripper: {pos_txt} / {open_txt}  {state}"
      self.button_grip_connect["text"] = "GripOff"
    except Exception:
      self.label_grip["text"] = "Gripper: error"
    self.root.after(500, self._poll_gripper)

  def _auto_reset_safety(self):
    self._pending_safety_reset = False
    if not self.robot:
      return
    try:
      self.robot.recover_if_protected()
    except Exception as e:
      self.add_log(f"Auto safety reset failed: {e}")