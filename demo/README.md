# Demo programs

IP and ports are read from `config/robot.yaml` (default `192.168.5.1`).

```
demo/
├── DobotDemo.py      # control helper
├── main.py           # CLI demo
├── ui.py             # Tk GUI
├── main_UI.py        # GUI entry
├── z_Cnv_test.py     # conveyor tracking
├── z_servo_test.py   # ServoP tracking
└── README.md
```

```bash
pip install -r requirements.txt
cd demo
python main.py
python main_UI.py
```

Put the robot in TCP mode first. Keep the e-stop reachable for motion demos.
