# Examples

Function-grouped Dobot SDK samples. Robot IP, ports, and payload come from `config/robot.yaml` (`DobotRobot.from_config()`, default `192.168.5.1`).

| # | File | What it shows |
|---|------|----------------|
| 00 | `00_logging_demo.py` | SDK logging |
| 01 | `01_basic_connection.py` | Connect, enable, disable |
| 01a | `01a_safe_motion.py` | Slow joint moves including Home |
| 02 | `02_motion_control.py` | MovJ / MovL / Arc / Circle |
| 03 | `03_error_monitor.py` | Error IDs and status |
| 04 | `04_io_control.py` | Digital / analog / tool IO |
| 05 | `05_coordinate_system.py` | User and tool frames |
| 06 | `06_force_and_conveyor.py` | FT sensor and conveyor |
| 07 | `07_status_monitor.py` | Full Feedback monitor |
| 08 | `08_multi_thread_control.py` | Multi-thread control |

```bash
python examples/01_basic_connection.py
```

HRC pick-and-place lives in `hrc/examples/pick_and_place.py`. Dataset / SafeSkin tools: `hrc/examples/collect_episode.py`, `stream_skins.py`, `visualize_episode.py`, `export_dataset.py`.

## Notes

1. Edit `config/robot.yaml` rather than per-file IPs.
2. Keep a clear workspace before any motion example.
3. The controller must be in TCP/IP mode.
4. `pip install numpy requests`

```python
from dobot_sdk import DobotRobot

with DobotRobot.from_config() as robot:
    robot.robot_control.RequestControl()
    robot.robot_control.ClearError()
    robot.robot_control.EnableRobot(load=1.0)
    robot.robot_control.DisableRobot()
```
