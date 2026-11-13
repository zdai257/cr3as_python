# Demo GUI

Tk control panel for the CS3AS cell. IP/ports from `config/robot.yaml`.

```bash
cd demo
python main_UI.py
```

| Panel | What it does |
|-------|----------------|
| Connect | Sockets, then **Take Cell** (TCP + Enable + SafeSkin + gripper attach + camera) |
| Dashboard | Enable / ClearError / speed / TCP / ResetSafety |
| HRC | Live SafeSkin bars (proximity / mm), Skin On/Off, gripper Open/Close |
| Feedback | Joints, Cartesian, DI/DO, errors |

**SafeSkin** works over TCP once the arm is Enabled (queued `EnableSafeSkin` only flushes then).

**Gripper:** Init once with pendant plugin **DHGrip_v2-1-6Stable**. Then Connect / Take Cell / Open / Close. Python does not replace plugin Init (`power_cycle: false`).

Keep the pendant in TCP mode, e-stop reachable.
