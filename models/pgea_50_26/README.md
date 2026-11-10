# DH Robotics PGEA-50-26 digital model

Parallel gripper mounted on the CS3AS flange.

| Item | Value |
|------|--------|
| Envelope (L x W x H) | 118 x 55 x 29 mm |
| Stroke | 26 mm (gap 1.2–27.2 mm) |
| Mass | 0.5 kg |
| Tool offset | x=0, y=0, **z=118 mm** |
| Force | 15–50 N / jaw |
| Bus | Modbus RTU on tool RS485 |

`urdf/pgea_50_26.urdf` is the model used for visualisation, collision, and TCP. Meshes are datasheet envelopes (run `python models/pgea_50_26/generate_meshes.py`). Official STEP is download-gated; see `cad/README.md`.

Python access:

```python
from hrc.models import PGEA5026Model, gripper_model_dir
model = PGEA5026Model()
print(model.urdf_path)
print(model.opening_mm(1000))  # fully open
```
