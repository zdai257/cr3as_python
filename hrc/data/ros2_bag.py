"""Write a rosbag2 sqlite3 bag of CS3AS cell streams (no rclpy required).

Messages are CDR-encoded `std_msgs/msg/Float64MultiArray` so `ros2 bag play`
and Foxglove can read them. Topics:

  /cs3as/joints
  /cs3as/pose
  /cs3as/gripper
  /cs3as/skins/approach
  /cs3as/skins/proximity
  /cs3as/skins/conductance
  /cs3as/action
"""

from __future__ import annotations

import sqlite3
import struct
from pathlib import Path
from typing import Sequence, Union

from .dataset import Episode

TOPICS = (
  ("/cs3as/joints", "joints"),
  ("/cs3as/pose", "pose"),
  ("/cs3as/gripper", "gripper"),
  ("/cs3as/skins/approach", "skin_approach"),
  ("/cs3as/skins/proximity", "skin_proximity"),
  ("/cs3as/skins/conductance", "skin_conductance"),
  ("/cs3as/action", "action"),
)

MSG_TYPE = "std_msgs/msg/Float64MultiArray"


def cdr_float64_multiarray(values: Sequence[float]) -> bytes:
  buf = bytearray()
  buf += b"\x00\x01\x00\x00"  # CDR LE encapsulation
  buf += struct.pack("<I", 0)  # layout.dim length
  buf += struct.pack("<I", 0)  # layout.data_offset
  buf += struct.pack("<I", len(values))
  buf += b"\x00\x00\x00\x00"  # align float64 to 8
  for v in values:
    buf += struct.pack("<d", float(v))
  return bytes(buf)


def _schema_sql() -> str:
  return """
  CREATE TABLE IF NOT EXISTS schema(
    schema_id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    encoding TEXT NOT NULL,
    encoded_data BLOB NOT NULL
  );
  CREATE TABLE IF NOT EXISTS metadata(
    id INTEGER PRIMARY KEY,
    metadata_version INTEGER NOT NULL,
    metadata TEXT NOT NULL
  );
  CREATE TABLE IF NOT EXISTS topics(
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    type TEXT NOT NULL,
    serialization_format TEXT NOT NULL,
    offered_qos_profiles TEXT NOT NULL,
    schema_id INTEGER NOT NULL
  );
  CREATE TABLE IF NOT EXISTS messages(
    id INTEGER PRIMARY KEY,
    topic_id INTEGER NOT NULL,
    timestamp INTEGER NOT NULL,
    data BLOB NOT NULL
  );
  """


def _metadata_yaml(
  rel_db: str,
  duration_ns: int,
  start_ns: int,
  counts: dict,
  message_count: int,
) -> str:
  lines = [
    "rosbag2_bagfile_information:",
    "  version: 5",
    "  storage_identifier: sqlite3",
    "  duration:",
    f"    nanoseconds: {duration_ns}",
    "  starting_time:",
    f"    nanoseconds_since_epoch: {start_ns}",
    f"  message_count: {message_count}",
    "  topics_with_message_count:",
  ]
  for name, count in counts.items():
    lines.extend(
      [
        "    - topic_metadata:",
        f"        name: {name}",
        f"        type: {MSG_TYPE}",
        "        serialization_format: cdr",
        '        offered_qos_profiles: ""',
        f"      message_count: {count}",
      ]
    )
  lines.extend(
    [
      '  compression_format: ""',
      '  compression_mode: ""',
      "  relative_file_paths:",
      f"    - {rel_db}",
      "  files:",
      f"    - path: {rel_db}",
      f"      starting_time:",
      f"        nanoseconds_since_epoch: {start_ns}",
      f"      duration:",
      f"        nanoseconds: {duration_ns}",
      f"      message_count: {message_count}",
    ]
  )
  return "\n".join(lines) + "\n"


def write_ros2_bag(episode: Episode, dest: Union[str, Path]) -> Path:
  dest = Path(dest)
  dest.mkdir(parents=True, exist_ok=True)
  db_name = "episode_0.db3"
  db_path = dest / db_name
  if db_path.exists():
    db_path.unlink()

  conn = sqlite3.connect(str(db_path))
  try:
    conn.executescript(_schema_sql())
    conn.execute(
      "INSERT INTO schema(schema_id, name, encoding, encoded_data) VALUES (1, ?, 'ros2msg', ?)",
      (MSG_TYPE, b""),
    )
    topic_ids = {}
    for i, (name, _) in enumerate(TOPICS, start=1):
      conn.execute(
        "INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles, schema_id) "
        "VALUES (?, ?, ?, 'cdr', '', 1)",
        (i, name, MSG_TYPE),
      )
      topic_ids[name] = i

    counts = {name: 0 for name, _ in TOPICS}
    msg_id = 1
    start_ns = 0
    last_ns = 0
    for frame in episode.frames:
      stamp_ns = int(frame.timestamp * 1e9)
      if start_ns == 0:
        start_ns = stamp_ns
      last_ns = stamp_ns
      payload = {
        "/cs3as/joints": frame.joints,
        "/cs3as/pose": frame.pose,
        "/cs3as/gripper": [frame.gripper_position],
        "/cs3as/skins/approach": frame.skin_approach,
        "/cs3as/skins/proximity": frame.skin_proximity,
        "/cs3as/skins/conductance": frame.skin_conductance,
        "/cs3as/action": frame.action,
      }
      for name, values in payload.items():
        conn.execute(
          "INSERT INTO messages(id, topic_id, timestamp, data) VALUES (?, ?, ?, ?)",
          (msg_id, topic_ids[name], stamp_ns, cdr_float64_multiarray(values)),
        )
        counts[name] += 1
        msg_id += 1
    conn.commit()
  finally:
    conn.close()

  message_count = sum(counts.values())
  duration = max(0, last_ns - start_ns)
  (dest / "metadata.yaml").write_text(
    _metadata_yaml(db_name, duration, start_ns, counts, message_count),
    encoding="utf-8",
  )
  return dest
