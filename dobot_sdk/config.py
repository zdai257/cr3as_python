# Copyright (c) 2026 Dobot
# Licensed under the MIT License

"""Load the repo-wide robot configuration (config/robot.yaml)."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union


_CACHE: Optional["RobotConfig"] = None
_CACHE_PATH: Optional[Path] = None

DEFAULT_IP = "192.168.5.1"
CONFIG_REL = Path("config") / "robot.yaml"


class Cfg:
  """Attribute access over a nested dict (read-only)."""

  def __init__(self, data: Dict[str, Any]):
    self._data = data

  def __getattr__(self, name: str) -> Any:
    if name.startswith("_"):
      raise AttributeError(name)
    try:
      value = self._data[name]
    except KeyError as exc:
      raise AttributeError(name) from exc
    return _wrap(value)

  def __getitem__(self, key: str) -> Any:
    return _wrap(self._data[key])

  def get(self, key: str, default: Any = None) -> Any:
    if key not in self._data:
      return default
    return _wrap(self._data[key])

  def to_dict(self) -> Dict[str, Any]:
    return self._data

  def __repr__(self) -> str:
    return f"Cfg({self._data!r})"


def _wrap(value: Any) -> Any:
  if isinstance(value, dict):
    return Cfg(value)
  if isinstance(value, list):
    return [_wrap(v) for v in value]
  return value


class RobotConfig(Cfg):
  """Root config with shortcuts used by DobotRobot.from_config()."""

  @property
  def ip(self) -> str:
    return str(self.robot.ip)

  @property
  def dashboard_port(self) -> int:
    return int(self.robot.dashboard_port)

  @property
  def feedback_port(self) -> int:
    return int(self.robot.feedback_port)

  @property
  def connect_timeout(self) -> float:
    return float(self.robot.connect_timeout)

  @property
  def receive_timeout(self) -> float:
    return float(self.robot.receive_timeout)


def _repo_root() -> Path:
  here = Path(__file__).resolve()
  return here.parents[1]


def find_config_path(explicit: Optional[Union[str, Path]] = None) -> Path:
  if explicit is not None:
    path = Path(explicit).expanduser().resolve()
    if not path.is_file():
      raise FileNotFoundError(f"Robot config not found: {path}")
    return path

  env = os.environ.get("DOBOT_CONFIG")
  if env:
    path = Path(env).expanduser().resolve()
    if path.is_file():
      return path

  candidates = [
    Path.cwd() / CONFIG_REL,
    _repo_root() / CONFIG_REL,
  ]
  for parent in Path.cwd().resolve().parents:
    candidates.append(parent / CONFIG_REL)

  seen = set()
  for path in candidates:
    path = path.resolve()
    if path in seen:
      continue
    seen.add(path)
    if path.is_file():
      return path

  raise FileNotFoundError(
    f"Could not find {CONFIG_REL}. Set DOBOT_CONFIG or run from the repo root."
  )


def _strip_comment(raw: str) -> str:
  in_single = False
  in_double = False
  out = []
  for ch in raw:
    if ch == "'" and not in_double:
      in_single = not in_single
    elif ch == '"' and not in_single:
      in_double = not in_double
    elif ch == "#" and not in_single and not in_double:
      break
    out.append(ch)
  return "".join(out).rstrip()


def _parse_scalar(text: str) -> Any:
  text = text.strip()
  if text in ("", "~", "null", "Null", "NULL", "None"):
    return None
  if text in ("true", "True", "yes", "Yes"):
    return True
  if text in ("false", "False", "no", "No"):
    return False
  if (text.startswith('"') and text.endswith('"')) or (
    text.startswith("'") and text.endswith("'")
  ):
    return text[1:-1]
  if text.startswith("[") and text.endswith("]"):
    inner = text[1:-1].strip()
    if not inner:
      return []
    return [_parse_scalar(part) for part in _split_top(inner)]
  if text.startswith("{") and text.endswith("}"):
    inner = text[1:-1].strip()
    if not inner:
      return {}
    data: Dict[str, Any] = {}
    for part in _split_top(inner):
      key, val = _split_key_value(part)
      data[key] = _parse_scalar(val)
    return data
  try:
    if any(ch in text for ch in ".eE") and text.replace(".", "", 1).replace(
      "e", "", 1
    ).replace("E", "", 1).replace("-", "", 1).replace("+", "", 1).isdigit():
      return float(text)
    return int(text)
  except ValueError:
    return text


def _split_top(text: str, sep: str = ",") -> List[str]:
  parts: List[str] = []
  buf: List[str] = []
  depth = 0
  in_single = False
  in_double = False
  for ch in text:
    if ch == "'" and not in_double:
      in_single = not in_single
    elif ch == '"' and not in_single:
      in_double = not in_double
    elif not in_single and not in_double:
      if ch in "[{":
        depth += 1
      elif ch in "]}":
        depth -= 1
      elif ch == sep and depth == 0:
        parts.append("".join(buf).strip())
        buf = []
        continue
    buf.append(ch)
  if buf:
    parts.append("".join(buf).strip())
  return [p for p in parts if p != ""]


def _split_key_value(text: str) -> Tuple[str, str]:
  depth = 0
  in_single = False
  in_double = False
  for i, ch in enumerate(text):
    if ch == "'" and not in_double:
      in_single = not in_single
    elif ch == '"' and not in_single:
      in_double = not in_double
    elif not in_single and not in_double:
      if ch in "[{":
        depth += 1
      elif ch in "]}":
        depth -= 1
      elif ch == ":" and depth == 0:
        key = text[:i].strip().strip("'\"")
        return key, text[i + 1 :].strip()
  raise ValueError(f"Invalid YAML mapping entry: {text!r}")


def _parse_block(
  items: List[Tuple[int, str]], index: int, indent: int
) -> Tuple[Any, int]:
  if index >= len(items):
    return {}, index

  first_indent, first = items[index]
  if first_indent < indent:
    return {}, index

  if first.startswith("- "):
    result: List[Any] = []
    while index < len(items):
      cur_indent, line = items[index]
      if cur_indent < first_indent:
        break
      if cur_indent > first_indent:
        raise ValueError(f"Invalid YAML indent: {line}")
      if not line.startswith("- "):
        break
      payload = line[2:].strip()
      index += 1
      child: Any
      if payload.startswith("{") or payload.startswith("["):
        child = _parse_scalar(payload)
      elif payload == "" or payload.endswith(":"):
        key = payload[:-1].strip() if payload.endswith(":") else None
        nested, index = _parse_block(items, index, first_indent + 1)
        child = {key: nested} if key else nested
      elif ":" in payload:
        key, val = _split_key_value(payload)
        nested, index = _parse_block(items, index, first_indent + 1)
        if nested:
          node = {key: _parse_scalar(val) if val != "" else nested}
          if isinstance(nested, dict):
            node.update(nested)
          child = node
        else:
          child = {key: _parse_scalar(val)}
      else:
        child = _parse_scalar(payload)
      result.append(child)
    return result, index

  result_dict: Dict[str, Any] = {}
  while index < len(items):
    cur_indent, line = items[index]
    if cur_indent < first_indent:
      break
    if cur_indent > first_indent:
      raise ValueError(f"Invalid YAML indent: {line}")
    if line.startswith("- "):
      break
    key, val = _split_key_value(line)
    index += 1
    if val == "" or val is None:
      nested, index = _parse_block(items, index, first_indent + 1)
      result_dict[key] = nested
    else:
      result_dict[key] = _parse_scalar(val)
  return result_dict, index


def load_yaml(path: Union[str, Path]) -> Dict[str, Any]:
  text = Path(path).read_text(encoding="utf-8")
  items: List[Tuple[int, str]] = []
  for raw in text.splitlines():
    stripped = _strip_comment(raw)
    if not stripped.strip():
      continue
    indent = len(stripped) - len(stripped.lstrip(" "))
    items.append((indent, stripped.strip()))
  data, _ = _parse_block(items, 0, 0)
  if not isinstance(data, dict):
    raise ValueError(f"Config root must be a mapping: {path}")
  return data


def load_robot_config(
  path: Optional[Union[str, Path]] = None, *, reload: bool = False
) -> RobotConfig:
  """Load config/robot.yaml (cached). Override with DOBOT_CONFIG or path=."""
  global _CACHE, _CACHE_PATH
  resolved = find_config_path(path)
  if not reload and _CACHE is not None and _CACHE_PATH == resolved:
    return _CACHE
  _CACHE = RobotConfig(load_yaml(resolved))
  _CACHE_PATH = resolved
  return _CACHE


def clear_config_cache() -> None:
  global _CACHE, _CACHE_PATH
  _CACHE = None
  _CACHE_PATH = None
