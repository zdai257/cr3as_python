"""Parse Dobot dashboard replies: ErrorID,{...},Command();"""

from __future__ import annotations

import ast
import re
from typing import Any, List, Optional, Sequence, Tuple


_DASHBOARD_RE = re.compile(
  r"^\s*(-?\d+)\s*,\s*(\{.*\}|)\s*,?\s*([A-Za-z_]\w*\(.*\))?;?\s*$",
  re.DOTALL,
)


def parse_dashboard(response: str) -> Tuple[int, List[Any], str]:
  """Return (error_id, values, command).

  Examples:
    0,{1.0,2.0,3.0,4.0,5.0,6.0},GetPose();
    0,{[1537,2048]},GetErrorID();
    0,{1},ModbusRTUCreate(1,115200,"N",8,1);
    -1,{},EnableRobot();
  """
  if response is None:
    raise ValueError("empty dashboard response")
  text = response.strip()
  match = _DASHBOARD_RE.match(text)
  if not match:
    # Some firmware omits the trailing command token.
    alt = re.match(r"^\s*(-?\d+)\s*,\s*(\{.*\})\s*;?\s*$", text, re.DOTALL)
    if not alt:
      raise ValueError(f"unrecognized dashboard response: {response!r}")
    error_id = int(alt.group(1))
    return error_id, _parse_braces(alt.group(2)), ""
  error_id = int(match.group(1))
  payload = match.group(2) or "{}"
  command = match.group(3) or ""
  return error_id, _parse_braces(payload), command


def _parse_braces(payload: str) -> List[Any]:
  payload = payload.strip()
  if payload in ("", "{}", "{ }"):
    return []
  inner = payload
  if inner.startswith("{") and inner.endswith("}"):
    inner = inner[1:-1].strip()
  if not inner:
    return []
  try:
    value = ast.literal_eval("[" + inner + "]")
    if isinstance(value, list) and len(value) == 1 and isinstance(value[0], list):
      return value[0]
    return list(value)
  except (SyntaxError, ValueError):
    return [part.strip().strip("'\"") for part in inner.split(",") if part.strip()]


def require_ok(response: str, what: str = "command") -> List[Any]:
  error_id, values, _ = parse_dashboard(response)
  if error_id != 0:
    raise RuntimeError(f"{what} failed (ErrorID={error_id}): {response}")
  return values


def as_vector(response: str, n: int = 6) -> List[float]:
  values = [float(v) for v in require_ok(response)]
  if len(values) < n:
    raise RuntimeError(f"expected {n} values, got {values} from {response!r}")
  return values[:n]


def taught(values: Optional[Sequence]) -> bool:
  if values is None:
    return False
  return all(v is not None for v in values)
