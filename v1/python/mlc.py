import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional


FUNDAMENTAL_FIELDS: List[str] = [
    "lat_rad", "lon_rad", "alt_m",
    "pn_m", "pe_m", "pd_m",
    "vn_mps", "ve_mps", "vd_mps",
    "u_mps", "v_mps", "w_mps",
    "qw", "qx", "qy", "qz",
    "roll_rad", "pitch_rad", "yaw_rad",
    "p_radps", "q_radps", "r_radps",
    "an_mps2", "ae_mps2", "ad_mps2",
    "ax_body_mps2", "ay_body_mps2", "az_body_mps2",
]


def _dump_line(obj: Dict[str, Any]) -> str:
    return json.dumps(obj, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def _as_double_list(values: List[Any], expected_len: int, name: str) -> List[float]:
    if len(values) != expected_len:
        raise ValueError(f"{name} expected {expected_len} values, got {len(values)}")

    out: List[float] = []
    for i, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError(f"{name}[{i}] must be numeric")
        value_f = float(value)
        if not math.isfinite(value_f):
            raise ValueError(f"{name}[{i}] must be finite")
        out.append(value_f)

    return out


# ---- LLA <-> NED (Section 3.2): exact WGS84 conversion through ECEF, radians ----

WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def _lla_to_ecef(lla: List[float]) -> List[float]:
    lat, lon, alt = (float(v) for v in lla)
    s = math.sin(lat)
    n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * s * s)
    return [(n + alt) * math.cos(lat) * math.cos(lon),
            (n + alt) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - WGS84_E2) + alt) * s]


def _ecef_to_lla(ecef: List[float]) -> List[float]:
    x, y, z = (float(v) for v in ecef)
    lon = math.atan2(y, x)
    horizontal = math.hypot(x, y)
    lat = math.atan2(z, horizontal * (1.0 - WGS84_E2))
    for _ in range(8):
        s = math.sin(lat)
        n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * s * s)
        alt = horizontal / math.cos(lat) - n
        lat = math.atan2(z, horizontal * (1.0 - WGS84_E2 * n / (n + alt)))
    s = math.sin(lat)
    n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * s * s)
    return [lat, lon, horizontal / math.cos(lat) - n]


def _ned_axes(origin_lla: List[float]) -> List[List[float]]:
    """Rows: north, east, down unit vectors at the origin, in ECEF."""
    lat0, lon0 = float(origin_lla[0]), float(origin_lla[1])
    sl, cl, so, co = math.sin(lat0), math.cos(lat0), math.sin(lon0), math.cos(lon0)
    return [[-sl * co, -sl * so, cl], [-so, co, 0.0], [-cl * co, -cl * so, -sl]]


def ned_to_lla(origin_lla: List[float], ned: List[float]) -> List[float]:
    """[n, e, d] metres from origin_lla -> [lat_rad, lon_rad, alt_m] (WGS84 ellipsoid height)."""
    o = _as_double_list(list(origin_lla), 3, "origin_lla")
    v = _as_double_list(list(ned), 3, "ned")
    axes = _ned_axes(o)
    base = _lla_to_ecef(o)
    return _ecef_to_lla([base[k] + sum(axes[i][k] * v[i] for i in range(3)) for k in range(3)])


def lla_to_ned(origin_lla: List[float], lla: List[float]) -> List[float]:
    """[lat_rad, lon_rad, alt_m] -> [n, e, d] metres from origin_lla."""
    o = _as_double_list(list(origin_lla), 3, "origin_lla")
    p = _as_double_list(list(lla), 3, "lla")
    axes = _ned_axes(o)
    d = [a - b for a, b in zip(_lla_to_ecef(p), _lla_to_ecef(o))]
    return [sum(axes[i][k] * d[k] for k in range(3)) for i in range(3)]


class MLCWriter:
    def __init__(
        self,
        path: str,
        *,
        label: str,
        origin_lla: List[float],
        producer: Optional[str] = None,
        mode: Optional[str] = None,
        scenario: Optional[str] = None,
        seed: Optional[int] = None,
        created_utc: Optional[str] = None,
        meta: Optional[Dict[str, Any]] = None,
    ) -> None:
        # Build and check the header before the file exists: a bad argument leaves no empty file behind.
        header: Dict[str, Any] = {
            "$": "header",
            "format": 1,
            "label": label,
            "origin_lla": _as_double_list(origin_lla, 3, "origin_lla"),
        }

        if created_utc is not None:
            header["created_utc"] = created_utc
        if producer is not None:
            header["producer"] = producer
        if mode is not None:
            header["mode"] = mode
        if scenario is not None:
            header["scenario"] = scenario
        if seed is not None:
            header["seed"] = seed
        if meta is not None:
            header["meta"] = dict(meta)
        _dump_line(header)   # raises on values JSON cannot hold (NaN, objects), before the file is opened

        self.path = Path(path)
        self.f = self.path.open("w", encoding="utf-8", newline="\n")

        self.next_body_id = 0
        self.next_action_id = 0
        self.next_reward_id = 0
        self.next_step_id = 1

        self.bodies: Dict[int, Dict[str, Any]] = {}
        self.action_specs: Dict[int, Dict[str, Any]] = {}
        self.reward_specs: Dict[int, Dict[str, Any]] = {}

        self.current_step: Optional[int] = None
        self.current_time: Optional[float] = None

        self._write(header)

    def _write(self, obj: Dict[str, Any]) -> None:
        self.f.write(_dump_line(obj) + "\n")

    def close(self) -> None:
        self.f.close()

    def __enter__(self) -> "MLCWriter":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def body(
        self,
        name: str,
        *,
        platform: Optional[str] = None,
        model: Optional[str] = None,
        role: Optional[str] = None,
        meta: Optional[Dict[str, Any]] = None,
    ) -> int:
        body_id = self.next_body_id
        self.next_body_id += 1

        obj: Dict[str, Any] = {
            "$": "body",
            "id": body_id,
            "name": name,
        }

        if platform is not None:
            obj["platform"] = platform
        if model is not None:
            obj["model"] = model
        if role is not None:
            obj["role"] = role
        if meta is not None:
            obj["meta"] = dict(meta)

        self.bodies[body_id] = obj
        self._write(obj)
        return body_id

    def action_spec(self, body_id: int, fields: List[str]) -> int:
        if body_id not in self.bodies:
            raise ValueError(f"Unknown body_id: {body_id}")
        if not fields:
            raise ValueError("action_spec fields must not be empty")

        action_id = self.next_action_id
        self.next_action_id += 1

        obj = {
            "$": "action_spec",
            "id": action_id,
            "b": body_id,
            "fields": list(fields),
        }

        self.action_specs[action_id] = obj
        self._write(obj)
        return action_id

    def reward_spec(self, fields: List[str], *, body_id: Optional[int] = None) -> int:
        if not fields:
            raise ValueError("reward_spec fields must not be empty")
        if fields[0] != "step_reward":
            raise ValueError('reward_spec fields[0] must be "step_reward"')
        if body_id is not None and body_id not in self.bodies:
            raise ValueError(f"Unknown body_id: {body_id}")

        reward_id = self.next_reward_id
        self.next_reward_id += 1

        obj: Dict[str, Any] = {
            "$": "reward_spec",
            "id": reward_id,
            "fields": list(fields),
        }

        if body_id is not None:
            obj["b"] = body_id

        self.reward_specs[reward_id] = obj
        self._write(obj)
        return reward_id

    def step(self, t: float, *, s: Optional[int] = None) -> int:
        if s is None:
            s = self.next_step_id
            self.next_step_id += 1
        else:
            self.next_step_id = max(self.next_step_id, s + 1)

        t_f = float(t)
        if not math.isfinite(t_f):
            raise ValueError("step time must be finite")

        self.current_step = s
        self.current_time = t_f

        self._write({"$": "step", "s": s, "t": t_f})
        return s

    def state(self, body_id: int, values: List[Any]) -> None:
        if body_id not in self.bodies:
            raise ValueError(f"Unknown body_id: {body_id}")
        if self.current_step is None:
            raise RuntimeError("state requires an active step")

        x = _as_double_list(values, len(FUNDAMENTAL_FIELDS), "fundamental_state")
        self._write({"b": body_id, "x": x})

    def action(self, action_id: int, values: List[Any]) -> None:
        if action_id not in self.action_specs:
            raise ValueError(f"Unknown action_id: {action_id}")
        if self.current_step is None:
            raise RuntimeError("action requires an active step")

        fields = self.action_specs[action_id]["fields"]
        if len(values) != len(fields):
            raise ValueError(f"action expected {len(fields)} values, got {len(values)}")

        self._write({"a": action_id, "x": list(values)})

    def reward(self, reward_id: int, values: List[Any]) -> None:
        if reward_id not in self.reward_specs:
            raise ValueError(f"Unknown reward_id: {reward_id}")
        if self.current_step is None:
            raise RuntimeError("reward requires an active step")

        fields = self.reward_specs[reward_id]["fields"]
        if len(values) != len(fields):
            raise ValueError(f"reward expected {len(fields)} values, got {len(values)}")

        self._write({"r": reward_id, "x": list(values)})

    def event(self, topic: str, data: Dict[str, Any], *, body_id: Optional[int] = None) -> None:
        if self.current_step is None:
            raise RuntimeError("event requires an active step")
        if body_id is not None and body_id not in self.bodies:
            raise ValueError(f"Unknown body_id: {body_id}")

        obj: Dict[str, Any] = {
            "$": "event",
            "topic": topic,
            "data": data,
        }

        if body_id is not None:
            obj["b"] = body_id

        self._write(obj)


def read_mlc(path: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "header": None,
        "bodies": {},
        "action_specs": {},
        "reward_specs": {},
        "steps": [],
        "states": [],
        "actions": [],
        "rewards": [],
        "events": [],
    }

    current_step: Optional[int] = None
    current_time: Optional[float] = None

    with Path(path).open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue

            obj = json.loads(line)
            kind = obj.get("$")

            if kind == "header":
                result["header"] = obj

            elif kind == "body":
                result["bodies"][obj["id"]] = obj

            elif kind == "action_spec":
                result["action_specs"][obj["id"]] = obj

            elif kind == "reward_spec":
                if obj["fields"][0] != "step_reward":
                    raise ValueError(f'Line {line_no}: reward_spec first field must be "step_reward"')
                result["reward_specs"][obj["id"]] = obj

            elif kind == "step":
                current_step = obj["s"]
                current_time = obj["t"]
                result["steps"].append(obj)

            elif kind == "event":
                if current_step is None and ("s" not in obj or "t" not in obj):
                    raise ValueError(
                        f"Line {line_no}: event appears before first step "
                        "and does not provide explicit s/t"
                    )
                event = dict(obj)
                event.setdefault("s", current_step)
                event.setdefault("t", current_time)
                result["events"].append(event)

            elif "b" in obj:
                if current_step is None and ("s" not in obj or "t" not in obj):
                    raise ValueError(
                        f"Line {line_no}: body state appears before first step "
                        "and does not provide explicit s/t"
                    )

                x = _as_double_list(obj["x"], len(FUNDAMENTAL_FIELDS), "fundamental_state")
                data = dict(zip(FUNDAMENTAL_FIELDS, x))

                record_s = obj.get("s", current_step)
                record_t = obj.get("t", current_time)

                result["states"].append({
                    "s": record_s,
                    "t": record_t,
                    "b": obj["b"],
                    "x": x,
                    "data": data,
                })

            elif "a" in obj:
                if current_step is None and ("s" not in obj or "t" not in obj):
                    raise ValueError(
                        f"Line {line_no}: action appears before first step "
                        "and does not provide explicit s/t"
                    )

                spec = result["action_specs"][obj["a"]]
                fields = spec["fields"]
                x = obj["x"]

                if len(x) != len(fields):
                    raise ValueError(f"Line {line_no}: action length mismatch")

                record_s = obj.get("s", current_step)
                record_t = obj.get("t", current_time)

                result["actions"].append({
                    "s": record_s,
                    "t": record_t,
                    "a": obj["a"],
                    "b": spec["b"],
                    "x": x,
                    "data": dict(zip(fields, x)),
                })

            elif "r" in obj:
                if current_step is None and ("s" not in obj or "t" not in obj):
                    raise ValueError(
                        f"Line {line_no}: reward appears before first step "
                        "and does not provide explicit s/t"
                    )

                spec = result["reward_specs"][obj["r"]]
                fields = spec["fields"]
                x = obj["x"]

                if len(x) != len(fields):
                    raise ValueError(f"Line {line_no}: reward length mismatch")

                record_s = obj.get("s", current_step)
                record_t = obj.get("t", current_time)

                result["rewards"].append({
                    "s": record_s,
                    "t": record_t,
                    "r": obj["r"],
                    "b": spec.get("b"),
                    "x": x,
                    "data": dict(zip(fields, x)),
                })

            else:
                raise ValueError(f"Line {line_no}: unknown MLC record")

    return result
