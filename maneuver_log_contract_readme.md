# Maneuver Log Contract v1

**A compact NDJSON log contract for moving things performing maneuvers.**

Maneuver Log Contract v1, or **MLC v1**, is a simple open format for logging the motion, actions, rewards, and events of vehicles, robots, agents, and simulated bodies.

It is built for one mission:

> **We play with moving things. We want to achieve intelligent autonomy.**

MLC v1 is designed for rockets, UAVs, quadcopters, fixed-wing aircraft, gliders, missiles, robots, and other maneuvering systems.

---

## Why MLC?

Many simulation and autonomy projects eventually need the same thing:

- replay a maneuver in 3D
- analyze position, velocity, attitude, rates, and acceleration
- compare model actions with interpreted control commands
- inspect rewards and task performance
- mark important events on a timeline
- share logs between tools, languages, and projects

MLC v1 provides a small, strict core for this.

The format is:

- **NDJSON-based** — one JSON object per line
- **stream-friendly** — easy to write during a run
- **replay-friendly** — every body has a complete maneuver state
- **tool-friendly** — easy to parse in Python, C, C++, Rust, Go, JavaScript, and more
- **autonomy-friendly** — actions and rewards are first-class optional records

---

## Core Idea

Every moving body is logged with a fixed **fundamental maneuver state**:

1. position
2. velocity
3. attitude
4. angular rate
5. acceleration

The core state always uses:

```text
SI units
WGS84 LLA
NED local frame
FRD body frame
quaternion attitude
Euler angles for readability
double precision for fundamental fields
```

A visualizer can reconstruct body motion from this state alone.

---

## Tiny Example

```ndjson
{"$":"header","format":1,"label":"run_001","origin_lla":[0.651733,2.216568,120.0]}
{"$":"body","id":0,"name":"booster_0","platform":"rocket"}
{"$":"action_spec","id":0,"b":0,"fields":["raw_action_0","raw_action_1","raw_action_2","throttle_cmd","tvc_x_cmd","tvc_y_cmd"]}
{"$":"reward_spec","id":0,"b":0,"fields":["step_reward","pos_err_reward","vel_err_reward","fuel_penalty","terminal_bonus"]}
{"$":"step","s":1,"t":0.000000}
{"b":0,"x":[0.651733,2.216568,970.0,18.2,-6.5,-850.0,-1.2,0.4,72.5,72.4,0.1,1.8,0.9998,0.0030,0.0190,0.0010,0.0061,0.0380,0.0021,0.0040,-0.0180,0.0020,0.1,-0.2,-12.8,0.5,0.0,-12.7]}
{"a":0,"x":[0.12,-0.44,0.91,0.58,0.012,-0.018]}
{"r":0,"x":[-0.812,-0.215,-0.441,-0.058,0.000]}
{"$":"event","b":0,"topic":"phase","data":{"from":"coast","to":"terminal_descent"}}
```

---

## Record Types

| Record | Purpose |
|---|---|
| `header` | Log identity and NED origin |
| `body` | Moving body definition |
| `step` | Time context |
| `b` sample | Fixed 28-field fundamental maneuver state |
| `action_spec` | Action vector layout |
| `a` sample | Action vector values |
| `reward_spec` | Reward vector layout |
| `r` sample | Reward vector values |
| `event` | Timeline annotation |

---

## Fundamental State

A body state sample has this compact form:

```json
{"b":0,"x":[...28 values...]}
```

The 28 values are fixed by the contract:

```text
lat_rad, lon_rad, alt_m,
pn_m, pe_m, pd_m,
vn_mps, ve_mps, vd_mps,
u_mps, v_mps, w_mps,
qw, qx, qy, qz,
roll_rad, pitch_rad, yaw_rad,
p_radps, q_radps, r_radps,
an_mps2, ae_mps2, ad_mps2,
ax_body_mps2, ay_body_mps2, az_body_mps2
```

This strict core is what makes MLC useful for replay, plotting, analysis, and cross-tool compatibility.

---

## Actions

Actions are optional but standardized.

An action spec names the action vector fields:

```ndjson
{"$":"action_spec","id":0,"b":0,"fields":["raw_action_0","raw_action_1","raw_action_2","throttle_cmd","tvc_x_cmd","tvc_y_cmd"]}
```

An action sample stores the values:

```ndjson
{"a":0,"x":[0.12,-0.44,0.91,0.58,0.012,-0.018]}
```

This allows one line to capture both model output and interpreted control command.

---

## Rewards

Rewards are optional but standardized.

The first reward field is always `step_reward`.

```ndjson
{"$":"reward_spec","id":0,"b":0,"fields":["step_reward","pos_err_reward","vel_err_reward","fuel_penalty","terminal_bonus"]}
{"r":0,"x":[-0.812,-0.215,-0.441,-0.058,0.000]}
```

Episode return can be computed from the logged step rewards.

---

## Events

Events annotate the maneuver timeline:

```ndjson
{"$":"event","b":0,"topic":"terminal","data":{"success":true,"reason":"soft_landing"}}
```

Events are useful for phase changes, terminal results, collisions, touchdowns, warnings, or other semantic markers.

---

## Recommended Extension

```text
.mlc.ndjson
```

Example:

```text
run_20260429_landing_eval_001.mlc.ndjson
```

---

## Status

MLC v1 is the first stable contract draft.

It intentionally keeps the guaranteed behavior layer small:

```text
header
body
step
fundamental maneuver state
```

Actions, rewards, and events are standardized optional records.

Everything else can be added as project-specific extensions.

---

## Philosophy

MLC is not trying to describe every vehicle system in the universe.

It defines the part that almost every maneuvering system needs:

> **Where is it? How is it moving? How is it oriented? What did the autonomy choose? What reward did it receive? What important thing happened?**

That is enough to build replay tools, analyzers, trainers, visualizers, and autonomy debugging workflows around a shared log language.

