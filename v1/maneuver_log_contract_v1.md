# Maneuver Log Contract v1

**Short name:** MLC v1  
**Recommended file extension:** `.mlc.ndjson`  
**Storage format:** NDJSON, one JSON object per line

---

## 1. Purpose

Maneuver Log Contract v1 defines a compact, strict, and implementation-friendly log format for moving bodies performing maneuvers.

The contract is designed around one central idea:

> We play with moving things, and we want to achieve intelligent autonomy through maneuvering.

MLC v1 is intended for:

- simulation
- training
- evaluation
- testing
- replay
- 3D visualization
- post-run analysis
- autonomy debugging
- real or simulated moving platforms

The contract supports moving bodies such as:

- rockets
- quadcopters
- fixed-wing UAVs
- gliders
- missiles
- ground vehicles
- surface vehicles
- robots
- other simulated or physical maneuvering bodies

The core of the contract is a fixed **fundamental maneuver state**.

Every logged moving body state must contain:

1. position
2. velocity
3. attitude
4. angular rate
5. acceleration

Actions, rewards, and events are standardized companion records.

All other records are optional extensions.

---

## 2. Core Philosophy

MLC v1 follows these principles:

1. A maneuver log must describe motion first.
2. The fundamental state must be fixed and complete.
3. The fundamental state must use double precision.
4. Static conventions must live in this document, so log files stay compact.
5. A visualizer must be able to animate bodies from the fundamental state alone.
6. Action logs should capture model outputs and interpreted commands in one compact vector.
7. Reward logs should capture step reward and task-specific reward terms in one compact vector.
8. Events should annotate the maneuver timeline with readable semantic information.
9. Optional extension records may carry extra values, plots, sensors, debug data, or project-specific information.
10. Tools must rely only on the fundamental state for guaranteed motion behavior.

---

## 3. Static Contract Conventions

The following conventions are fixed by MLC v1 and are not repeated in every log file.

### 3.1 Units

All fundamental numeric fields use SI units.

| Quantity | Unit |
|---|---|
| latitude | rad |
| longitude | rad |
| altitude | m |
| local position | m |
| velocity | m/s |
| acceleration | m/s² |
| attitude Euler angles | rad |
| angular rate | rad/s |
| quaternion components | unitless |

### 3.2 Earth and Position Frames

MLC v1 uses:

```text
Earth model: WGS84
Global position: LLA = [lat_rad, lon_rad, alt_m]
Local position frame: NED = North-East-Down
```

The log header must define the local NED origin using LLA:

```json
"origin_lla": [lat_rad, lon_rad, alt_m]
```

### 3.3 Body Frame

MLC v1 uses the aerospace body frame:

```text
Body frame: FRD = Forward-Right-Down
```

Body-frame velocity uses:

```text
uvw = [u, v, w]
```

where:

```text
u = forward velocity
v = right velocity
w = down velocity
```

Angular rate uses:

```text
pqr = [p, q, r]
```

where:

```text
p = roll rate about body forward axis
q = pitch rate about body right axis
r = yaw rate about body down axis
```

### 3.4 Attitude Convention

The canonical attitude representation is quaternion.

```text
Quaternion order: [qw, qx, qy, qz]
Quaternion meaning: body_to_ned
```

The quaternion rotates a vector from the body FRD frame into the local NED frame.

Euler angles are also stored for readability and plotting:

```text
Euler order: [roll, pitch, yaw]
Euler unit: rad
Euler convention: derived from body_to_ned attitude
```

---

## 4. Numeric Precision Rule

All fundamental maneuver-state numeric fields must be interpreted as double-precision floating-point values.

```text
Precision: IEEE-754 binary64
C: double
C++: double
Python: float
NumPy: float64
```

JSON stores numbers as text. Readers and writers must parse and produce the fundamental maneuver-state values as double precision.

Optional extension records may define their own value types.

---

## 5. Top-Level Record Types

MLC v1 defines these standard record types:

| Record | Purpose |
|---|---|
| `header` | Defines the log identity and NED origin |
| `body` | Defines a moving body |
| `step` | Defines the current time context |
| body state sample | Stores the fixed fundamental maneuver state |
| `action_spec` | Defines an action vector layout |
| action sample | Stores one action vector |
| `reward_spec` | Defines a reward vector layout |
| reward sample | Stores one reward vector |
| `event` | Stores a readable timeline annotation |

---

## 6. Header Record

The first line of an MLC v1 file must be a header record.

### 6.1 Form

```json
{"$":"header","format":1,"label":"run_001","origin_lla":[0.651733,2.216568,120.0]}
```

### 6.2 Recommended Full Form

```json
{"$":"header","format":1,"label":"run_20260429_landing_eval_001","origin_lla":[0.651733,2.216568,120.0],"created_utc":"2026-04-29T11:20:00Z","producer":"project-nightfall-rocketsim","mode":"simulation","scenario":"rocket_landing_with_chase_quad","seed":42017}
```

### 6.3 Fields

| Field | Required | Meaning |
|---|---:|---|
| `$` | yes | must be `"header"` |
| `format` | yes | must be `1` for MLC v1 |
| `label` | yes | human-readable log label |
| `origin_lla` | yes | NED origin as `[lat_rad, lon_rad, alt_m]` |
| `created_utc` | recommended | UTC creation timestamp |
| `producer` | recommended | producing tool, simulator, trainer, or logger |
| `mode` | recommended | run mode such as simulation, training, evaluation, testing, replay, or real_flight |
| `scenario` | optional | scenario or task name |
| `seed` | optional | random seed |
| `meta` | optional | project-specific metadata |

---

## 7. Body Record

A body record defines one moving body.

### 7.1 Form

```json
{"$":"body","id":0,"name":"booster_0","platform":"rocket","model":"generic_vtv_booster"}
```

### 7.2 Fields

| Field | Required | Meaning |
|---|---:|---|
| `$` | yes | must be `"body"` |
| `id` | yes | unique body ID within the log |
| `name` | yes | human-readable body name |
| `platform` | recommended | platform family such as rocket, quadcopter, fixed_wing, glider, missile, robot |
| `model` | optional | model name |
| `role` | optional | role such as primary, target, observer, leader, follower |
| `meta` | optional | project-specific body metadata |

---

## 8. Step Record

A step record defines the current time context.

Samples, actions, rewards, and events following a step inherit the step time until the next step record appears.

### 8.1 Form

```json
{"$":"step","s":1,"t":0.000000}
```

### 8.2 Fields

| Field | Required | Meaning |
|---|---:|---|
| `$` | yes | must be `"step"` |
| `s` | yes | step index |
| `t` | yes | maneuver time in seconds |
| `meta` | optional | step-specific metadata |

### 8.3 Rules

- `s` must increase over time.
- `t` must represent maneuver time in seconds.
- `t` should increase or stay equal as the log progresses.
- Records after a step inherit that step's `s` and `t`.
- A record may include explicit `s` or `t` when a local override is required.

---

## 9. Fundamental Maneuver State

The body state sample is the mandatory core record of MLC v1.

It uses this compact form:

```json
{"b":0,"x":[...]}
```

where:

```text
b = body ID
x = fixed 28-value fundamental maneuver-state vector
```

A valid fundamental state sample must contain exactly 28 double-precision values in the fixed order below.

### 9.1 Fixed Field Order

| Index | Field | Meaning | Unit / Type |
|---:|---|---|---|
| 0 | `lat_rad` | latitude | rad |
| 1 | `lon_rad` | longitude | rad |
| 2 | `alt_m` | altitude above WGS84 ellipsoid | m |
| 3 | `pn_m` | NED north position | m |
| 4 | `pe_m` | NED east position | m |
| 5 | `pd_m` | NED down position | m |
| 6 | `vn_mps` | NED north velocity | m/s |
| 7 | `ve_mps` | NED east velocity | m/s |
| 8 | `vd_mps` | NED down velocity | m/s |
| 9 | `u_mps` | body forward velocity | m/s |
| 10 | `v_mps` | body right velocity | m/s |
| 11 | `w_mps` | body down velocity | m/s |
| 12 | `qw` | quaternion scalar component | double |
| 13 | `qx` | quaternion x component | double |
| 14 | `qy` | quaternion y component | double |
| 15 | `qz` | quaternion z component | double |
| 16 | `roll_rad` | roll angle | rad |
| 17 | `pitch_rad` | pitch angle | rad |
| 18 | `yaw_rad` | yaw angle | rad |
| 19 | `p_radps` | body roll rate | rad/s |
| 20 | `q_radps` | body pitch rate | rad/s |
| 21 | `r_radps` | body yaw rate | rad/s |
| 22 | `an_mps2` | NED north acceleration | m/s² |
| 23 | `ae_mps2` | NED east acceleration | m/s² |
| 24 | `ad_mps2` | NED down acceleration | m/s² |
| 25 | `ax_body_mps2` | body forward acceleration | m/s² |
| 26 | `ay_body_mps2` | body right acceleration | m/s² |
| 27 | `az_body_mps2` | body down acceleration | m/s² |

### 9.2 Example

```json
{"b":0,"x":[0.651733,2.216568,970.0,18.2,-6.5,-850.0,-1.2,0.4,72.5,72.4,0.1,1.8,0.9998,0.0030,0.0190,0.0010,0.0061,0.0380,0.0021,0.0040,-0.0180,0.0020,0.1,-0.2,-12.8,0.5,0.0,-12.7]}
```

### 9.3 Fundamental Completeness Rule

A body state sample must include every fundamental field.

A visualizer must use the fundamental maneuver state as the guaranteed source of body motion.

---

## 10. Action Spec and Action Sample

Actions are standardized optional records.

Actions are used to log what an autonomy model, policy, controller, planner, or decision process produced, and how that output was interpreted into commands.

### 10.1 Action Spec Form

```json
{"$":"action_spec","id":0,"b":0,"fields":["raw_action_0","raw_action_1","raw_action_2","throttle_cmd","tvc_x_cmd","tvc_y_cmd"]}
```

### 10.2 Action Spec Fields

| Field | Required | Meaning |
|---|---:|---|
| `$` | yes | must be `"action_spec"` |
| `id` | yes | unique action spec ID |
| `b` | yes | body ID |
| `fields` | yes | ordered action field names |
| `meta` | optional | project-specific action metadata |

### 10.3 Action Sample Form

```json
{"a":0,"x":[0.12,-0.44,0.91,0.58,0.012,-0.018]}
```

where:

```text
a = action spec ID
x = ordered action values
```

### 10.4 Action Rules

- `len(x)` must equal `len(action_spec.fields)`.
- An action spec may combine raw model output, scaled action, semantic command, and vehicle command in one vector.
- Field names must carry the meaning.

Example interpretation:

```text
raw_action_0 = 0.12
raw_action_1 = -0.44
raw_action_2 = 0.91
throttle_cmd = 0.58
tvc_x_cmd = 0.012
tvc_y_cmd = -0.018
```

---

## 11. Reward Spec and Reward Sample

Rewards are standardized optional records.

Rewards are used to log task feedback, especially for reinforcement learning, evaluation, and autonomy analysis.

### 11.1 Reward Spec Form

```json
{"$":"reward_spec","id":0,"b":0,"fields":["step_reward","pos_err_reward","vel_err_reward","att_err_reward","fuel_penalty","terminal_bonus"]}
```

### 11.2 Reward Spec Fields

| Field | Required | Meaning |
|---|---:|---|
| `$` | yes | must be `"reward_spec"` |
| `id` | yes | unique reward spec ID |
| `b` | optional | body ID when reward belongs to one body |
| `fields` | yes | ordered reward field names |
| `meta` | optional | project-specific reward metadata |

### 11.3 Reward Field Rule

The first reward field must be:

```text
step_reward
```

The first reward value must be the scalar reward for the current step.

All following reward fields are task-defined.

### 11.4 Reward Sample Form

```json
{"r":0,"x":[-0.812,-0.215,-0.441,-0.031,-0.058,0.000]}
```

where:

```text
r = reward spec ID
x = ordered reward values
```

### 11.5 Reward Rules

- `fields[0]` must equal `step_reward`.
- `x[0]` must be the current step reward.
- `len(x)` must equal `len(reward_spec.fields)`.
- Episode return can be computed by summing `step_reward` over steps.

---

## 12. Event Record

Events are standardized optional records.

Events annotate the maneuver timeline with readable semantic information.

### 12.1 Form

```json
{"$":"event","b":0,"topic":"terminal","data":{"success":true,"reason":"soft_landing"}}
```

### 12.2 Fields

| Field | Required | Meaning |
|---|---:|---|
| `$` | yes | must be `"event"` |
| `topic` | yes | event topic in snake_case |
| `data` | yes | event payload object |
| `b` | optional | related body ID |
| `s` | optional | explicit step index override |
| `t` | optional | explicit time override |
| `meta` | optional | project-specific event metadata |

### 12.3 Event Examples

```json
{"$":"event","b":0,"topic":"phase","data":{"from":"coast","to":"terminal_descent"}}
```

```json
{"$":"event","b":0,"topic":"landing_contact","data":{"vertical_speed_mps":-0.62,"lateral_speed_mps":0.18,"tilt_rad":0.006}}
```

```json
{"$":"event","b":0,"topic":"terminal","data":{"success":true,"reason":"soft_landing"}}
```

Visualizer tools may display events as timeline markers, text, overlays, or plots.

---

## 13. Complete Example

```ndjson
{"$":"header","format":1,"label":"run_20260429_landing_eval_001","origin_lla":[0.651733,2.216568,120.0],"created_utc":"2026-04-29T11:20:00Z","producer":"project-nightfall-rocketsim","mode":"simulation","scenario":"rocket_landing_with_chase_quad","seed":42017}
{"$":"body","id":0,"name":"booster_0","platform":"rocket","model":"nightfall_generic_vtv_booster"}
{"$":"body","id":1,"name":"quad_0","platform":"quadcopter","model":"nightfall_chase_quad_mk1"}
{"$":"action_spec","id":0,"b":0,"fields":["raw_action_0","raw_action_1","raw_action_2","throttle_cmd","tvc_x_cmd","tvc_y_cmd"]}
{"$":"reward_spec","id":0,"b":0,"fields":["step_reward","pos_err_reward","vel_err_reward","att_err_reward","fuel_penalty","terminal_bonus"]}
{"$":"step","s":1,"t":0.000000}
{"$":"event","b":0,"topic":"phase","data":{"from":"coast","to":"terminal_descent"}}
{"b":0,"x":[0.651733,2.216568,970.0,18.2,-6.5,-850.0,-1.2,0.4,72.5,72.4,0.1,1.8,0.9998,0.0030,0.0190,0.0010,0.0061,0.0380,0.0021,0.0040,-0.0180,0.0020,0.1,-0.2,-12.8,0.5,0.0,-12.7]}
{"b":1,"x":[0.651730,2.216560,240.0,-45.0,22.0,-120.0,5.0,-1.5,0.0,5.2,0.1,-0.2,0.9910,0.0000,0.0000,0.1340,0.0000,0.0000,0.2688,0.0,0.0,0.0,0.2,-0.1,0.0,0.2,-0.1,0.0]}
{"a":0,"x":[0.12,-0.44,0.91,0.58,0.012,-0.018]}
{"r":0,"x":[-0.812,-0.215,-0.441,-0.031,-0.058,0.000]}
{"$":"step","s":2,"t":0.001600}
{"b":0,"x":[0.651733,2.216568,969.884,18.198,-6.499,-849.884,-1.202,0.401,72.486,72.38,0.1,1.8,0.9998,0.0030,0.0190,0.0010,0.0061,0.0380,0.0021,0.0040,-0.0180,0.0020,0.1,-0.2,-12.8,0.5,0.0,-12.7]}
{"a":0,"x":[0.10,-0.42,0.88,0.59,0.012,-0.017]}
{"r":0,"x":[-0.801,-0.214,-0.432,-0.031,-0.059,0.000]}
{"$":"step","s":10188,"t":16.300000}
{"$":"event","b":0,"topic":"terminal","data":{"success":true,"reason":"soft_landing","landing_error_m":0.144,"vertical_speed_mps":-0.62,"lateral_speed_mps":0.18,"tilt_rad":0.006}}
```

---

## 14. Visualizer Contract

A visualizer must animate body motion using only the fundamental maneuver state records:

```json
{"b":body_id,"x":[28 fundamental values]}
```

A visualizer may display actions, rewards, events, and optional extensions as:

- timeline markers
- text overlays
- side panels
- plots
- debug tables

The guaranteed behavior layer is the fundamental maneuver state.

---

## 15. Reader Requirements

An MLC v1 reader must:

1. Parse the file line by line as NDJSON.
2. Read the header first.
3. Store body definitions by body ID.
4. Store action specs by action spec ID.
5. Store reward specs by reward spec ID.
6. Track the current step from step records.
7. Parse body state samples using the fixed 28-field order.
8. Interpret all fundamental state numeric values as double precision.
9. Validate action samples against action specs.
10. Validate reward samples against reward specs.
11. Interpret events using topic and data.
12. Associate step-scoped records with the latest step unless they include explicit `s` or `t`.

---

## 16. Writer Requirements

An MLC v1 writer must:

1. Write a header record first.
2. Write body records before body state samples.
3. Write action specs before action samples.
4. Write reward specs before reward samples.
5. Write step records to establish time context.
6. Write one complete 28-value fundamental state vector for each body state sample.
7. Write fundamental numeric values using double precision.
8. Write action samples with the same field count as their action spec.
9. Write reward samples with the same field count as their reward spec.
10. Write reward specs with `step_reward` as the first field.
11. Write events as readable JSON objects.
12. Write one JSON object per line.

---

## 17. Final Summary

MLC v1 is built around a strict center and flexible edges.

Strict center:

```text
header
body
step
fundamental maneuver state
```

Standard optional records:

```text
action_spec
action sample
reward_spec
reward sample
event
```

Flexible extensions:

```text
any project-specific record added later
```

The core promise is simple:

> If a tool understands MLC v1, it can reconstruct and visualize the maneuver of every logged body from the fixed fundamental state.

---

## Appendix A. Topic Convention: `target` Gate Geometry

This appendix is a usage convention inside MLC v1. It does not change any record type or the fundamental state, and it does not change `format`. Readers that do not know these fields keep working.

A fly-through gate is logged as:

1. a `body` record with `"role":"target"` (and `"platform":"other"` for a static gate), with a state sample every step (static pose), and
2. one `event` with `"topic":"target"` whose `data` carries the geometry the body record cannot.

### A.1 Form

```json
{"$":"event","b":0,"topic":"target","data":{"name":"gate_center","position_ned":[20.0,0.0,-30.0],"lla":[3.1568e-06,0.0,30.0],"yaw_rad":3.141592653589793,"gate_body":1,"aperture_m":[2.0,2.0],"aperture_uv_m":[[-1.0,-1.0],[1.0,-1.0],[1.0,1.0],[-1.0,1.0]],"frame_thickness_m":0.1}}
```

### A.2 `data` Fields

| Field | Required | Meaning |
|---|---:|---|
| `name` | recommended | readable name of the target point |
| `position_ned` | recommended | gate center in NED meters (same as the gate body state) |
| `lla` | optional | gate center as `[lat_rad, lon_rad, alt_m]` |
| `yaw_rad` | optional | gate yaw, informative only (the gate body quaternion is authoritative) |
| `gate_body` | recommended | body ID of the gate this geometry belongs to |
| `aperture_uv_m` | recommended | closed opening outline `[[u, v], ...]` or `[[u, v, w], ...]` in meters, at least 3 vertices |
| `frame_thickness_m` | optional | frame bar thickness in meters |
| `aperture_m` | recommended | `[width, height]` bounding box of the outline, for readers without `aperture_uv_m` |

The event `b` keeps its Section 12 meaning (the related body, usually the primary vehicle). The gate is identified by `gate_body`.

### A.3 Outline Convention

```text
Frame:     gate body frame (FRD, Section 3.3), origin at the gate body position
u:         along body y (right)
v:         along body z (down)
w:         optional third value, along body x (the passage axis); missing = 0
Opening:   the body y-z plane; the passage axis is body x
Vertices:  one closed loop, last vertex joins the first, no self-intersection
Winding:   either direction
Shape:     convex or concave; curves are sampled as polygons (e.g. 48 vertices for a circle)
```

The outline is the opening the pass check uses. The frame lies outside it, `frame_thickness_m` wide.

When `aperture_uv_m` is absent, readers use the rectangle `aperture_m` centered on the gate body.

### A.4 Non-Flat (Twisted) Gates

A vertex may carry a third value `w`, which moves it along body x. With `w` the outline is a closed loop in 3D, for example a square whose corners are pushed alternately forward and back:

```json
"aperture_uv_m":[[-1.0,-1.0,0.5],[1.0,-1.0,-0.5],[1.0,1.0,0.5],[-1.0,1.0,-0.5]]
```

```text
Projection: the (u, v) loop, seen along body x, must still be a simple closed polygon
Mixing:     vertices without w use w = 0
Readers:    a reader that ignores w shows the flat projection; this stays valid MLC v1
aperture_m: still the (u, v) bounding box
```

`w` describes the frame shape only. How a pass through a non-flat opening is judged is up to the producer and is not defined here.

---

## Appendix B. Topic Convention: `camera` and `gimbal`

This appendix is a usage convention inside MLC v1. It does not change any record type or the fundamental state, and it does not change `format`. Readers that do not know these topics keep working.

A body that carries a camera (optionally on a gimbal) logs:

1. one `event` with `"topic":"camera"` before the first step: the camera and its mounting, and
2. one `event` with `"topic":"gimbal"` per step (or whenever the angles change): where the camera actually points.

Both events use `b` = the body the camera is mounted on.

### B.1 Form

```json
{"$":"event","b":0,"topic":"camera","data":{"name":"d435","hfov_rad":1.211259,"vfov_rad":0.742833,"image_px":[1280,720],"mount_offset_frd_m":[0.15,-0.03,0.01],"gimbal":"yaw_pitch","gimbal_limits_rad":[-1.047198,1.047198],"gimbal_yaw_limits_rad":[-1.570796,1.570796]}}
{"$":"event","b":0,"topic":"gimbal","data":{"pitch_rad":-0.466263,"yaw_rad":0.001511}}
```

### B.2 `camera` `data` Fields

| Field | Required | Meaning |
|---|---:|---|
| `name` | optional | camera name |
| `hfov_rad` | yes | horizontal field of view, radians |
| `vfov_rad` | recommended | vertical field of view, radians (readers may assume 16:9 from `hfov_rad` without it) |
| `image_px` | optional | `[width, height]` in pixels |
| `mount_offset_frd_m` | recommended | gimbal pivot (or fixed camera) position in the body frame (FRD, Section 3.3), meters |
| `gimbal` | recommended | gimbal axes: `"none"` (fixed camera), `"pitch"` (tilt only), `"yaw_pitch"` (pan-tilt) |
| `gimbal_limits_rad` | optional | `[min, max]` pitch travel, radians |
| `gimbal_yaw_limits_rad` | optional | `[min, max]` yaw travel, radians |

### B.3 `gimbal` `data` Fields

| Field | Required | Meaning |
|---|---:|---|
| `pitch_rad` | when not 0 | camera tilt, radians: rotation about body y, `+` = camera up, `0` = along the body nose (x) |
| `yaw_rad` | when not 0 | camera pan, radians: rotation about body z, `+` = camera right, `0` = along the body nose (x) |

```text
Order:     yaw (pan) first, then pitch (tilt) about the panned axis - a pan-tilt gimbal
Rest:      yaw 0, pitch 0 = the camera looks along body x (nose)
Whole:     one event is the whole gimbal state; an axis it leaves out is 0
Holding:   the angles hold until the next gimbal event of the same body
Values:    the direction the camera's optical axis actually has relative to the body, INCLUDING any fixed
           mounting tilt of the camera (not the commanded angles; commands belong in actions)
Range:     any angle; producers should send (-pi, pi]
```

The angles are the camera direction, not only the gimbal travel: a camera mounted 25 deg up on a gimbal at 0 logs `pitch_rad` 0.436. There is no separate mounting rotation field. A pitch-only gimbal sends `pitch_rad` only. A fixed camera (`"gimbal":"none"`) along the body nose sends no `gimbal` events; a fixed camera mounted at an angle sends one `gimbal` event (before or in the first step) with that angle, for example an FPV camera tilted 25 deg up:

```json
{"$":"event","b":0,"topic":"camera","data":{"hfov_rad":2.094395,"mount_offset_frd_m":[0.08,0.0,-0.02],"gimbal":"none"}}
{"$":"event","b":0,"topic":"gimbal","data":{"pitch_rad":0.436332}}
```

### B.4 Placement

Write the `camera` event after the body records and before the first `step`. Write a `gimbal` event inside the step it belongs to, after the body's state sample. The viewer applies a `gimbal` event to the state of the same step whether it comes before or after the state sample.

---

## Appendix C. Body `model` Names

This appendix is a naming convention inside MLC v1. It does not change any record type and does not change `format`. `model` stays optional (Section 7.2); any string remains valid MLC.

`model` tells a visualizer which airframe to draw. The names below are the ones the RL Cinema viewer recognizes. Using one of them gets that airframe; any other name (or no `model`) is still drawn, as a generic body.

### C.1 Rules

```text
Spelling:   lowercase ASCII, digits, "_" or "-", no spaces
Matching:   readers compare case-insensitively
Unknown:    a name not in the list is valid; the body is shown with a generic shape
New names:  add them to this table before producers start using them
```

### C.2 Names

| `model` | Airframe | Live telemetry `vehicle_type_id` |
|---|---|---:|
| `coyote` | Coyote X26 | 0 |
| `f16` | DALE F-16 | 10 |
| `f16new` | DALE F-16 (new) | 11 |
| `xq58a` | DALE XQ-58A | 12 |
| `suca` | DALE SUCA | 13 |
| `mucca` | DALE MUCCA | 14 |
| `kf21` | DALE KF-21 | 15 |
| `vtol` | DALE VTOL (Neptune V370) | 16 |
| `lah` | DALE LAH | 17 |
| `basic_quad` | generic quadcopter | 20 |
| `mbqd_xp23` | XP23 quadcopter (same airframe as XP26, older simulator engine) | 21 |
| `mbqd_xp26` | XP26 quadcopter | 21 |
| `gbu-39` | GBU-39 glide bomb | - |

The `vehicle_type_id` column links the name to the live telemetry ID of the same airframe, so a replay and a live session show the same model.

`muca` is an older spelling of `mucca`; readers should accept both, writers should use `mucca`.

### C.3 Example

```json
{"$":"body","id":0,"name":"ownship","platform":"fixed_wing","model":"coyote","role":"ownship"}
{"$":"body","id":1,"name":"quad_0","platform":"quadcopter","model":"basic_quad"}
```
