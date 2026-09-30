"""Write the appendix examples: MLC logs that use Appendix A (gate), B (camera / gimbal) and C (model names).

  example_quad_gate_camera.mlc.ndjson       quadcopter "mbqd_xp26" flying through a twisted star gate, a pan-tilt camera
                                             tracking the gate (camera + per-step gimbal events), CTBR actions
  example_fixedwing_fixed_camera.mlc.ndjson fixed-wing "coyote" circling a waypoint, a fixed camera mounted 30 deg
                                             down (one gimbal event carrying the mounting angle)
  example_quad_round_gate_miss.mlc.ndjson   quadcopter "basic_quad" drifting past a round gate (a miss), a
                                             tilt-only camera (gimbal "pitch", pitch_rad only)
  example_suca_track_vehicle.mlc.ndjson     fixed-wing UAV "suca" orbiting a moving ground vehicle, a pan-tilt
                                             belly camera tracking it
  example_quad_gate_course.mlc.ndjson       quadcopter "mbqd_xp23" racing three gate bodies (triangle, arrow,
                                             twisted square), one target event per gate tied by gate_body,
                                             fixed FPV camera tilted up
  example_formation_turn.mlc.ndjson         formation: "kf21" lead and two "xq58a" wingmen turning, V to
                                             echelon right; one action / reward spec per body
  example_heli_recon.mlc.ndjson             helicopter "lah": ingress and flare to hover, gimbal search sweep,
                                             detect a vehicle, pedal turn and track; phase / detect events
  example_vtol_transition.mlc.ndjson        VTOL "vtol": vertical takeoff, transition to fixed-wing, cruise with
                                             a turn, back-transition, vertical landing; landing_contact / terminal
  example_quad_moving_pad_landing.mlc.ndjson quadcopter "basic_quad" landing on a moving truck: gust, go_around,
                                             second approach, landing_contact relative to the truck
  example_glide_bomb_release.mlc.ndjson     Appendix D: a "kf21" releases two "gbu-39" glide bombs that exist only
                                             from release to impact; spawn / despawn events

Every state sample is kinematically consistent: position, velocity, attitude (quaternion and Euler angles), body
rates and accelerations come from one analytic trajectory. Run from the repository root:

  python v1/examples/write_appendix_examples.py
"""
import math
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from v1.python.mlc import MLCWriter

HERE = Path(__file__).resolve().parent
G = 9.80665
EARTH_R = 6378137.0
ORIGIN_LLA = [math.radians(36.5), math.radians(127.5), 100.0]   # lat_rad, lon_rad, alt_m


# --------------------------------------------------------------------------------------------------------------------
# Kinematics helpers (NED world, FRD body, quaternion = body -> NED, [w, x, y, z])
# --------------------------------------------------------------------------------------------------------------------

def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy]


def body_to_ned_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
            [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
            [-sp, sr * cp, cr * cp]]


def ned_to_body(m, v):
    return [sum(m[r][c] * v[r] for r in range(3)) for c in range(3)]


def lla_from_ned(n, e, d):
    lat0, lon0, alt0 = ORIGIN_LLA
    return [lat0 + n / EARTH_R, lon0 + e / (EARTH_R * math.cos(lat0)), alt0 - d]


def state_vector(pos, vel, acc, att, att_rate):
    """28 fundamental values from NED position / velocity / acceleration and Euler angles with their rates."""
    roll, pitch, yaw = att
    droll, dpitch, dyaw = att_rate
    m = body_to_ned_matrix(roll, pitch, yaw)
    uvw = ned_to_body(m, vel)
    acc_b = ned_to_body(m, acc)
    # Euler rates -> body rates p, q, r
    p = droll - dyaw * math.sin(pitch)
    q = dpitch * math.cos(roll) + dyaw * math.cos(pitch) * math.sin(roll)
    r = -dpitch * math.sin(roll) + dyaw * math.cos(pitch) * math.cos(roll)
    return (lla_from_ned(*pos) + list(pos) + list(vel) + uvw + euler_to_quat(roll, pitch, yaw)
            + [roll, pitch, yaw, p, q, r] + list(acc) + acc_b)


def sample(f, t, dt=1e-3):
    """Position, velocity, acceleration of a trajectory function f(t) -> (n, e, d) by central differences."""
    p0, p1, p2 = f(t - dt), f(t), f(t + dt)
    vel = [(b - a) / (2 * dt) for a, b in zip(p0, p2)]
    acc = [(a - 2 * b + c) / (dt * dt) for a, b, c in zip(p0, p1, p2)]
    return list(p1), vel, acc


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rounded(values, digits=6):
    return [round(v, digits) for v in values]


# --------------------------------------------------------------------------------------------------------------------
# Example 1: quadcopter through a gate, pan-tilt camera tracking the gate
# --------------------------------------------------------------------------------------------------------------------

def write_quad_gate_camera():
    rate, duration = 20.0, 10.0
    gate_ned = (25.0, 0.0, -3.0)          # gate centre, 3 m above the origin, passage axis = north
    mount_frd = (0.15, 0.0, 0.03)         # camera on the nose, slightly below the body centre
    hfov = math.radians(69.4)
    vfov = 2.0 * math.atan(math.tan(hfov / 2) * 720 / 1280)
    yaw_lim, pitch_lim = math.radians(90.0), math.radians(60.0)
    max_rate = 3.0                         # rad/s, normalises the CTBR rate commands

    t_gate = 5.0 + (gate_ned[0] - 15.0) / 6.0      # when the north position reaches the gate

    def traj(t):
        # accelerate north through the gate, climbing to the gate height and weaving east-west on the
        # way in; the weave is back to zero exactly at the gate, so the pass is through the centre
        n = 0.5 * 1.2 * t * t if t < 5 else 15.0 + 6.0 * (t - 5)
        e = 1.2 * math.sin(2.0 * math.pi * t / t_gate)
        d = -3.0 + 2.0 * math.exp(-t)
        return (n, e, d)

    # Appendix A: a twisted five-point star. Outline in the gate body frame, u = right, v = down (m),
    # w = along the passage axis: the tips lean 0.3 m forward, the inner corners 0.3 m back.
    star = []
    for k in range(10):
        a = -math.pi / 2 + k * math.pi / 5          # first tip straight up (v negative)
        rad, w = (2.4, 0.3) if k % 2 == 0 else (1.1, -0.3)
        star.append([round(rad * math.cos(a), 4), round(rad * math.sin(a), 4), w])
    us, vs = [p[0] for p in star], [p[1] for p in star]
    aperture_bbox = [round(max(us) - min(us), 4), round(max(vs) - min(vs), 4)]

    def attitude(vel, acc):
        yaw = 0.0   # nose kept on the gate (north); a quad does not need to face its velocity
        # a quad tilts along the horizontal acceleration: forward -> nose down, right -> right wing down
        cy, sy = math.cos(yaw), math.sin(yaw)
        a_fwd = cy * acc[0] + sy * acc[1]
        a_right = -sy * acc[0] + cy * acc[1]
        return (math.atan2(a_right, G), -math.atan2(a_fwd, G), yaw)

    def att_at(t):
        _, v, a = sample(traj, t)
        return attitude(v, a)

    path = HERE / "example_quad_gate_camera.mlc.ndjson"
    with MLCWriter(str(path), label="example_quad_gate_camera", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="twisted star gate pass with a pan-tilt camera", seed=0) as log:
        quad = log.body("uav_0", platform="quadcopter", model="mbqd_xp26", role="primary")      # Appendix C
        gate = log.body("gate_0", platform="other", role="target")                                  # Appendix A
        act = log.action_spec(quad, ["thrust_norm", "p_cmd_norm", "q_cmd_norm", "r_cmd_norm"])
        rew = log.reward_spec(["step_reward", "progress", "gate_alignment", "terminal"], body_id=quad)

        passed = False
        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            pos, vel, acc = sample(traj, t)
            att = attitude(vel, acc)
            h = 1e-3
            a0, a1 = att_at(t - h), att_at(t + h)
            att_rate = [wrap(b - a) / (2 * h) for a, b in zip(a0, a1)]
            quad_x = state_vector(pos, vel, acc, att, att_rate)
            log.state(quad, rounded(quad_x))
            log.state(gate, rounded(state_vector(gate_ned, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0))))

            if i == 0:
                # Appendix B: the camera, once. Appendix A: the gate geometry, once.
                log.event("camera", {
                    "name": "d435",
                    "hfov_rad": round(hfov, 6), "vfov_rad": round(vfov, 6), "image_px": [1280, 720],
                    "mount_offset_frd_m": list(mount_frd),
                    "gimbal": "yaw_pitch",
                    "gimbal_limits_rad": rounded([-pitch_lim, pitch_lim]),
                    "gimbal_yaw_limits_rad": rounded([-yaw_lim, yaw_lim]),
                }, body_id=quad)
                log.event("target", {
                    "name": "gate_center",
                    "position_ned": list(gate_ned),
                    "lla": rounded(lla_from_ned(*gate_ned), 9),
                    "yaw_rad": 0.0,
                    "gate_body": gate,
                    "aperture_m": aperture_bbox,
                    "aperture_uv_m": star,
                    "frame_thickness_m": 0.12,
                }, body_id=quad)

            # Appendix B: where the camera points (body frame), tracking the gate while it is ahead
            m = body_to_ned_matrix(*att)
            cam_ned = [pos[k] + sum(m[k][j] * mount_frd[j] for j in range(3)) for k in range(3)]
            to_gate = ned_to_body(m, [g - c for g, c in zip(gate_ned, cam_ned)])
            if to_gate[0] > 0.5:
                yaw = math.atan2(to_gate[1], to_gate[0])
                pitch = math.atan2(-to_gate[2], math.hypot(to_gate[0], to_gate[1]))
            else:
                yaw, pitch = 0.0, 0.0      # gate behind: look ahead again
            yaw = max(-yaw_lim, min(yaw_lim, yaw))
            pitch = max(-pitch_lim, min(pitch_lim, pitch))
            log.event("gimbal", {"pitch_rad": round(pitch, 6), "yaw_rad": round(yaw, 6)}, body_id=quad)

            if not passed and pos[0] >= gate_ned[0]:
                passed = True
                log.event("gate_pass", {"gate_body": gate}, body_id=quad)

            # CTBR: collective thrust (hover ~ 0.5) and normalised body-rate commands
            p, q, r = quad_x[19:22]
            thrust = 0.5 * math.sqrt(acc[0] ** 2 + acc[1] ** 2 + (G - acc[2]) ** 2) / G
            log.action(act, rounded([thrust, p / max_rate, q / max_rate, r / max_rate]))
            progress = 0.05 * vel[0]
            alignment = -0.02 * abs(pos[1] - gate_ned[1])
            terminal = 1.0 if (passed and pos[0] - gate_ned[0] < 6.0 / rate) else 0.0
            log.reward(rew, rounded([progress + alignment + terminal, progress, alignment, terminal]))
    return path


# --------------------------------------------------------------------------------------------------------------------
# Example 2: fixed-wing circling a waypoint, fixed camera mounted 30 deg down
# --------------------------------------------------------------------------------------------------------------------

def write_fixedwing_fixed_camera():
    rate, duration = 20.0, 20.0
    radius, speed, height = 150.0, 25.0, 120.0
    omega = speed / radius
    center = (0.0, 0.0)
    mount_pitch = math.radians(-30.0)       # camera mounted 30 deg down, no gimbal

    def traj(t):
        a = omega * t
        return (center[0] - radius * math.cos(a), center[1] - radius * math.sin(a), -height)   # clockwise seen from above

    def attitude(vel):
        yaw = math.atan2(vel[1], vel[0])
        roll = math.atan(speed * omega / G)       # coordinated right turn
        return (roll, 0.0, yaw)

    path = HERE / "example_fixedwing_fixed_camera.mlc.ndjson"
    with MLCWriter(str(path), label="example_fixedwing_fixed_camera", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="loiter over a waypoint with a fixed down-tilted camera", seed=0) as log:
        uav = log.body("ownship", platform="fixed_wing", model="coyote", role="ownship")   # Appendix C
        wp = log.body("waypoint", platform="other", role="target")
        act = log.action_spec(uav, ["p_rate_cmd", "q_rate_cmd", "r_rate_cmd"])
        rew = log.reward_spec(["step_reward", "radius_error", "altitude_error"], body_id=uav)

        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            pos, vel, acc = sample(traj, t)
            att = attitude(vel)
            x = state_vector(pos, vel, acc, att, (0.0, 0.0, omega))
            log.state(uav, rounded(x))
            log.state(wp, rounded(state_vector((center[0], center[1], 0.0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0))))
            if i == 0:
                # Appendix B: a fixed camera ("gimbal":"none") mounted at an angle sends that angle once
                log.event("camera", {
                    "name": "eo_fixed",
                    "hfov_rad": round(math.radians(40.0), 6),
                    "mount_offset_frd_m": [0.35, 0.0, 0.05],
                    "gimbal": "none",
                }, body_id=uav)
                log.event("gimbal", {"pitch_rad": round(mount_pitch, 6)}, body_id=uav)
            log.action(act, rounded(x[19:22]))
            radius_err = math.hypot(pos[0] - center[0], pos[1] - center[1]) - radius
            alt_err = -pos[2] - height
            log.reward(rew, rounded([-abs(radius_err) - abs(alt_err), radius_err, alt_err]))
    return path


# --------------------------------------------------------------------------------------------------------------------
# Example 3: quadcopter missing a round gate, tilt-only camera
# --------------------------------------------------------------------------------------------------------------------

def write_quad_round_gate_miss():
    rate, duration = 20.0, 8.0
    gate_ned = (20.0, 0.0, -3.0)          # round gate, 3 m above the origin, passage axis = north
    gate_r = 1.25
    mount_frd = (0.12, 0.0, 0.02)
    hfov = math.radians(87.0)
    pitch_lim = math.radians(90.0)
    max_rate = 3.0
    t_gate = 20.0 / 4.0                   # 4 m/s north: reaches the gate plane at 5 s
    miss = 1.8                            # drifted this far east at the gate: outside the 1.25 m ring

    def traj(t):
        n = 4.0 * t
        e = miss * (t / t_gate) ** 2      # a growing drift the controller never corrects
        d = -1.2 + 0.7 * math.exp(-1.5 * t)   # stays low (1.2 m): the camera has to tilt up to the gate
        return (n, e, d)

    def attitude(vel, acc):
        # nose kept north; tilt along the horizontal acceleration
        return (math.atan2(acc[1], G), -math.atan2(acc[0], G), 0.0)

    def att_at(t):
        _, v, a = sample(traj, t)
        return attitude(v, a)

    ring = [[round(gate_r * math.cos(2 * math.pi * k / 32), 4), round(gate_r * math.sin(2 * math.pi * k / 32), 4)]
            for k in range(32)]

    path = HERE / "example_quad_round_gate_miss.mlc.ndjson"
    with MLCWriter(str(path), label="example_quad_round_gate_miss", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="round gate missed, tilt-only camera", seed=0) as log:
        quad = log.body("uav_0", platform="quadcopter", model="basic_quad", role="primary")      # Appendix C
        gate = log.body("gate_0", platform="other", role="target")                                  # Appendix A
        act = log.action_spec(quad, ["thrust_norm", "p_cmd_norm", "q_cmd_norm", "r_cmd_norm"])
        rew = log.reward_spec(["step_reward", "progress", "gate_alignment", "terminal"], body_id=quad)

        crossed = False
        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            pos, vel, acc = sample(traj, t)
            att = attitude(vel, acc)
            h = 1e-3
            a0, a1 = att_at(t - h), att_at(t + h)
            att_rate = [wrap(b - a) / (2 * h) for a, b in zip(a0, a1)]
            quad_x = state_vector(pos, vel, acc, att, att_rate)
            log.state(quad, rounded(quad_x))
            log.state(gate, rounded(state_vector(gate_ned, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0))))

            if i == 0:
                # Appendix B: a tilt-only gimbal. Appendix A: a round gate (a flat 32-point outline).
                log.event("camera", {
                    "name": "fpv_wide",
                    "hfov_rad": round(hfov, 6),
                    "mount_offset_frd_m": list(mount_frd),
                    "gimbal": "pitch",
                    "gimbal_limits_rad": rounded([-pitch_lim, pitch_lim]),
                }, body_id=quad)
                log.event("target", {
                    "name": "gate_center",
                    "position_ned": list(gate_ned),
                    "lla": rounded(lla_from_ned(*gate_ned), 9),
                    "yaw_rad": 0.0,
                    "gate_body": gate,
                    "aperture_m": [2 * gate_r, 2 * gate_r],
                    "aperture_uv_m": ring,
                    "frame_thickness_m": 0.1,
                }, body_id=quad)

            # Appendix B: tilt only - the camera follows the gate up / down in the body x-z plane, no yaw_rad
            m = body_to_ned_matrix(*att)
            cam_ned = [pos[k] + sum(m[k][j] * mount_frd[j] for j in range(3)) for k in range(3)]
            to_gate = ned_to_body(m, [g - c for g, c in zip(gate_ned, cam_ned)])
            pitch = math.atan2(-to_gate[2], to_gate[0]) if to_gate[0] > 0.5 else 0.0
            pitch = max(-pitch_lim, min(pitch_lim, pitch))
            log.event("gimbal", {"pitch_rad": round(pitch, 6)}, body_id=quad)

            terminal = 0.0
            if not crossed and pos[0] >= gate_ned[0]:
                crossed = True
                off = math.hypot(pos[1] - gate_ned[1], pos[2] - gate_ned[2])
                # a producer-defined event: readers that do not know the topic skip it
                log.event("gate_miss", {"gate_body": gate, "miss_distance_m": round(off - gate_r, 3)}, body_id=quad)
                terminal = -1.0

            p, q, r = quad_x[19:22]
            thrust = 0.5 * math.sqrt(acc[0] ** 2 + acc[1] ** 2 + (G - acc[2]) ** 2) / G
            log.action(act, rounded([thrust, p / max_rate, q / max_rate, r / max_rate]))
            progress = 0.05 * vel[0]
            alignment = -0.05 * abs(pos[1] - gate_ned[1])
            log.reward(rew, rounded([progress + alignment + terminal, progress, alignment, terminal]))
    return path


# --------------------------------------------------------------------------------------------------------------------
# Example 4: fixed-wing UAV orbiting a moving ground vehicle, pan-tilt camera tracking it
# --------------------------------------------------------------------------------------------------------------------

def write_suca_track_vehicle():
    rate, duration = 20.0, 30.0
    height, radius, airspeed = 300.0, 400.0, 30.0
    car_speed = 5.0                        # the vehicle drives east
    omega = airspeed / radius              # orbit rate around the moving vehicle (right turn, clockwise)
    mount_frd = (1.2, 0.0, -0.02)         # gimbal ball at the nose (where the "suca" airframe carries it)
    hfov = math.radians(20.0)             # zoomed EO camera
    vfov = 2.0 * math.atan(math.tan(hfov / 2) * 9 / 16)
    pitch_lim = (math.radians(-90.0), math.radians(10.0))

    def car(t):
        return (0.0, car_speed * t, 0.0)

    def traj(t):
        c = car(t)
        a = omega * t
        return (c[0] - radius * math.cos(a), c[1] - radius * math.sin(a), -height)

    def heading(t):
        _, v, _ = sample(traj, t)
        return math.atan2(v[1], v[0])

    def attitude(t):
        _, v, _ = sample(traj, t)
        h = 1e-3
        yaw_rate = wrap(heading(t + h) - heading(t - h)) / (2 * h)
        roll = math.atan(math.hypot(v[0], v[1]) * yaw_rate / G)   # coordinated turn
        return (roll, 0.0, heading(t))

    path = HERE / "example_suca_track_vehicle.mlc.ndjson"
    with MLCWriter(str(path), label="example_suca_track_vehicle", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="orbit a moving vehicle, pan-tilt camera tracking it", seed=0) as log:
        uav = log.body("suca_0", platform="fixed_wing", model="suca", role="ownship")    # Appendix C
        veh = log.body("truck_0", platform="ground_vehicle", role="target")
        act = log.action_spec(uav, ["p_rate_cmd", "q_rate_cmd", "r_rate_cmd", "throttle_cmd"])
        rew = log.reward_spec(["step_reward", "range_error", "target_in_view"], body_id=uav)

        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            pos, vel, acc = sample(traj, t)
            att = attitude(t)
            h = 1e-3
            a0, a1 = attitude(t - h), attitude(t + h)
            att_rate = [wrap(b - a) / (2 * h) for a, b in zip(a0, a1)]
            x = state_vector(pos, vel, acc, att, att_rate)
            log.state(uav, rounded(x))
            cpos, cvel, cacc = sample(car, t)
            log.state(veh, rounded(state_vector(cpos, cvel, cacc, (0.0, 0.0, math.pi / 2), (0, 0, 0))))

            if i == 0:
                # Appendix B: a pan-tilt gimbal under the belly
                log.event("camera", {
                    "name": "eo_zoom",
                    "hfov_rad": round(hfov, 6), "vfov_rad": round(vfov, 6), "image_px": [1920, 1080],
                    "mount_offset_frd_m": list(mount_frd),
                    "gimbal": "yaw_pitch",
                    "gimbal_limits_rad": rounded(list(pitch_lim)),
                    "gimbal_yaw_limits_rad": rounded([-math.pi, math.pi]),
                }, body_id=uav)
                log.event("track_start", {"target_body": veh}, body_id=uav)   # a producer-defined event

            # Appendix B: pan (yaw_rad) and tilt (pitch_rad) that keep the vehicle on the optical axis
            m = body_to_ned_matrix(*att)
            cam_ned = [pos[k] + sum(m[k][j] * mount_frd[j] for j in range(3)) for k in range(3)]
            to_car = ned_to_body(m, [c - q for c, q in zip(cpos, cam_ned)])
            yaw = math.atan2(to_car[1], to_car[0])
            pitch = math.atan2(-to_car[2], math.hypot(to_car[0], to_car[1]))
            pitch = max(pitch_lim[0], min(pitch_lim[1], pitch))
            log.event("gimbal", {"pitch_rad": round(pitch, 6), "yaw_rad": round(yaw, 6)}, body_id=uav)

            p, q, r = x[19:22]
            log.action(act, rounded([p, q, r, 0.55]))
            rng = math.dist(cam_ned, cpos)
            range_err = rng - math.hypot(radius, height)
            log.reward(rew, rounded([1.0 - 0.001 * abs(range_err), range_err, 1.0]))
    return path


# --------------------------------------------------------------------------------------------------------------------
# Example 5: quadcopter racing a three-gate course, fixed FPV camera tilted up
# --------------------------------------------------------------------------------------------------------------------

def hermite(p0, p1, m0, m1, u):
    h00, h10 = 2 * u ** 3 - 3 * u ** 2 + 1, u ** 3 - 2 * u ** 2 + u
    h01, h11 = -2 * u ** 3 + 3 * u ** 2, u ** 3 - u ** 2
    return [h00 * a + h10 * b + h01 * c + h11 * d for a, b, c, d in zip(p0, m0, p1, m1)]


def write_quad_gate_course():
    rate, speed = 20.0, 6.0
    uptilt = math.radians(25.0)            # racing FPV camera, fixed 25 deg up

    # Appendix A outlines (gate body frame: u = right, v = down, optional w = along the passage axis)
    triangle = [[0.0, -1.25], [1.4, 1.25], [-1.4, 1.25]]                                   # apex up
    arrow = [[-0.5, 1.2], [-0.5, 0.0], [-1.2, 0.0], [0.0, -1.3], [1.2, 0.0], [0.5, 0.0], [0.5, 1.2]]   # points up
    twisted_square = [[-1.0, -1.0, 0.4], [1.0, -1.0, -0.4], [1.0, 1.0, 0.4], [-1.0, 1.0, -0.4]]
    # (name, outline, centre NED, yaw of the passage axis)
    gates = [("triangle", triangle, (15.0, 0.0, -3.0), 0.0),
             ("arrow", arrow, (30.0, 12.0, -4.0), math.radians(60.0)),
             ("twisted_square", twisted_square, (35.0, 30.0, -3.0), math.radians(90.0))]

    # Path: start -> each gate centre along its passage axis -> 10 m past the last gate (Hermite segments)
    start = (0.0, 0.0, -1.0)
    last_yaw = gates[-1][3]
    end = (gates[-1][2][0] + 10 * math.cos(last_yaw), gates[-1][2][1] + 10 * math.sin(last_yaw), gates[-1][2][2])
    knots = [start] + [g[2] for g in gates] + [end]
    dirs = ([(1.0, 0.0, 0.0)] + [(math.cos(g[3]), math.sin(g[3]), 0.0) for g in gates]
            + [(math.cos(last_yaw), math.sin(last_yaw), 0.0)])
    seg_t = [math.dist(a, b) / speed for a, b in zip(knots, knots[1:])]
    t_knot = [0.0]
    for d in seg_t:
        t_knot.append(t_knot[-1] + d)
    duration = t_knot[-1]

    def traj(t):
        # straight lines before the start and past the end, so the samples there stay smooth
        if t < 0.0:
            return tuple(a + c * speed * t for a, c in zip(knots[0], dirs[0]))
        if t > duration:
            return tuple(a + c * speed * (t - duration) for a, c in zip(knots[-1], dirs[-1]))
        k = min(len(seg_t) - 1, max(i for i in range(len(seg_t)) if t_knot[i] <= t))
        u = (t - t_knot[k]) / seg_t[k]
        scale = seg_t[k] * speed
        return tuple(hermite(knots[k], knots[k + 1], [c * scale for c in dirs[k]], [c * scale for c in dirs[k + 1]], u))

    def attitude(t):
        _, v, a = sample(traj, t)
        yaw = math.atan2(v[1], v[0])                     # racing: nose along the path
        cy, sy = math.cos(yaw), math.sin(yaw)
        a_fwd = cy * a[0] + sy * a[1]
        a_right = -sy * a[0] + cy * a[1]
        return (math.atan2(a_right, G), -math.atan2(a_fwd, G), yaw)

    path = HERE / "example_quad_gate_course.mlc.ndjson"
    with MLCWriter(str(path), label="example_quad_gate_course", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="three-gate course, fixed FPV camera", seed=0) as log:
        quad = log.body("racer_0", platform="quadcopter", model="mbqd_xp23", role="primary")      # Appendix C
        gate_ids = [log.body("gate_%d_%s" % (k, g[0]), platform="other", role="target") for k, g in enumerate(gates)]
        act = log.action_spec(quad, ["thrust_norm", "p_cmd_norm", "q_cmd_norm", "r_cmd_norm"])
        rew = log.reward_spec(["step_reward", "progress", "gate_bonus"], body_id=quad)

        passed = [False] * len(gates)
        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            pos, vel, acc = sample(traj, t)
            att = attitude(t)
            h = 1e-3
            a0, a1 = attitude(t - h), attitude(t + h)
            att_rate = [wrap(b - a) / (2 * h) for a, b in zip(a0, a1)]
            x = state_vector(pos, vel, acc, att, att_rate)
            log.state(quad, rounded(x))
            for gid, g in zip(gate_ids, gates):
                log.state(gid, rounded(state_vector(g[2], (0, 0, 0), (0, 0, 0), (0.0, 0.0, g[3]), (0, 0, 0))))

            if i == 0:
                # Appendix B: fixed camera, mounted 25 deg up -> one gimbal event with that angle
                log.event("camera", {"name": "fpv", "hfov_rad": round(math.radians(120.0), 6),
                                     "mount_offset_frd_m": [0.08, 0.0, -0.03], "gimbal": "none"}, body_id=quad)
                log.event("gimbal", {"pitch_rad": round(uptilt, 6)}, body_id=quad)
                # Appendix A: one target event per gate; gate_body ties each outline to its gate body
                for gid, (name, outline, centre, yaw) in zip(gate_ids, gates):
                    us, vs = [q[0] for q in outline], [q[1] for q in outline]
                    log.event("target", {
                        "name": name,
                        "position_ned": list(centre),
                        "lla": rounded(lla_from_ned(*centre), 9),
                        "yaw_rad": round(yaw, 6),
                        "gate_body": gid,
                        "aperture_m": [round(max(us) - min(us), 4), round(max(vs) - min(vs), 4)],
                        "aperture_uv_m": outline,
                        "frame_thickness_m": 0.1,
                    }, body_id=quad)

            bonus = 0.0
            for k, (gid, g) in enumerate(zip(gate_ids, gates)):
                along = (pos[0] - g[2][0]) * math.cos(g[3]) + (pos[1] - g[2][1]) * math.sin(g[3])
                if not passed[k] and along >= 0.0 and math.dist(pos, g[2]) < 3.0:
                    passed[k] = True
                    bonus = 1.0
                    log.event("gate_pass", {"gate_body": gid, "gate_index": k}, body_id=quad)

            p, q, r = x[19:22]
            thrust = 0.5 * math.sqrt(acc[0] ** 2 + acc[1] ** 2 + (G - acc[2]) ** 2) / G
            log.action(act, rounded([thrust, p / 3.0, q / 3.0, r / 3.0]))
            progress = 0.05 * math.hypot(vel[0], vel[1])
            log.reward(rew, rounded([progress + bonus, progress, bonus]))
    return path





# --------------------------------------------------------------------------------------------------------------------
# Example 6: three-ship formation (manned lead + two unmanned wingmen), per-body action / reward specs
# --------------------------------------------------------------------------------------------------------------------

def write_formation_turn():
    rate = 10.0                            # three bodies: 10 Hz keeps the file small
    speed, height, turn_r = 200.0, 1500.0, 2000.0
    w0, ramp = speed / turn_r, 3.0         # turn rate, roll-in / roll-out time
    t1 = 10.0                              # straight north, then a 90 deg right turn
    t2 = t1 + (math.pi / 2) / w0 + ramp    # turn rate ramps 0 -> w0 -> 0, so the heading change is exactly 90 deg
    t_change, t_blend = t2 + 2.0, 20.0     # then east: V -> echelon right over 20 s
    duration = t_change + t_blend + 5.0

    # slots in the lead's heading frame: (metres back, metres right)
    v_slots = [(150.0, -150.0), (150.0, 150.0)]
    echelon_slots = [(150.0, 150.0), (300.0, 300.0)]

    def ramp_area(u):                      # integral of smoothstep from 0 to u (in units of the ramp time)
        u = min(1.0, max(0.0, u))
        return u ** 3 - u ** 4 / 2

    def lead_heading(t):
        u_in, u_out = (t - t1) / ramp, (t - t2 + ramp) / ramp
        a = ramp * (ramp_area(u_in) + min(1.0, max(0.0, u_out)) - ramp_area(u_out))
        return w0 * (a + max(0.0, min(t, t2 - ramp) - (t1 + ramp)))

    def lead(t):                           # integrate the heading (Simpson), smooth in t
        n = 200
        hs = t / n
        acc = [0.0, 0.0]
        for j in range(n + 1):
            wgt = 1 if j in (0, n) else (4 if j % 2 else 2)
            psi = lead_heading(j * hs)
            acc[0] += wgt * math.cos(psi)
            acc[1] += wgt * math.sin(psi)
        return (speed * hs / 3 * acc[0], speed * hs / 3 * acc[1], -height)

    def slot(k, t):
        u = min(1.0, max(0.0, (t - t_change) / t_blend))
        u = u ** 3 * (u * (6 * u - 15) + 10)   # smootherstep: no acceleration jumps
        return tuple(a + (b - a) * u for a, b in zip(v_slots[k], echelon_slots[k]))

    def wingman(k):
        def f(t):
            n, e, d = lead(t)
            h = lead_heading(t)
            back, right = slot(k, t)
            return (n - back * math.cos(h) - right * math.sin(h), e - back * math.sin(h) + right * math.cos(h), d)
        return f

    def attitude(f, t):
        _, v, _ = sample(f, t)
        yaw = math.atan2(v[1], v[0])
        h = 1e-3
        _, v0, _ = sample(f, t - h)
        _, v1, _ = sample(f, t + h)
        yaw_rate = wrap(math.atan2(v1[1], v1[0]) - math.atan2(v0[1], v0[0])) / (2 * h)
        return (math.atan(math.hypot(v[0], v[1]) * yaw_rate / G), 0.0, yaw)   # coordinated, level

    ships = [("lead", "kf21", "leader", lead), ("wing_1", "xq58a", "follower", wingman(0)),
             ("wing_2", "xq58a", "follower", wingman(1))]

    path = HERE / "example_formation_turn.mlc.ndjson"
    with MLCWriter(str(path), label="example_formation_turn", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="three-ship formation turn, V to echelon right", seed=0) as log:
        bodies = [log.body(name, platform="fixed_wing", model=model, role=role) for name, model, role, _ in ships]   # Appendix C
        # one action spec and one reward spec per body: the lead flies, the wingmen keep their slots
        acts = [log.action_spec(bodies[0], ["roll_cmd", "pitch_cmd", "throttle_cmd"])]
        acts += [log.action_spec(b, ["slot_back_err_m", "slot_right_err_m", "roll_cmd", "throttle_cmd"]) for b in bodies[1:]]
        rews = [log.reward_spec(["step_reward", "heading_error"], body_id=bodies[0])]
        rews += [log.reward_spec(["step_reward", "slot_error"], body_id=b) for b in bodies[1:]]

        shape = None
        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            states = []
            for b, (_, _, _, f) in zip(bodies, ships):
                pos, vel, acc = sample(f, t)
                att = attitude(f, t)
                h = 1e-3
                att_rate = [wrap(q1 - q0) / (2 * h) for q0, q1 in zip(attitude(f, t - h), attitude(f, t + h))]
                x = state_vector(pos, vel, acc, att, att_rate)
                states.append(x)
                log.state(b, rounded(x))

            # a producer-defined event: the formation shape and each wingman's slot (back, right) in metres
            new_shape = "v" if t < t_change else "echelon_right"
            if new_shape != shape:
                target = v_slots if new_shape == "v" else echelon_slots
                log.event("formation", {"shape": new_shape, "lead_body": bodies[0],
                                        "slots_back_right_m": {str(b): list(s) for b, s in zip(bodies[1:], target)},
                                        "blend_s": 0.0 if new_shape == "v" else t_blend}, body_id=bodies[0])
                shape = new_shape

            roll_lead = states[0][16]
            log.action(acts[0], rounded([roll_lead / math.radians(60.0), 0.0, 0.8]))
            log.reward(rews[0], rounded([1.0, 0.0]))
            for k in range(2):
                back, right = slot(k, t)
                err_back = 2.0 * math.sin(0.4 * t + k)          # small, bounded slot-keeping error
                err_right = 1.5 * math.cos(0.3 * t + 2 * k)
                roll = states[k + 1][16]
                log.action(acts[k + 1], rounded([err_back, err_right, roll / math.radians(60.0), 0.8]))
                err = math.hypot(err_back, err_right)
                log.reward(rews[k + 1], rounded([1.0 - 0.1 * err, err]))
    return path



# --------------------------------------------------------------------------------------------------------------------
# Example 7: helicopter reconnaissance: ingress, hover search sweep, detect, pedal turn and track
# --------------------------------------------------------------------------------------------------------------------

def smootherstep(u):
    u = min(1.0, max(0.0, u))
    return u ** 3 * (u * (6 * u - 15) + 10)


def write_heli_recon():
    rate, height = 10.0, 100.0
    v0, t_cruise, t_decel = 40.0, 5.0, 20.0     # ingress north at 40 m/s, then a 20 s deceleration to hover
    t_hover = t_cruise + t_decel
    x_hover = v0 * t_cruise + v0 * t_decel * 0.5
    road_n, car_speed, car_e0 = 300.0, 30.0, 2150.0   # a road 300 m north of the hover point, vehicle driving west
    sweep_amp, sweep_period, search_tilt = math.radians(60.0), 16.0, math.radians(-12.0)
    hfov = math.radians(30.0)                 # vfov omitted: 16:9
    vfov = 2.0 * math.atan(math.tan(hfov / 2) * 9 / 16)
    t_turn, t_slew = 6.0, 1.5                 # pedal turn onto the target, gimbal slew after detection

    def car(t):
        return (road_n, car_e0 - car_speed * t, 0.0)

    def sweep(t):
        return -sweep_amp * math.sin(2 * math.pi * (t - t_hover) / sweep_period)   # left first

    def bearing(t):                           # vehicle bearing from the hover point, 0 = north
        c = car(t)
        return math.atan2(c[1], c[0])

    # detection: the first step at which the vehicle is inside the sweeping camera's image
    t_det = None
    for i in range(int(rate * t_hover), int(rate * 200)):
        t = i / rate
        c = car(t)
        depression = math.atan2(-height, math.hypot(c[0], c[1]))
        if abs(wrap(sweep(t) - bearing(t))) < hfov / 2 and abs(depression - search_tilt) < vfov / 2:
            t_det = t
            break
    duration = (car_e0 + road_n) / car_speed   # until the vehicle is 45 deg left

    def ingress_n(t):
        if t < t_cruise:
            return v0 * t
        u = min(1.0, (t - t_cruise) / t_decel)
        return v0 * t_cruise + v0 * t_decel * (u - (u ** 6 - 3 * u ** 5 + 2.5 * u ** 4))   # integral of 1 - smootherstep

    def traj(t):
        sway = smootherstep((t - t_hover + 5.0) / 5.0)   # small hover drift, faded in
        return (ingress_n(t) - x_hover + sway * 0.4 * math.sin(0.7 * t),
                sway * 0.4 * math.sin(0.5 * t + 1.0), -height - sway * 0.2 * math.sin(0.3 * t))

    def yaw_of(t):                            # nose north, then a pedal turn onto the vehicle and follow it
        return smootherstep((t - t_det) / t_turn) * bearing(t)

    def attitude(t):
        _, v, a = sample(traj, t)
        yaw = yaw_of(t)
        cy, sy = math.cos(yaw), math.sin(yaw)
        v_fwd = cy * v[0] + sy * v[1]
        a_fwd, a_right = cy * a[0] + sy * a[1], -sy * a[0] + cy * a[1]
        # rotor tilts toward the acceleration, plus a nose-down trim with forward speed
        return (math.atan2(a_right, G), -math.atan2(a_fwd, G) - 0.0025 * v_fwd, yaw)

    path = HERE / "example_heli_recon.mlc.ndjson"
    with MLCWriter(str(path), label="example_heli_recon", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="helicopter recon: ingress, hover search sweep, detect, track", seed=0) as log:
        heli = log.body("lah_0", platform="helicopter", model="lah", role="ownship")          # Appendix C
        veh = log.body("vehicle_0", platform="ground_vehicle", role="target")
        act = log.action_spec(heli, ["collective", "cyclic_lon", "cyclic_lat", "pedal"])
        rew = log.reward_spec(["step_reward", "target_in_view", "off_axis_rad"], body_id=heli)

        phase = None
        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            pos, vel, acc = sample(traj, t)
            att = attitude(t)
            h = 1e-3
            att_rate = [wrap(b - a) / (2 * h) for a, b in zip(attitude(t - h), attitude(t + h))]
            x = state_vector(pos, vel, acc, att, att_rate)
            log.state(heli, rounded(x))
            cpos, cvel, cacc = sample(car, t)
            log.state(veh, rounded(state_vector(cpos, cvel, cacc, (0.0, 0.0, -math.pi / 2), (0, 0, 0))))

            if i == 0:
                # Appendix B: nose turret. mount_offset_frd_m omitted: the viewer keeps the airframe's own gimbal point
                log.event("camera", {"name": "eo_ir", "hfov_rad": round(hfov, 6), "gimbal": "yaw_pitch",
                                     "gimbal_limits_rad": rounded([math.radians(-100.0), math.radians(20.0)]),
                                     "gimbal_yaw_limits_rad": rounded([math.radians(-120.0), math.radians(120.0)])},
                          body_id=heli)

            # Section 12 phase events, plus producer-defined detect / terminal events
            new_phase = "ingress" if t < t_hover else ("search" if t < t_det else "track")
            if new_phase != phase:
                log.event("phase", {"from": phase or "start", "to": new_phase}, body_id=heli)
                phase = new_phase
            m = body_to_ned_matrix(*att)
            to_car = ned_to_body(m, [c - q for c, q in zip(cpos, pos)])
            track_yaw = math.atan2(to_car[1], to_car[0])
            track_pitch = math.atan2(-to_car[2], math.hypot(to_car[0], to_car[1]))
            if abs(t - t_det) < 0.5 / rate:
                log.event("detect", {"target_body": veh, "range_m": round(math.dist(pos, cpos), 1),
                                     "bearing_rad": round(bearing(t), 6)}, body_id=heli)

            # Appendix B gimbal: forward look on ingress, sweep while searching, then lock on the vehicle
            if t < t_hover:
                g_yaw, g_pitch = 0.0, search_tilt
            elif t < t_det:
                g_yaw, g_pitch = sweep(t), search_tilt
            else:                             # slew from where the sweep was to the vehicle, then track it
                s = smootherstep((t - t_det) / t_slew)
                g_yaw = sweep(t_det) + s * wrap(track_yaw - sweep(t_det))
                g_pitch = search_tilt + s * (track_pitch - search_tilt)
            log.event("gimbal", {"pitch_rad": round(g_pitch, 6), "yaw_rad": round(g_yaw, 6)}, body_id=heli)

            axis = (math.cos(g_pitch) * math.cos(g_yaw), math.cos(g_pitch) * math.sin(g_yaw), -math.sin(g_pitch))
            los = [c / math.sqrt(sum(q * q for q in to_car)) for c in to_car]
            off_axis = math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(axis, los)))))
            in_view = 1.0 if off_axis < hfov / 2 else 0.0
            if i == int(rate * duration):
                log.event("terminal", {"success": True, "reason": "target_tracked"}, body_id=heli)

            roll, pitch = x[16], x[17]
            log.action(act, rounded([0.5 - 0.05 * acc[2] / G, -pitch / 0.4, roll / 0.4, x[21] / 0.5]))
            log.reward(rew, rounded([0.1 + in_view, in_view, off_axis]))
    return path




# --------------------------------------------------------------------------------------------------------------------
# Example 8: VTOL (quadplane) mission: vertical takeoff, transition, cruise, back-transition, vertical landing
# --------------------------------------------------------------------------------------------------------------------

def write_vtol_transition():
    rate = 10.0
    v_cruise, t_accel = 22.0, 10.0            # pusher: 0 -> 22 m/s in 10 s (and back)
    h_hover, h_cruise = 50.0, 120.0
    t_lift, t_climb, t_hover = 2.0, 15.0, 3.0
    leg, turn_r, ramp_s = 500.0, 150.0, 60.0  # north leg, 180 deg right turn (curvature ramped over 60 m), south leg

    # Path by arc length: heading psi(s) with a ramped turn rate, position by integrating it
    k0 = 1.0 / turn_r
    s_turn = math.pi * turn_r + ramp_s        # heading change k0 * (s_turn - ramp_s) = 180 deg
    s_total = leg + s_turn + leg

    def ramp_area(u):
        u = min(1.0, max(0.0, u))
        return u ** 3 - u ** 4 / 2

    def heading(s):
        u_in, u_out = (s - leg) / ramp_s, (s - leg - s_turn + ramp_s) / ramp_s
        a = ramp_s * (ramp_area(u_in) + min(1.0, max(0.0, u_out)) - ramp_area(u_out))
        return k0 * (a + max(0.0, min(s, leg + s_turn - ramp_s) - (leg + ramp_s)))

    def turn_point(s):                        # Simpson over the turn, from its start to s
        n = 100
        hs = (s - leg) / n
        acc = [0.0, 0.0]
        for j in range(n + 1):
            wgt = 1 if j in (0, n) else (4 if j % 2 else 2)
            psi = heading(leg + j * hs)
            acc[0] += wgt * math.cos(psi)
            acc[1] += wgt * math.sin(psi)
        return (leg + hs / 3 * acc[0], hs / 3 * acc[1])

    turn_end = turn_point(leg + s_turn)

    def path(s):
        if s <= leg:
            return (s, 0.0)
        if s <= leg + s_turn:
            return turn_point(s)
        return (turn_end[0] - (s - leg - s_turn), turn_end[1])   # heading south

    # Timeline
    t_tr = t_lift + t_climb + t_hover         # start of the transition to fixed-wing
    t_cruise = t_tr + t_accel
    t_dec = t_cruise + (s_total - v_cruise * t_accel) / v_cruise   # start of the back-transition
    t_hov2 = t_dec + t_accel
    t_desc = t_hov2 + t_hover
    t_desc_len = 25.0
    t_alt = 25.0                              # cruise climb 50 -> 120 m and descent back

    def s_of(t):                              # distance along the path; speed ramps with smootherstep
        si = lambda u: u ** 6 - 3 * u ** 5 + 2.5 * u ** 4   # integral of smootherstep
        if t <= t_tr:
            return 0.0
        if t <= t_cruise:
            return v_cruise * t_accel * si((t - t_tr) / t_accel)
        if t <= t_dec:
            return v_cruise * t_accel / 2 + v_cruise * (t - t_cruise)
        if t <= t_hov2:
            u = (t - t_dec) / t_accel
            return s_total - v_cruise * t_accel / 2 + v_cruise * t_accel * (u - si(u))
        return s_total

    def height_free(t):                       # the planned height; the descent aims 0.2 m below ground
        h = h_hover * smootherstep((t - t_lift) / t_climb)
        h += (h_cruise - h_hover) * smootherstep((t - t_cruise - 2.0) / t_alt)
        h -= (h_cruise - h_hover) * smootherstep((t - t_dec + t_alt + 2.0) / t_alt)
        h -= (h_hover + 0.2) * smootherstep((t - t_desc) / t_desc_len)
        return h

    lo, hi = t_desc, t_desc + t_desc_len      # touchdown: height_free crosses 0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if height_free(mid) > 0 else (lo, mid)
    t_contact = hi
    duration = t_contact + 2.0

    def sway(t):                              # small drift while hovering down (multicopter mode)
        f = smootherstep((t - t_hov2) / 3.0) * (1.0 if t < t_contact else 0.0)
        return (0.25 * f * math.sin(0.6 * t), 0.3 * f * math.sin(0.45 * t + 0.5))

    def free_traj(t):
        n, e = path(s_of(t))
        dn, de = sway(min(t, t_contact))
        return (n + dn, e + de, -height_free(t))

    def traj(t):
        p = free_traj(min(t, t_contact))
        return (p[0], p[1], min(p[2], 0.0) if t < t_contact else 0.0)

    def fw_weight(t):                         # 0 = multicopter, 1 = fixed-wing (follows the airspeed)
        return min(1.0, s_rate(t) / v_cruise)

    def s_rate(t, h=1e-3):
        return (s_of(t + h) - s_of(t - h)) / (2 * h)

    def attitude(t):
        w = fw_weight(t)
        s = s_of(t)
        yaw = heading(s)
        _, v, _ = sample(traj, t)
        vh = math.hypot(v[0], v[1])
        h = 1e-3
        yaw_rate = (heading(s_of(t + h)) - heading(s_of(t - h))) / (2 * h)
        gamma = math.atan2(-v[2], vh) if vh > 1.0 else 0.0
        roll = w * math.atan(vh * yaw_rate / G)       # coordinated turn
        pitch = w * (gamma + math.radians(3.0))      # flight path + angle of attack
        _, _, a_sway = sample(sway, t)               # multicopter tilt from the hover drift
        cy, sy = math.cos(yaw), math.sin(yaw)
        roll += (1 - w) * math.atan2(-sy * a_sway[0] + cy * a_sway[1], G)
        pitch += (1 - w) * -math.atan2(cy * a_sway[0] + sy * a_sway[1], G)
        return (roll, pitch, yaw)

    # Camera plan: look ahead, stare at a point of interest (the turn centre) in cruise, look straight down to land
    hfov = math.radians(40.0)
    pitch_lim = (math.radians(-100.0), math.radians(20.0))
    mid = turn_point(leg + s_turn / 2)        # northernmost point of the turn (heading east)
    poi = (mid[0] - turn_r, mid[1], 0.0)      # the turn centre, on the ground

    def gimbal_plan(t, pos, att):
        m = body_to_ned_matrix(*att)
        to_poi = ned_to_body(m, [c - q for c, q in zip(poi, pos)])
        look = {"ahead": (math.radians(-20.0), 0.0), "down": (math.radians(-90.0), 0.0),
                "poi": (max(pitch_lim[0], math.atan2(-to_poi[2], math.hypot(to_poi[0], to_poi[1]))),
                        math.atan2(to_poi[1], to_poi[0]))}
        blend = lambda a, b, u: tuple(x + smootherstep(u) * (y - x) for x, y in zip(look[a], look[b]))
        if t < t_dec:
            return blend("ahead", "poi", (t - t_cruise) / 3.0)
        if t < t_desc - 3.0:
            return blend("poi", "ahead", (t - t_dec) / 3.0)
        return blend("ahead", "down", (t - t_desc + 3.0) / 3.0)

    phases = [(0.0, "ground"), (t_lift, "takeoff"), (t_lift + t_climb, "hover"), (t_tr, "transition_to_fw"),
              (t_cruise, "cruise"), (t_dec, "transition_to_mc"), (t_hov2, "hover"), (t_desc, "descent"),
              (t_contact, "landed")]

    path_out = HERE / "example_vtol_transition.mlc.ndjson"
    with MLCWriter(str(path_out), label="example_vtol_transition", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="VTOL takeoff, transition, cruise, back-transition, landing", seed=0) as log:
        uav = log.body("vtol_0", platform="vtol", model="vtol", role="ownship")        # Appendix C
        act = log.action_spec(uav, ["lift_throttle", "pusher_throttle", "roll_cmd", "pitch_cmd", "yaw_cmd"])
        rew = log.reward_spec(["step_reward", "progress", "landing_bonus"], body_id=uav)

        phase_i, landed = -1, False
        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            pos, vel, acc = sample(traj, t)
            att = attitude(t)
            h = 1e-3
            att_rate = [wrap(b - a) / (2 * h) for a, b in zip(attitude(t - h), attitude(t + h))]
            x = state_vector(pos, vel, acc, att, att_rate)
            log.state(uav, rounded(x))

            if i == 0:
                # Appendix B: pan-tilt ball; vfov omitted (16:9), mount omitted (the airframe's own gimbal point)
                log.event("camera", {"name": "eo_ir", "hfov_rad": round(hfov, 6), "image_px": [1280, 720],
                                     "gimbal": "yaw_pitch", "gimbal_limits_rad": rounded(list(pitch_lim)),
                                     "gimbal_yaw_limits_rad": rounded([-math.pi, math.pi])}, body_id=uav)
            g_pitch, g_yaw = gimbal_plan(t, pos, att)
            log.event("gimbal", {"pitch_rad": round(g_pitch, 6), "yaw_rad": round(g_yaw, 6)}, body_id=uav)

            # Section 12 events: phase changes, touchdown, end of the episode
            while phase_i + 1 < len(phases) and t >= phases[phase_i + 1][0] - 1e-9:
                phase_i += 1
                log.event("phase", {"from": phases[phase_i - 1][1] if phase_i else "start", "to": phases[phase_i][1]}, body_id=uav)
            bonus = 0.0
            if not landed and t >= t_contact:
                landed, bonus = True, 10.0
                _, v_in, _ = sample(free_traj, t_contact - 0.05)   # just before touchdown
                r_in, p_in, _ = attitude(t_contact - 0.05)
                log.event("landing_contact", {"vertical_speed_mps": round(v_in[2] * -1, 3),
                                              "lateral_speed_mps": round(math.hypot(v_in[0], v_in[1]), 3),
                                              "tilt_rad": round(math.acos(math.cos(r_in) * math.cos(p_in)), 4)}, body_id=uav)
            if i == int(rate * duration):
                log.event("terminal", {"success": True, "reason": "soft_landing"}, body_id=uav)

            w = fw_weight(t)
            airborne = 0.0 < -pos[2] or t_lift <= t < t_contact
            lift = (1 - w) * 0.5 * (G - acc[2]) / G if airborne else 0.0
            pusher = w * 0.6 + 0.4 * max(0.0, (s_rate(t + 0.05) - s_rate(t - 0.05)) / 0.1) / 4.0
            roll, pitch = x[16], x[17]
            log.action(act, rounded([lift, min(1.0, pusher), roll / 0.5, pitch / 0.3, x[21] / 0.5]))
            progress = 0.01 * s_rate(t)
            log.reward(rew, rounded([progress + bonus, progress, bonus]))
    return path_out


# --------------------------------------------------------------------------------------------------------------------
# Example 9: quadcopter landing on a moving truck: approach, gust, go-around, second approach, touchdown
# --------------------------------------------------------------------------------------------------------------------

def write_quad_moving_pad_landing():
    rate = 20.0
    truck_speed = 8.0                          # north
    pad_frd = (-2.0, 0.0, -2.5)                # pad on the truck bed: 2 m behind the truck origin, 2.5 m up
    mount_frd = (0.05, 0.0, 0.04)              # camera ball under the quad's nose
    pitch_lim = (math.radians(-90.0), math.radians(30.0))
    hfov = math.radians(90.0)

    def truck(t):
        return (truck_speed * t, 0.0, 0.0)

    def pad(t):
        c = truck(t)
        return (c[0] + pad_frd[0], c[1] + pad_frd[1], c[2] + pad_frd[2])

    # Crosswind gust: pushes the quad right (east) between 20 s and 26 s
    def gust(t):
        u = (t - 20.0) / 6.0
        return 1.4 * 64 * (u * (1 - u)) ** 3 if 0.0 <= u <= 1.0 else 0.0   # smooth up to the acceleration

    # Go-around: the first step, while low over the pad, at which the gust offset exceeds 0.6 m
    t_ga = next(i / rate for i in range(int(rate * 15), int(rate * 30)) if gust(i / rate) > 0.6)

    # Planned position relative to the pad (north, east, height above it), smootherstep between the keys
    keys = [(0.0, (-60.0, -25.0, 25.0)), (12.0, (0.0, 0.0, 6.0)), (15.0, (0.0, 0.0, 6.0)),
            (21.0, (0.0, 0.0, 1.2)), (t_ga + 0.5, (0.0, 0.0, 1.2)), (t_ga + 5.5, (0.0, 0.0, 6.0)),
            (29.0, (0.0, 0.0, 6.0)), (37.0, (0.0, 0.0, -0.15))]    # the last descent aims just below the pad

    def rel_plan(t):
        if t <= keys[0][0]:
            return keys[0][1]
        for (t0, a), (t1, b) in zip(keys, keys[1:]):
            if t <= t1:
                s = smootherstep((t - t0) / (t1 - t0))
                return tuple(x + s * (y - x) for x, y in zip(a, b))
        return keys[-1][1]

    lo, hi = 29.0, 37.0                        # touchdown: planned height above the pad reaches 0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if rel_plan(mid)[2] > 0 else (lo, mid)
    t_contact = hi
    duration = t_contact + 3.0                 # then rides on the truck

    def free_traj(t):
        p = pad(t)
        dn, de, dh = rel_plan(t)
        return (p[0] + dn, p[1] + de + gust(t), p[2] - dh)

    def traj(t):
        if t < t_contact:
            return free_traj(t)
        return pad(t)                          # on the pad

    def attitude(t):
        _, v, a = sample(traj, t)
        if t >= t_contact:
            return (0.0, 0.0, 0.0)
        # multirotor: tilt toward the acceleration, plus a nose-down trim against drag at speed
        return (math.atan2(a[1], G), -math.atan2(a[0], G) - 0.01 * v[0], 0.0)

    phases = [(0.0, "approach"), (15.0, "descent"), (t_ga, "go_around"), (t_ga + 5.5, "approach"),
              (29.0, "descent"), (t_contact, "landed")]

    path = HERE / "example_quad_moving_pad_landing.mlc.ndjson"
    with MLCWriter(str(path), label="example_quad_moving_pad_landing", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="land on a moving truck, gust, go-around, second approach", seed=0) as log:
        quad = log.body("quad_0", platform="quadcopter", model="basic_quad", role="ownship")     # Appendix C
        veh = log.body("truck_0", platform="ground_vehicle", role="target")
        act = log.action_spec(quad, ["thrust_norm", "roll_cmd", "pitch_cmd", "yaw_rate_cmd"])
        rew = log.reward_spec(["step_reward", "pad_offset_m", "landing_bonus"], body_id=quad)

        phase_i, landed = -1, False
        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            pos, vel, acc = sample(traj, t)
            att = attitude(t)
            h = 1e-3
            att_rate = [wrap(b - a) / (2 * h) for a, b in zip(attitude(t - h), attitude(t + h))]
            x = state_vector(pos, vel, acc, att, att_rate)
            log.state(quad, rounded(x))
            tpos, tvel, tacc = sample(truck, t)
            log.state(veh, rounded(state_vector(tpos, tvel, tacc, (0.0, 0.0, 0.0), (0, 0, 0))))

            if i == 0:
                # Appendix B: pan-tilt ball with its mount position given this time
                log.event("camera", {"name": "landing_cam", "hfov_rad": round(hfov, 6), "vfov_rad": round(math.radians(58.7), 6),
                                     "mount_offset_frd_m": list(mount_frd), "gimbal": "yaw_pitch",
                                     "gimbal_limits_rad": rounded(list(pitch_lim)),
                                     "gimbal_yaw_limits_rad": rounded([-math.pi, math.pi])}, body_id=quad)

            # Section 12 phase events and producer-defined go_around / landing events
            p_now = pad(t)
            offset = math.hypot(pos[0] - p_now[0], pos[1] - p_now[1])
            while phase_i + 1 < len(phases) and t >= phases[phase_i + 1][0] - 1e-9:
                phase_i += 1
                log.event("phase", {"from": phases[phase_i - 1][1] if phase_i else "start", "to": phases[phase_i][1]}, body_id=quad)
                if phases[phase_i][1] == "go_around":
                    log.event("go_around", {"reason": "pad_offset", "pad_offset_m": round(offset, 3),
                                            "height_above_pad_m": round(p_now[2] - pos[2], 3), "pad_body": veh}, body_id=quad)
            bonus = 0.0
            if not landed and t >= t_contact:
                landed, bonus = True, 10.0
                t_in = t_contact - 0.02                                # just before touchdown, relative to the pad
                _, v_in, _ = sample(free_traj, t_in)
                _, v_pad, _ = sample(pad, t_in)
                rv = [a - b for a, b in zip(v_in, v_pad)]
                r_in, p_in, _ = attitude(t_in)
                log.event("landing_contact", {"vertical_speed_mps": round(-rv[2], 3),
                                              "lateral_speed_mps": round(math.hypot(rv[0], rv[1]), 3),
                                              "tilt_rad": round(math.acos(math.cos(r_in) * math.cos(p_in)), 4),
                                              "relative_to_body": veh}, body_id=quad)
            if i == int(rate * duration):
                log.event("terminal", {"success": True, "reason": "landed_on_moving_vehicle"}, body_id=quad)

            # Appendix B: keep the pad on the optical axis (pan fades out when the pad is right below)
            m = body_to_ned_matrix(*att)
            cam = [pos[k] + sum(m[k][j] * mount_frd[j] for j in range(3)) for k in range(3)]
            to_pad = ned_to_body(m, [a - b for a, b in zip(p_now, cam)])
            horiz = math.hypot(to_pad[0], to_pad[1])
            g_pitch = max(pitch_lim[0], math.atan2(-to_pad[2], horiz))
            g_yaw = math.atan2(to_pad[1], to_pad[0]) * min(1.0, horiz / 2.0)
            if landed:                         # on the pad: stow the camera forward
                g_pitch, g_yaw = pitch_lim[0] * (1 - smootherstep((t - t_contact) / 1.5)), 0.0
            log.event("gimbal", {"pitch_rad": round(g_pitch, 6), "yaw_rad": round(g_yaw, 6)}, body_id=quad)

            thrust = 0.0 if landed else 0.5 * math.sqrt(acc[0] ** 2 + acc[1] ** 2 + (G - acc[2]) ** 2) / G
            log.action(act, rounded([thrust, x[16] / 0.5, x[17] / 0.5, 0.0]))
            penalty = -1.0 if phases[phase_i][1] == "go_around" and t - t_ga < 0.5 / rate else 0.0
            log.reward(rew, rounded([0.1 - 0.05 * offset + bonus + penalty, offset, bonus]))
    return path

# --------------------------------------------------------------------------------------------------------------------
# Example 10: bodies that appear and end mid-episode (Appendix D): two glide bombs released by a fighter
# --------------------------------------------------------------------------------------------------------------------

def write_glide_bomb_release():
    rate = 10.0
    speed, height = 240.0, 5000.0
    t_turn, ramp, bank = 13.0, 4.0, math.radians(60.0)   # egress: 180 deg right turn after the release
    w0 = G * math.tan(bank) / speed
    t_turn_end = t_turn + math.pi / w0 + ramp
    releases = [(10.0, 55.0), (11.0, 55.0)]              # (release time, glide time) per bomb
    targets = [(12000.0, -150.0, 0.0), (12240.0, 150.0, 0.0)]
    dive, v_end = math.radians(60.0), 250.0              # terminal dive angle and speed
    duration = max(r + g for r, g in releases) + 4.0

    def ramp_area(u):
        u = min(1.0, max(0.0, u))
        return u ** 3 - u ** 4 / 2

    def heading(t):
        u_in, u_out = (t - t_turn) / ramp, (t - t_turn_end + ramp) / ramp
        a = ramp * (ramp_area(u_in) + min(1.0, max(0.0, u_out)) - ramp_area(u_out))
        return w0 * (a + max(0.0, min(t, t_turn_end - ramp) - (t_turn + ramp)))

    def jet(t):                                # heading integrated (Simpson) from the turn start
        if t <= t_turn:
            return (speed * t, 0.0, -height)
        n = 120
        hs = (t - t_turn) / n
        acc = [0.0, 0.0]
        for j in range(n + 1):
            wgt = 1 if j in (0, n) else (4 if j % 2 else 2)
            psi = heading(t_turn + j * hs)
            acc[0] += wgt * math.cos(psi)
            acc[1] += wgt * math.sin(psi)
        return (speed * t_turn + speed * hs / 3 * acc[0], speed * hs / 3 * acc[1], -height)

    def jet_attitude(t):
        h = 1e-3
        return (math.atan(speed * (heading(t + h) - heading(t - h)) / (2 * h) / G), math.radians(2.0), heading(t))

    def bomb(k):                               # cubic Hermite from the release (jet's velocity) to the target (dive)
        t_r, t_g = releases[k]
        p0 = jet(t_r)
        p0 = (p0[0], p0[1], p0[2] + 2.0)       # separates 2 m below the jet
        _, v0, _ = sample(jet, t_r)
        tgt = targets[k]
        brg = math.atan2(tgt[1] - p0[1], tgt[0] - p0[0])
        v1 = (v_end * math.cos(dive) * math.cos(brg), v_end * math.cos(dive) * math.sin(brg), v_end * math.sin(dive))

        def f(t):
            u = (t - t_r) / t_g
            return tuple(hermite(p0, tgt, [c * t_g for c in v0], [c * t_g for c in v1], u))
        return f

    bombs = [bomb(k) for k in range(len(releases))]

    def bomb_attitude(f, t):                   # nose along the velocity
        _, v, _ = sample(f, t)
        return (0.0, math.atan2(-v[2], math.hypot(v[0], v[1])), math.atan2(v[1], v[0]))

    path = HERE / "example_glide_bomb_release.mlc.ndjson"
    with MLCWriter(str(path), label="example_glide_bomb_release", origin_lla=ORIGIN_LLA,
                   producer="maneuver-log-contract v1/examples/write_appendix_examples.py",
                   mode="simulation", scenario="two glide bombs released, glide to two targets, jet egresses", seed=0) as log:
        lead = log.body("lead", platform="fixed_wing", model="kf21", role="ownship")          # Appendix C
        tgt_ids = [log.body("target_%s" % c, platform="ground_vehicle", role="target") for c in "ab"]
        # Appendix D: the bombs are declared here, with the other bodies, but exist only from release to impact
        bomb_ids = [log.body("bomb_%d" % k, platform="munition", model="gbu-39", role="weapon") for k in range(len(releases))]
        act = log.action_spec(lead, ["roll_cmd", "pitch_cmd", "throttle_cmd", "release_cmd"])
        rew = log.reward_spec(["step_reward", "hit_bonus"], body_id=lead)

        for i in range(int(rate * duration) + 1):
            t = i / rate
            log.step(t)
            pos, vel, acc = sample(jet, t)
            att = jet_attitude(t)
            h = 1e-3
            att_rate = [wrap(b - a) / (2 * h) for a, b in zip(jet_attitude(t - h), jet_attitude(t + h))]
            x = state_vector(pos, vel, acc, att, att_rate)
            log.state(lead, rounded(x))
            for tid, tg in zip(tgt_ids, targets):                  # targets exist the whole episode
                log.state(tid, rounded(state_vector(tg, (0, 0, 0), (0, 0, 0), (0.0, 0.0, 0.0), (0, 0, 0))))

            release, hit = 0.0, 0.0
            for k, (bid, f) in enumerate(zip(bomb_ids, bombs)):
                t_r, t_g = releases[k]
                if not (t_r - 1e-9 <= t <= t_r + t_g + 1e-9):
                    continue                                        # not alive: no state sample (Appendix D)
                bp, bv, ba = sample(f, t)
                batt = bomb_attitude(f, t)
                brate = [wrap(b - a) / (2 * h) for a, b in zip(bomb_attitude(f, t - h), bomb_attitude(f, t + h))]
                log.state(bid, rounded(state_vector(bp, bv, ba, batt, brate)))
                if abs(t - t_r) < 0.5 / rate:                      # first sample: spawn
                    release = 1.0
                    log.event("spawn", {"parent_body": lead}, body_id=bid)
                    log.event("weapon_release", {"weapon_body": bid, "target_body": tgt_ids[k]}, body_id=lead)
                if abs(t - t_r - t_g) < 0.5 / rate:                # last sample: despawn at the impact
                    hit = 10.0
                    log.event("despawn", {"reason": "impact", "hit_body": tgt_ids[k],
                                          "impact_speed_mps": round(math.sqrt(sum(c * c for c in bv)), 1)}, body_id=bid)
                    log.event("destroyed", {"by_body": bid}, body_id=tgt_ids[k])

            log.action(act, rounded([x[16] / bank, 0.1, 0.85, release]))
            log.reward(rew, rounded([0.01 + hit, hit]))
    return path


if __name__ == "__main__":
    for p in (write_quad_gate_camera(), write_fixedwing_fixed_camera(), write_quad_round_gate_miss(),
              write_suca_track_vehicle(), write_quad_gate_course(),
              write_formation_turn(), write_heli_recon(),
              write_vtol_transition(), write_quad_moving_pad_landing(),
              write_glide_bomb_release()):
        print(p)
