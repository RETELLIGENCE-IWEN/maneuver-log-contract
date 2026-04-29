from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from v1.python.mlc import MLCWriter


EXAMPLE_LOG = Path(__file__).resolve().with_name("example_run.mlc.ndjson")


STATE_0 = [
    0.651733, 2.216568, 970.0,
    18.2, -6.5, -850.0,
    -1.2, 0.4, 72.5,
    72.4, 0.1, 1.8,
    0.9998, 0.0030, 0.0190, 0.0010,
    0.0061, 0.0380, 0.0021,
    0.0040, -0.0180, 0.0020,
    0.1, -0.2, -12.8,
    0.5, 0.0, -12.7,
]

STATE_1 = [
    0.651733, 2.216568, 969.884,
    18.198, -6.499, -849.884,
    -1.202, 0.401, 72.486,
    72.38, 0.1, 1.8,
    0.9998, 0.0030, 0.0190, 0.0010,
    0.0061, 0.0380, 0.0021,
    0.0040, -0.0180, 0.0020,
    0.1, -0.2, -12.8,
    0.5, 0.0, -12.7,
]


def main() -> None:
    with MLCWriter(
        str(EXAMPLE_LOG),
        label="example_run",
        origin_lla=[0.651733, 2.216568, 120.0],
        producer="mlc-python-example",
        mode="simulation",
        scenario="minimal_demo",
        seed=1,
    ) as log:
        booster = log.body(
            "booster_0",
            platform="rocket",
            model="generic_vtv_booster",
        )

        action = log.action_spec(
            booster,
            [
                "raw_action_0",
                "raw_action_1",
                "raw_action_2",
                "throttle_cmd",
                "tvc_x_cmd",
                "tvc_y_cmd",
            ],
        )

        reward = log.reward_spec(
            [
                "step_reward",
                "pos_err_reward",
                "vel_err_reward",
                "fuel_penalty",
                "terminal_bonus",
            ],
            body_id=booster,
        )

        log.step(0.0)
        log.event("phase", {"from": "coast", "to": "terminal_descent"}, body_id=booster)
        log.state(booster, STATE_0)
        log.action(action, [0.12, -0.44, 0.91, 0.58, 0.012, -0.018])
        log.reward(reward, [-0.812, -0.215, -0.441, -0.058, 0.0])

        log.step(0.0016)
        log.state(booster, STATE_1)
        log.action(action, [0.10, -0.42, 0.88, 0.59, 0.012, -0.017])
        log.reward(reward, [-0.801, -0.214, -0.432, -0.059, 0.0])

        log.event(
            "terminal",
            {"success": True, "reason": "demo_finished"},
            body_id=booster,
        )


if __name__ == "__main__":
    main()
