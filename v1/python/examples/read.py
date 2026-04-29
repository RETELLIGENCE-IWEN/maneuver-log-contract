from reference.python.mlc import read_mlc


def main() -> None:
    log = read_mlc("example_run.mlc.ndjson")

    print("Label:", log["header"]["label"])
    print("Bodies:", len(log["bodies"]))
    print("Steps:", len(log["steps"]))
    print("States:", len(log["states"]))
    print("Actions:", len(log["actions"]))
    print("Rewards:", len(log["rewards"]))
    print("Events:", len(log["events"]))

    if log["states"]:
        first = log["states"][0]
        print()
        print("First state:")
        print("  t:", first["t"])
        print("  body:", first["b"])
        print("  pn_m:", first["data"]["pn_m"])
        print("  pe_m:", first["data"]["pe_m"])
        print("  pd_m:", first["data"]["pd_m"])
        print("  qw:", first["data"]["qw"])

    if log["actions"]:
        first_action = log["actions"][0]
        print()
        print("First action:")
        print("  t:", first_action["t"])
        print("  data:", first_action["data"])

    if log["rewards"]:
        total_return = sum(r["data"]["step_reward"] for r in log["rewards"])
        print()
        print("Computed episode return:", total_return)

    if log["events"]:
        print()
        print("Events:")
        for event in log["events"]:
            print(f"  t={event['t']} topic={event['topic']} data={event['data']}")


if __name__ == "__main__":
    main()