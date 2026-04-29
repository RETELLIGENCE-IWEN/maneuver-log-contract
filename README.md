# Maneuver Log Contract

**A small open contract for logging moving things.**

Maneuver Log Contract, or **MLC**, is a compact NDJSON-based log format for vehicles, robots, agents, and simulated bodies performing maneuvers.

It exists for one mission:

> **We play with moving things. We want to achieve intelligent autonomy.**

This repository hosts the MLC specification, examples, and future reference tools.

---

## What MLC is

MLC is a shared log language for maneuvering systems.

It is designed to help tools answer a few essential questions:

- Where is the body?
- How is it moving?
- How is it oriented?
- What action did the autonomy system choose?
- What reward did the task provide?
- What important event happened?

The format is intentionally simple:

- **NDJSON**: one JSON object per line
- **Strict core state**: every body state has a fixed maneuver-state vector
- **Double precision**: fundamental motion fields use IEEE-754 binary64 semantics
- **Replay-ready**: visualizers can animate bodies from the core state alone
- **Autonomy-ready**: actions and rewards are standardized optional records
- **Extension-friendly**: projects can add their own records around the stable core

---

## Current version

The current contract is:

```text
Maneuver Log Contract v1
Recommended extension: .mlc.ndjson