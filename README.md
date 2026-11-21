# sokoban-solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Rasidi007/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Rasidi007/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A room-by-room push-optimal Sokoban solver: it splits each level into its hallway and rooms, solves every room alone with A* over pushes (admissible box-goal matching heuristic, dead-square and frozen-2x2 pruning), and falls back to greedy best-first when a room resists. Budgets count work, never the clock, so two runs write identical bytes.

## Run it

```sh
MAKEFUN_INPUT_DIR=/path/to/instances MAKEFUN_OUTPUT_DIR=out node main.ts
```

It reads `level.txt` from each instance directory under the input directory and writes `solution.txt` for each under the output directory.

---

<sub>Entered on [make.fun](https://make.fun/?ref=pesfxty1&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
