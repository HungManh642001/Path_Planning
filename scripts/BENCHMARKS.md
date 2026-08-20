# Planner benchmarks

Protocol and reference numbers for `scripts/ab_planners.py`, the gate for every
change to `core/kinodynamic_astar_v0.py` (production) and
`core/kinodynamic_astar.py`.

The dumps themselves live in `docs/benchmarks/` — untracked (`docs/` is
gitignored), because they are large and regenerate in 40-180 s. The numbers
below are the tracked record of what they contained.

## Protocol

```bash
export PYTHONPATH=.
python scripts/ab_planners.py run --planner v0 --seeds 300 --mode free --out /tmp/new.json
python scripts/ab_planners.py compare docs/benchmarks/baseline-v0-free.json /tmp/new.json
```

Regenerate the four baselines with:

```bash
for m in free fixed; do for p in v0 main; do
  python scripts/ab_planners.py run --planner $p --seeds 300 --mode $m \
    --out docs/benchmarks/baseline-$p-$m.json
done; done
```

Two kinds of gate, decided BEFORE the change is written:

- **bit-identical** — for anything claimed to be a pure optimisation (reordering
  pure predicates, caching a value derived from immutable fields, hoisting a
  loop invariant). `compare` prints `bit-identical paths: N/N`; anything less
  than N/N means the change is not what it was claimed to be. Summary stats
  matching is NOT evidence: two different routes can agree on total length to
  four decimals.
- **A/B** — for anything that changes which successors exist. Read `solved`
  (gained/lost seeds), then `length`, then `iterations`. Read `time` last and
  only from paired repeats: single runs on identical code drift ~5%.

## Modes

- `free` (default) — `goal_heading = None`. This is what `batch_random_test.py`
  runs, i.e. the production distribution. `_try_goal_shot` returns immediately
  in this mode, so a goal-shot change cannot be measured here.
- `fixed` — same obstacle field, same start/goal, plus an approach heading drawn
  from a SEPARATE rng so the map stays byte-identical to the `free` run of the
  same seed. Exercises the terminal-turn constraint, the approach-ray rule in
  `smooth_path`, and the goal shot.

A seed means the same map `batch_random_test.py` draws for that seed:
`make_scenario` mirrors `generate_random_scenario` call for call.

## Validity of comparisons over time

`core/map_generator.py` decides the obstacle field. **Any** change there moves
every number in this directory, so dumps are only comparable within one
generator version. When the generator changes, re-run the whole baseline and
note the commit here.

## Baseline, 300 seeds, generator at 5400d9c (2026-08-20)

| planner | mode | solved | time | length | waypoints |
| --- | --- | --- | --- | --- | --- |
| v0 | free | 294/300 | 40.8 s | 74744.3 km | 1401 |
| main | free | 294/300 | 102.1 s | 74347.6 km | 1691 |
| v0 | fixed | 243/300 | 53.0 s | 65386.7 km | 1189 |
| main | fixed | 243/300 | 177.2 s | 64957.6 km | 1424 |

Same missions solved by both planners in both modes; main buys 0.53% of path
length with 2.5x the time and 21% more waypoints. v0 is the production planner
(`batch_random_test.py` imports it), and this is the sample that says the choice
costs nothing in missions solved.
