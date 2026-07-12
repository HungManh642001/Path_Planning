"""Smoke tests for the pure-planner part of the benchmark (no model / no GPU).

Offline eval and the trainer are not unit-tested (they need a model file / a
GPU); they are CLIs verified manually per ml_planner/EVAL.md.
"""
from ml_planner.benchmark import planner_benchmark, CSV_COLUMNS


def test_rows_have_all_csv_columns_and_bound_holds():
    rows = planner_benchmark(guidance=None, easy_seeds=[1, 2], hard_seeds=[], eps=0.05)
    assert len(rows) == 2
    for r in rows:
        for col in CSV_COLUMNS:
            assert col in r, f"missing column {col}"
        # Whenever base and focal-hand both solve, the epsilon bound must hold.
        if r['base_success'] and r['hand_success']:
            assert r['hand_bound_ok'] is True


def test_guided_falls_back_to_hand_without_model():
    # No guidance model -> guided path uses the hand-crafted secondary, so its
    # result is identical to the hand-crafted run.
    rows = planner_benchmark(guidance=None, easy_seeds=[2], hard_seeds=[], eps=0.05)
    r = rows[0]
    if r['hand_success'] and r['guided_success']:
        assert r['guided_mission'] == r['hand_mission']
        assert r['guided_iters'] == r['hand_iters']
