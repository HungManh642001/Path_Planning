"""Smoke tests for the pure-planner part of the benchmark (no model / no GPU).

Offline eval and the trainer are not unit-tested (they need a model file / a
GPU); they are CLIs verified manually per ml_planner/EVAL.md.
"""
import ml_planner.benchmark as bm
from ml_planner.benchmark import planner_benchmark, CSV_COLUMNS
from batch_random_test import generate_random_scenario


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


def test_compare_one_emits_gnn_columns_without_model():
    row = bm.compare_one(generate_random_scenario, 7003, 'easy',
                         guidance=None, eps=0.05, graph_guidance=None)
    for col in ('gnn_success', 'gnn_iters', 'gnn_time', 'gnn_mission',
                'gnn_flight', 'gnn_cost_ratio', 'gnn_bound_ok',
                'gnn_beats_hand_iters'):
        assert col in row
    assert all(c in bm.CSV_COLUMNS for c in
               ('gnn_success', 'gnn_iters', 'gnn_time', 'gnn_cost_ratio'))


def test_gnn_acceptance_logic():
    # speed win + quality not-worse => PASS
    assert bm.gnn_acceptance(it_g=100, it_h=200, t_g=1.0, t_h=2.0,
                             cost_g=1.010, cost_h=1.009) is True
    # quality win + time within 5% => PASS
    assert bm.gnn_acceptance(it_g=250, it_h=200, t_g=2.09, t_h=2.0,
                             cost_g=1.001, cost_h=1.009) is True
    # no win on either axis => FAIL
    assert bm.gnn_acceptance(it_g=250, it_h=200, t_g=2.5, t_h=2.0,
                             cost_g=1.010, cost_h=1.009) is False
    # quality win but time blown past 5% => FAIL
    assert bm.gnn_acceptance(it_g=250, it_h=200, t_g=2.5, t_h=2.0,
                             cost_g=1.001, cost_h=1.009) is False


def test_compare_one_emits_lazy_and_lcor_columns():
    row = bm.compare_one(generate_random_scenario, 7003, 'easy',
                         guidance=None, eps=0.05, graph_guidance=None)
    for col in ('lazy_success', 'lazy_iters', 'lazy_time', 'lazy_checks',
                'lcor_success', 'lcor_iters', 'lcor_time', 'lcor_checks',
                'hand_checks', 'lazy_bound_ok', 'lcor_bound_ok'):
        assert col in row
    assert all(c in bm.CSV_COLUMNS for c in
               ('lazy_iters', 'lcor_iters', 'hand_checks', 'lazy_checks', 'lcor_checks'))


def test_lazy_verdict_layers(capsys):
    hard = dict(n=5, t_h=10.0, it_h=1000, checks_h=50000,
                t_lz=8.0, it_lz=1000, checks_lz=20000, viol_lz=0,
                t_lc=7.0, it_lc=900, checks_lc=15000, viol_lc=0)
    bm.lazy_verdict(hard)
    out = capsys.readouterr().out
    assert 'LAZY' in out and 'PASS' in out
    # bound violation flips the verdict regardless of speed
    hard['viol_lc'] = 1
    bm.lazy_verdict(hard)
    out = capsys.readouterr().out
    assert 'FAIL' in out
