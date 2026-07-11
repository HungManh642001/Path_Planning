from ml_planner.run_ml import compare_seed, run_benchmark


def test_compare_seed_reports_bound_and_keys():
    row = compare_seed(1, focal_eps=0.05)
    for key in ('seed', 'base_success', 'focal_success', 'base_cost',
                'focal_cost', 'cost_ratio', 'within_bound',
                'base_iters', 'focal_iters', 'base_time', 'focal_time'):
        assert key in row
    # Whenever both solve, the epsilon bound must hold.
    if row['base_success'] and row['focal_success']:
        assert row['within_bound'] is True


def test_run_benchmark_multiple_seeds():
    rows = run_benchmark([1, 2], focal_eps=0.05)
    assert len(rows) == 2
    assert all(r['within_bound'] for r in rows)
