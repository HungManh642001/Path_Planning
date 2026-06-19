import gui.scenario_io as sio


def test_roundtrip_preserves_state():
    state = {
        'start': (1000.0, 2000.0), 'start_heading': 0.5,
        'goal': (400000.0, 250000.0), 'goal_heading': 0.0,
        'obstacles': [
            {'type': 'circle', 'center': (100000.0, 50000.0), 'radius': 20000.0},
            {'type': 'polygon', 'polygon': [(10.0, 10.0), (20.0, 10.0), (15.0, 25.0)]},
        ],
    }
    text = sio.scenario_to_json(state)
    back = sio.scenario_from_json(text)
    assert back['start'] == (1000.0, 2000.0)
    assert back['goal_heading'] == 0.0
    assert back['obstacles'][0]['center'] == (100000.0, 50000.0)
    assert back['obstacles'][1]['polygon'][2] == (15.0, 25.0)


def test_from_json_handles_missing_points():
    state = {'start': None, 'start_heading': 0.0, 'goal': None, 'goal_heading': 0.0,
             'obstacles': []}
    back = sio.scenario_from_json(sio.scenario_to_json(state))
    assert back['start'] is None
    assert back['obstacles'] == []
