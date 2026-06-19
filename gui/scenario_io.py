"""Serialize / deserialize the GUI scenario state to JSON.

Tuples become lists in JSON; deserialization restores tuples so downstream code
that unpacks (x, y) keeps working.
"""

import json


def _pt(p):
    return None if p is None else [float(p[0]), float(p[1])]


def scenario_to_json(state):
    obstacles = []
    for o in state['obstacles']:
        if o['type'] == 'circle':
            obstacles.append({'type': 'circle', 'center': _pt(o['center']),
                              'radius': float(o['radius'])})
        else:
            obstacles.append({'type': 'polygon',
                              'polygon': [_pt(v) for v in o['polygon']]})
    doc = {
        'start': _pt(state['start']),
        'start_heading': float(state['start_heading']),
        'goal': _pt(state['goal']),
        'goal_heading': float(state['goal_heading']),
        'obstacles': obstacles,
    }
    return json.dumps(doc, indent=2)


def _tup(p):
    return None if p is None else (float(p[0]), float(p[1]))


def scenario_from_json(text):
    doc = json.loads(text)
    obstacles = []
    for o in doc.get('obstacles', []):
        if o['type'] == 'circle':
            obstacles.append({'type': 'circle', 'center': _tup(o['center']),
                              'radius': float(o['radius'])})
        else:
            obstacles.append({'type': 'polygon',
                              'polygon': [_tup(v) for v in o['polygon']]})
    return {
        'start': _tup(doc.get('start')),
        'start_heading': float(doc.get('start_heading', 0.0)),
        'goal': _tup(doc.get('goal')),
        'goal_heading': float(doc.get('goal_heading', 0.0)),
        'obstacles': obstacles,
    }
