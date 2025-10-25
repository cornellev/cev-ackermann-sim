"""
obstacle_loader.py

JSON schema (recommended) for obstacle list:

{
  "obstacles": [
    {"type": "circle", "x": 1.0, "y": 2.0, "radius": 0.5},
    {"type": "polygon", "vertices": [[0,0],[1,0],[1,1]], "color": [255,0,0]},
    {"type": "line", "start": [0,0], "end": [2,0], "width": 0.1, "color": [255,255,255]}
  ]
}

Supported obstacle objects and required fields:
- circle: {"type":"circle", "x": float, "y": float, "radius": float, "color": [r,g,b] (optional)}
- polygon: {"type":"polygon", "vertices": [[x,y],...], "color": [r,g,b] (optional)}
- line: {"type":"line", "start": [x,y], "end": [x,y], "width": float, "color": [r,g,b] (optional)}

This module provides `load_obstacles_from_json(path_or_dict)` which accepts either a
path to a .json file or a Python dict already loaded from JSON, validates the structure,
and returns a list of obstacle instances from `objects.py` (CircleObstacle, PolygonObstacle,
LineObstacle).

It performs light validation and will raise ValueError on malformed inputs.
"""

import json
from typing import List, Union

from objects import CircleObstacle, PolygonObstacle, LineObstacle
from constants import *

def load_obstacles_from_json(data: Union[str, dict]) -> List[object]:
    """Load obstacles from a JSON file path or a dict.

    Returns a list of obstacle objects.
    """
    if isinstance(data, str):
        with open(data, 'r') as fh:
            obj = json.load(fh)
    elif isinstance(data, dict):
        obj = data
    else:
        raise ValueError("data must be a file path or dict")

    if 'obstacles' not in obj or not isinstance(obj['obstacles'], list):
        raise ValueError("JSON must contain an 'obstacles' list")

    obstacles = []
    for i, item in enumerate(obj['obstacles']):
        if not isinstance(item, dict):
            raise ValueError(f"obstacle at index {i} must be an object")
        t = item.get('type', '').lower()
        if t == 'circle':
            x = float(item.get('x'))
            y = float(item.get('y'))
            r = float(item.get('radius'))
            color = item.get('color')
            obs = CircleObstacle(x, y, r)
            if color is not None:
                obs.color = color
            obstacles.append(obs)
        elif t == 'polygon' or t == 'poly':
            verts = item.get('vertices')
            if not isinstance(verts, list) or len(verts) < 3:
                raise ValueError(f"polygon obstacle at index {i} requires 'vertices' list of length>=3")
            verts = [(float(v[0]), float(v[1])) for v in verts]
            color = item.get('color') or ORANGE
            obs = PolygonObstacle(verts, color)
            obstacles.append(obs)
        elif t == 'line':
            start = item.get('start')
            end = item.get('end')
            width = float(item.get('width', 0.1))
            if not (isinstance(start, (list, tuple)) and isinstance(end, (list, tuple))):
                raise ValueError(f"line obstacle at index {i} requires 'start' and 'end' arrays")
            start = (float(start[0]), float(start[1]))
            end = (float(end[0]), float(end[1]))
            color = item.get('color') or WHITE
            obs = LineObstacle(start, end, width, color)
            obstacles.append(obs)
        else:
            raise ValueError(f"unknown obstacle type '{t}' at index {i}")

    return obstacles


# Note: This module is not intended to be executed as a script. Import
# `load_obstacles_from_json` from other modules instead.
