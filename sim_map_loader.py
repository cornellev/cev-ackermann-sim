"""
sim_map_loader.py

Loader for the simulator map files. Provides helper functions to load obstacles
and to parse the full scene JSON (including start/goal/waypoints).

This module exposes:
- load_obstacles_from_json(path_or_dict) -> List[ObstacleInstances]
- load_map_file(path_or_dict) -> (obstacles_list, scene_obj_dict)

It accepts either a file path to a .json map or a Python dict already loaded
from JSON.
"""

import json
from typing import List, Tuple, Union

from objects import CircleObstacle, PolygonObstacle, LineObstacle
from constants import *


def _load_obj(data: Union[str, dict]) -> dict:
	"""Internal helper: return parsed JSON dict from path or dict."""
	if isinstance(data, str):
		with open(data, 'r') as fh:
			return json.load(fh)
	elif isinstance(data, dict):
		return data
	else:
		raise ValueError("data must be a file path or a dict")


def load_obstacles_from_json(data: Union[str, dict]) -> List[object]:
	"""Load obstacles from a JSON file path or a dict.

	Returns a list of obstacle objects (CircleObstacle, PolygonObstacle, LineObstacle).
	"""
	obj = _load_obj(data)

	if 'obstacles' not in obj or not isinstance(obj['obstacles'], list):
		# No obstacles key is acceptable: return empty list
		return []

	obstacles: List[object] = []

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
		elif t in ('polygon', 'poly'):
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


def load_map_file(data: Union[str, dict]) -> Tuple[List[object], dict]:
	"""Load a full map file (path or dict) and return (obstacles, scene_obj).

	scene_obj is the raw parsed JSON dictionary (may include 'start', 'goal', 'waypoints').
	"""
	obj = _load_obj(data)
	obstacles = load_obstacles_from_json(obj)
	return obstacles, obj

