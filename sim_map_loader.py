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
	Includes lanes (type: "lane") as regular obstacles for rendering.
	"""
	obj = _load_obj(data)

	if 'obstacles' not in obj or not isinstance(obj['obstacles'], list):
		# No obstacles key is acceptable: return empty list
		return []

	obstacles: List[object] = []

	for i, item in enumerate(obj['obstacles']):
		if not isinstance(item, dict):
			raise ValueError(f"obstacle at index {i} must be an object")
		
		# Skip legacy lane marker (is_lane: true), but load type: "lane"
		if item.get('is_lane', False):
			continue
		
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
		elif t in ('polygon', 'poly', 'lane'):
			verts = item.get('vertices')
			if not isinstance(verts, list) or len(verts) < 3:
				raise ValueError(f"polygon/lane obstacle at index {i} requires 'vertices' list of length>=3")
			verts = [(float(v[0]), float(v[1])) for v in verts]
			# Lanes default to white (line markings); polygons default to orange (barriers)
			default_color = WHITE if t == 'lane' else ORANGE
			color = item.get('color') or default_color
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


def load_lanes_from_json(data: Union[str, dict]) -> List[List[Tuple[float, float]]]:
	"""Load lane centerlines from a JSON file path or a dict.

	Extracts obstacles marked with:
	- is_lane: true (for backwards compatibility)
	- type: "lane" (primary method)
	
	For polygon/lane obstacles, returns their vertices as the lane centerline.

	Returns a list of lane definitions, where each lane is a list of (x, y) points.
	"""
	obj = _load_obj(data)

	if 'obstacles' not in obj or not isinstance(obj['obstacles'], list):
		return []

	lanes: List[List[Tuple[float, float]]] = []

	for i, item in enumerate(obj['obstacles']):
		if not isinstance(item, dict):
			continue
		
		t = item.get('type', '').lower()
		
		# Extract as lane if type is "lane" or marked with is_lane: true
		is_lane_type = (t == 'lane')
		is_lane_marked = item.get('is_lane', False)
		
		if not (is_lane_type or is_lane_marked):
			continue
		
		if t in ('polygon', 'poly', 'lane'):
			verts = item.get('vertices')
			if isinstance(verts, list) and len(verts) >= 2:
				verts = [(float(v[0]), float(v[1])) for v in verts]
				lanes.append(verts)
		elif t == 'line':
			start = item.get('start')
			end = item.get('end')
			if isinstance(start, (list, tuple)) and isinstance(end, (list, tuple)):
				start_pt = (float(start[0]), float(start[1]))
				end_pt = (float(end[0]), float(end[1]))
				lanes.append([start_pt, end_pt])

	return lanes


def load_lane_objects_from_json(data: Union[str, dict]) -> List[object]:
	"""Load lane objects as polygon obstacles from a JSON file path or dict.

	Extracts obstacles marked with:
	- is_lane: true (for backwards compatibility)
	- type: "lane" (primary method)
	
	Returns them as PolygonObstacle or LineObstacle objects for the lane waypoint generator.

	Returns a list of obstacle objects representing lanes.
	"""
	obj = _load_obj(data)

	if 'obstacles' not in obj or not isinstance(obj['obstacles'], list):
		return []

	lane_objects: List[object] = []

	for i, item in enumerate(obj['obstacles']):
		if not isinstance(item, dict):
			continue
		
		t = item.get('type', '').lower()
		
		# Extract as lane if type is "lane" or marked with is_lane: true
		is_lane_type = (t == 'lane')
		is_lane_marked = item.get('is_lane', False)
		
		if not (is_lane_type or is_lane_marked):
			continue
		
		# Load lane polygons
		if t in ('polygon', 'poly', 'lane'):
			verts = item.get('vertices')
			if isinstance(verts, list) and len(verts) >= 3:
				verts = [(float(v[0]), float(v[1])) for v in verts]
				color = item.get('color') or (0, 255, 255)  # Cyan default
				lane_obj = PolygonObstacle(verts, color)
				lane_objects.append(lane_obj)
		elif t == 'line':
			start = item.get('start')
			end = item.get('end')
			width = float(item.get('width', 0.1))
			if isinstance(start, (list, tuple)) and isinstance(end, (list, tuple)):
				start_pt = (float(start[0]), float(start[1]))
				end_pt = (float(end[0]), float(end[1]))
				color = item.get('color') or (0, 255, 255)  # Cyan default
				lane_obj = LineObstacle(start_pt, end_pt, width, color)
				lane_objects.append(lane_obj)

	return lane_objects


def load_map_file(data: Union[str, dict]) -> Tuple[List[object], dict]:
	"""Load a full map file (path or dict) and return (obstacles, scene_obj).

	scene_obj is the raw parsed JSON dictionary with added 'lanes' key.
	Lanes are extracted from obstacles marked with is_lane: true.
	"""
	obj = _load_obj(data)
	obstacles = load_obstacles_from_json(obj)
	lanes = load_lanes_from_json(obj)
	
	# Add lanes to scene_obj for easy access
	obj['lanes'] = lanes
	
	return obstacles, obj

