"""
Quite suspect.
"""

import math
import time


class RunEvaluator:
    def __init__(self, waypoints, obstacles, duration=30.0):
        """
        waypoints: list of (x, y) GPS waypoints (in order)
        obstacles: list of obstacle objects (with .check_collision, optional .obs_type / .radius / .vertices)
        duration: evaluation window in seconds
        """
        self.waypoints = list(waypoints) if waypoints else []
        self.obstacles = list(obstacles) if obstacles else []
        self.duration = duration

        # Timing
        self.start_time = None
        self.elapsed = 0.0
        self.finished = False

        # Progress tracking
        self.initial_wp_index = 0
        self.current_wp_index = 0
        self.waypoints_reached = 0
        self.total_distance_traveled = 0.0
        self._prev_pos = None

        # Collision tracking
        self.collision_frames = 0
        self.total_frames = 0
        self.collision_events = []  # list of {time, obstacle_type, x, y}
        self._was_colliding = False  # edge detector — log once per entry

        # Direction tracking: how often the car is heading toward the next waypoint
        self.good_heading_frames = 0

        # Mode tracking
        self.lane_frames = 0
        self.gps_frames = 0

        # Score
        self.score = 0.0

        # Snapshot of starting position for net-displacement calculation
        self._start_pos = None

    def tick(self, vehicle, collision_detector, planner_mode='GPS', wp_index=0):
        """Call once per sim frame.
        
        vehicle: object with .x, .y, .heading, .speed, .get_corners()
        collision_detector: CollisionDetector instance
        planner_mode: 'LANE' or 'GPS'
        wp_index: current GPS waypoint index the planner targets
        """
        if self.finished:
            return

        now = time.time()
        if self.start_time is None:
            self.start_time = now
            self._start_pos = (vehicle.x, vehicle.y)
            self._prev_pos = (vehicle.x, vehicle.y)
            self.initial_wp_index = wp_index

        self.elapsed = now - self.start_time
        if self.elapsed >= self.duration:
            self.finished = True
            self._compute_score()
            return

        self.total_frames += 1
        pos = (vehicle.x, vehicle.y)

        # Distance traveled
        if self._prev_pos is not None:
            dx = pos[0] - self._prev_pos[0]
            dy = pos[1] - self._prev_pos[1]
            self.total_distance_traveled += math.hypot(dx, dy)
        self._prev_pos = pos

        # Waypoint progress
        self.current_wp_index = wp_index
        wps_advanced = wp_index - self.initial_wp_index
        if wps_advanced > self.waypoints_reached:
            self.waypoints_reached = wps_advanced

        # Collision check
        colliding = collision_detector.check_collision(vehicle.get_corners(), self.obstacles)
        is_colliding = len(colliding) > 0
        if is_colliding:
            self.collision_frames += 1
            if not self._was_colliding:
                # New collision event — log it
                for obs in colliding:
                    obs_type = getattr(obs, 'obs_type', 'unknown')
                    if obs_type == 'unknown':
                        if hasattr(obs, 'radius'):
                            obs_type = 'cone'
                        elif hasattr(obs, 'vertices'):
                            obs_type = 'lane'
                    self.collision_events.append({
                        'time': round(self.elapsed, 2),
                        'type': obs_type,
                        'x': round(vehicle.x, 2),
                        'y': round(vehicle.y, 2),
                    })
        self._was_colliding = is_colliding

        # Direction check: is the car heading toward the next waypoint?
        if self.waypoints and wp_index < len(self.waypoints):
            wx, wy = self.waypoints[wp_index]
            dx = wx - vehicle.x
            dy = wy - vehicle.y
            dist = math.hypot(dx, dy)
            if dist > 0.5:
                # Dot product of heading unit vector and direction to waypoint
                heading_dot = (dx / dist) * math.cos(vehicle.heading) + \
                              (dy / dist) * math.sin(vehicle.heading)
                if heading_dot > 0.3:  # within ~72 degrees of correct direction
                    self.good_heading_frames += 1

        # Mode tracking
        if planner_mode == 'LANE':
            self.lane_frames += 1
        else:
            self.gps_frames += 1

    def _compute_score(self):
        """Compute a single numeric score. Higher = better."""
        if self.total_frames == 0:
            self.score = 0.0
            return

        # Component 1: waypoint progress (big reward per waypoint)
        wp_score = self.waypoints_reached * 100.0

        # Component 2: forward progress toward final waypoint (net displacement)
        progress_score = 0.0
        if self._start_pos and self.waypoints:
            final_wp = self.waypoints[-1]
            start_dist = math.hypot(final_wp[0] - self._start_pos[0],
                                     final_wp[1] - self._start_pos[1])
            end_dist = math.hypot(final_wp[0] - self._prev_pos[0],
                                   final_wp[1] - self._prev_pos[1])
            progress_score = max(0, start_dist - end_dist) * 5.0

        # Component 3: collision penalty (both rate and event count)
        collision_pct = self.collision_frames / self.total_frames
        collision_penalty = collision_pct * 200.0 + len(self.collision_events) * 25.0

        # Component 4: heading quality bonus (are we pointed at the next WP?)
        heading_pct = self.good_heading_frames / self.total_frames
        heading_score = heading_pct * 40.0

        # Component 5: efficiency (penalize circling — ratio of net displacement / total dist)
        efficiency_penalty = 0.0
        if self._start_pos and self._prev_pos and self.total_distance_traveled > 1.0:
            net_displacement = math.hypot(self._prev_pos[0] - self._start_pos[0],
                                           self._prev_pos[1] - self._start_pos[1])
            ratio = net_displacement / self.total_distance_traveled
            efficiency_penalty = max(0, (1.0 - ratio)) * 25.0

        self.score = wp_score + progress_score + heading_score - collision_penalty - efficiency_penalty
        self._score_breakdown = {
            'wp_score': wp_score,
            'progress_score': progress_score,
            'heading_score': heading_score,
            'collision_penalty': -collision_penalty,
            'efficiency_penalty': -efficiency_penalty,
        }

    def summary(self):
        """Return a formatted summary string."""
        if not self.finished:
            self._compute_score()

        bd = getattr(self, '_score_breakdown', {})
        lines = []
        lines.append("=" * 60)
        lines.append(f"  RUN EVALUATION  ({self.elapsed:.1f}s / {self.duration:.0f}s)")
        lines.append("=" * 60)
        lines.append(f"  SCORE: {self.score:+.1f}")
        lines.append(f"    WP progress:      {bd.get('wp_score', 0):+.1f}  ({self.waypoints_reached} waypoints reached)")
        lines.append(f"    Forward progress: {bd.get('progress_score', 0):+.1f}")
        lines.append(f"    Heading quality:  {bd.get('heading_score', 0):+.1f}  ({100*self.good_heading_frames//max(1,self.total_frames)}% frames aimed right)")
        lines.append(f"    Collision penalty:{bd.get('collision_penalty', 0):+.1f}  ({100*self.collision_frames//max(1,self.total_frames)}% frames, {len(self.collision_events)} events)")
        lines.append(f"    Efficiency:       {bd.get('efficiency_penalty', 0):+.1f}")
        lines.append("-" * 60)
        lines.append(f"  Distance traveled: {self.total_distance_traveled:.1f}m")
        if self._start_pos and self._prev_pos:
            net = math.hypot(self._prev_pos[0] - self._start_pos[0],
                              self._prev_pos[1] - self._start_pos[1])
            lines.append(f"  Net displacement:  {net:.1f}m")
            lines.append(f"  End position:      ({self._prev_pos[0]:.1f}, {self._prev_pos[1]:.1f})")
        lines.append(f"  Mode split:        LANE={self.lane_frames} GPS={self.gps_frames}  ({100*self.lane_frames//max(1,self.total_frames)}% lane)")
        if self.collision_events:
            lines.append("-" * 60)
            lines.append("  COLLISION LOG:")
            for ev in self.collision_events:
                lines.append(f"    t={ev['time']:5.1f}s  {ev['type']:8s}  at ({ev['x']}, {ev['y']})")
        lines.append("=" * 60)
        return "\n".join(lines)
