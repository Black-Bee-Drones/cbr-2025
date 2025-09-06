import math
from typing import List, Dict, Tuple


class GridWaypoints:
    """
    Grid waypoint
    """

    def __init__(
        self,
        search_width: float = 6.5,
        search_height: float = 6.5,
        grid_spacing: float = 1.5,
        origin_offset: Tuple[float, float] = (1.0, 0.75),
    ):
        """
        Initialize grid waypoint system.

        Args:
            search_width: Width of search area (meters)
            search_height: Height of search area (meters)
            grid_spacing: Distance between grid points (meters)
            origin_offset: Offset from arena origin to start of search grid (meters)
        """
        self.search_width = search_width
        self.search_height = search_height
        self.grid_spacing = grid_spacing
        self.origin_offset = origin_offset

        # Calculate grid dimensions
        self.grid_cols = max(1, int(math.ceil(search_width / grid_spacing))) + 1
        self.grid_rows = max(1, int(math.ceil(search_height / grid_spacing))) + 1

        # Generate waypoints
        self._waypoints = self._generate_grid_waypoints()
        self._current_index = 0
        self._visited_waypoints = set()

    def _generate_grid_waypoints(self) -> List[Dict[str, float]]:
        """Generate grid waypoints covering the search area."""
        waypoints = []

        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                x = self.origin_offset[0] + col * self.grid_spacing
                y = self.origin_offset[1] + row * self.grid_spacing

                if (
                    x <= self.search_width + self.origin_offset[0]
                    and y <= self.search_height + self.origin_offset[1]
                ):
                    waypoints.append(
                        {
                            "x": x,
                            "y": y,
                            "row": row,
                            "col": col,
                            "index": len(waypoints),
                            "visited": False,
                        }
                    )

        return waypoints

    def get_next_waypoint(self) -> Dict[str, float]:
        """Get the next unvisited waypoint."""
        while self._current_index < len(self._waypoints):
            waypoint = self._waypoints[self._current_index]
            if not waypoint["visited"]:
                return waypoint
            self._current_index += 1

        return None

    def mark_waypoint_visited(self, waypoint_index: int):
        """Mark a waypoint as visited."""
        if 0 <= waypoint_index < len(self._waypoints):
            self._waypoints[waypoint_index]["visited"] = True
            self._visited_waypoints.add(waypoint_index)

    def get_waypoint_by_position(
        self, x: float, y: float, tolerance: float = 0.5
    ) -> Dict[str, float]:
        """Find waypoint closest to given position within tolerance."""
        closest_waypoint = None
        min_distance = float("inf")

        for waypoint in self._waypoints:
            distance = math.sqrt((waypoint["x"] - x) ** 2 + (waypoint["y"] - y) ** 2)
            if distance < min_distance and distance <= tolerance:
                min_distance = distance
                closest_waypoint = waypoint

        return closest_waypoint

    def advance_to_next(self):
        """Advance the current index to next waypoint."""
        self._current_index += 1

    def get_progress(self) -> Dict[str, float]:
        """Get current progress through waypoints."""
        total_waypoints = len(self._waypoints)
        visited_count = len(self._visited_waypoints)

        return {
            "total_waypoints": total_waypoints,
            "visited_waypoints": visited_count,
            "remaining_waypoints": total_waypoints - visited_count,
            "progress_percent": (
                (visited_count / total_waypoints) * 100 if total_waypoints > 0 else 0.0
            ),
            "current_index": self._current_index,
        }

    def is_complete(self) -> bool:
        """Check if all waypoints have been visited."""
        return len(self._visited_waypoints) >= len(self._waypoints)

    def get_all_waypoints(self) -> List[Dict[str, float]]:
        """Get all waypoints."""
        return self._waypoints.copy()

    def reset(self):
        """Reset waypoint system to start."""
        self._current_index = 0
        self._visited_waypoints.clear()
        for waypoint in self._waypoints:
            waypoint["visited"] = False

    def get_summary(self) -> Dict[str, any]:
        """Get summary of grid configuration."""
        return {
            "search_area": f"{self.search_width}m x {self.search_height}m",
            "grid_spacing": f"{self.grid_spacing}m",
            "grid_dimensions": f"{self.grid_cols} x {self.grid_rows}",
            "total_waypoints": len(self._waypoints),
            "origin_offset": self.origin_offset,
            "coverage_efficiency": f"{(len(self._waypoints) * self.grid_spacing**2) / (self.search_width * self.search_height) * 100:.1f}%",
        }
