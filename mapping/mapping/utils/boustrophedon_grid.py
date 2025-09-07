import math
from typing import List, Dict, Tuple, Optional
from enum import Enum


class PatternType(Enum):
    COLUMNS = "COLUMNS"  # Move in columns (forward/backward), transition right/left
    ROWS = "ROWS"  # Move in rows (right/left), transition forward/backward


class Direction(Enum):
    FORWARD = "FORWARD"  # +X direction
    BACKWARD = "BACKWARD"  # -X direction
    RIGHT = "RIGHT"  # -Y direction
    LEFT = "LEFT"  # +Y direction


class BoustrophedonGrid:
    """
    Boustrophedon (lawnmower) grid pattern generator for CBR 2025 Phase 1.

    Supports configurable movement patterns:
    - Column-based: Move forward/backward in columns, transition right/left
    - Row-based: Move right/left in rows, transition forward/backward

    Adapts to initial drone position and desired movement directions.
    """

    def __init__(
        self,
        search_width: float = 6.5,
        search_height: float = 6.5,
        grid_spacing: float = 1.5,
        initial_position: Tuple[float, float] = (0.0, 0.0),
        start_offset: Tuple[float, float] = (1.0, 0.75),
        pattern_type: str = "COLUMNS",
        primary_direction: str = "FORWARD",
        transition_direction: str = "RIGHT",
    ):
        """
        Initialize boustrophedon grid pattern.

        Args:
            search_width: Width of search area (meters)
            search_height: Height of search area (meters)
            grid_spacing: Distance between grid points (meters)
            initial_position: Drone's initial position (x, y)
            start_offset: Offset from initial position to start search (x, y)
            pattern_type: "COLUMNS" or "ROWS"
            primary_direction: Primary movement direction ("FORWARD", "BACKWARD", "RIGHT", "LEFT")
            transition_direction: Transition direction between columns/rows
        """
        self.search_width = search_width
        self.search_height = search_height
        self.grid_spacing = grid_spacing
        self.initial_position = initial_position
        self.start_offset = start_offset

        self.pattern_type = PatternType(pattern_type)
        self.primary_direction = Direction(primary_direction)
        self.transition_direction = Direction(transition_direction)

        # Calculate search origin (drone start + offset)
        self.search_origin = (
            initial_position[0] + start_offset[0],
            initial_position[1] + start_offset[1],
        )

        # Generate waypoints using boustrophedon pattern
        self._waypoints = self._generate_boustrophedon_pattern()
        self._current_index = 0
        self._visited_waypoints = set()

        print(f"Generated {len(self._waypoints)} waypoints in boustrophedon pattern")
        print(
            f"Pattern: {pattern_type}, Primary: {primary_direction}, Transition: {transition_direction}"
        )

    def _generate_boustrophedon_pattern(self) -> List[Dict[str, any]]:
        """Generate boustrophedon waypoints based on configuration."""
        waypoints = []

        if self.pattern_type == PatternType.COLUMNS:
            waypoints = self._generate_column_pattern()
        else:
            waypoints = self._generate_row_pattern()

        return waypoints

    def _generate_column_pattern(self) -> List[Dict[str, any]]:
        """Generate column-based boustrophedon pattern.

        Columns pattern: Move forward/backward along X-axis, transition right/left along Y-axis
        """
        waypoints = []

        num_columns = max(1, int(math.ceil(self.search_width / self.grid_spacing)))
        points_per_column = (
            max(1, int(math.ceil(self.search_height / self.grid_spacing))) + 1
        )

        # Calculate the starting position to center the pattern within search area
        # The pattern should be centered around search_origin
        half_pattern_width = (num_columns - 1) * self.grid_spacing / 2.0
        half_pattern_height = (points_per_column - 1) * self.grid_spacing / 2.0

        for col in range(num_columns):
            if self.transition_direction == Direction.RIGHT:
                # Start from leftmost column and move right
                y_pos = (
                    self.search_origin[1] + half_pattern_width - col * self.grid_spacing
                )
            else:  # LEFT
                # Start from rightmost column and move left
                y_pos = (
                    self.search_origin[1] - half_pattern_width + col * self.grid_spacing
                )

            # Determine movement direction for this column (alternating)
            if col % 2 == 0:
                # Even columns: use primary direction
                column_direction = self.primary_direction
            else:
                # Odd columns: reverse direction
                column_direction = self._reverse_direction(self.primary_direction)

            column_points = self._generate_column_points(
                y_pos, col, column_direction, points_per_column
            )
            waypoints.extend(column_points)

        return waypoints

    def _generate_row_pattern(self) -> List[Dict[str, any]]:
        """Generate row-based boustrophedon pattern.

        Rows pattern: Move right/left along Y-axis, transition forward/backward along X-axis
        """
        waypoints = []

        num_rows = max(1, int(math.ceil(self.search_height / self.grid_spacing)))
        points_per_row = (
            max(1, int(math.ceil(self.search_width / self.grid_spacing))) + 1
        )

        half_pattern_width = (points_per_row - 1) * self.grid_spacing / 2.0
        half_pattern_height = (num_rows - 1) * self.grid_spacing / 2.0

        for row in range(num_rows):
            if self.transition_direction == Direction.FORWARD:
                # Start from bottom row and move up
                x_pos = (
                    self.search_origin[0]
                    - half_pattern_height
                    + row * self.grid_spacing
                )
            else:  # BACKWARD
                # Start from top row and move down
                x_pos = (
                    self.search_origin[0]
                    + half_pattern_height
                    - row * self.grid_spacing
                )

            if row % 2 == 0:
                # Even rows: use primary direction
                row_direction = self.primary_direction
            else:
                # Odd rows: reverse direction for boustrophedon
                row_direction = self._reverse_direction(self.primary_direction)

            row_points = self._generate_row_points(
                x_pos, row, row_direction, points_per_row
            )
            waypoints.extend(row_points)

        return waypoints

    def _generate_column_points(
        self, y_pos: float, col: int, direction: Direction, num_points: int
    ) -> List[Dict[str, any]]:
        """Generate waypoints for a single column.

        Args:
            y_pos: Y position of the column (fixed for all points in column)
            col: Column index
            direction: Movement direction (FORWARD or BACKWARD along X-axis)
            num_points: Number of points in the column
        """
        points = []

        half_pattern_height = (num_points - 1) * self.grid_spacing / 2.0

        for point_idx in range(num_points):
            if direction == Direction.FORWARD:
                # Start from bottom and move up (negative X to positive X)
                x_pos = (
                    self.search_origin[0]
                    - half_pattern_height
                    + point_idx * self.grid_spacing
                )
            else:  # BACKWARD
                # Start from top and move down (positive X to negative X)
                x_pos = (
                    self.search_origin[0]
                    + half_pattern_height
                    - point_idx * self.grid_spacing
                )

            if (
                abs(x_pos - self.search_origin[0]) <= self.search_height / 2.0
                and abs(y_pos - self.search_origin[1]) <= self.search_width / 2.0
            ):

                points.append(
                    {
                        "x": x_pos,
                        "y": y_pos,
                        "column": col,
                        "point_in_column": point_idx,
                        "direction": direction.value,
                        "index": len(points),
                        "visited": False,
                    }
                )

        return points

    def _generate_row_points(
        self, x_pos: float, row: int, direction: Direction, num_points: int
    ) -> List[Dict[str, any]]:
        """Generate waypoints for a single row.

        Args:
            x_pos: X position of the row (fixed for all points in row)
            row: Row index
            direction: Movement direction (RIGHT or LEFT along Y-axis)
            num_points: Number of points in the row
        """
        points = []

        half_pattern_width = (num_points - 1) * self.grid_spacing / 2.0

        for point_idx in range(num_points):
            if direction == Direction.RIGHT:
                # Start from left and move right (negative Y to positive Y)
                y_pos = (
                    self.search_origin[1]
                    - half_pattern_width
                    + point_idx * self.grid_spacing
                )
            else:  # LEFT
                # Start from right and move left (positive Y to negative Y)
                y_pos = (
                    self.search_origin[1]
                    + half_pattern_width
                    - point_idx * self.grid_spacing
                )

            if (
                abs(x_pos - self.search_origin[0]) <= self.search_height / 2.0
                and abs(y_pos - self.search_origin[1]) <= self.search_width / 2.0
            ):

                points.append(
                    {
                        "x": x_pos,
                        "y": y_pos,
                        "row": row,
                        "point_in_row": point_idx,
                        "direction": direction.value,
                        "index": len(points),
                        "visited": False,
                    }
                )

        return points

    def _reverse_direction(self, direction: Direction) -> Direction:
        """Reverse a direction for boustrophedon pattern."""
        reverse_map = {
            Direction.FORWARD: Direction.BACKWARD,
            Direction.BACKWARD: Direction.FORWARD,
            Direction.RIGHT: Direction.LEFT,
            Direction.LEFT: Direction.RIGHT,
        }
        return reverse_map[direction]

    def get_next_waypoint(self) -> Optional[Dict[str, any]]:
        """Get the next unvisited waypoint in boustrophedon order."""
        while self._current_index < len(self._waypoints):
            waypoint = self._waypoints[self._current_index]
            if not waypoint["visited"]:
                return waypoint
            self._current_index += 1

        return None  # All waypoints visited

    def mark_waypoint_visited(self, waypoint_index: int):
        """Mark a waypoint as visited."""
        if 0 <= waypoint_index < len(self._waypoints):
            self._waypoints[waypoint_index]["visited"] = True
            self._visited_waypoints.add(waypoint_index)

    def advance_to_next(self):
        """Advance to next waypoint in sequence."""
        self._current_index += 1

    def get_progress(self) -> Dict[str, any]:
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

    def get_all_waypoints(self) -> List[Dict[str, any]]:
        """Get all waypoints in order."""
        return self._waypoints.copy()

    def reset(self):
        """Reset waypoint system to start."""
        self._current_index = 0
        self._visited_waypoints.clear()
        for waypoint in self._waypoints:
            waypoint["visited"] = False

    def update_initial_position(self, new_position: Tuple[float, float]):
        """
        Update initial position and regenerate waypoints.

        Args:
            new_position: New initial position (x, y)
        """
        self.initial_position = new_position
        self.search_origin = (
            new_position[0] + self.start_offset[0],
            new_position[1] + self.start_offset[1],
        )

        # Regenerate waypoints with new origin
        self._waypoints = self._generate_boustrophedon_pattern()
        self._current_index = 0
        self._visited_waypoints.clear()

        print(
            f"Updated grid origin to ({self.search_origin[0]:.2f}, {self.search_origin[1]:.2f})"
        )
        print(f"Regenerated {len(self._waypoints)} waypoints")

    def get_pattern_summary(self) -> Dict[str, any]:
        """Get summary of the boustrophedon pattern."""
        return {
            "pattern_type": self.pattern_type.value,
            "primary_direction": self.primary_direction.value,
            "transition_direction": self.transition_direction.value,
            "search_area": f"{self.search_width}m x {self.search_height}m",
            "grid_spacing": f"{self.grid_spacing}m",
            "total_waypoints": len(self._waypoints),
            "search_origin": self.search_origin,
            "initial_position": self.initial_position,
            "start_offset": self.start_offset,
        }

    def visualize_pattern(self) -> str:
        """
        Create a simple text visualization of the waypoint pattern.

        Returns:
            String representation of the pattern
        """
        if not self._waypoints:
            return "No waypoints generated"

        # Find bounds
        min_x = min(wp["x"] for wp in self._waypoints)
        max_x = max(wp["x"] for wp in self._waypoints)
        min_y = min(wp["y"] for wp in self._waypoints)
        max_y = max(wp["y"] for wp in self._waypoints)

        visualization = f"\nBoustrophedon Pattern Visualization:\n"
        visualization += f"Pattern: {self.pattern_type.value}, Primary: {self.primary_direction.value}\n"
        visualization += (
            f"Bounds: X[{min_x:.1f}, {max_x:.1f}], Y[{min_y:.1f}, {max_y:.1f}]\n"
        )
        visualization += f"Total waypoints: {len(self._waypoints)}\n"

        # Show first few waypoints in order
        visualization += "\nFirst 10 waypoints in sequence:\n"
        for i, wp in enumerate(self._waypoints[:10]):
            visualization += f"  {i+1:2d}: ({wp['x']:.1f}, {wp['y']:.1f}) - {wp.get('direction', 'N/A')}\n"

        if len(self._waypoints) > 10:
            visualization += f"  ... and {len(self._waypoints) - 10} more waypoints\n"

        return visualization
