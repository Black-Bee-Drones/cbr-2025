"""Maze exploration data management"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import numpy as np
from enum import Enum
import time


class Direction(Enum):
    """Direction enumeration for maze navigation."""

    NORTH = 0  # Forward
    EAST = 90  # Right
    SOUTH = 180  # Backward
    WEST = 270  # Left

    @classmethod
    def from_angle(cls, angle: int):
        """Convert angle to direction."""
        normalized = angle % 360
        for direction in cls:
            if direction.value == normalized:
                return direction
        # Return closest direction
        angles = [d.value for d in cls]
        closest_idx = np.argmin([abs(a - normalized) for a in angles])
        return list(cls)[closest_idx]

    def to_relative(self, other: "Direction") -> int:
        """Get relative angle to another direction."""
        diff = (other.value - self.value) % 360
        return diff


@dataclass
class ScanData:
    """Data from a single scan position."""

    position_id: int
    grid_position: Tuple[int, int]  # Grid coordinates (x, y)
    height_level: str  # "high" or "low"
    photos: List[np.ndarray] = field(default_factory=list)
    lidar_readings: List[float] = field(default_factory=list)
    directions: List[Direction] = field(default_factory=list)
    qr_codes: List[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

    def add_scan(
        self, direction: Direction, photo: np.ndarray, lidar: float, qr_code: str = None
    ):
        """Add scan data for a specific direction."""
        self.directions.append(direction)
        self.photos.append(photo)
        self.lidar_readings.append(lidar)
        if qr_code:
            self.qr_codes.append(qr_code)

    def get_passages(self, threshold: float = 0.5) -> List[Direction]:
        """Get directions with passages (lidar > threshold)."""
        passages = []
        for i, reading in enumerate(self.lidar_readings):
            if reading > threshold:
                passages.append(self.directions[i])
        return passages


@dataclass
class GridPoint:
    """Represents a point in the maze grid."""

    x: int
    y: int
    visited: bool = False
    scan_data_high: Optional[ScanData] = None
    scan_data_low: Optional[ScanData] = None
    entry_direction: Optional[Direction] = None
    available_passages: List[Direction] = field(default_factory=list)

    @property
    def position(self) -> Tuple[int, int]:
        """Get grid position as tuple."""
        return (self.x, self.y)

    def is_fully_scanned(self) -> bool:
        """Check if both height levels have been scanned."""
        return self.scan_data_high is not None and self.scan_data_low is not None


class MazeMap:
    """
    Manages the maze exploration data and mapping.
    """

    def __init__(self, grid_size: float = 1.0):
        """
        Initialize maze map.

        Parameters
        ----------
        grid_size : float
            Size of each grid cell in meters
        """
        self.grid_size = grid_size
        self.grid_points: Dict[Tuple[int, int], GridPoint] = {}
        self.current_position = (0, 0)  # Start at origin
        self.current_direction = Direction.NORTH
        self.visited_count = 0
        self.scan_history: List[ScanData] = []
        self.qr_codes_found: Dict[str, Tuple[int, int]] = {}

    def add_grid_point(self, x: int, y: int) -> GridPoint:
        """Add or get a grid point."""
        if (x, y) not in self.grid_points:
            self.grid_points[(x, y)] = GridPoint(x, y)
        return self.grid_points[(x, y)]

    def mark_visited(self, x: int, y: int, entry_direction: Direction = None):
        """Mark a grid point as visited."""
        point = self.add_grid_point(x, y)
        if not point.visited:
            point.visited = True
            point.entry_direction = entry_direction
            self.visited_count += 1
        self.current_position = (x, y)

    def add_scan_data(
        self, position: Tuple[int, int], height_level: str, scan_data: ScanData
    ):
        """Add scan data for a position."""
        point = self.add_grid_point(position[0], position[1])

        if height_level == "high":
            point.scan_data_high = scan_data
        else:
            point.scan_data_low = scan_data

        self.scan_history.append(scan_data)

        # Update available passages
        passages = scan_data.get_passages()
        for passage in passages:
            if passage not in point.available_passages:
                point.available_passages.append(passage)

        # Update QR codes found
        for qr_code in scan_data.qr_codes:
            if qr_code not in self.qr_codes_found:
                self.qr_codes_found[qr_code] = position

    def get_next_unvisited_passage(self) -> Optional[Tuple[Direction, Tuple[int, int]]]:
        """
        Get the next unvisited passage from current position.

        Returns
        -------
        Optional[Tuple[Direction, Tuple[int, int]]]
            Direction and target position, or None if no unvisited passages
        """
        current_point = self.grid_points.get(self.current_position)
        if not current_point:
            return None

        # Check available passages for unvisited neighbors
        for direction in current_point.available_passages:
            target_pos = self.get_neighbor_position(self.current_position, direction)
            if (
                target_pos not in self.grid_points
                or not self.grid_points[target_pos].visited
            ):
                return (direction, target_pos)

        return None

    def get_neighbor_position(
        self, position: Tuple[int, int], direction: Direction
    ) -> Tuple[int, int]:
        """Get neighbor position in given direction."""
        x, y = position
        if direction == Direction.NORTH:
            return (x, y + 1)
        elif direction == Direction.EAST:
            return (x + 1, y)
        elif direction == Direction.SOUTH:
            return (x, y - 1)
        elif direction == Direction.WEST:
            return (x - 1, y)
        return position

    def get_unvisited_neighbors(
        self, position: Tuple[int, int] = None
    ) -> List[Tuple[int, int]]:
        """Get list of unvisited neighbor positions."""
        if position is None:
            position = self.current_position

        neighbors = []
        for direction in Direction:
            neighbor_pos = self.get_neighbor_position(position, direction)
            if (
                neighbor_pos not in self.grid_points
                or not self.grid_points[neighbor_pos].visited
            ):
                neighbors.append(neighbor_pos)

        return neighbors

    def find_path_to(self, target: Tuple[int, int]) -> List[Direction]:
        """
        Find path from current position to target using BFS.

        Parameters
        ----------
        target : Tuple[int, int]
            Target grid position

        Returns
        -------
        List[Direction]
            List of directions to reach target
        """
        from collections import deque

        start = self.current_position
        if start == target:
            return []

        # BFS to find shortest path
        queue = deque([(start, [])])
        visited = {start}

        while queue:
            pos, path = queue.popleft()

            for direction in Direction:
                next_pos = self.get_neighbor_position(pos, direction)

                if next_pos == target:
                    return path + [direction]

                if next_pos not in visited:
                    # Check if we know this position is accessible
                    if next_pos in self.grid_points or len(path) < 3:
                        visited.add(next_pos)
                        queue.append((next_pos, path + [direction]))

        return []  # No path found

    def get_exploration_summary(self) -> Dict:
        """Get summary of exploration progress."""
        total_scans = len(self.scan_history)
        high_scans = sum(1 for s in self.scan_history if s.height_level == "high")
        low_scans = total_scans - high_scans

        return {
            "visited_points": self.visited_count,
            "total_scans": total_scans,
            "high_scans": high_scans,
            "low_scans": low_scans,
            "qr_codes_found": list(self.qr_codes_found.keys()),
            "qr_code_count": len(self.qr_codes_found),
            "current_position": self.current_position,
            "current_direction": self.current_direction.name,
        }

    def update_direction(self, new_direction: Direction):
        """Update current facing direction."""
        self.current_direction = new_direction
