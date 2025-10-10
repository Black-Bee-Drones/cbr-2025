"""
Detection Position Calculator

Converts pixel coordinates of detections to world coordinates using camera geometry.
Uses ImageCalculus for projection calculations.
"""

import numpy as np
import math
from typing import Dict, List, Tuple, Optional
from tf_transformations import euler_from_quaternion

from mirela_sdk.image_processing.camera.image_calculus import ImageCalculus
from mapping.constants import (
    CAMERA_RESOLUTION_WIDTH,
    CAMERA_RESOLUTION_HEIGHT,
    CAMERA_FOV_HORIZONTAL,
    CAMERA_FOV_VERTICAL,
    CAMERA_PITCH,
)


class DetectionPositionCalculator:
    """
    Calculate world positions of detected objects using camera geometry.
    
    This class uses ImageCalculus to project detection pixel coordinates
    to world coordinates, accounting for drone position, orientation, and altitude.
    """

    def __init__(self):
        """Initialize the position calculator with camera parameters."""
        self.image_calc = ImageCalculus()
        
        # Configure camera parameters
        self.image_calc.update_camera_resolution(
            width=CAMERA_RESOLUTION_WIDTH,
            height=CAMERA_RESOLUTION_HEIGHT
        )
        
        self.image_calc.update_camera_orientation(
            pitch=CAMERA_PITCH,
            roll=0.0,
            rotation=0.0
        )
        
        # Calculate pixels per degree for this camera
        # Using horizontal FOV and width
        pixels_per_degree_h = CAMERA_RESOLUTION_WIDTH / CAMERA_FOV_HORIZONTAL
        pixels_per_degree_v = CAMERA_RESOLUTION_HEIGHT / CAMERA_FOV_VERTICAL
        
        # Use average for conversion function
        pixels_per_degree = (pixels_per_degree_h + pixels_per_degree_v) / 2.0
        
        # Create conversion function (pixels to degrees)
        self.image_calc.update_pixels_to_degree(
            pixels_to_degree=lambda pixels: pixels / pixels_per_degree
        )

    def calculate_detection_world_position(
        self,
        detection: Dict,
        drone_position: Tuple[float, float, float],
        drone_orientation_quaternion: Tuple[float, float, float, float],
        altitude: float,
    ) -> Optional[Tuple[float, float]]:
        """
        Calculate the world position (x, y) of a detected object.

        Args:
            detection: Detection dict with 'center' key containing (pixel_x, pixel_y)
            drone_position: Current drone position (x, y, z) in world frame
            drone_orientation_quaternion: Drone orientation as quaternion (x, y, z, w)
            altitude: Drone altitude above ground (from lidar)

        Returns:
            Tuple (world_x, world_y) of the detection in world coordinates,
            or None if calculation fails
        """
        try:
            # Extract detection center in pixels
            pixel_x, pixel_y = detection["center"]
            
            # Get drone orientation (roll, pitch, yaw) from quaternion
            roll, pitch, yaw = euler_from_quaternion(drone_orientation_quaternion)
            
            # Calculate ground intersection vector from drone to detection
            # Returns vector in drone's local frame
            vector = self.image_calc.calculate_ground_intersection(
                pixel_x=float(pixel_x),
                pixel_y=float(pixel_y),
                altitude=altitude,
                pitch=pitch,
                roll=roll
            )
            
            if vector is None:
                return None
            
            # Vector is in drone's body frame: (forward, right, up)
            # Need to rotate it to world frame using drone's yaw
            forward_local = vector[0]
            right_local = vector[1]
            
            # Rotate by yaw to get world frame coordinates
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            # Transform from body frame to world frame
            x_offset = forward_local * cos_yaw - right_local * sin_yaw
            y_offset = forward_local * sin_yaw + right_local * cos_yaw
            
            # Add drone's current world position to get absolute position
            world_x = drone_position[0] + x_offset
            world_y = drone_position[1] + y_offset
            
            return (world_x, world_y)
            
        except Exception as e:
            print(f"Error calculating detection position: {e}")
            return None

    def filter_and_select_best_detection(
        self,
        detections: List[Dict],
        drone_position: Tuple[float, float, float],
        drone_orientation_quaternion: Tuple[float, float, float, float],
        altitude: float,
        visited_bases: List[Dict],
        duplicate_radius: float = 1.2,
    ) -> Optional[Dict]:
        """
        Filter detections to remove duplicates and select the best one.

        Strategy:
        1. Calculate world position for each detection
        2. Filter out detections near already visited bases
        3. From remaining detections, select the one closest to drone

        Args:
            detections: List of detection dicts from YOLO
            drone_position: Current drone position (x, y, z)
            drone_orientation_quaternion: Drone orientation (x, y, z, w)
            altitude: Altitude above ground from lidar
            visited_bases: List of visited base positions
            duplicate_radius: Distance threshold to consider as duplicate (meters)

        Returns:
            Best detection dict with added 'world_position' key,
            or None if all detections are duplicates or invalid
        """
        if not detections:
            return None

        # Calculate world positions for all detections
        detections_with_positions = []
        
        for detection in detections:
            world_pos = self.calculate_detection_world_position(
                detection=detection,
                drone_position=drone_position,
                drone_orientation_quaternion=drone_orientation_quaternion,
                altitude=altitude
            )
            
            if world_pos is not None:
                detection_copy = detection.copy()
                detection_copy["world_position"] = world_pos
                detections_with_positions.append(detection_copy)

        if not detections_with_positions:
            return None

        # Filter out detections near visited bases
        valid_detections = []
        
        for detection in detections_with_positions:
            world_x, world_y = detection["world_position"]
            
            # Check distance to all visited bases
            is_duplicate = False
            min_distance_to_visited = float('inf')
            
            for visited_base in visited_bases:
                # Calculate distance to visited base
                dx = abs(world_x - visited_base["x"])
                dy = abs(world_y - visited_base["y"])
                distance = math.sqrt(dx**2 + dy**2)
                
                min_distance_to_visited = min(min_distance_to_visited, distance)
                
                if distance < duplicate_radius:
                    is_duplicate = True
                    print(
                        f"  [FILTER] Detection at ({world_x:.2f}, {world_y:.2f}) "
                        f"is duplicate (dist={distance:.2f}m from visited base)"
                    )
                    break
            
            if not is_duplicate:
                detection["distance_to_nearest_visited"] = min_distance_to_visited
                valid_detections.append(detection)
                print(
                    f"  [FILTER] Detection at ({world_x:.2f}, {world_y:.2f}) "
                    f"is VALID (confidence={detection['confidence']:.2f}, "
                    f"min_dist_to_visited={min_distance_to_visited:.2f}m)"
                )

        if not valid_detections:
            print("  [FILTER] All detections filtered as duplicates")
            return None

        # Select detection closest to drone
        drone_x, drone_y = drone_position[0], drone_position[1]
        
        for detection in valid_detections:
            world_x, world_y = detection["world_position"]
            distance_to_drone = math.sqrt(
                (world_x - drone_x)**2 + (world_y - drone_y)**2
            )
            detection["distance_to_drone"] = distance_to_drone

        # Sort by distance to drone (closest first)
        valid_detections.sort(key=lambda d: d["distance_to_drone"])
        
        best_detection = valid_detections[0]
        world_x, world_y = best_detection["world_position"]
        
        print(
            f"  [FILTER] Selected detection at ({world_x:.2f}, {world_y:.2f}), "
            f"drone_dist={best_detection['distance_to_drone']:.2f}m, "
            f"confidence={best_detection['confidence']:.2f}"
        )
        
        return best_detection

