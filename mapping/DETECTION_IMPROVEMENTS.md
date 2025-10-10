# Detection Position-Based Filtering System

## Overview

This document describes the improved detection system that calculates actual world positions of detected landing bases and uses position-based filtering to handle multiple detections and duplicates more robustly.

## Problem Statement

### Previous Approach Issues:
1. **Simple selection**: Only picked detection with highest `confidence * area`
2. **Drone-based duplicate checking**: Compared drone's position to visited bases
3. **No multi-detection handling**: Couldn't distinguish between multiple bases in same image
4. **Inaccurate duplicate filtering**: A base could appear at different positions depending on drone's location

### Example Problem Scenario:
```
Drone at (5.0, 5.0) detects two bases:
- Base A at pixel (400, 300) → actually at world position (3.2, 4.1)
- Base B at pixel (800, 600) → actually at world position (5.8, 5.9)

Old system: Would pick one based on confidence*area, check if DRONE is near visited
New system: Calculates both base positions, filters if BASES are near visited, picks closest to drone
```

## New Solution Architecture

### Components

#### 1. **DetectionPositionCalculator** (`utils/detection_position_calculator.py`)
Core utility that performs geometric calculations to convert pixel coordinates to world positions.

**Key Features:**
- Uses `ImageCalculus` from `mirela_sdk` for camera projection math
- Accounts for camera FOV, orientation, drone altitude, and pose
- Converts pixel coordinates to world (x, y) positions
- Filters detections based on their calculated world positions
- Selects best detection based on proximity to drone

**Main Methods:**

```python
calculate_detection_world_position(
    detection: Dict,
    drone_position: Tuple[float, float, float],
    drone_orientation_quaternion: Tuple[float, float, float, float],
    altitude: float
) -> Optional[Tuple[float, float]]
```
Calculates world position (x, y) for a single detection.

```python
filter_and_select_best_detection(
    detections: List[Dict],
    drone_position: Tuple[float, float, float],
    drone_orientation_quaternion: Tuple[float, float, float, float],
    altitude: float,
    visited_bases: List[Dict],
    duplicate_radius: float = 1.2
) -> Optional[Dict]
```
Processes all detections: calculates positions, filters duplicates, selects best.

#### 2. **YOLODetector Enhancement** (`utils/yolo_detector.py`)
Modified to support returning all detections instead of just the best one.

**Changes:**
- Added `return_all` parameter to `detect()` method
- Returns `List[Dict]` when `return_all=True`
- Returns single `Dict` (best detection) when `return_all=False` (backward compatible)

#### 3. **CaptureAndDetect State** (`states/navigation_states.py`)
Updated to use position-based filtering for multi-detection handling.

**New Detection Flow:**
```python
1. Capture image
2. Get ALL detections from YOLO (return_all=True)
3. Get drone position, orientation, altitude
4. For each detection:
   - Calculate world position using camera geometry
   - Check distance to all visited bases
   - Filter if too close to visited base
5. From remaining detections, select closest to drone
6. Store selected detection with world_position
```

#### 4. **LandAndWait State** (`states/detection_states.py`)
Updated to save the actual base position instead of drone position.

**Changes:**
- Extracts `world_position` from detection
- Saves base's world position to `visited_bases` list
- Falls back to drone position if world_position unavailable

## Algorithm Details

### Position Calculation Process

```
1. Input:
   - Detection center pixel: (pixel_x, pixel_y)
   - Drone world position: (x, y, z)
   - Drone orientation: quaternion (x, y, z, w)
   - Altitude above ground: lidar_range

2. Camera Geometry:
   - Convert quaternion → roll, pitch, yaw
   - Use ImageCalculus.calculate_ground_intersection()
     → Returns vector in drone body frame: (forward, right, up)

3. Frame Transformation:
   - Rotate vector from body frame to world frame using yaw:
     x_world = forward * cos(yaw) - right * sin(yaw)
     y_world = forward * sin(yaw) + right * cos(yaw)

4. Absolute Position:
   - world_position = drone_position + (x_world, y_world)
```

### Duplicate Filtering Logic

```python
For each detection with calculated world_position (wx, wy):
    For each visited_base at (bx, by):
        distance = sqrt((wx - bx)² + (wy - by)²)
        
        if distance < DUPLICATE_BASE_RADIUS (1.2m):
            Mark detection as DUPLICATE
            Discard detection
            Break
    
    if not DUPLICATE:
        Add to valid_detections list

# Select best from valid detections
If valid_detections is empty:
    Return None (all duplicates)
Else:
    Calculate distance from each valid detection to drone
    Return detection with minimum distance to drone
```

### Selection Strategy

**Priority Order:**
1. **Filter duplicates**: Remove detections near visited bases (< 1.2m)
2. **Prefer proximity**: From remaining, select detection closest to drone
3. **Confidence as tiebreaker**: Already incorporated via YOLO confidence threshold

**Rationale:**
- Closest base is easier/faster to center on
- Less flight time between detection and landing
- More robust against detection noise

## Configuration Parameters

### Camera Calibration (`constants.py`)
```python
CAMERA_RESOLUTION_WIDTH = 1640  # pixels
CAMERA_RESOLUTION_HEIGHT = 1232  # pixels
CAMERA_FOV_HORIZONTAL = 62.2  # degrees (Arducam IMX219)
CAMERA_FOV_VERTICAL = 48.8  # degrees
CAMERA_PITCH = -90.0  # degrees (pointing down)
```

### Detection Parameters
```python
DUPLICATE_BASE_RADIUS = 1.2  # meters - threshold for duplicate detection
YOLO_CONFIDENCE_THRESHOLD = 0.65  # minimum confidence for detection
```

## Usage Examples

### Example 1: Single Detection
```
Input: 1 detection at pixel (820, 616)
Drone at: (3.5, 2.1, 3.0), altitude: 3.0m

Process:
1. Calculate world position → (3.8, 2.3)
2. Check against visited bases → None within 1.2m
3. Result: VALID detection at (3.8, 2.3)
```

### Example 2: Multiple Detections
```
Input: 3 detections
Drone at: (4.0, 4.0, 3.0), altitude: 3.0m
Visited bases: [(2.0, 2.0), (6.0, 6.0)]

Detection A: pixel (300, 400)
  → world pos (2.1, 2.2)
  → distance to visited[0] = 0.14m < 1.2m → DUPLICATE

Detection B: pixel (820, 616)
  → world pos (4.2, 4.1)
  → distance to all visited > 1.2m → VALID
  → distance to drone = 0.22m

Detection C: pixel (1200, 800)
  → world pos (5.5, 5.0)
  → distance to all visited > 1.2m → VALID
  → distance to drone = 1.80m

Result: Select Detection B (closest to drone)
```

### Example 3: All Duplicates
```
Input: 2 detections
Drone at: (3.0, 3.0, 3.0)
Visited bases: [(2.5, 2.8), (4.0, 4.2)]

Detection A: world pos (2.4, 2.9) → 0.14m from visited[0] → DUPLICATE
Detection B: world pos (3.9, 4.3) → 0.14m from visited[1] → DUPLICATE

Result: None (all filtered as duplicates)
Continue to next waypoint
```

## Benefits of New System

### 1. **Accurate Duplicate Detection**
- Compares actual base positions, not drone positions
- Works even when same base viewed from different angles/distances
- Reduces false positive landings on same base

### 2. **Multi-Detection Handling**
- Can distinguish multiple bases in single image
- Intelligently selects which base to approach
- Maximizes mission efficiency

### 3. **Geometric Accuracy**
- Uses proper camera projection math
- Accounts for drone orientation and altitude
- More precise than simple pixel-based heuristics

### 4. **Robust to Viewing Angle**
- Same base at different viewing angles correctly identified as duplicate
- Position calculation adjusts for drone orientation

### 5. **Efficient Mission Planning**
- Selects closest unvisited base when multiple options
- Reduces total flight distance
- Faster mission completion

## Debug Output

The system provides detailed logging for troubleshooting:

```
Found 3 detection(s) in image
Drone position: (4.00, 4.00), altitude: 3.00m
Filtering detections using world position calculation...
  [FILTER] Detection at (2.10, 2.20) is duplicate (dist=0.14m from visited base)
  [FILTER] Detection at (4.20, 4.10) is VALID (confidence=0.85, min_dist_to_visited=2.50m)
  [FILTER] Detection at (5.50, 5.00) is VALID (confidence=0.78, min_dist_to_visited=1.80m)
  [FILTER] Selected detection at (4.20, 4.10), drone_dist=0.22m, confidence=0.85
- NEW landing base detected at world position (4.20, 4.10)!
  Confidence: 0.85, Distance to drone: 0.22m
```

## Testing Recommendations

### Unit Tests
1. Test `DetectionPositionCalculator` with known pixel → world mappings
2. Verify duplicate filtering with various scenarios
3. Test selection logic with multiple valid detections

### Integration Tests
1. Test with simulated multi-detection scenarios
2. Verify behavior when all detections are duplicates
3. Test position accuracy at different altitudes

### Field Tests
1. Fly over known base positions, verify calculated positions
2. Test with multiple bases in camera view
3. Validate duplicate rejection works across multiple visits

## Maintenance Notes

### When to Adjust Parameters

**DUPLICATE_BASE_RADIUS (1.2m):**
- Increase if getting false positive duplicates
- Decrease if missing actual duplicates
- Consider base size (1m plates) + positioning error

**Camera FOV values:**
- Update if camera is changed
- Verify with ground truth measurements
- Check at operational altitude (3m)

**Camera orientation:**
- Adjust CAMERA_PITCH if camera mounting changes
- Currently -90° (straight down)

## Future Improvements

### Potential Enhancements:
1. **Confidence weighting**: Combine distance and confidence in selection
2. **Historical tracking**: Track base positions across multiple detections
3. **Kalman filtering**: Improve position estimates with multiple observations
4. **Adaptive duplicate radius**: Adjust based on altitude or detection confidence
5. **3D position estimation**: Include height of base in calculations

## Dependencies

- `mirela_sdk.image_processing.camera.image_calculus.ImageCalculus`
- `tf_transformations.euler_from_quaternion`
- NumPy for vector operations
- Math for trigonometric calculations

## References

- ImageCalculus documentation: `mirela-sdk/mirela_sdk/mirela_sdk/image_processing/camera/image_calculus.py`
- Camera specs: Arducam IMX219 (62.2° × 48.8° FOV)
- Mission specs: CBR 2025 Phase 1 rules

