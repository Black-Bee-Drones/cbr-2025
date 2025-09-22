# Delivery constants

TAKEOFF_ALTITUDE = 3.0 # Meters

# Package Info
STARTING_PACKAGE_IDX = 0
LAST_PACKAGE_IDX = 2

# Movement tolerances
VELOCITY_TOLERANCE = 0.1  # m/s - velocity considered as stopped
ALTITUDE_TOLERANCE = 0.1  # meters - altitude precision tolerance
POSITION_TOLERANCE = 0.3  # meters - position precision tolerance

# Height reference management
MAINTAIN_ABSOLUTE_HEIGHT = True  # Keep same height above original ground level
TARGET_HEIGHT_ABOVE_GROUND = 3.0  # meters - desired height above original ground
ALTITUDE_COMPENSATION_TIME = 5.0  # seconds - time for drone to auto-compensate height
ALTITUDE_COMPENSATION_GAIN = 0.4  # gain for counteracting auto-compensation

# Position control parameters
POSITION_CONTROLLER_KP_XY = 0.6  # Proportional gain for XY movement
POSITION_CONTROLLER_KP_Z = 0.6  # Proportional gain for Z movement
POSITION_CONTROLLER_KP_YAW = 0.5  # Proportional gain for yaw
MAX_VELOCITY_XY = 0.8
MAX_VELOCITY_Z = 0.5
