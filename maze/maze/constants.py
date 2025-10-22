"""Constants for CBR 2025 Phase 4 - Maze Navigation"""

# Arena dimensions
MAZE_WIDTH = 2.0  # meters
MAZE_HEIGHT = 6.0  # meters
MAZE_CEILING = 1.5  # meters
GATE_WIDTH = 0.8  # meters
GATE_HEIGHT = 0.8  # meters
GRID_SIZE = 1.0  # 1m x 1m grid sections

# Mission parameters
MAX_POINTS_TO_VISIT = 12  # Maximum number of points to explore
TOTAL_POSITIONS = 24  # 12 high + 12 low positions

# Flight parameters
ENTRY_HEIGHT = 1.2  # meters - initial height for entering maze (1.2m absolute)
HIGH_SCAN_HEIGHT = 1.2  # meters - upper scanning height (1.2m absolute)
LOW_SCAN_HEIGHT = 0.4  # meters - lower scanning height (0.4m absolute)
NAVIGATION_SPEED = 0.2  # m/s - default navigation speed
ROTATION_SPEED = 30  # degrees/second

# Movement distances
ENTRY_DISTANCE = 1.0  # meters - distance to move when entering a section
ADVANCE_DISTANCE = 1.0  # meters - distance to move between grid points
CENTERING_OFFSET = 0.5  # meters - offset for centering in grid

# Lidar parameters
LIDAR_THRESHOLD = 0.5  # meters - minimum distance to consider as passage
LIDAR_TIMEOUT = 2.0  # seconds - timeout for lidar reading
LIDAR_WEBSOCKET_URL = "ws://192.168.10.1:8765"  # WebSocket URL for lidar data

# Camera parameters
PHOTO_DELAY = 0.5  # seconds - delay after movement before taking photo
ROTATION_ANGLES = [0, 90, 180, 270]  # degrees - angles for scanning
PHOTOS_PER_POSITION = 4  # Number of photos at each position

# QR Code detection
QR_CODE_SIZE = 10  # cm - size of QR codes
QR_CODES_EXPECTED = ["A", "B", "C", "D", "E"]  # Expected QR code values
DETECTION_CONFIDENCE = 0.8  # Confidence threshold for QR detection

# Timeouts
NAVIGATION_TIMEOUT = 600  # seconds - 10 minutes max for navigation
TAKEOFF_TIMEOUT = 10  # seconds - takeoff timeout
LAND_TIMEOUT = 10  # seconds - landing timeout
CONNECTION_TIMEOUT = 30  # seconds - connection timeout

# State machine outcomes
SUCCESS = "success"
FAILURE = "failure"
TIMEOUT = "timeout"
CONTINUE = "continue"
COMPLETE = "complete"
