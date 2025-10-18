import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

import time
import cv2
import numpy as np

from navigation.constants import (
    COMMAND_SLEEP, GREEN, RED, YELLOW, RESET
)

class BaseSearch(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        
    def execute(self, blackboard: Blackboard):
        try:
            tello = blackboard["tello"]
            tello.go_xyz_speed(x = -350, y = 150, z = 100, speed = 100)
            return SUCCEED
            
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{RED}Base search failed: {e}{RESET}")
            return ABORT

        
