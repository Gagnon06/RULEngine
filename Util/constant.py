# Under MIT License, see LICENSE.txt
""" Module définissant des constantes de programmations python pour l'IA """
from .Position import Position
__author__ = 'RoboCupULaval'

ROBOT_RADIUS = 90
PLAYER_PER_TEAM = 6
KICK_MAX_SPD = 8.0

# Field Parameters
FIELD_Y_TOP = 3000
FIELD_Y_BOTTOM = -3000
FIELD_X_LEFT = -4500
FIELD_X_RIGHT = 4500
FIELD_GOAL_RADIUS = 1000
FIELD_GOAL_SEGMENT = 500

# Goal Parameters
FIELD_GOAL_Y_TOP = FIELD_GOAL_RADIUS + FIELD_GOAL_SEGMENT / 2
FIELD_GOAL_Y_BOTTOM = (FIELD_GOAL_RADIUS + FIELD_GOAL_SEGMENT / 2) * -1
FIELD_GOAL_BLUE_X_LEFT = FIELD_X_LEFT
FIELD_GOAL_BLUE_X_RIGHT = FIELD_X_LEFT + FIELD_GOAL_RADIUS
FIELD_GOAL_YELLOW_X_LEFT = FIELD_X_RIGHT - FIELD_GOAL_RADIUS
FIELD_GOAL_YELLOW_X_RIGHT = FIELD_X_RIGHT

# Field Positions
FIELD_GOAL_BLUE_TOP_CIRCLE = Position(FIELD_X_LEFT, FIELD_GOAL_SEGMENT / 2)
FIELD_GOAL_BLUE_BOTTOM_CIRCLE = Position(FIELD_X_LEFT, FIELD_GOAL_SEGMENT / 2 * -1)
FIELD_GOAL_YELLOW_TOP_CIRCLE = Position(FIELD_X_RIGHT, FIELD_GOAL_SEGMENT / 2)
FIELD_GOAL_YELLOW_BOTTOM_CIRCLE = Position(FIELD_X_RIGHT, FIELD_GOAL_SEGMENT / 2 * -1)

# Simulation param
DELTA_T = 17 #ms, hack, à éviter

# Communication information
DEBUG_RECEIVE_BUFFER_SIZE = 100
