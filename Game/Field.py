# Under MIT License, see LICENSE.txt
from RULEngine.Util.Position import Position


class Field:

    def __init__(self, ball, terrain_type="sim"):
        self.ball = ball

        if terrain_type == "sim":
            self.constant = simulation
        elif terrain_type == "real":
            self.constant = real_life
        else:
            print("ERREUR lors de la création de l'objet field\n Mauvais terrain_type passé - simulation choisi\n")
            self.constant = simulation

    def move_ball(self, position, delta):
        self.ball.set_position(position, delta)


simulation = {
    "ROBOT_RADIUS": 90,
    "BALL_RADIUS": 22,
    "PLAYER_PER_TEAM": 6,
    "KICK_MAX_SPD": 8.0,

    # Field Parameters
    "FIELD_Y_TOP": 3000,
    "FIELD_Y_BOTTOM": -3000,
    "FIELD_X_LEFT": -4500,
    "FIELD_X_RIGHT": 4500,
    "FIELD_GOAL_RADIUS": 1000,
    "FIELD_GOAL_SEGMENT": 500,

    # Goal Parameters
    "FIELD_GOAL_Y_TOP": 1250,  # FIELD_GOAL_RADIUS + FIELD_GOAL_SEGMENT / 2
    "FIELD_GOAL_Y_BOTTOM": -1250,  # (FIELD_GOAL_RADIUS + FIELD_GOAL_SEGMENT / 2) * -1
    "FIELD_GOAL_BLUE_X_LEFT": -4500,  # FIELD_X_LEFT
    "FIELD_GOAL_BLUE_X_RIGHT": -3500,  # FIELD_X_LEFT + FIELD_GOAL_RADIUS
    "FIELD_GOAL_YELLOW_X_LEFT": 3500,  # FIELD_X_RIGHT - FIELD_GOAL_RADIUS
    "FIELD_GOAL_YELLOW_X_RIGHT": 4500,  # FIELD_X_RIGHT

    # Field Positions
    "FIELD_GOAL_BLUE_TOP_CIRCLE": Position(-4500, 250),  # FIELD_X_LEFT, FIELD_GOAL_SEGMENT / 2)
    "FIELD_GOAL_BLUE_BOTTOM_CIRCLE": Position( -4500,-250),  # FIELD_X_LEFT, FIELD_GOAL_SEGMENT / 2 * -1)
    "FIELD_GOAL_YELLOW_TOP_CIRCLE": Position(4500, 250),  # FIELD_X_RIGHT, FIELD_GOAL_SEGMENT / 2)
    "FIELD_GOAL_YELLOW_BOTTOM_CIRCLE": Position(4500, -250),  # FIELD_X_RIGHT, FIELD_GOAL_SEGMENT / 2 * -1)

    # Legal field dimensions
    "LEGAL_Y_TOP": 3000,
    # LEGAL_Y_TOP": 0
    "LEGAL_Y_BOTTOM": -3000,
    "LEGAL_X_LEFT": -4500,
    "LEGAL_X_RIGHT": 4500,
    # LEGAL_X_RIGHT": 0

    # Simulation param
    "DELTA_T": 17,  # ms, hack, à éviter

    # Communication information
    "DEBUG_RECEIVE_BUFFER_SIZE": 100,

    # Deadzones
    "SPEED_DEAD_ZONE_DISTANCE": 150,
    "POSITION_DEADZONE": 135,  # ROBOT_RADIUS*1.5

    # Radius and angles for tactics
    "DISTANCE_BEHIND": 120,  # ROBOT_RADIUS + 30  # in millimeters
    "ANGLE_TO_GRAB_BALL": 1,  # in radians; must be large in case ball moves fast
    "RADIUS_TO_GRAB_BALL": 120,  # ROBOT_RADIUS + 30
    "ANGLE_TO_HALT": 0.09,
    "RADIUS_TO_HALT": 102,  # ROBOT_RADIUS + BALL_RADIUS

    # Orientation abs_tol
    "ORIENTATION_ABSOLUTE_TOLERANCE": 1e-4,
    "SPEED_ABSOLUTE_TOLERANCE": 1e-3,

    # Speed
    "DEFAULT_MAX_SPEED": 1,
    "DEFAULT_MIN_SPEED": 0.65
}

real_life = {
    "ROBOT_RADIUS": 90,
    "BALL_RADIUS": 22,
    "PLAYER_PER_TEAM": 6,
    "KICK_MAX_SPD": 8.0,

    # Field Parameters
    "FIELD_Y_TOP": 1090,
    "FIELD_Y_BOTTOM": -1090,
    "FIELD_X_LEFT": -1450,
    "FIELD_X_RIGHT": 1450,
    "FIELD_GOAL_RADIUS": 363,
    "FIELD_GOAL_SEGMENT": 181,

    # Goal Parameters
    "FIELD_GOAL_Y_TOP": 436,  # FIELD_GOAL_RADIUS + FIELD_GOAL_SEGMENT / 2
    "FIELD_GOAL_Y_BOTTOM": -436,  # (FIELD_GOAL_RADIUS + FIELD_GOAL_SEGMENT / 2) * -1
    "FIELD_GOAL_BLUE_X_LEFT": -1636,  # FIELD_X_LEFT
    "FIELD_GOAL_BLUE_X_RIGHT": -1272,  # FIELD_X_LEFT + FIELD_GOAL_RADIUS
    "FIELD_GOAL_YELLOW_X_LEFT": 1272,  # FIELD_X_RIGHT - FIELD_GOAL_RADIUS
    "FIELD_GOAL_YELLOW_X_RIGHT": 1636,  # FIELD_X_RIGHT

    # Field Positions
    "FIELD_GOAL_BLUE_TOP_CIRCLE": Position(-4500, 250),  # FIELD_X_LEFT, FIELD_GOAL_SEGMENT / 2)
    "FIELD_GOAL_BLUE_BOTTOM_CIRCLE": Position(-4500, -250),  # FIELD_X_LEFT, FIELD_GOAL_SEGMENT / 2 * -1)
    "FIELD_GOAL_YELLOW_TOP_CIRCLE": Position(4500, 250),  # FIELD_X_RIGHT, FIELD_GOAL_SEGMENT / 2)
    "FIELD_GOAL_YELLOW_BOTTOM_CIRCLE": Position(4500, -250),  # FIELD_X_RIGHT, FIELD_GOAL_SEGMENT / 2 * -1)

    # Legal field dimensions
    "LEGAL_Y_TOP": 3000,
    # LEGAL_Y_TOP": 0
    "LEGAL_Y_BOTTOM": -3000,
    "LEGAL_X_LEFT": -4500,
    "LEGAL_X_RIGHT": 4500,
    # LEGAL_X_RIGHT": 0

    # Simulation param
    "DELTA_T": 17,  # ms, hack, à éviter

    # Communication information
    "DEBUG_RECEIVE_BUFFER_SIZE": 100,

    # Deadzones
    "SPEED_DEAD_ZONE_DISTANCE": 150,
    "POSITION_DEADZONE": 200,  # ROBOT_RADIUS*1.5

    # Radius and angles for tactics
    "DISTANCE_BEHIND": 120,  # ROBOT_RADIUS + 30  # in millimeters
    "ANGLE_TO_GRAB_BALL": 1,  # in radians; must be large in case ball moves fast
    "RADIUS_TO_GRAB_BALL": 120,  # ROBOT_RADIUS + 30
    "ANGLE_TO_HALT": 0.09,
    "RADIUS_TO_HALT": 102,  # ROBOT_RADIUS + BALL_RADIUS

    # Orientation abs_tol
    "ORIENTATION_ABSOLUTE_TOLERANCE": 1e-4,
    "SPEED_ABSOLUTE_TOLERANCE": 1e-3,

    # Speed
    "DEFAULT_MAX_SPEED": 1,
    "DEFAULT_MIN_SPEED": 0.65
}
