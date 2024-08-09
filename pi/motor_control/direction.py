# A class to specify what pin to use to move the sub in a direction,
# e.g. VFL or HBR, in an integer
# (V = vertical / H = horizontal)(F = front / B = back)(L = left / R = right)
# E.g. HFL = horizontal front left motor

# ALL PLACES USING MOTORS SHOULD REFER TO THIS, so if needed the indices can change

# haha bad code
directions = {
    "VFL": 6,
    "VFR": 7,
    "VBL": 4,
    "VBR": 5,
    "HFL": 8,
    "HFR": 9,
    "HBL": 2,
    "HBR": 3
}

vertical = ["VFL", "VFR", "VBL", "VBR"]
horizontal = ["HFL", "HFR", "HBL", "HBR"]


def to_direction(upwards, forwards, leftwards):
    """Parameters are booleans. True-true-true = VFL, false-false-false=HBR"""
    up = "V" if upwards else "H"
    forward = "F" if forwards else "B"
    side = "L" if leftwards else "R"
    return directions[up + forward + side]


def direction(key):
    """Get the direction directly from a string, e.g. 'VFL'"""
    return directions[key]


def key_from_direction(direction_to_find):
    """Gets the key (e.g. HFL) from the direction (e.g. 0)"""
    for key, loop_direction in directions:
        if loop_direction == direction_to_find:
            return key
