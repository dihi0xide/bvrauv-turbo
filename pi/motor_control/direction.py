# A class to specify what pin to use to move the sub in a direction,
# e.g. VFL or HBR, in an integer
# (V = vertical / H = horizontal)(F = front / B = back)(L = left / R = right)
# E.g. HFL = horizontal front left motor

# ALL PLACES USING MOTORS SHOULD REFER TO THIS, so if needed the indices can change

# haha bad code
directions = {
    "VFL": 0,
    "VFR": 1,
    "VBL": 2,
    "VBR": 3,
    "HFL": 4,
    "HFR": 5,
    "HBL": 6,
    "HBR": 7
}


def to_direction(upwards, forwards, leftwards):
    """Parameters are booleans. True-true-true = VFL, false-false-false=HBR"""
    up = "V" if upwards else "H"
    forward = "F" if forwards else "B"
    side = "L" if leftwards else "R"
    return directions[up + forward + side]


def direction(key):
    """Get the direction directly from a string, e.g. 'VFL'"""
    return directions[key]
