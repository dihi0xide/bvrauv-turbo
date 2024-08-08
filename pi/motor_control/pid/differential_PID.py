from pi.motor_control.pid.PID import PID

# This code is to fix the issue that a lot of our motors have a minimum speed, stopping us
# from correcting at small turns and leading to constant overshooting. It does this by having
# two diagonally opposite motors turning at one speed (this is just normal turning),
# but if it would go below the minimum instead it stays at that value and the other diagonal
# spins at the difference of the wanted and the minimum; eg. if you want to spin at 20 and 
# the minimum is 10, then two can just spin at 20 normally, but if you want to spin at 5
# then two can spin at 15 and the other two at 10 in the other direction, and 15 - 10 = 5

# this can be thought of generally as the first diagonal goes at the min + the speed you want,
# and the other diagonal goes at the min in the other direction; so min + speed - min = speed


class DifferentialPID:
    def __init__(self, pid, motor_min):
        self.pid = pid
        self.min = motor_min

    def signal(self, current):
        """
        Returns a tuple (x, y) where x is the speed to turn the two diagonal motors at,
        and y is the speed to turn the other two diagonal motors to slow down if the first two
        have reached their minimum

        The motors turning at y should be turning to slow down (opposite to) the other motors,
        not speeding them up.
        """
        value = self.pid.signal(current)

        underflow = self.min > value
        # whether the value would go below the minimum threshold
        
        overcorrected = value + self.min if underflow else value
        # e.g. if value is 10 but min is 15,
        # overcorrected = 10 + 15 = 25

        correction = self.min if underflow else 0
        # e.g. overcorrected is 25, correction is self.min = 15

        # so, you'll have two motors spinning at 25 in one direction,
        # and two spinning at 15 in the other direction; it should be about equivalent
        # to just spinning at 10

        return overcorrected, -correction
        # negative to turn in other direction
