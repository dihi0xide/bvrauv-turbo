class BoundedPID:
    def __init__(self, PID, max, min, allow_negative=True):
        """Allow negative means whether to reflect the bounds across 0"""
        self.PID = PID
        self.max = max
        self.min = min
        self.allow_negative = allow_negative

    def signal(self, current):
        sign = 1 if (current >= 0 or self.allow_negative) else -1

        if self.allow_negative:
            current = abs(current)

        if current > self.max:
            current = self.max

        if self.min > current:
            current = self.min

        return self.PID.signal(current * sign)
