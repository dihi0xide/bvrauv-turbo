# TODO make sure this can be safely removed and then remove it; bounds should be implemented
# TODO per motor, not per PID

class BoundedPID:
    def __init__(self, PID, max, min, allow_negative=True):
        """Allow negative means whether to reflect the bounds across 0"""
        self.PID = PID
        self.max = max
        self.min = min
        self.allow_negative = allow_negative
        raise DeprecationWarning()

    def signal(self, current):
        signal = self.PID.signal(current)
        sign = 1 if (signal >= 0 or not self.allow_negative) else -1

        if self.allow_negative:
            signal = abs(signal)

        if signal > self.max:
            signal = self.max

        if self.min > signal:
            signal = self.min

        return signal * sign