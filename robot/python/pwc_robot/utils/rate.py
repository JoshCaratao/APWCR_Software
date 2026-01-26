import time


class Rate:
    """
    Simple non-blocking rate limiter.

    Usage:
        rate = Rate(hz=10)
        while True:
            now = time.perf_counter()
            if rate.ready(now):
                do_work()
    """

    def __init__(self, hz: float):
        if hz <= 0:
            raise ValueError("Rate hz must be > 0")

        self.hz = float(hz)
        self.period = 1.0 / self.hz
        self._last_t = 0.0

    def ready(self, now: float | None = None) -> bool:
        """
        Returns True if enough time has elapsed since last trigger.
        Does NOT block or sleep.
        """
        if now is None:
            now = time.perf_counter()

        if (now - self._last_t) >= self.period:
            self._last_t = now
            return True

        return False

    def reset(self):
        """
        Reset timer so next ready() triggers immediately.
        """
        self._last_t = 0.0
