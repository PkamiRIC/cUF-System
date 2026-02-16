import time

class InterruptibleSleeper:
    def __init__(self, stop_flag):
        self.stop_flag = stop_flag

    def sleep(self, seconds):
        interval = 0.1
        steps = int(seconds / interval)
        for _ in range(steps):
            if self.stop_flag():
                raise InterruptedError("[Sleep Interrupted by STOP]")
            time.sleep(interval)

