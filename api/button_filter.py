import datetime

class ButtonFilter:
    def __init__(self, delay=0.5):
        self.delay = delay
        self.last_press = {}

    def allow(self, button_code: int) -> bool:
        """Check if button press should be allowed based on delay filtering."""

        now = datetime.datetime.now().timestamp()
        last_time = self.last_press.get(button_code, 0)

        if now - last_time > self.delay:
            self.last_press[button_code] = now
            return True
        return False
