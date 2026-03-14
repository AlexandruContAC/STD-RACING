"""
Proportional steering controller.

error = image_center_x - detected_track_center
steering = clamp(Kp * error, -1.0, 1.0)

Positive steering → turn right, negative → turn left.
"""

import config


class SteeringController:
    """Simple P (with PID stubs) steering from a track center offset."""

    def __init__(
        self,
        image_width: int = config.FRAME_WIDTH,
        kp: float = config.STEERING_KP,
        ki: float = config.STEERING_KI,
        kd: float = config.STEERING_KD,
    ) -> None:
        self._image_center = image_width / 2.0
        self._kp = kp
        self._ki = ki
        self._kd = kd

        # PID state (for future use)
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, track_center: float | None) -> float:
        """
        Return a normalised steering value in [-1.0, 1.0].

        If *track_center* is None (track lost), returns 0.0.
        """
        if track_center is None:
            return 0.0

        error = self._image_center - track_center

        # P term
        p = self._kp * error

        # I term (stubbed)
        self._integral += error
        i = self._ki * self._integral

        # D term
        d = self._kd * (error - self._prev_error)
        self._prev_error = error

        steering = p + i + d
        return max(-1.0, min(1.0, steering))
