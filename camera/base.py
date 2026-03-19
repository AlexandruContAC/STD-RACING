"""Abstract base class for camera backends."""

from abc import ABC, abstractmethod
import numpy as np


class CameraBase(ABC):
    """Every camera backend must implement these three methods."""

    @abstractmethod
    def open(self) -> None:
        """Initialise the camera / USB device."""
        ...

    @abstractmethod
    def read_frame(self) -> np.ndarray:
        """Return the next frame as a BGR numpy array (H, W, 3)."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Release resources."""
        ...
