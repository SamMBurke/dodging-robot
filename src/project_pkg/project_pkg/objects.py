from dataclasses import dataclass
import numpy as np

@dataclass
class DetectedObject:
    id: int
    center: np.ndarray
    heading: float # orientiation of the object in the world frame
    length: float
    width: float
    corners: np.ndarray
