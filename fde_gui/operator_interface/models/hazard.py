from dataclasses import dataclass
from typing import Tuple


@dataclass
class Hazard:
    hazard_id: str
    hazard_type: str
    status: str
    coords: Tuple[float, float, float]

    detected_by: str
    time_detected: str

    husky_assignment: str
    assigned_time: str

    resolved_by: str
    resolved_time: str

    map_pos: Tuple[float, float]