"""Pure JSON protocol parsing helpers for the Sharp IR serial stream."""

import json
import math

MIN_DISTANCE_CM = 10.0
MAX_DISTANCE_CM = 80.0


def parse_sensor_line(line: str):
    """Parse one Arduino JSON line.

    Returns a dict with distance_cm (float or None), valid (bool), status (str),
    and raw_json (the original stripped line). Returns None for non-JSON lines.
    The parser accepts both the current sharp_* keys and shorter aliases.
    """
    line = line.strip()
    if not line:
        return None

    try:
        payload = json.loads(line)
    except (json.JSONDecodeError, TypeError):
        return None

    if not isinstance(payload, dict):
        return None

    distance = payload.get('sharp_distance_cm', payload.get('distance_cm'))
    declared_valid = payload.get('sharp_valid', payload.get('valid', False))
    status = payload.get('sharp_status', payload.get('status', 'UNKNOWN'))

    if distance is None:
        distance_value = None
    else:
        try:
            distance_value = float(distance)
        except (TypeError, ValueError):
            distance_value = None

    valid = bool(declared_valid)
    if distance_value is None or not math.isfinite(distance_value):
        valid = False
    if valid and not (MIN_DISTANCE_CM <= distance_value <= MAX_DISTANCE_CM):
        valid = False
        status = 'OUT_OF_RANGE'

    if not valid and status == 'VALID':
        status = 'INVALID_READING'

    return {
        'distance_cm': distance_value if valid else None,
        'valid': valid,
        'status': str(status),
        'raw_json': line,
    }
