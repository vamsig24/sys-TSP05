"""Serial protocol between the TSP05 Arduino firmware and ROS 2.

Pure Python (no ROS imports) so it can be unit tested without ROS.

Arduino -> host: one JSON object per line, with "t" giving the type:
    boot   {"t":"boot","tof_ok":true,"dc_on":false,"mode":"GUI"}
    state  {"t":"state","ms":..,"mode":"GUI","safety":false, ...sensor/motor fields}
    ack    {"t":"ack","cmd":"SERVO","ok":true}  /  {..,"ok":false,"err":"..."}
    event  {"t":"event","event":"mode_changed","mode":"SENSOR"}

Host -> Arduino: one text command per line (see format_* helpers).
"""

import json
import math

MODES = ('GUI', 'SENSOR')

SHARP_MIN_CM = 10.0
SHARP_MAX_CM = 80.0

# Fields every state message must have
STATE_FIELDS = (
    'ms', 'mode', 'safety', 'safety_cm',
    'flex_raw', 'flex_adc', 'flex_kohm', 'flex_deg',
    'flex_force', 'flex_flat_k', 'flex_bent_k',
    'pot_raw', 'pot_v', 'pot_pct',
    'tof_ok', 'tof_mm',
    'sharp_v', 'sharp_cm', 'sharp_st',
    'btn', 'btn_n',
    'servo_deg', 'step_deg', 'step_tgt',
    'dc_on', 'dc_tgt', 'dc_rpm', 'dc_pwm', 'dc_pol',
)


def parse_line(line):
    """Parse one serial line into a dict.

    Returns None for blank lines, non-JSON text, JSON that is not an object
    with a "t" field, and state messages with missing fields (for example a
    line cut short when the port was opened mid-message).
    """
    if isinstance(line, bytes):
        line = line.decode('utf-8', errors='replace')
    line = line.strip()
    if not line:
        return None

    try:
        msg = json.loads(line)
    except (json.JSONDecodeError, TypeError, ValueError):
        return None

    if not isinstance(msg, dict) or 't' not in msg:
        return None

    if msg['t'] == 'state':
        if any(k not in msg for k in STATE_FIELDS):
            return None
        msg.update(derive_state(msg))

    return msg


def derive_state(msg):
    """Values computed from a raw state message (units, validity, NaN)."""
    tof_ok = bool(msg['tof_ok'])
    tof_cm = msg['tof_mm'] / 10.0 if tof_ok else math.nan

    sharp_valid = (msg['sharp_st'] == 'VALID' and
                   SHARP_MIN_CM <= float(msg['sharp_cm']) <= SHARP_MAX_CM)
    sharp_cm = float(msg['sharp_cm']) if sharp_valid else math.nan

    return {
        'tof_cm': tof_cm,
        'sharp_valid': sharp_valid,
        'sharp_dist_cm': sharp_cm,
    }


# ----------------------------------------------------------------------
# Commands (host -> Arduino)
# ----------------------------------------------------------------------

def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


def format_mode(mode):
    mode = str(mode).strip().upper()
    if mode not in MODES + ('TOGGLE',):
        raise ValueError(f'mode must be GUI, SENSOR or TOGGLE, got {mode!r}')
    return f'MODE {mode}\n'


def format_servo(angle_deg):
    angle = _finite(angle_deg, 'servo angle')
    angle = min(max(angle, 0.0), 180.0)
    return f'SERVO {angle:.0f}\n'


def format_dc(rpm):
    rpm = _finite(rpm, 'DC rpm')
    rpm = min(max(rpm, -100.0), 100.0)
    return f'DC {rpm:.1f}\n'


def format_dc_pwm(pwm):
    """Open-loop DC test (GUI mode); the Arduino stops it after 2 s."""
    pwm = _finite(pwm, 'DC pwm')
    pwm = min(max(pwm, -230.0), 230.0)
    return f'DCPWM {pwm:.0f}\n'


def format_dc_polarity(value):
    """value: "<motor> <encoder>" with 0/1 each, e.g. "1 0"."""
    parts = str(value).split()
    if len(parts) != 2 or any(p not in ('0', '1') for p in parts):
        raise ValueError(f'DC polarity must be "<0|1> <0|1>", got {value!r}')
    return f'DCPOL {parts[0]} {parts[1]}\n'


def format_step(delta_deg):
    delta = _finite(delta_deg, 'stepper delta')
    return f'STEP {delta:.1f}\n'


def format_calibration(point):
    point = str(point).strip().upper()
    if point not in ('FLAT', 'BENT', 'RESET'):
        raise ValueError(f'calibration must be FLAT, BENT or RESET, got {point!r}')
    return f'CAL {point}\n'


def format_safety(threshold_cm):
    threshold = _finite(threshold_cm, 'safety threshold')
    return f'SAFETY {max(threshold, 0.0):.0f}\n'
