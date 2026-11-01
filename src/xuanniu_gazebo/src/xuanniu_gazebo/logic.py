"""Pure front-view angle and detent rules (radians throughout)."""
import math

HALF = math.pi/4
MODES = {'MANUAL': HALF, 'STOP': 0.0, 'AUTO': -HALF}


def clamp_angle(angle):
    return max(-HALF, min(HALF, angle))


def snap(angle):
    if not math.isfinite(angle):
        raise ValueError('Angle must be finite')
    return HALF if angle > HALF/2+1e-12 else (-HALF if angle < -HALF/2-1e-12 else 0.0)


def cursor_angle(x, y):
    return math.atan2(-x, -y)


def drag_angle(initial, initial_cursor, current_cursor):
    delta = (current_cursor-initial_cursor+math.pi) % (2*math.pi)-math.pi
    return clamp_angle(initial+delta)


def adjacent(mode, left):
    return clamp_angle(MODES[mode]+(HALF if left else -HALF))
