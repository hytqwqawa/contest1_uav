"""Pure-Python continuity checker; it vetoes, never modifies, setpoints."""
import math
MAX_POS_M = 0.25
MAX_YAW_RAD = math.radians(12.)
REQUIRED_VALID = 1 | 2 | 4 | 1024  # PX/PY/PZ/YAW ignored bits

def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))

def check(previous, candidate):
    if candidate is None: return 'candidate_missing'
    if int(candidate.type_mask) & REQUIRED_VALID: return 'xyz_or_yaw_ignored'
    vals = [candidate.position.x, candidate.position.y,
            candidate.position.z, candidate.yaw]
    if not all(math.isfinite(float(v)) for v in vals): return 'nonfinite'
    if previous is None: return None
    if int(previous.type_mask) & REQUIRED_VALID: return 'reference_xyz_or_yaw_ignored'
    if int(previous.coordinate_frame) != int(candidate.coordinate_frame):
        return 'frame_mismatch'
    d = math.sqrt(sum((a-b)**2 for a,b in zip(
        (candidate.position.x,candidate.position.y,candidate.position.z),
        (previous.position.x,previous.position.y,previous.position.z))))
    if d > MAX_POS_M: return 'position_jump_%.3fm' % d
    a = abs(angle_delta(candidate.yaw, previous.yaw))
    if a > MAX_YAW_RAD: return 'yaw_jump_%.1fdeg' % math.degrees(a)
    return None
