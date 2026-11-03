SLOT_TABLE = {
    "limo2": {"x":  0.0,       "y": -0.103, "yaw": -1.5708},
    "limo3": {"x": -0.300909,  "y": -0.103, "yaw": -1.5708},
    "limo4": {"x": +0.350909,  "y": -0.103, "yaw": -1.5708},
}

SLOT_LIDAR_CLEAR_M = 0.65
SLOT_SCAN_SECTOR_DEG = 8.0


def detect_empty_slot(scan_msg, current_pose):
    import math
    x, y, yaw = current_pose
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    all_points = []
    for i, d in enumerate(scan_msg.ranges):
        if math.isnan(d) or math.isinf(d):
            continue
        angle = scan_msg.angle_min + i * scan_msg.angle_increment
        lx = d * math.cos(angle)
        ly = d * math.sin(angle)
        ox = x + cos_y * lx - sin_y * ly
        oy = y + sin_y * lx + cos_y * ly
        all_points.append((ox, oy))

    for name, slot in SLOT_TABLE.items():
        sy = slot["y"]
        near = [p for p in all_points
                if abs(p[1] - sy) < 0.30
                and abs(p[0] - slot["x"]) < 0.40]
        if len(near) == 0:
            return name
        distances = [math.hypot(p[0] - slot["x"],
                                p[1] - sy) for p in near]
        distances.sort()
        median = distances[len(distances) // 2]
        if median >= SLOT_LIDAR_CLEAR_M:
            return name

    return None
