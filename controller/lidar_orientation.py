"""Operator-selected LiDAR yaw and driver handedness for a known physical scene."""
import json
import math
import os
from pathlib import Path
import re

MOUNTS = Path(__file__).with_name('sensor_mounts.json')
PARAMS = Path(__file__).resolve().parents[2] / 'Kenny---Automotive-self-driving-trash-can/configs/pi5-lidar.yaml'
INVERTED = re.compile(r'(?m)^(\s*inverted:\s*)(true|false)(\s*)$')
YAWS = (0, 90, 180, 270)


def settings():
    config = json.loads(MOUNTS.read_text())
    laser = next(m for m in config['mounts'] if m['frame'] == 'laser_frame')
    text = PARAMS.read_text()
    match = INVERTED.search(text)
    if match is None:
        raise ValueError('LiDAR driver inversion setting missing')
    return {'yaw_deg': laser['rpy_deg'][2], 'inverted': match.group(2) == 'true'}


def set_settings(yaw_deg, inverted):
    if type(yaw_deg) is not int or yaw_deg not in YAWS or type(inverted) is not bool:
        raise ValueError('Choose a 0/90/180/270-degree yaw and a true/false inversion')
    config = json.loads(MOUNTS.read_text())
    laser = next(m for m in config['mounts'] if m['frame'] == 'laser_frame')
    text = PARAMS.read_text()
    if len(INVERTED.findall(text)) != 1:
        raise ValueError('Expected exactly one LiDAR inversion setting')
    laser['rpy_deg'][2] = yaw_deg
    laser['orientation_note'] = 'Operator-selected mounting yaw; verify front/left/right with a known obstacle.'
    from pi.receiver import save
    save(MOUNTS, config)
    changed = INVERTED.sub(lambda m: m.group(1) + str(inverted).lower() + m.group(3), text)
    temporary = PARAMS.with_suffix('.yaml.tmp')
    try:
        temporary.write_text(changed)
        os.replace(temporary, PARAMS)
    finally:
        temporary.unlink(missing_ok=True)


def sectors(scan, yaw_deg):
    """Minimum measured ranges in base_link quadrants, after driver inversion."""
    result = {name: [] for name in ('front', 'left', 'right', 'rear')}
    if scan is None:
        return {name: None for name in result}
    centers = {'front': 0., 'left': math.pi/2, 'right': -math.pi/2, 'rear': math.pi}
    for index, distance in enumerate(scan['ranges']):
        if distance is None:
            continue
        angle = scan['angle_min'] + index*scan['angle_increment'] + math.radians(yaw_deg)
        for name, center in centers.items():
            error = (angle-center+math.pi) % (2*math.pi)-math.pi
            if abs(error) < math.pi/6:
                result[name].append(distance)
    return {name: {'minimum_m': min(values), 'samples': len(values)} if values else None
            for name, values in result.items()}
