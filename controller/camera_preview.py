"""Small diagnostic JPEGs; raw RGB and depth remain separate camera frames."""
import cv2
import numpy as np


def preview(msg, depth=False):
    if not 0 < msg.width * msg.height <= 4_000_000:
        raise ValueError('Invalid camera image dimensions')
    if depth:
        if msg.encoding not in ('16UC1', 'mono16', '32FC1'):
            raise ValueError(f'Unsupported depth encoding: {msg.encoding}')
        dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('f4' if msg.encoding == '32FC1' else 'u2'))
        channels = 1
    else:
        if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'):
            raise ValueError(f'Unsupported RGB encoding: {msg.encoding}')
        dtype = np.dtype('u1')
        channels = 1 if msg.encoding == 'mono8' else 4 if 'a8' in msg.encoding else 3
    if msg.step < msg.width * dtype.itemsize * channels or len(msg.data) < msg.step * msg.height:
        raise ValueError('Truncated camera image or invalid row stride')
    shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels)
    strides = (msg.step, dtype.itemsize) if channels == 1 else (msg.step, channels, 1)
    image = np.ndarray(shape, dtype=dtype, buffer=msg.data, strides=strides)
    metadata = {'encoding': msg.encoding, 'width': msg.width, 'height': msg.height}
    if depth:
        meters = image.astype(np.float32) * (1. if msg.encoding == '32FC1' else .001)
        valid = np.isfinite(meters) & (meters > 0)
        scaled = np.clip((np.where(valid, meters, 4.) - .3) / 3.7, 0, 1)
        image = cv2.applyColorMap(((1 - scaled) * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        image[~valid] = 0
        metadata.update(valid_fraction=float(valid.mean()), minimum_m=float(meters[valid].min()) if valid.any() else None)
        central = meters[msg.height//4:3*msg.height//4, msg.width//4:3*msg.width//4]
        central_valid = np.isfinite(central) & (central > 0)
        metadata.update(central_valid_fraction=float(central_valid.mean()),
                        central_minimum_m=float(central[central_valid].min()) if central_valid.any() else None)
    else:
        conversions = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR}
        if msg.encoding in conversions:
            image = cv2.cvtColor(image, conversions[msg.encoding])
    width = min(msg.width, 480)
    image = cv2.resize(image, (width, max(1, round(msg.height * width / msg.width))), interpolation=cv2.INTER_NEAREST if depth else cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if not ok:
        raise ValueError('Could not encode camera preview')
    return encoded.tobytes(), metadata
