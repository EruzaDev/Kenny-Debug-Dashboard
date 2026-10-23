"""Wait for a correctly enumerated Astra instead of flooding failed-open retries."""
from pathlib import Path
import time


def devices():
    result = []
    for path in Path('/sys/bus/usb/devices').glob('*'):
        try:
            if (path / 'idVendor').read_text().strip() == '2bc5' and (path / 'idProduct').read_text().strip() == '0403':
                result.append({'port': path.name, 'interfaces': int((path / 'bNumInterfaces').read_text().strip(), 16)})
        except (OSError, ValueError):
            continue
    return result


if __name__ == '__main__':
    previous = None
    while True:
        found = devices()
        if any(d['interfaces'] >= 3 for d in found):
            print(f'Astra USB ready: {found}', flush=True)
            break
        state = str(found)
        if state != previous:
            print(f'Waiting for Astra USB: {found or "not connected"}. If interfaces=0, unplug camera for 10 seconds and reconnect; keep Pi powered.', flush=True)
            previous = state
        time.sleep(1.)
