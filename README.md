# Kenny Debug Dashboard

ROS 2 operations and motor-debug dashboard for Kenny. It serves the web UI on
`127.0.0.1:8002` and listens to Kenny's ROS topics for live diagnostics. The
dashboard starts with the emergency stop latched and is intended to run on the
robot computer, not on an internet-facing host.

## Requirements

- Raspberry Pi or Linux robot computer
- ROS 2 Jazzy with the required Kenny sensor and message nodes running
- Python 3.10+
- A Kenny `environment.json` map export
- ROS packages used by the dashboard, including `rclpy`, standard messages,
  `tf2_ros`, and sensor/navigation message packages

Install the Python dependencies in the same virtual environment used with ROS:

```sh
cd Kenny-Debug-Dashboard
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r controller/requirements.txt
```

The virtual environment must be able to import the ROS Python packages. If
your ROS installation is not Jazzy, source the matching ROS setup file instead.

## Run On The Robot

From the repository root, source ROS and start the dashboard:

```sh
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m controller.dashboard \
  --environment /path/to/environment.json
```

The default map path is `~/Downloads/kenny-environment/environment.json`. Use
`--port` to change the local HTTP port and `--log` to change the operations log:

```sh
.venv/bin/python -m controller.dashboard \
  --port 8002 \
  --environment /home/kenny/Downloads/kenny-environment/environment.json \
  --log /home/kenny/Kenny-Debug-Dashboard/data/operations.jsonl
```

Open `http://127.0.0.1:8002` on the robot computer. To open it from another
computer, keep this SSH tunnel running:

```sh
ssh -N -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  -L 8002:127.0.0.1:8002 \
  kenny@kenny-desktop.local
```

Then open `http://localhost:8002`. The dashboard must remain bound to loopback;
do not expose port `8002` directly to the network.

## Decision Debugger

Open **Decision debugger** to inspect the latest controller decision and its
bounded timeline. It shows the controller belief pose, an independent pose
reference, goal and route revision, policy feature groups, requested versus
guarded motion, safety reason, and measured context. The **Operator feedback**
form writes timestamped observations to `data/debug-feedback.jsonl`; feedback is
never applied directly to motion or safety settings.

Use **Export AI review packet** to produce a JSON bundle for offline review. An
AI or engineer should reproduce the decision, add a regression test, and make a
reviewed code/configuration change rather than allowing free text to alter the
robot. Map geometry is edited in **Map creator**, then synchronized through the
existing map deployment flow.

## Browser Launcher

After the dashboard service is running in a graphical desktop session, the
browser helper waits for `/api/status` and opens the page:

```sh
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m controller.open_dashboard
```

Use a readiness check without opening a browser:

```sh
.venv/bin/python -m controller.open_dashboard --check
```

## Safety

This dashboard contains emergency-stop, motor, calibration, and bounded-pulse
controls. Keep the stop latched until the robot, wheels, motor power cutoff,
and surrounding area have been physically checked. Raised-wheel tests require
the wheels to be secured and inaccessible to people. A lost dashboard heartbeat
is designed to latch STOP, but it is not a replacement for a physical stop.

## Tests

Unit tests that do not require a live ROS graph can be run with:

```sh
.venv/bin/python -m unittest discover -s controller/tests -v
```

The browser smoke test requires a running dashboard, Playwright, and Chromium:

```sh
.venv/bin/python controller/tests/browser_dashboard.py
```

The dashboard is a local robot diagnostic tool. It does not replace the Kenny
API, map editor, motor firmware, or ROS sensor/controller services.
