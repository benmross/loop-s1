"""Draw waypoints and plans into the Gazebo GUI.

Markers go to the GUI's /marker service through the ``gz service`` command on
a worker thread, so a headless run costs one timed-out call every few seconds
and nothing else. They are visuals only: nothing collides with them, and they
are drawn flat on the ground below the lidar's scan plane.
"""

import shutil
import subprocess
import threading


def _material(rgb):
    r, g, b = rgb
    color = f'r: {r} g: {g} b: {b} a: 1'
    return f'material {{ ambient {{ {color} }} diffuse {{ {color} }} emissive {{ {color} }} }}'


class GzMarkers:

    def __init__(self, logger, timeout_ms=1000, retry_s=3.0):
        self._log = logger
        self._timeout = timeout_ms
        self._retry = retry_s
        self._state = {}
        self._dirty = set()
        self._resend = True
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._exe = shutil.which('gz')
        if self._exe:
            threading.Thread(target=self._run, daemon=True).start()

    def disc(self, ns, marker_id, x, y, radius, rgb, z=0.02):
        self._put(ns, marker_id,
                  f'type: CYLINDER pose {{ position {{ x: {x:.3f} y: {y:.3f} z: {z} }} }} '
                  f'scale {{ x: {2 * radius} y: {2 * radius} z: 0.02 }} {_material(rgb)}')

    def line(self, ns, marker_id, points, rgb, z=0.05):
        pts = ' '.join(f'point {{ x: {x:.2f} y: {y:.2f} z: {z} }}' for x, y in points)
        self._put(ns, marker_id, f'type: LINE_STRIP {_material(rgb)} {pts}')

    def _put(self, ns, marker_id, body):
        req = f'action: ADD_MODIFY ns: "{ns}" id: {marker_id} {body}'
        with self._lock:
            self._state[(ns, marker_id)] = req
            self._dirty.add((ns, marker_id))
        self._wake.set()

    def _run(self):
        while True:
            self._wake.wait(timeout=self._retry)
            with self._lock:
                self._wake.clear()
                keys = list(self._state) if self._resend else list(self._dirty)
                todo = [self._state[k] for k in keys]
                self._dirty.clear()
                self._resend = False
            for req in todo:
                if not self._call(req):
                    # No GUI yet, or it restarted: send everything again later.
                    with self._lock:
                        self._resend = True
                    break

    def _call(self, req):
        cmd = [self._exe, 'service', '-s', '/marker', '--reqtype', 'gz.msgs.Marker',
               '--reptype', 'gz.msgs.Empty', '--timeout', str(self._timeout), '--req', req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=self._timeout / 1000 + 5)
        except subprocess.TimeoutExpired:
            return False
        text = (out.stdout + out.stderr).lower()
        return out.returncode == 0 and 'timed out' not in text and 'error' not in text
