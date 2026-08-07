#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Stand between the camera and the arm, and let a person decide.

The stock monitor only reports what it sees, and the task manager only moves
when something asks it to.  This node is what joins the two: it raises an alarm
when a bin has read empty long enough to be believed, serves that alarm on a
web page, and forwards a refill request to the task manager only once somebody
has pressed Approve.

Run the task manager with ``auto_refill:=false`` alongside this node, otherwise
it acts on the monitor directly and the approval gate is decorative.

An approval is not published the instant it is given.  The task manager drops
requests that arrive while it is busy, so approvals are held in a queue here and
released one at a time, each when the arm is actually idle.  Nothing is lost
because the operator looked away at the wrong moment.

The page also carries the monitor's debug image, so the decision is made while
looking at the bins rather than at a bin name.  The stream is dropped silently
if OpenCV is unavailable; every other part of the gate still works.

Open http://<robot>:8080/ once running.  There is no login: this is a gate
against the robot moving unattended, not against a hostile network, so keep it
on the cell's own network.
"""

import functools
import http.server
import json
import os
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import yaml

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - the gate runs without the stream
    cv2 = None
    CvBridge = None

ALARM_PENDING = 'pending'
ALARM_APPROVED = 'approved'
ALARM_MUTED = 'muted'

ARM_IDLE = 'idle'


class FrameBuffer:
    """Hands the newest JPEG to every streaming client without a busy loop."""

    def __init__(self):
        self._condition = threading.Condition()
        self._jpeg = None
        self._seq = 0

    def put(self, jpeg):
        with self._condition:
            self._jpeg = jpeg
            self._seq += 1
            self._condition.notify_all()

    def get(self, last_seq, timeout):
        """Block until a frame newer than ``last_seq`` arrives, or time out."""
        with self._condition:
            if self._seq == last_seq:
                self._condition.wait(timeout)
            if self._seq == last_seq:
                return last_seq, None
            return self._seq, self._jpeg


class Alarm:
    """One bin's journey from 'looks empty' to 'the arm was told to refill it'."""

    def __init__(self, bin_id, raised_at):
        self.bin_id = bin_id
        self.raised_at = raised_at
        self.state = ALARM_PENDING
        self.decided_at = None
        self.muted_until = None

    def to_dict(self, now):
        return {
            'id': self.bin_id,
            'state': self.state,
            'age_sec': round(now - self.raised_at, 1),
            'waiting_sec': (
                None if self.decided_at is None else round(now - self.decided_at, 1)
            ),
            'mute_left_sec': (
                None if self.muted_until is None else round(max(0.0, self.muted_until - now), 1)
            ),
        }


class StockApproval(Node):
    """Web approval gate between the stock monitor and the task manager."""

    def __init__(self):
        super().__init__('stock_approval')

        self.declare_parameter('layout_file', '')
        self.declare_parameter('status_topic', '/stock/status')
        self.declare_parameter('task_state_topic', '/stock/task_state')
        self.declare_parameter('request_topic', '/stock/refill_request')
        self.declare_parameter('debug_image_topic', '/stock/debug_image')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('serve_stream', True)
        self.declare_parameter('stream_fps', 5.0)
        self.declare_parameter('stream_quality', 70)
        # How long a rejected bin stays quiet before it may raise an alarm
        # again.  A bin that reads filled clears its own mute immediately.
        self.declare_parameter('mute_sec', 300.0)
        # A bin has to still look empty this long after being seen before the
        # alarm is raised, so a part picked up by hand does not summon the arm.
        self.declare_parameter('confirm_sec', 2.0)

        self.mute_sec = float(self.get_parameter('mute_sec').value)
        self.confirm_sec = float(self.get_parameter('confirm_sec').value)

        # Guards everything the HTTP threads share with the ROS executor.
        self.lock = threading.Lock()
        self.alarms = {}
        self.approved_queue = []
        self.history = []
        self.arm = {'state': 'unknown', 'job': None, 'parts_left': None}
        self.bins = {}
        self.empty_since = {}
        self.known_bins = self._load_bin_ids()
        self.last_status_at = None
        self.running = True

        self.request_pub = self.create_publisher(
            String, self.get_parameter('request_topic').value, 10
        )
        self.create_subscription(
            String, self.get_parameter('status_topic').value, self._on_status, 10
        )
        self.create_subscription(
            String, self.get_parameter('task_state_topic').value, self._on_task_state, 10
        )

        self.frames = None
        if bool(self.get_parameter('serve_stream').value):
            self._start_stream()

        self.create_timer(0.2, self._dispatch)

        self.httpd = None
        self._start_server()

    # ------------------------------------------------------------------ setup

    def _load_bin_ids(self):
        """Read the bin names from the layout, so all of them can be listed.

        Optional: without a layout the page still works, it just cannot offer a
        bin the camera has not reported on yet.
        """
        path = self.get_parameter('layout_file').value
        if not path:
            return []
        if not os.path.exists(path):
            self.get_logger().warn(f'layout_file not found: {path!r}')
            return []
        with open(path) as handle:
            layout = yaml.safe_load(handle)
        return [entry['id'] for entry in layout.get('bins', [])]

    def _start_stream(self):
        """Mirror the monitor's debug image into an MJPEG stream."""
        if cv2 is None:
            self.get_logger().warn(
                'OpenCV or cv_bridge is missing - serving the page without video'
            )
            return

        self.bridge = CvBridge()
        self.frames = FrameBuffer()
        self.stream_quality = int(self.get_parameter('stream_quality').value)
        fps = float(self.get_parameter('stream_fps').value)
        self.stream_period = 1.0 / fps if fps > 0.0 else 0.0
        self.last_frame_at = 0.0

        self.create_subscription(
            Image,
            self.get_parameter('debug_image_topic').value,
            self._on_image,
            qos_profile_sensor_data,
        )

    def _start_server(self):
        """Serve the page from a thread of its own, leaving ROS on the main one."""
        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        handler = functools.partial(ApprovalHandler, self)
        try:
            self.httpd = http.server.ThreadingHTTPServer((host, port), handler)
        except OSError as error:
            raise RuntimeError(
                f'Cannot serve the approval page on {host}:{port} ({error}). '
                f'Another node may already hold the port; pass port:=<other>.'
            ) from error
        self.httpd.daemon_threads = True

        self.server_thread = threading.Thread(
            target=self.httpd.serve_forever, name='approval_http', daemon=True
        )
        self.server_thread.start()
        shown = 'localhost' if host in ('0.0.0.0', '') else host
        self.get_logger().info(f'Approval page on http://{shown}:{port}/')

    # -------------------------------------------------------------- ros input

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_status(self, msg):
        """Raise or clear alarms from the monitor's view of the bins."""
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Ignoring malformed stock status')
            return

        now = self._now()
        with self.lock:
            self.last_status_at = now
            for entry in status.get('bins', []):
                bin_id = entry.get('id')
                if bin_id is None:
                    continue
                self.bins[bin_id] = entry
                if entry.get('state') == 'empty' and entry.get('stable'):
                    self._saw_empty(bin_id, now)
                else:
                    self.empty_since.pop(bin_id, None)
                    if entry.get('state') == 'filled' and entry.get('stable'):
                        self._saw_filled(bin_id)

    def _saw_empty(self, bin_id, now):
        """Note an empty reading, and raise the alarm once it has held."""
        first_seen = self.empty_since.setdefault(bin_id, now)
        if now - first_seen < self.confirm_sec:
            return

        alarm = self.alarms.get(bin_id)
        if alarm is not None:
            if alarm.state != ALARM_MUTED:
                return
            if alarm.muted_until is not None and now < alarm.muted_until:
                return

        self.alarms[bin_id] = Alarm(bin_id, now)
        self.get_logger().info(f'{bin_id} reads empty - waiting for approval')

    def _saw_filled(self, bin_id):
        """A bin that is full again has nothing left to approve."""
        alarm = self.alarms.get(bin_id)
        if alarm is None or alarm.state == ALARM_APPROVED:
            return
        del self.alarms[bin_id]

    def _on_task_state(self, msg):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.arm = {
                'state': state.get('state', 'unknown'),
                'job': state.get('job'),
                'parts_left': state.get('parts_left'),
            }

    def _on_image(self, msg):
        """Keep the newest debug frame encoded and ready to hand out."""
        now = self._now()
        if self.stream_period and now - self.last_frame_at < self.stream_period:
            return
        self.last_frame_at = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:  # noqa: BLE001 - a bad frame must not kill the gate
            self.get_logger().warn(f'Cannot decode debug image: {error}')
            return

        ok, buffer = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.stream_quality]
        )
        if ok:
            self.frames.put(buffer.tobytes())

    # ------------------------------------------------------------- decisions

    def decide(self, bin_id, decision):
        """Apply an operator's decision.  Returns (accepted, message)."""
        now = self._now()
        with self.lock:
            if decision == 'request':
                return self._approve_manual(bin_id, now)

            alarm = self.alarms.get(bin_id)
            if alarm is None:
                return False, f'No alarm for {bin_id}'

            if decision == 'approve':
                if alarm.state == ALARM_APPROVED:
                    return False, f'{bin_id} is already approved'
                alarm.state = ALARM_APPROVED
                alarm.decided_at = now
                alarm.muted_until = None
                self.approved_queue.append(bin_id)
                self.get_logger().info(f'{bin_id} approved by the operator')
                return True, f'{bin_id} approved'

            if decision == 'reject':
                alarm.state = ALARM_MUTED
                alarm.decided_at = now
                alarm.muted_until = now + self.mute_sec
                if bin_id in self.approved_queue:
                    self.approved_queue.remove(bin_id)
                self.empty_since.pop(bin_id, None)
                self._record(bin_id, 'rejected', now)
                self.get_logger().info(
                    f'{bin_id} rejected - quiet for {self.mute_sec:.0f}s'
                )
                return True, f'{bin_id} rejected'

            if decision == 'cancel':
                if alarm.state != ALARM_APPROVED:
                    return False, f'{bin_id} is not waiting to be sent'
                if bin_id not in self.approved_queue:
                    return False, f'{bin_id} has already gone to the arm'
                self.approved_queue.remove(bin_id)
                alarm.state = ALARM_PENDING
                alarm.decided_at = None
                self._record(bin_id, 'cancelled', now)
                self.get_logger().info(f'{bin_id} approval cancelled before dispatch')
                return True, f'{bin_id} approval cancelled'

        return False, f'Unknown decision {decision!r}'

    def _approve_manual(self, bin_id, now):
        """Queue a refill the operator asked for without an alarm."""
        if self.known_bins and bin_id not in self.known_bins:
            return False, f'Unknown bin {bin_id}'
        if bin_id in self.approved_queue:
            return False, f'{bin_id} is already queued'

        alarm = self.alarms.get(bin_id)
        if alarm is None:
            alarm = Alarm(bin_id, now)
            self.alarms[bin_id] = alarm
        alarm.state = ALARM_APPROVED
        alarm.decided_at = now
        alarm.muted_until = None
        self.approved_queue.append(bin_id)
        self.get_logger().info(f'{bin_id} refill requested by hand')
        return True, f'{bin_id} queued'

    def _dispatch(self):
        """Release one approved refill, once the arm is free to take it.

        The task manager ignores a request that lands while it is busy, so an
        approval waits here rather than being published into the void.
        """
        with self.lock:
            if not self.approved_queue:
                return
            if self.arm.get('state') != ARM_IDLE:
                return
            bin_id = self.approved_queue.pop(0)
            now = self._now()
            self._record(bin_id, 'sent', now)
            self.alarms.pop(bin_id, None)
            self.empty_since.pop(bin_id, None)

        self.request_pub.publish(String(data=bin_id))
        self.get_logger().info(f'Refill request sent for {bin_id}')

    def _record(self, bin_id, outcome, now):
        """Keep a short trail of what was decided, newest first."""
        self.history.insert(0, {'id': bin_id, 'outcome': outcome, 'at': now})
        del self.history[20:]

    # ---------------------------------------------------------------- reading

    def snapshot(self):
        """Everything the page needs, taken in one consistent pass."""
        now = self._now()
        with self.lock:
            stale = (
                None if self.last_status_at is None
                else round(now - self.last_status_at, 1)
            )
            return {
                'arm': dict(self.arm),
                'bins': [
                    dict(entry, alarm=(
                        None if bin_id not in self.alarms
                        else self.alarms[bin_id].state
                    ))
                    for bin_id, entry in sorted(self.bins.items())
                ],
                'alarms': [
                    alarm.to_dict(now)
                    for _, alarm in sorted(self.alarms.items())
                ],
                'queue': list(self.approved_queue),
                'history': [
                    dict(item, age_sec=round(now - item['at'], 1))
                    for item in self.history
                ],
                'known_bins': list(self.known_bins),
                'monitor_age_sec': stale,
                'stream': self.frames is not None,
            }

    # --------------------------------------------------------------- teardown

    def shutdown(self):
        self.running = False
        if self.httpd is not None:
            # serve_forever runs on its own thread, so this returns promptly.
            self.httpd.shutdown()
            self.httpd.server_close()


class ApprovalHandler(http.server.BaseHTTPRequestHandler):
    """Serves the page, its state, the decisions and the camera stream."""

    protocol_version = 'HTTP/1.1'

    def __init__(self, node, *args, **kwargs):
        self.node = node
        super().__init__(*args, **kwargs)

    def log_message(self, fmt, *args):
        """Keep request logging out of the console the operator is watching."""
        self.node.get_logger().debug(fmt % args)

    # ------------------------------------------------------------------- get

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            self._send(200, 'text/html; charset=utf-8', PAGE.encode('utf-8'))
        elif path == '/api/state':
            body = json.dumps(self.node.snapshot()).encode('utf-8')
            self._send(200, 'application/json', body)
        elif path == '/stream.mjpg':
            self._stream()
        else:
            self._send(404, 'text/plain', b'not found\n')

    def do_POST(self):
        if self.path.split('?', 1)[0] != '/api/decide':
            self._send(404, 'text/plain', b'not found\n')
            return

        length = int(self.headers.get('Content-Length') or 0)
        try:
            payload = json.loads(self.rfile.read(length) or b'{}')
            bin_id = str(payload['id'])
            decision = str(payload['decision'])
        except (json.JSONDecodeError, KeyError, ValueError):
            self._send(400, 'application/json', b'{"ok": false, "message": "bad request"}')
            return

        ok, message = self.node.decide(bin_id, decision)
        body = json.dumps({'ok': ok, 'message': message}).encode('utf-8')
        self._send(200, 'application/json', body)

    # ---------------------------------------------------------------- helpers

    def _send(self, code, content_type, body):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _stream(self):
        """Write frames until the browser goes away or the node stops."""
        frames = self.node.frames
        if frames is None:
            self._send(503, 'text/plain', b'no video\n')
            return

        boundary = 'omxframe'
        self.send_response(200)
        self.send_header(
            'Content-Type', f'multipart/x-mixed-replace; boundary={boundary}'
        )
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()

        seq = 0
        try:
            while self.node.running:
                seq, jpeg = frames.get(seq, 1.0)
                if jpeg is None:
                    continue
                self.wfile.write(
                    f'--{boundary}\r\nContent-Type: image/jpeg\r\n'
                    f'Content-Length: {len(jpeg)}\r\n\r\n'.encode('ascii')
                )
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Stock refill approval</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; background: #14161a; color: #e7e9ee;
         font: 15px/1.5 system-ui, sans-serif; }
  header { padding: 14px 20px; background: #1b1e24; border-bottom: 1px solid #2a2f38;
           display: flex; gap: 20px; align-items: baseline; flex-wrap: wrap; }
  h1 { font-size: 17px; margin: 0; font-weight: 600; }
  main { padding: 20px; display: grid; gap: 20px;
         grid-template-columns: minmax(320px, 1fr) minmax(320px, 1fr); }
  @media (max-width: 800px) { main { grid-template-columns: 1fr; } }
  section { background: #1b1e24; border: 1px solid #2a2f38; border-radius: 10px;
            padding: 16px; }
  h2 { font-size: 13px; text-transform: uppercase; letter-spacing: .08em;
       color: #8b93a3; margin: 0 0 12px; font-weight: 600; }
  .alarm { border: 1px solid #4a3320; background: #241a12; border-radius: 8px;
           padding: 14px; margin-bottom: 10px; }
  .alarm.approved { border-color: #204a2c; background: #12241a; }
  .alarm.muted { border-color: #2a2f38; background: #191c22; color: #8b93a3; }
  .alarm b { font-size: 16px; }
  .row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  .spacer { flex: 1; }
  button { font: inherit; padding: 7px 16px; border-radius: 6px; cursor: pointer;
           border: 1px solid #3a4150; background: #262b34; color: #e7e9ee; }
  button:hover { background: #303643; }
  button.go { background: #1f7a45; border-color: #1f7a45; }
  button.go:hover { background: #26904f; }
  button.no { background: #7a2f2f; border-color: #7a2f2f; }
  button.no:hover { background: #8f3838; }
  table { width: 100%; border-collapse: collapse; }
  td { padding: 5px 0; border-bottom: 1px solid #23272f; }
  td.right { text-align: right; color: #8b93a3; }
  .tag { padding: 2px 8px; border-radius: 999px; font-size: 12px; }
  .empty { background: #4a2020; color: #ff9b9b; }
  .filled { background: #17351f; color: #86e0a4; }
  .unknown { background: #2a2f38; color: #8b93a3; }
  img { width: 100%; border-radius: 8px; display: block; background: #000; }
  .quiet { color: #8b93a3; }
  #toast { position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%);
           background: #2a2f38; padding: 10px 18px; border-radius: 8px;
           opacity: 0; transition: opacity .2s; pointer-events: none; }
  #toast.show { opacity: 1; }
</style>
</head>
<body>
<header>
  <h1>Stock refill approval</h1>
  <span class="quiet">arm <b id="arm">-</b></span>
  <span class="quiet">parts left <b id="parts">-</b></span>
  <span class="spacer"></span>
  <span class="quiet" id="link">-</span>
</header>
<main>
  <div>
    <section>
      <h2>Waiting for you</h2>
      <div id="alarms"></div>
    </section>
    <section style="margin-top:20px">
      <h2>Bins</h2>
      <table id="bins"></table>
    </section>
    <section style="margin-top:20px">
      <h2>Recent</h2>
      <table id="history"></table>
    </section>
  </div>
  <section>
    <h2>Camera</h2>
    <div id="camera" class="quiet">no video</div>
  </section>
</main>
<div id="toast"></div>
<script>
let streaming = false;

function tag(state) {
  const cls = state === 'empty' ? 'empty' : state === 'filled' ? 'filled' : 'unknown';
  return `<span class="tag ${cls}">${state}</span>`;
}

function toast(message) {
  const el = document.getElementById('toast');
  el.textContent = message;
  el.classList.add('show');
  clearTimeout(el.timer);
  el.timer = setTimeout(() => el.classList.remove('show'), 2000);
}

async function decide(id, decision) {
  const response = await fetch('/api/decide', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({id, decision}),
  });
  const result = await response.json();
  toast(result.message);
  refresh();
}

function renderAlarms(state) {
  const box = document.getElementById('alarms');
  if (!state.alarms.length) {
    box.innerHTML = '<div class="quiet">Nothing needs approving.</div>';
    return;
  }
  box.innerHTML = state.alarms.map(a => {
    let detail, buttons;
    if (a.state === 'pending') {
      detail = `empty for ${a.age_sec}s`;
      buttons = `<button class="go" onclick="decide('${a.id}','approve')">Approve</button>
                 <button class="no" onclick="decide('${a.id}','reject')">Reject</button>`;
    } else if (a.state === 'approved') {
      const place = state.queue.indexOf(a.id);
      detail = state.arm.state === 'idle'
        ? 'sending to the arm...'
        : `approved ${a.waiting_sec}s ago, waiting for the arm (${state.arm.state})`
          + (place > 0 ? `, ${place} ahead` : '');
      buttons = `<button onclick="decide('${a.id}','cancel')">Cancel</button>`;
    } else {
      detail = `rejected, quiet for another ${a.mute_left_sec}s`;
      buttons = `<button class="go" onclick="decide('${a.id}','approve')">Approve anyway</button>`;
    }
    return `<div class="alarm ${a.state}">
              <div class="row"><b>${a.id}</b><span class="quiet">${detail}</span></div>
              <div class="row" style="margin-top:10px">${buttons}</div>
            </div>`;
  }).join('');
}

function renderBins(state) {
  const known = new Set(state.bins.map(b => b.id));
  const rows = state.bins.map(b => `
    <tr><td><b>${b.id}</b> ${tag(b.state)}
        ${b.stable ? '' : '<span class="quiet">settling</span>'}</td>
        <td class="right">
          <button onclick="decide('${b.id}','request')">Refill</button></td></tr>`);
  for (const id of state.known_bins) {
    if (!known.has(id)) {
      rows.push(`<tr><td><b>${id}</b> ${tag('unknown')}</td>
        <td class="right">
          <button onclick="decide('${id}','request')">Refill</button></td></tr>`);
    }
  }
  document.getElementById('bins').innerHTML =
    rows.join('') || '<tr><td class="quiet">The monitor has not reported yet.</td></tr>';
}

function renderHistory(state) {
  document.getElementById('history').innerHTML = state.history.map(h =>
    `<tr><td>${h.id} <span class="quiet">${h.outcome}</span></td>
         <td class="right">${h.age_sec}s ago</td></tr>`
  ).join('') || '<tr><td class="quiet">Nothing yet.</td></tr>';
}

async function refresh() {
  let state;
  try {
    state = await (await fetch('/api/state')).json();
  } catch (error) {
    document.getElementById('link').textContent = 'disconnected';
    return;
  }
  document.getElementById('arm').textContent = state.arm.state
    + (state.arm.job ? ` (${state.arm.job})` : '');
  document.getElementById('parts').textContent =
    state.arm.parts_left === null ? '-' : state.arm.parts_left;
  document.getElementById('link').textContent = state.monitor_age_sec === null
    ? 'no monitor yet'
    : state.monitor_age_sec > 5
      ? `monitor silent for ${state.monitor_age_sec}s`
      : 'monitor live';

  renderAlarms(state);
  renderBins(state);
  renderHistory(state);

  if (state.stream && !streaming) {
    streaming = true;
    document.getElementById('camera').innerHTML =
      '<img src="/stream.mjpg" alt="bins">';
  }
}

refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = StockApproval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
