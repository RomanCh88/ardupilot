#!/usr/bin/env python3
"""
Auto-fly ArduPlane SITL from Dnipro to Kyiv.
Serves live Cesium.js telemetry map at http://localhost:8765
"""

import subprocess
import time
import os
import math
import threading
import json
import webbrowser
from http.server import HTTPServer, BaseHTTPRequestHandler
from pymavlink import mavutil

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
BINARY    = os.path.join(REPO_ROOT, 'build/sitl/bin/arduplane')
DEFAULTS  = os.path.join(REPO_ROOT, 'Tools/autotest/models/plane.parm')
MISSION_FILE = os.path.join(REPO_ROOT,
    'Tools/autotest/ArduPlane_Tests/Dnipro_to_Kyiv/mission.txt')

SITL_PORT = 5760
SPEEDUP   = 50
WEB_PORT  = 8765

# ── shared telemetry state ───────────────────────────────────────────────────
positions = []       # list of {lat, lon, alt, spd, wp, t}
pos_lock  = threading.Lock()
flight_complete = False

# Populated at startup from mission file
HOME             = ''
WP_NAMES         = {}
MISSION_WAYPOINTS = []
MISSION_TITLE    = 'ArduPlane SITL'


# ── MAVLink command labels ────────────────────────────────────────────────────
_CMD_LABELS = {
    16:  'Waypoint',
    17:  'Loiter',
    20:  'RTL',
    21:  'Land',
    22:  'Takeoff',
    92:  'Loiter',
    177: 'Loiter to Alt',
    189: 'Land (abort)',
}


def _bearing(lat1, lon1, lat2, lon2):
    """Initial bearing (degrees) from point 1 to point 2."""
    la1, lo1, la2, lo2 = map(math.radians, [lat1, lon1, lat2, lon2])
    x = math.sin(lo2 - lo1) * math.cos(la2)
    y = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def build_mission_config(waypoints):
    """Derive HOME, WP_NAMES, MISSION_WAYPOINTS and MISSION_TITLE from parsed waypoints."""
    global HOME, WP_NAMES, MISSION_WAYPOINTS, MISSION_TITLE

    home_wp = waypoints[0]
    home_lat, home_lon, home_alt = home_wp['x'], home_wp['y'], home_wp['z']

    # Heading: bearing from home toward first WP with real coordinates
    heading = 0
    for wp in waypoints[1:]:
        if abs(wp['x']) > 0.01 or abs(wp['y']) > 0.01:
            heading = _bearing(home_lat, home_lon, wp['x'], wp['y'])
            break

    HOME = f'{home_lat},{home_lon},{home_alt:.1f},{heading:.1f}'

    # WP_NAMES: seq → human label
    WP_NAMES = {}
    for wp in waypoints:
        seq = wp['seq']
        cmd = wp['command']
        if seq == 0:
            label = f'Home ({home_lat:.4f}, {home_lon:.4f})'
        elif cmd == 22:
            label = 'Takeoff'
        elif cmd == 21:
            label = f'Land ({wp["x"]:.4f}, {wp["y"]:.4f})'
        elif cmd == 20:
            label = 'RTL'
        else:
            label = f'{_CMD_LABELS.get(cmd, f"CMD{cmd}")} {seq}'
        WP_NAMES[seq] = label

    # MISSION_WAYPOINTS: only WPs with real lat/lon for Cesium markers
    MISSION_WAYPOINTS = []
    nav_cmds = {16, 17, 20, 21, 22, 92, 177, 189}
    last_land_seq = max(
        (wp['seq'] for wp in waypoints if wp['command'] == 21),
        default=-1)

    for wp in waypoints:
        lat, lon = wp['x'], wp['y']
        if abs(lat) < 0.001 and abs(lon) < 0.001:
            continue   # skip WPs with no real coordinates (e.g. relative takeoff)
        if wp['command'] not in nav_cmds:
            continue
        if wp['seq'] == 0:
            wtype = 'home'
        elif wp['seq'] == last_land_seq:
            wtype = 'land'
        else:
            wtype = 'wp'
        MISSION_WAYPOINTS.append({
            'lat': lat, 'lon': lon,
            'name': WP_NAMES[wp['seq']],
            'type': wtype,
        })

    # Title: "Home → Land" or just coordinates
    first = MISSION_WAYPOINTS[0]  if MISSION_WAYPOINTS else None
    last  = MISSION_WAYPOINTS[-1] if len(MISSION_WAYPOINTS) > 1 else None
    if first and last and first is not last:
        MISSION_TITLE = (f'ArduPlane SITL  '
                         f'({first["lat"]:.3f},{first["lon"]:.3f}) → '
                         f'({last["lat"]:.3f},{last["lon"]:.3f})')
    else:
        MISSION_TITLE = 'ArduPlane SITL'

    print(f'Home: {HOME}')
    print(f'Title: {MISSION_TITLE}')
    print(f'Map waypoints: {len(MISSION_WAYPOINTS)}')

# ── Cesium HTML ───────────────────────────────────────────────────────────────
HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>__TITLE__</title>
  <script src="https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Cesium.js"></script>
  <link  href="https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Widgets/widgets.css" rel="stylesheet">
  <style>
    * { margin:0; padding:0; box-sizing:border-box; }
    html, body { width:100%; height:100%; background:#000; }
    #map  { width:100%; height:100%; }

    #hud {
      position:absolute; top:12px; left:12px;
      background:rgba(10,20,40,0.82); color:#e0f4ff;
      font-family:'Courier New',monospace; font-size:13px;
      padding:14px 18px; border-radius:8px;
      border:1px solid rgba(100,180,255,0.3);
      min-width:280px; z-index:10;
      backdrop-filter:blur(4px);
    }
    #hud h2 { font-size:15px; color:#7dd3fc; margin-bottom:10px; letter-spacing:.5px; }
    #hud table { border-collapse:collapse; width:100%; }
    #hud td   { padding:2px 6px; }
    #hud td:first-child { color:#94a3b8; white-space:nowrap; }
    #hud td:last-child  { color:#f0f9ff; font-weight:bold; }
    #hud .sep { border-top:1px solid rgba(100,180,255,0.2); height:6px; }
    #hud .status-fly  { color:#4ade80; }
    #hud .status-done { color:#facc15; }

    #legend {
      position:absolute; bottom:30px; left:12px;
      background:rgba(10,20,40,0.82); color:#cbd5e1;
      font-family:'Courier New',monospace; font-size:12px;
      padding:10px 14px; border-radius:8px;
      border:1px solid rgba(100,180,255,0.2);
      z-index:10;
    }
    #legend div { margin:3px 0; display:flex; align-items:center; gap:8px; }
    .dot { width:10px; height:10px; border-radius:50%; flex-shrink:0; }
  </style>
</head>
<body>
<div id="map"></div>

<div id="hud">
  <h2>✈ __TITLE__</h2>
  <table id="tbl">
    <tr><td>Status</td><td id="v-status" class="status-fly">Initialising…</td></tr>
    <tr class="sep"><td colspan="2"></td></tr>
    <tr><td>Latitude</td> <td id="v-lat">—</td></tr>
    <tr><td>Longitude</td><td id="v-lon">—</td></tr>
    <tr><td>Alt AGL</td>  <td id="v-alt">—</td></tr>
    <tr><td>Speed</td>    <td id="v-spd">—</td></tr>
    <tr class="sep"><td colspan="2"></td></tr>
    <tr><td>Waypoint</td> <td id="v-wp">—</td></tr>
    <tr><td>Track pts</td><td id="v-pts">0</td></tr>
    <tr class="sep"><td colspan="2"></td></tr>
    <tr><td>Speedup</td>  <td>50×</td></tr>
  </table>
</div>

<div id="legend">
  <div><span class="dot" style="background:#22c55e"></span>Home</div>
  <div><span class="dot" style="background:#facc15"></span>Waypoints</div>
  <div><span class="dot" style="background:#ef4444"></span>Land</div>
  <div><span class="dot" style="background:rgba(148,163,184,.5);width:24px;height:2px;border-radius:0"></span>Planned route</div>
  <div><span class="dot" style="background:#38bdf8;width:24px;height:2px;border-radius:0"></span>Flight trail</div>
</div>

<script>
// ── Cesium init (no Ion required – using OSM tiles) ─────────────────────────
window.CESIUM_BASE_URL = 'https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/';

const viewer = new Cesium.Viewer('cesiumContainer', {
  imageryProvider: new Cesium.UrlTemplateImageryProvider({
    url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
    minimumLevel: 0, maximumLevel: 18,
    credit: '© OpenStreetMap contributors'
  }),
  terrainProvider : new Cesium.EllipsoidTerrainProvider(),
  baseLayerPicker : false,
  geocoder        : false,
  homeButton      : false,
  sceneModePicker : true,
  navigationHelpButton: false,
  timeline        : false,
  animation       : false,
  fullscreenButton: true,
  infoBox         : true,
});
viewer.scene.globe.enableLighting = true;

// ── Mission data injected from Python ───────────────────────────────────────
const WAYPOINTS = __WAYPOINTS__;

// ── Plot planned route line ─────────────────────────────────────────────────
const routeCoords = WAYPOINTS.flatMap(w => [w.lon, w.lat, 1100]);
viewer.entities.add({
  polyline: {
    positions       : Cesium.Cartesian3.fromDegreesArrayHeights(routeCoords),
    width           : 2,
    material        : new Cesium.PolylineDashMaterialProperty({
                        color    : Cesium.Color.fromCssColorString('#94a3b8').withAlpha(.6),
                        dashLength: 16
                      }),
    clampToGround   : false,
  }
});

// ── Plot waypoint markers ────────────────────────────────────────────────────
WAYPOINTS.forEach(w => {
  const color = w.type === 'home' ? Cesium.Color.fromCssColorString('#22c55e')
              : w.type === 'land' ? Cesium.Color.fromCssColorString('#ef4444')
              :                     Cesium.Color.fromCssColorString('#facc15');
  viewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(w.lon, w.lat, 1100),
    point   : { pixelSize: w.type !== 'wp' ? 14 : 10, color, outlineColor: Cesium.Color.BLACK, outlineWidth: 1.5 },
    label   : {
      text            : w.name,
      font            : '12px "Courier New"',
      fillColor       : Cesium.Color.WHITE,
      outlineColor    : Cesium.Color.BLACK,
      outlineWidth    : 2,
      style           : Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset     : new Cesium.Cartesian2(0, -20),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    }
  });
});

// ── Aircraft entity ──────────────────────────────────────────────────────────
const acPos = new Cesium.CallbackProperty(() => acCartesian, false);

let acCartesian = Cesium.Cartesian3.fromDegrees(35.0462, 48.4647, 200);
const acEntity  = viewer.entities.add({
  position : acPos,
  billboard: {
    image           : buildPlaneIcon(),
    width           : 36, height: 36,
    verticalOrigin  : Cesium.VerticalOrigin.CENTER,
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
    heightReference : Cesium.HeightReference.NONE,
  },
  label: {
    text            : '',
    font            : '11px "Courier New"',
    fillColor       : Cesium.Color.fromCssColorString('#7dd3fc'),
    outlineColor    : Cesium.Color.BLACK,
    outlineWidth    : 2,
    style           : Cesium.LabelStyle.FILL_AND_OUTLINE,
    pixelOffset     : new Cesium.Cartesian2(0, 28),
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
  }
});

// ── Flight trail ─────────────────────────────────────────────────────────────
let trailCarts = [];
const trailEntity = viewer.entities.add({
  polyline: {
    positions     : new Cesium.CallbackProperty(() => trailCarts, false),
    width         : 2.5,
    material      : new Cesium.PolylineGlowMaterialProperty({
                      glowPower: .12,
                      color: Cesium.Color.fromCssColorString('#38bdf8'),
                    }),
    clampToGround : false,
  }
});

// ── Initial camera: overview of Ukraine ─────────────────────────────────────
viewer.camera.flyTo({
  destination : Cesium.Cartesian3.fromDegrees(32.8, 49.2, 900000),
  orientation : { heading: 0, pitch: Cesium.Math.toRadians(-55), roll: 0 },
  duration    : 2,
});

let followAc = false;
let lastCount = 0;
let isDone    = false;

// ── Polling loop ─────────────────────────────────────────────────────────────
function poll() {
  fetch('/positions')
    .then(r => r.json())
    .then(data => {
      const pts = data.positions;
      isDone = data.complete;

      if (pts.length === 0) {
        document.getElementById('v-status').textContent = 'Waiting for SITL…';
        return;
      }

      const p = pts[pts.length - 1];
      const alt = typeof p.alt === 'number' ? p.alt : 0;

      // Update aircraft position
      acCartesian = Cesium.Cartesian3.fromDegrees(p.lon, p.lat, alt + 70);

      // Rebuild trail only when new points arrive
      if (pts.length !== lastCount) {
        trailCarts = pts.map(q =>
          Cesium.Cartesian3.fromDegrees(q.lon, q.lat, (q.alt || 0) + 70));
        lastCount = pts.length;
      }

      // Update HUD
      const wpName = __WPNAMES__[p.wp] || ('WP ' + p.wp);
      document.getElementById('v-status').textContent =
        isDone ? '✅ Landed in Kyiv' : '🛫 Flying';
      document.getElementById('v-status').className =
        isDone ? 'status-done' : 'status-fly';
      document.getElementById('v-lat').textContent  = p.lat.toFixed(5) + '°';
      document.getElementById('v-lon').textContent  = p.lon.toFixed(5) + '°';
      document.getElementById('v-alt').textContent  = alt.toFixed(0) + ' m';
      document.getElementById('v-spd').textContent  =
        (p.spd || 0).toFixed(1) + ' m/s (' + ((p.spd||0)*3.6).toFixed(0) + ' km/h)';
      document.getElementById('v-wp').textContent   = wpName;
      document.getElementById('v-pts').textContent  = pts.length;
      acEntity.label.text = alt.toFixed(0) + 'm';

      // Auto-follow during flight (first 3 seconds after first point)
      if (!followAc && pts.length === 1) {
        followAc = true;
        viewer.camera.flyTo({
          destination : Cesium.Cartesian3.fromDegrees(p.lon, p.lat, 300000),
          orientation : { heading: 0, pitch: Cesium.Math.toRadians(-50), roll: 0 },
          duration    : 3,
        });
      }
    })
    .catch(() => {});
}

setInterval(poll, 800);

// ── Plane SVG icon ────────────────────────────────────────────────────────────
function buildPlaneIcon() {
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64" width="64" height="64">
    <g transform="rotate(-45 32 32)">
      <polygon points="32,4 40,28 60,32 40,36 32,60 24,36 4,32 24,28"
               fill="#38bdf8" stroke="#0ea5e9" stroke-width="2"/>
    </g>
  </svg>`;
  return 'data:image/svg+xml;base64,' + btoa(svg);
}
</script>
</body>
</html>
"""

# ── HTTP server ───────────────────────────────────────────────────────────────
class TelemetryHandler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass  # silence access log

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            # Inject Python data into JS template placeholders
            html = HTML.replace(
                '__WAYPOINTS__', json.dumps(MISSION_WAYPOINTS)
            ).replace(
                '__WPNAMES__',   json.dumps(WP_NAMES)
            ).replace(
                '__TITLE__',     MISSION_TITLE
            )
            body = html.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == '/positions':
            with pos_lock:
                payload = json.dumps({
                    'positions': positions,
                    'complete':  flight_complete,
                }).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        else:
            self.send_error(404)


def start_web_server():
    server = HTTPServer(('127.0.0.1', WEB_PORT), TelemetryHandler)
    server.serve_forever()


# ── SITL helpers ─────────────────────────────────────────────────────────────
def start_sitl():
    cmd = [
        BINARY,
        '--model', 'plane',
        '--speedup', str(SPEEDUP),
        '--defaults', DEFAULTS,
        '--sim-address=127.0.0.1',
        '-I0',
        '--home', HOME,
    ]
    print(f"Starting SITL: {' '.join(cmd)}")
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def connect(port, retries=30):
    print(f"Connecting to SITL on tcp:127.0.0.1:{port} ...")
    for i in range(retries):
        try:
            mav = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=255)
            for _ in range(20):
                hb = mav.wait_heartbeat(timeout=3)
                if hb and mav.target_system > 0:
                    break
            if mav.target_system  == 0: mav.target_system  = 1
            if mav.target_component == 0: mav.target_component = 1
            print(f"Connected. sysid={mav.target_system} compid={mav.target_component}")
            return mav
        except Exception as e:
            print(f"  attempt {i+1}/{retries}: {e}")
            time.sleep(1)
    raise RuntimeError("Could not connect to SITL")


def parse_mission(filepath):
    waypoints = []
    with open(filepath) as f:
        lines = f.readlines()
    assert lines[0].strip().startswith('QGC WPL')
    for line in lines[1:]:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split('\t')
        if len(parts) < 12:
            parts = line.split()
        waypoints.append({
            'seq': int(parts[0]), 'current': int(parts[1]),
            'frame': int(parts[2]), 'command': int(parts[3]),
            'param1': float(parts[4]), 'param2': float(parts[5]),
            'param3': float(parts[6]), 'param4': float(parts[7]),
            'x': float(parts[8]), 'y': float(parts[9]),
            'z': float(parts[10]), 'autocontinue': int(parts[11]),
        })
    return waypoints


def upload_mission(mav, mission_file):
    """Upload mission using MISSION_ITEM_INT (MAVLink 2) with prior clear."""
    from pymavlink import mavwp
    loader = mavwp.MAVWPLoader(target_system=mav.target_system,
                               target_component=mav.target_component)
    n = loader.load(mission_file)
    print(f"Uploading {n} waypoints ...")
    for i in range(n):
        wp = loader.wp(i)
        print(f"  WP{wp.seq}: cmd={wp.command} frame={wp.frame} "
              f"lat={wp.x:.5f} lon={wp.y:.5f} alt={wp.z:.1f}m")

    # ── Step 1: clear existing mission so old WPs don't persist ──────────────
    mav.mav.mission_clear_all_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    clr = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    print(f"  Clear: {'OK' if clr and clr.type == 0 else clr}")
    time.sleep(0.3)

    # ── Step 2: upload ────────────────────────────────────────────────────────
    mav.mav.mission_count_send(
        mav.target_system, mav.target_component, n,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    success   = False
    last_sent = -1   # seq of last WP we actually sent; used to skip stale re-requests

    for _ in range(n * 6 + 20):
        msg = mav.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=5)
        if msg is None:
            print("  Timeout waiting for MISSION_REQUEST"); break
        if msg.get_type() == 'MISSION_ACK':
            success = (msg.type == 0)
            status  = 'ACCEPTED' if success else f'FAILED result={msg.type}'
            print(f"  Upload: {status}")
            break

        req_seq  = msg.seq
        req_type = msg.get_type()

        # KEY FIX: the vehicle queues multiple MISSION_REQUESTs before our first
        # reply arrives.  On TCP there is no packet loss, so any re-request for
        # a seq we already answered is a stale queued duplicate — replying again
        # would push the vehicle back to an earlier state → INVALID_SEQUENCE (13).
        if req_seq <= last_sent:
            print(f"  ← {req_type} seq={req_seq}  SKIP (stale, last_sent={last_sent})")
            continue

        wp = loader.wp(req_seq)
        if req_type == 'MISSION_REQUEST_INT':
            mav.mav.mission_item_int_send(
                mav.target_system, mav.target_component,
                wp.seq, wp.frame, wp.command,
                wp.current, wp.autocontinue,
                wp.param1, wp.param2, wp.param3, wp.param4,
                int(wp.x * 1e7), int(wp.y * 1e7), wp.z,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        else:
            mav.mav.send(wp)
        last_sent = req_seq
        print(f"  ← {req_type} seq={req_seq}  → WP{wp.seq} "
              f"cmd={wp.command} lat={wp.x:.5f} lon={wp.y:.5f} alt={wp.z:.0f}m")

    if not success:
        print("  WARNING: upload may be incomplete — verifying anyway")


def verify_mission(mav):
    """Download mission back from vehicle and print it to confirm correct upload."""
    print("\nVerifying uploaded mission ...")
    mav.mav.mission_request_list_send(mav.target_system, mav.target_component)
    cnt_msg = mav.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if cnt_msg is None:
        print("  No MISSION_COUNT received — cannot verify"); return
    n = cnt_msg.count
    print(f"  Vehicle reports {n} waypoints:")
    for i in range(n):
        mav.mav.mission_request_int_send(mav.target_system, mav.target_component, i)
        wp = mav.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        if wp is None:
            print(f"  WP{i}: no response"); continue
        lat = wp.x / 1e7
        lon = wp.y / 1e7
        print(f"  WP{wp.seq}: cmd={wp.command} frame={wp.frame} "
              f"lat={lat:.5f} lon={lon:.5f} alt={wp.z:.1f}m")
    # Send ACK
    mav.mav.mission_ack_send(mav.target_system, mav.target_component, 0,
                             mavutil.mavlink.MAV_MISSION_TYPE_MISSION)


def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"Mode {mode_name} not found!"); return
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    print(f"Mode → {mode_name} (id={mode_id})")


def arm(mav):
    print("Arming...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + 20
    while time.time() < deadline:
        msg = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == 0:
                print("Armed!"); return True
            print(f"Arm failed result={msg.result}, force-arming...")
            time.sleep(1)
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0)
    print("Warning: arm timeout"); return False


def monitor_flight(mav, n_waypoints):
    global flight_complete
    print(f"\n=== FLIGHT IN PROGRESS (Speedup {SPEEDUP}×) ===")
    print(f"    {MISSION_TITLE}\n")

    last_print  = 0
    last_wp     = -1
    last_record = 0
    last_seq    = n_waypoints - 1  # last waypoint seq index

    # Build a seq→(lat,lon) lookup from MISSION_WAYPOINTS for direction verification
    wp_coords = {w['name']: (w['lat'], w['lon']) for w in MISSION_WAYPOINTS}

    while True:
        msg = mav.recv_match(
            type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT', 'STATUSTEXT', 'HEARTBEAT',
                  'NAV_CONTROLLER_OUTPUT'],
            blocking=True, timeout=2)
        if msg is None:
            continue

        mtype = msg.get_type()

        if mtype == 'MISSION_CURRENT' and msg.seq != last_wp:
            last_wp = msg.seq
            name = WP_NAMES.get(msg.seq, f'WP {msg.seq}')
            # Find target coords from MISSION_WAYPOINTS list by index
            idx = msg.seq - 1  # MISSION_WAYPOINTS skips seq=0 (takeoff has no real coords)
            target_info = ''
            nav_wps = [w for w in MISSION_WAYPOINTS if w.get('lat', 0) and w.get('lon', 0)]
            if 0 <= idx < len(nav_wps):
                tw = nav_wps[idx]
                target_info = f'  → target ({tw["lat"]:.5f}, {tw["lon"]:.5f})'
            print(f"\n>>> WP {msg.seq}: {name}{target_info}")

        elif mtype == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000
            spd = (msg.vx**2 + msg.vy**2)**0.5 / 100
            now = time.time()

            # Record position every ~0.5s real time for smooth trail
            if now - last_record >= 0.5:
                with pos_lock:
                    positions.append({
                        'lat': round(lat, 6),
                        'lon': round(lon, 6),
                        'alt': round(alt, 1),
                        'spd': round(spd, 1),
                        'wp' : last_wp,
                        't'  : round(now, 2),
                    })
                last_record = now

            if now - last_print >= 5:
                print(f"  {lat:.4f},{lon:.4f}  {alt:.0f}m AGL  {spd:.0f} m/s  WP:{last_wp}")
                last_print = now

        elif mtype == 'STATUSTEXT':
            print(f"  MSG: {msg.text}")
            if 'Disarmed' in msg.text or 'landed' in msg.text.lower():
                flight_complete = True
                print("\n=== FLIGHT COMPLETE — Mission finished! ===")
                return

        elif mtype == 'HEARTBEAT':
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                if last_wp >= last_seq - 1:
                    flight_complete = True
                    print("\n=== FLIGHT COMPLETE — Aircraft disarmed! ===")
                    return


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    # ── Read mission and build config FIRST (before web server / SITL start)
    waypoints = parse_mission(MISSION_FILE)
    build_mission_config(waypoints)

    # Start web server in background thread
    t = threading.Thread(target=start_web_server, daemon=True)
    t.start()
    print(f"Cesium map → http://127.0.0.1:{WEB_PORT}/")

    # Open browser after a short delay
    threading.Timer(2.0, lambda: webbrowser.open(f'http://127.0.0.1:{WEB_PORT}/')).start()

    sitl_proc = start_sitl()
    try:
        time.sleep(5)
        mav = connect(SITL_PORT)

        print("Waiting for vehicle to initialise...")
        time.sleep(5)
        for _ in range(50):
            mav.recv_match(blocking=True, timeout=0.1)

        upload_mission(mav, MISSION_FILE)
        time.sleep(1)
        verify_mission(mav)
        time.sleep(1)

        set_mode(mav, 'AUTO')
        time.sleep(1)
        arm(mav)
        time.sleep(2)

        monitor_flight(mav, len(waypoints))

        # Keep server alive so user can inspect the final map
        print(f"\nServer still running at http://127.0.0.1:{WEB_PORT}/  (Ctrl+C to stop)")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        sitl_proc.terminate()
        print("SITL stopped.")
