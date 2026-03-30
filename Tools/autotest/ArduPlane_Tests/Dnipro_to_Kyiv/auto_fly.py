#!/usr/bin/env python3
"""
ArduPlane SITL auto-flight with live Cesium.js telemetry map.

Usage:
    python3 auto_fly.py [mission.txt] [--speedup N] [--port P] [--web-port W]

Defaults:
    mission : mission.txt next to this script
    speedup : 50
    port    : 5760  (SITL MAVLink TCP port)
    web-port: 8765  (Cesium HTTP server)
"""

import argparse
import math
import json
import os
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer

from pymavlink import mavutil

# ── Paths derived from repo layout ───────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.abspath(os.path.join(_SCRIPT_DIR, '../../../..'))
BINARY      = os.path.join(REPO_ROOT, 'build/sitl/bin/arduplane')
DEFAULTS    = os.path.join(REPO_ROOT, 'Tools/autotest/models/plane.parm')
# Model name is derived from the defaults filename (e.g. plane.parm → "plane")
MODEL       = os.path.splitext(os.path.basename(DEFAULTS))[0]

# ── Shared telemetry state ────────────────────────────────────────────────────
positions       = []     # list of {lat, lon, alt, spd, wp, t}
pos_lock        = threading.Lock()
flight_complete = False

# Populated at startup from the mission file
HOME             = ''
HOME_LAT         = 0.0
HOME_LON         = 0.0
ROUTE_LAT        = 0.0   # midpoint of route, used for initial camera
ROUTE_LON        = 0.0
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


# ── Geometry helpers ──────────────────────────────────────────────────────────
def _bearing(lat1, lon1, lat2, lon2):
    """Initial bearing (degrees true) from point 1 to point 2."""
    la1, lo1, la2, lo2 = map(math.radians, [lat1, lon1, lat2, lon2])
    x = math.sin(lo2 - lo1) * math.cos(la2)
    y = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def _midpoint(lats, lons):
    """Geographic midpoint of a list of lat/lon pairs."""
    if not lats:
        return 0.0, 0.0
    xs = [math.cos(math.radians(la)) * math.cos(math.radians(lo))
          for la, lo in zip(lats, lons)]
    ys = [math.cos(math.radians(la)) * math.sin(math.radians(lo))
          for la, lo in zip(lats, lons)]
    zs = [math.sin(math.radians(la)) for la in lats]
    mx, my, mz = sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)
    lon = math.degrees(math.atan2(my, mx))
    lat = math.degrees(math.atan2(mz, math.sqrt(mx**2 + my**2)))
    return lat, lon


# ── Mission config builder ────────────────────────────────────────────────────
def build_mission_config(waypoints):
    """Populate all globals from the parsed mission waypoints."""
    global HOME, HOME_LAT, HOME_LON, ROUTE_LAT, ROUTE_LON
    global WP_NAMES, MISSION_WAYPOINTS, MISSION_TITLE

    home_wp = waypoints[0]
    HOME_LAT, HOME_LON = home_wp['x'], home_wp['y']
    home_alt = home_wp['z']

    # Initial heading: bearing toward first WP with real coordinates
    heading = 0.0
    for wp in waypoints[1:]:
        if abs(wp['x']) > 0.01 or abs(wp['y']) > 0.01:
            heading = _bearing(HOME_LAT, HOME_LON, wp['x'], wp['y'])
            break
    HOME = f'{HOME_LAT},{HOME_LON},{home_alt:.1f},{heading:.1f}'

    # WP_NAMES: seq → human label
    WP_NAMES = {}
    for wp in waypoints:
        seq, cmd = wp['seq'], wp['command']
        if seq == 0:
            label = f'Home ({HOME_LAT:.4f}, {HOME_LON:.4f})'
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
    nav_cmds = {16, 17, 20, 21, 22, 92, 177, 189}
    last_land_seq = max(
        (wp['seq'] for wp in waypoints if wp['command'] == 21),
        default=-1)
    MISSION_WAYPOINTS = []
    for wp in waypoints:
        lat, lon = wp['x'], wp['y']
        if abs(lat) < 0.001 and abs(lon) < 0.001:
            continue   # skip WPs without real coordinates (relative takeoff etc.)
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

    # Route midpoint for initial camera position
    if MISSION_WAYPOINTS:
        ROUTE_LAT, ROUTE_LON = _midpoint(
            [w['lat'] for w in MISSION_WAYPOINTS],
            [w['lon'] for w in MISSION_WAYPOINTS])
    else:
        ROUTE_LAT, ROUTE_LON = HOME_LAT, HOME_LON

    # Title: "Home → Last" based on actual coordinates
    first = MISSION_WAYPOINTS[0]  if MISSION_WAYPOINTS else None
    last  = MISSION_WAYPOINTS[-1] if len(MISSION_WAYPOINTS) > 1 else None
    if first and last and first is not last:
        MISSION_TITLE = (f'ArduPlane SITL  '
                         f'({first["lat"]:.3f},{first["lon"]:.3f}) → '
                         f'({last["lat"]:.3f},{last["lon"]:.3f})')
    else:
        MISSION_TITLE = 'ArduPlane SITL'

    print(f'Home  : {HOME}')
    print(f'Title : {MISSION_TITLE}')
    print(f'Centre: {ROUTE_LAT:.4f}, {ROUTE_LON:.4f}')
    print(f'Map WPs: {len(MISSION_WAYPOINTS)}')


# ── Cesium HTML template ──────────────────────────────────────────────────────
# Placeholders replaced at serve-time:
#   __TITLE__      mission title string
#   __SPEEDUP__    simulation speedup factor
#   __HOME_LAT__   home latitude  (initial aircraft position)
#   __HOME_LON__   home longitude
#   __ROUTE_LAT__  route midpoint latitude  (initial camera target)
#   __ROUTE_LON__  route midpoint longitude
#   __WAYPOINTS__  JSON array of {lat,lon,name,type}
#   __WPNAMES__    JSON object {seq: label}
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
    #cesiumContainer { width:100%; height:100%; }

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
<div id="cesiumContainer"></div>

<div id="hud">
  <h2>✈ __TITLE__</h2>
  <table>
    <tr><td>Status</td>   <td id="v-status" class="status-fly">Initialising…</td></tr>
    <tr class="sep"><td colspan="2"></td></tr>
    <tr><td>Latitude</td> <td id="v-lat">—</td></tr>
    <tr><td>Longitude</td><td id="v-lon">—</td></tr>
    <tr><td>Alt AGL</td>  <td id="v-alt">—</td></tr>
    <tr><td>Speed</td>    <td id="v-spd">—</td></tr>
    <tr class="sep"><td colspan="2"></td></tr>
    <tr><td>Waypoint</td> <td id="v-wp">—</td></tr>
    <tr><td>Track pts</td><td id="v-pts">0</td></tr>
    <tr class="sep"><td colspan="2"></td></tr>
    <tr><td>Speedup</td>  <td>__SPEEDUP__×</td></tr>
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
// ── Cesium init (OSM tiles — no Ion token required) ───────────────────────
const viewer = new Cesium.Viewer('cesiumContainer', {
  imageryProvider: new Cesium.UrlTemplateImageryProvider({
    url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
    minimumLevel: 0, maximumLevel: 18,
    credit: '© OpenStreetMap contributors',
  }),
  terrainProvider     : new Cesium.EllipsoidTerrainProvider(),
  baseLayerPicker     : false,
  geocoder            : false,
  homeButton          : false,
  sceneModePicker     : true,
  navigationHelpButton: false,
  timeline            : false,
  animation           : false,
  fullscreenButton    : true,
  infoBox             : true,
});
viewer.scene.globe.enableLighting = true;

// ── Mission data injected by Python ──────────────────────────────────────
const WAYPOINTS = __WAYPOINTS__;
const WP_NAMES  = __WPNAMES__;

// ── Planned-route dashed line ─────────────────────────────────────────────
if (WAYPOINTS.length >= 2) {
  viewer.entities.add({
    polyline: {
      positions: Cesium.Cartesian3.fromDegreesArrayHeights(
        WAYPOINTS.flatMap(w => [w.lon, w.lat, 1100])),
      width   : 2,
      material: new Cesium.PolylineDashMaterialProperty({
        color    : Cesium.Color.fromCssColorString('#94a3b8').withAlpha(0.6),
        dashLength: 16,
      }),
    },
  });
}

// ── Waypoint markers ──────────────────────────────────────────────────────
WAYPOINTS.forEach(w => {
  const color = w.type === 'home' ? Cesium.Color.fromCssColorString('#22c55e')
              : w.type === 'land' ? Cesium.Color.fromCssColorString('#ef4444')
              :                     Cesium.Color.fromCssColorString('#facc15');
  viewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(w.lon, w.lat, 1100),
    point   : { pixelSize: w.type === 'wp' ? 10 : 14, color,
                outlineColor: Cesium.Color.BLACK, outlineWidth: 1.5 },
    label   : {
      text     : w.name,
      font     : '12px "Courier New"',
      fillColor: Cesium.Color.WHITE,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style    : Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new Cesium.Cartesian2(0, -20),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });
});

// ── Aircraft entity (starts at home position) ─────────────────────────────
let acCartesian = Cesium.Cartesian3.fromDegrees(__HOME_LON__, __HOME_LAT__, 200);
const acEntity  = viewer.entities.add({
  position : new Cesium.CallbackProperty(() => acCartesian, false),
  billboard: {
    image : buildPlaneIcon(),
    width : 36, height: 36,
    verticalOrigin          : Cesium.VerticalOrigin.CENTER,
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
  },
  label: {
    text        : '',
    font        : '11px "Courier New"',
    fillColor   : Cesium.Color.fromCssColorString('#7dd3fc'),
    outlineColor: Cesium.Color.BLACK,
    outlineWidth: 2,
    style       : Cesium.LabelStyle.FILL_AND_OUTLINE,
    pixelOffset : new Cesium.Cartesian2(0, 28),
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
  },
});

// ── Flight trail ──────────────────────────────────────────────────────────
let trailCarts = [];
viewer.entities.add({
  polyline: {
    positions: new Cesium.CallbackProperty(() => trailCarts, false),
    width    : 2.5,
    material : new Cesium.PolylineGlowMaterialProperty({
      glowPower: 0.12,
      color    : Cesium.Color.fromCssColorString('#38bdf8'),
    }),
  },
});

// ── Initial camera — centred on route midpoint ────────────────────────────
viewer.camera.flyTo({
  destination : Cesium.Cartesian3.fromDegrees(__ROUTE_LON__, __ROUTE_LAT__, 900000),
  orientation : { heading: 0, pitch: Cesium.Math.toRadians(-55), roll: 0 },
  duration    : 2,
});

let followAc  = false;
let lastCount = 0;

// ── Polling loop ──────────────────────────────────────────────────────────
function poll() {
  fetch('/positions')
    .then(r => r.json())
    .then(data => {
      const pts  = data.positions;
      const done = data.complete;
      if (pts.length === 0) {
        document.getElementById('v-status').textContent = 'Waiting for SITL…';
        return;
      }

      const p   = pts[pts.length - 1];
      const alt = typeof p.alt === 'number' ? p.alt : 0;

      acCartesian = Cesium.Cartesian3.fromDegrees(p.lon, p.lat, alt + 70);

      if (pts.length !== lastCount) {
        trailCarts = pts.map(q =>
          Cesium.Cartesian3.fromDegrees(q.lon, q.lat, (q.alt || 0) + 70));
        lastCount = pts.length;
      }

      document.getElementById('v-status').textContent =
        done ? '✅ Mission complete' : '🛫 Flying';
      document.getElementById('v-status').className =
        done ? 'status-done' : 'status-fly';
      document.getElementById('v-lat').textContent = p.lat.toFixed(5) + '°';
      document.getElementById('v-lon').textContent = p.lon.toFixed(5) + '°';
      document.getElementById('v-alt').textContent = alt.toFixed(0) + ' m';
      document.getElementById('v-spd').textContent =
        (p.spd || 0).toFixed(1) + ' m/s (' + ((p.spd || 0) * 3.6).toFixed(0) + ' km/h)';
      document.getElementById('v-wp').textContent  =
        WP_NAMES[p.wp] || ('WP ' + p.wp);
      document.getElementById('v-pts').textContent = pts.length;
      acEntity.label.text = alt.toFixed(0) + 'm';

      // Zoom to first fix
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

// ── Plane icon (SVG) ──────────────────────────────────────────────────────
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
        pass

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            html = (HTML
                .replace('__TITLE__',     MISSION_TITLE)
                .replace('__SPEEDUP__',   str(SPEEDUP))
                .replace('__HOME_LAT__',  str(HOME_LAT))
                .replace('__HOME_LON__',  str(HOME_LON))
                .replace('__ROUTE_LAT__', str(round(ROUTE_LAT, 4)))
                .replace('__ROUTE_LON__', str(round(ROUTE_LON, 4)))
                .replace('__WAYPOINTS__', json.dumps(MISSION_WAYPOINTS))
                .replace('__WPNAMES__',   json.dumps(WP_NAMES))
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
                    'complete' : flight_complete,
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
    HTTPServer(('127.0.0.1', WEB_PORT), TelemetryHandler).serve_forever()


# ── SITL helpers ──────────────────────────────────────────────────────────────
def start_sitl():
    cmd = [
        BINARY,
        '--model',        MODEL,
        '--speedup',      str(SPEEDUP),
        '--defaults',     DEFAULTS,
        '--sim-address=127.0.0.1',
        '-I0',
        '--home',         HOME,
    ]
    print(f"Starting SITL: {' '.join(cmd)}")
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def connect(port, retries=30):
    print(f"Connecting to tcp:127.0.0.1:{port} ...")
    for i in range(retries):
        try:
            mav = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=255)
            for _ in range(20):
                if mav.wait_heartbeat(timeout=3) and mav.target_system > 0:
                    break
            if mav.target_system   == 0: mav.target_system   = 1
            if mav.target_component == 0: mav.target_component = 1
            print(f"Connected. sysid={mav.target_system} compid={mav.target_component}")
            return mav
        except Exception as e:
            print(f"  attempt {i+1}/{retries}: {e}")
            time.sleep(1)
    raise RuntimeError("Could not connect to SITL")


def parse_mission(filepath):
    """Parse a QGC WPL 110 mission file into a list of waypoint dicts."""
    waypoints = []
    with open(filepath) as f:
        lines = f.readlines()
    if not lines[0].strip().startswith('QGC WPL'):
        raise ValueError(f"{filepath} is not a QGC WPL file")
    for line in lines[1:]:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split('\t')
        if len(parts) < 12:
            parts = line.split()
        waypoints.append({
            'seq'         : int(parts[0]),
            'current'     : int(parts[1]),
            'frame'       : int(parts[2]),
            'command'     : int(parts[3]),
            'param1'      : float(parts[4]),
            'param2'      : float(parts[5]),
            'param3'      : float(parts[6]),
            'param4'      : float(parts[7]),
            'x'           : float(parts[8]),   # latitude
            'y'           : float(parts[9]),   # longitude
            'z'           : float(parts[10]),  # altitude
            'autocontinue': int(parts[11]),
        })
    return waypoints


def upload_mission(mav, mission_file):
    """Upload mission via MAVLink mission protocol with stale-request deduplication."""
    from pymavlink import mavwp
    loader = mavwp.MAVWPLoader(target_system=mav.target_system,
                               target_component=mav.target_component)
    n = loader.load(mission_file)
    print(f"Uploading {n} waypoints ...")
    for i in range(n):
        wp = loader.wp(i)
        print(f"  WP{wp.seq}: cmd={wp.command} frame={wp.frame} "
              f"lat={wp.x:.5f} lon={wp.y:.5f} alt={wp.z:.1f}m")

    # Clear any previously stored mission so stale waypoints don't persist
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component,
                                   mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    clr = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    print(f"  Clear: {'OK' if clr and clr.type == 0 else str(clr)}")
    time.sleep(0.3)

    mav.mav.mission_count_send(mav.target_system, mav.target_component,
                               n, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    success   = False
    last_sent = -1   # highest seq we have responded to

    for _ in range(n * 6 + 20):
        msg = mav.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=5)
        if msg is None:
            print("  Timeout waiting for MISSION_REQUEST"); break

        if msg.get_type() == 'MISSION_ACK':
            success = (msg.type == 0)
            print(f"  Upload: {'ACCEPTED' if success else f'FAILED result={msg.type}'}")
            break

        req_seq  = msg.seq
        req_type = msg.get_type()

        # The vehicle queues multiple MISSION_REQUESTs before our first reply arrives.
        # On TCP (no packet loss) any re-request for a seq already answered is a
        # stale queued duplicate.  Answering it would rewind the vehicle's sequence
        # counter → MAV_MISSION_INVALID_SEQUENCE (13).
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
    """Download mission back and print it to confirm correct upload."""
    print("\nVerifying mission on vehicle ...")
    mav.mav.mission_request_list_send(mav.target_system, mav.target_component)
    cnt = mav.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if cnt is None:
        print("  No MISSION_COUNT — cannot verify"); return
    print(f"  Vehicle reports {cnt.count} waypoints:")
    for i in range(cnt.count):
        mav.mav.mission_request_int_send(mav.target_system, mav.target_component, i)
        wp = mav.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        if wp is None:
            print(f"  WP{i}: no response"); continue
        print(f"  WP{wp.seq}: cmd={wp.command} frame={wp.frame} "
              f"lat={wp.x/1e7:.5f} lon={wp.y/1e7:.5f} alt={wp.z:.1f}m")
    mav.mav.mission_ack_send(mav.target_system, mav.target_component,
                             0, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)


def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"Mode {mode_name} not found in mapping!"); return
    mav.mav.set_mode_send(mav.target_system,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          mode_id)
    print(f"Mode → {mode_name} (id={mode_id})")


def arm(mav):
    print("Arming ...")
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
            # Force-arm (bypass pre-arm checks for SITL)
            print(f"  Arm refused (result={msg.result}), force-arming ...")
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

    last_print  = 0.0
    last_wp     = -1
    last_record = 0.0
    last_seq    = n_waypoints - 1   # highest seq index in the mission

    while True:
        msg = mav.recv_match(
            type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT',
                  'STATUSTEXT', 'HEARTBEAT'],
            blocking=True, timeout=2)
        if msg is None:
            continue

        mtype = msg.get_type()

        if mtype == 'MISSION_CURRENT' and msg.seq != last_wp:
            last_wp = msg.seq
            print(f"\n>>> WP {msg.seq}: {WP_NAMES.get(msg.seq, f'WP {msg.seq}')}")

        elif mtype == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000
            spd = (msg.vx**2 + msg.vy**2)**0.5 / 100
            now = time.time()

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
                print(f"  {lat:.4f},{lon:.4f}  {alt:.0f}m AGL  "
                      f"{spd:.0f} m/s  WP:{last_wp}")
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


# ── Entry point ───────────────────────────────────────────────────────────────
def parse_args():
    parser = argparse.ArgumentParser(
        description='ArduPlane SITL auto-flight with live Cesium.js map')
    parser.add_argument(
        'mission', nargs='?',
        default=os.path.join(_SCRIPT_DIR, 'mission.txt'),
        help='Path to QGC WPL mission file (default: mission.txt next to this script)')
    parser.add_argument(
        '--speedup', type=int, default=50,
        help='Simulation speedup factor (default: 50)')
    parser.add_argument(
        '--port', type=int, default=5760,
        help='SITL MAVLink TCP port (default: 5760)')
    parser.add_argument(
        '--web-port', type=int, default=8765,
        help='Cesium HTTP server port (default: 8765)')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()

    # Expose CLI values as module-level names used throughout
    MISSION_FILE = args.mission
    SPEEDUP      = args.speedup
    SITL_PORT    = args.port
    WEB_PORT     = args.web_port

    # Parse mission and build all derived config before anything else starts
    waypoints = parse_mission(MISSION_FILE)
    build_mission_config(waypoints)

    # Start Cesium web server in background
    threading.Thread(target=start_web_server, daemon=True).start()
    print(f"Cesium map → http://127.0.0.1:{WEB_PORT}/")
    threading.Timer(2.0, lambda: webbrowser.open(f'http://127.0.0.1:{WEB_PORT}/')).start()

    sitl_proc = start_sitl()
    try:
        time.sleep(5)
        mav = connect(SITL_PORT)

        print("Waiting for vehicle to initialise ...")
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

        print(f"\nServer still running → http://127.0.0.1:{WEB_PORT}/  (Ctrl+C to stop)")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        sitl_proc.terminate()
        print("SITL stopped.")
