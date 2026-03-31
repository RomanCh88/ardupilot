#!/usr/bin/env python3
"""
ArduPlane SITL auto-flight with live Cesium.js telemetry map.
Single-drone or swarm mode (--drones N) with mesh-network visualization.

Usage:
    python3 auto_fly.py [mission.txt] [--speedup N] [--port P] [--web-port W]
                        [--drones N] [--mesh-range M] [--launch-delay D]

Defaults:
    mission      : mission.txt next to this script
    speedup      : 50
    port         : 5760  (base SITL TCP port; instance i uses port + i*10)
    web-port     : 8765
    drones       : 1
    mesh-range   : 300   (metres — max radio range between adjacent drones)
    launch-delay : 5.0   (seconds between sequential drone launches)
"""

import argparse
import copy
import math
import json
import os
import random
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer

from pymavlink import mavutil

# ── Paths derived from repo layout ────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.abspath(os.path.join(_SCRIPT_DIR, '../../../..'))
BINARY      = os.path.join(REPO_ROOT, 'build/sitl/bin/arduplane')
DEFAULTS    = os.path.join(REPO_ROOT, 'Tools/autotest/models/plane.parm')
MODEL       = os.path.splitext(os.path.basename(DEFAULTS))[0]

# ── Logging (stdout + file tee) ───────────────────────────────────────────────

class _Tee:
    """Write every print() to both the real stdout and a log file."""
    def __init__(self, primary, secondary):
        self._p = primary
        self._s = secondary
        self._lock = threading.Lock()

    def write(self, data):
        with self._lock:
            try:  self._p.write(data);  self._p.flush()
            except Exception: pass
            try:  self._s.write(data);  self._s.flush()
            except Exception: pass

    def flush(self):
        try:  self._p.flush()
        except Exception: pass
        try:  self._s.flush()
        except Exception: pass

    # Proxy everything else (isatty, fileno, …) to the primary stream
    def __getattr__(self, name):
        return getattr(self._p, name)


_log_file   = None   # opened in setup_logging()
_http_server = None  # set in start_web_server()


def setup_logging():
    """Open a timestamped log file and tee stdout/stderr into it."""
    global _log_file
    # Use a sub-folder so Python .log files don't mix with ArduPilot .BIN logs
    log_dir  = os.path.join(_SCRIPT_DIR, 'logs', 'python')
    os.makedirs(log_dir, exist_ok=True)
    ts       = time.strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(log_dir, f'flight_{ts}.log')
    _log_file        = open(log_path, 'w', buffering=1, encoding='utf-8')
    sys.stdout = _Tee(sys.__stdout__, _log_file)
    sys.stderr = _Tee(sys.__stderr__, _log_file)
    print(f'Log file : {log_path}')
    return log_path


# ── Shared swarm state ────────────────────────────────────────────────────────
# drone_id (int) → {'positions': [...], 'complete': bool}
_swarm      = {}
_swarm_lock = threading.Lock()

# ── Mission config globals (populated by build_mission_config) ────────────────
HOME             = ''    # "lat,lon,alt,hdg" string for SITL --home
HOME_LAT         = 0.0
HOME_LON         = 0.0
HOME_ALT         = 0.0
ROUTE_BEARING    = 0.0   # initial bearing home → first real WP (degrees)
ROUTE_LAT        = 0.0   # geographic midpoint of route (for initial camera)
ROUTE_LON        = 0.0
WP_NAMES         = {}    # seq (int) → human-readable label
MISSION_WAYPOINTS = []   # list of {lat, lon, name, type} for base route markers
MISSION_TITLE    = 'ArduPlane SITL'

# ── Runtime parameters (set in __main__) ─────────────────────────────────────
SPEEDUP      = 50
SITL_PORT    = 5760
WEB_PORT     = 8765
NUM_DRONES   = 1
MESH_RANGE_M = 300
LAUNCH_DELAY = 5.0

# ── MAVLink command labels ────────────────────────────────────────────────────
_CMD_LABELS = {
    16: 'Waypoint',  17: 'Loiter',   20: 'RTL',          21: 'Land',
    22: 'Takeoff',   92: 'Loiter',  177: 'Loiter→Alt',   189: 'Land(abort)',
}


# ── Geometry helpers ──────────────────────────────────────────────────────────

def _bearing(lat1, lon1, lat2, lon2):
    """Initial bearing (degrees true) from point 1 to point 2."""
    la1, lo1, la2, lo2 = map(math.radians, [lat1, lon1, lat2, lon2])
    x = math.sin(lo2 - lo1) * math.cos(la2)
    y = (math.cos(la1) * math.sin(la2)
         - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def _midpoint(lats, lons):
    """Geographic midpoint via 3-D Cartesian average."""
    if not lats:
        return 0.0, 0.0
    xs = [math.cos(math.radians(la)) * math.cos(math.radians(lo))
          for la, lo in zip(lats, lons)]
    ys = [math.cos(math.radians(la)) * math.sin(math.radians(lo))
          for la, lo in zip(lats, lons)]
    zs = [math.sin(math.radians(la)) for la in lats]
    mx, my, mz = sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)
    return (math.degrees(math.atan2(mz, math.sqrt(mx**2 + my**2))),
            math.degrees(math.atan2(my, mx)))


def _offset_latlon(lat, lon, bearing_deg, dist_m):
    """Move (lat, lon) by dist_m along bearing_deg; returns (new_lat, new_lon)."""
    R  = 6_371_000.0
    d  = dist_m / R
    b  = math.radians(bearing_deg)
    la1 = math.radians(lat)
    lo1 = math.radians(lon)
    la2 = math.asin(math.sin(la1) * math.cos(d) +
                    math.cos(la1) * math.sin(d) * math.cos(b))
    lo2 = lo1 + math.atan2(math.sin(b) * math.sin(d) * math.cos(la1),
                            math.cos(d) - math.sin(la1) * math.sin(la2))
    return math.degrees(la2), math.degrees(lo2)


def _haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres between two lat/lon points."""
    R = 6_371_000.0
    la1, lo1, la2, lo2 = map(math.radians, [lat1, lon1, lat2, lon2])
    a = (math.sin((la2 - la1) / 2) ** 2
         + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2)
    return R * 2 * math.asin(math.sqrt(a))


# ── Mission config builder ────────────────────────────────────────────────────

def build_mission_config(waypoints):
    """Populate all mission-derived globals from a parsed waypoint list."""
    global HOME, HOME_LAT, HOME_LON, HOME_ALT, ROUTE_BEARING
    global ROUTE_LAT, ROUTE_LON, WP_NAMES, MISSION_WAYPOINTS, MISSION_TITLE

    home_wp  = waypoints[0]
    HOME_LAT = home_wp['x']
    HOME_LON = home_wp['y']
    HOME_ALT = home_wp['z']

    # Route bearing: home → first WP with real coordinates
    ROUTE_BEARING = 0.0
    for wp in waypoints[1:]:
        if abs(wp['x']) > 0.01 or abs(wp['y']) > 0.01:
            ROUTE_BEARING = _bearing(HOME_LAT, HOME_LON, wp['x'], wp['y'])
            break
    HOME = f'{HOME_LAT},{HOME_LON},{HOME_ALT:.1f},{ROUTE_BEARING:.1f}'

    # Human-readable WP labels
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

    # Waypoints visible on Cesium map (must have real lat/lon)
    nav_cmds      = {16, 17, 20, 21, 22, 92, 177, 189}
    last_land_seq = max((wp['seq'] for wp in waypoints if wp['command'] == 21),
                        default=-1)
    MISSION_WAYPOINTS = []
    for wp in waypoints:
        lat, lon = wp['x'], wp['y']
        if abs(lat) < 0.001 and abs(lon) < 0.001:
            continue
        if wp['command'] not in nav_cmds:
            continue
        if wp['seq'] == 0:
            wtype = 'home'
        elif wp['seq'] == last_land_seq:
            wtype = 'land'
        else:
            wtype = 'wp'
        MISSION_WAYPOINTS.append(
            {'lat': lat, 'lon': lon,
             'name': WP_NAMES[wp['seq']], 'type': wtype})

    if MISSION_WAYPOINTS:
        ROUTE_LAT, ROUTE_LON = _midpoint(
            [w['lat'] for w in MISSION_WAYPOINTS],
            [w['lon'] for w in MISSION_WAYPOINTS])
    else:
        ROUTE_LAT, ROUTE_LON = HOME_LAT, HOME_LON

    first = MISSION_WAYPOINTS[0]  if MISSION_WAYPOINTS           else None
    last  = MISSION_WAYPOINTS[-1] if len(MISSION_WAYPOINTS) > 1  else None
    if first and last and first is not last:
        MISSION_TITLE = (f'ArduPlane SITL  '
                         f'({first["lat"]:.3f},{first["lon"]:.3f}) → '
                         f'({last["lat"]:.3f},{last["lon"]:.3f})')
    else:
        MISSION_TITLE = 'ArduPlane SITL'

    print(f'Home    : {HOME}')
    print(f'Bearing : {ROUTE_BEARING:.1f}°')
    print(f'Title   : {MISSION_TITLE}')
    print(f'Centre  : {ROUTE_LAT:.4f}, {ROUTE_LON:.4f}')
    print(f'Map WPs : {len(MISSION_WAYPOINTS)}')


# ── Swarm helpers ─────────────────────────────────────────────────────────────

def generate_swarm_offsets(n, mesh_range_m):
    """
    Return n (lateral_m, alt_m) tuples for drone fleet placement.

    Guarantees:
    - Adjacent drones are separated by 0.45–0.65 × mesh_range_m laterally,
      ensuring the mesh chain is never broken.
    - All offsets are centred at 0 (no "lead" drone).
    - Altitude offsets use a random walk ±12 m per step, also centred.
    - Final assignments are shuffled so drone IDs are not spatially ordered.
    """
    if n == 1:
        return [(0.0, 0.0)]

    max_step = mesh_range_m * 0.65
    min_step = mesh_range_m * 0.45

    lat_seq = [0.0]
    for _ in range(n - 1):
        lat_seq.append(lat_seq[-1] + random.uniform(min_step, max_step))
    centre_lat = (lat_seq[0] + lat_seq[-1]) / 2
    lat_seq = [v - centre_lat for v in lat_seq]

    alt_seq = [0.0]
    for _ in range(n - 1):
        alt_seq.append(alt_seq[-1] + random.uniform(-12.0, 12.0))
    centre_alt = sum(alt_seq) / len(alt_seq)
    alt_seq = [v - centre_alt for v in alt_seq]

    combined = list(zip(lat_seq, alt_seq))
    random.shuffle(combined)
    return combined


def make_offset_mission(base_wps, lat_off_m, alt_off_m):
    """
    Deep-copy base_wps, shifting every WP with real coordinates:
    - laterally by lat_off_m metres perpendicular to ROUTE_BEARING
    - vertically by alt_off_m metres
    WPs with lat≈0 / lon≈0 (relative-frame) are left unchanged.
    """
    perp = (ROUTE_BEARING + 90.0) % 360.0
    wps  = copy.deepcopy(base_wps)
    for wp in wps:
        lat, lon = wp['x'], wp['y']
        if abs(lat) < 0.001 and abs(lon) < 0.001:
            continue
        new_lat, new_lon = _offset_latlon(lat, lon, perp, lat_off_m)
        wp['x'] = new_lat
        wp['y'] = new_lon
        wp['z'] = max(0.0, wp['z'] + alt_off_m)
    return wps


def write_temp_mission(wps, drone_id):
    """Write waypoints to a QGC WPL 110 temp file; return the file path."""
    tmp_dir = os.path.join(_SCRIPT_DIR, '.tmp_missions')
    os.makedirs(tmp_dir, exist_ok=True)
    path = os.path.join(tmp_dir, f'mission_drone{drone_id}.txt')
    with open(path, 'w') as fh:
        fh.write('QGC WPL 110\n')
        for wp in wps:
            fh.write(
                f"{wp['seq']}\t{wp['current']}\t{wp['frame']}\t{wp['command']}\t"
                f"{wp['param1']}\t{wp['param2']}\t{wp['param3']}\t{wp['param4']}\t"
                f"{wp['x']:.8f}\t{wp['y']:.8f}\t{wp['z']:.6f}\t{wp['autocontinue']}\n"
            )
    return path


# ── Cesium HTML template ──────────────────────────────────────────────────────

HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>__TITLE__</title>
<script src="https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Cesium.js"></script>
<link href="https://cesium.com/downloads/cesiumjs/releases/1.115/Build/Cesium/Widgets/widgets.css" rel="stylesheet">
<style>
* { margin:0; padding:0; box-sizing:border-box; }
html, body { width:100%; height:100%; background:#000; overflow:hidden; }
#cesiumContainer { width:100%; height:100%; }

#hud {
  position:absolute; top:12px; left:12px; z-index:10;
  background:rgba(8,18,38,0.88);
  color:#e0f4ff; font-family:'Courier New',monospace; font-size:12px;
  padding:14px 18px; border-radius:10px;
  border:1px solid rgba(100,180,255,0.25);
  min-width:340px; max-width:460px;
  backdrop-filter:blur(6px);
  box-shadow:0 4px 24px rgba(0,0,0,0.5);
}
#hud h2 {
  font-size:13px; color:#7dd3fc; margin-bottom:10px;
  letter-spacing:.4px; white-space:nowrap; overflow:hidden;
  text-overflow:ellipsis;
}
#hud table { border-collapse:collapse; width:100%; }
#hud th {
  color:#475569; font-weight:normal; padding:2px 6px 4px;
  text-align:left; border-bottom:1px solid rgba(100,180,255,0.18);
  font-size:11px;
}
#hud td { padding:3px 6px; }
#hud .meta-table { margin-top:10px; border-top:1px solid rgba(100,180,255,0.15); padding-top:8px; }
#hud .meta-table td:first-child { color:#475569; }
#hud .meta-table td:last-child  { color:#f0f9ff; font-weight:bold; padding-left:8px; }
.status-fly  { color:#4ade80; }
.status-wait { color:#64748b; }
.status-done { color:#facc15; }

#legend {
  position:absolute; bottom:34px; left:12px; z-index:10;
  background:rgba(8,18,38,0.82);
  color:#cbd5e1; font-family:'Courier New',monospace; font-size:11px;
  padding:10px 14px; border-radius:8px;
  border:1px solid rgba(100,180,255,0.18);
  backdrop-filter:blur(4px);
}
#legend .row { margin:3px 0; display:flex; align-items:center; gap:8px; }
.dot      { width:10px; height:10px; border-radius:50%; flex-shrink:0; }
.line-seg { width:24px; height:2px;  border-radius:1px; flex-shrink:0; }
</style>
</head>
<body>
<div id="cesiumContainer"></div>

<div id="hud">
  <h2>&#9992; __TITLE__</h2>
  <table id="drone-table">
    <thead>
      <tr>
        <th>Drone</th>
        <th>Status</th>
        <th>Alt</th>
        <th>Speed</th>
        <th>Waypoint</th>
      </tr>
    </thead>
    <tbody id="drone-tbody"></tbody>
  </table>
  <table class="meta-table">
    <tr><td>Speedup</td><td>__SPEEDUP__&times;</td></tr>
    <tr><td>Drones</td><td>__NUM_DRONES__</td></tr>
    <tr id="mesh-row"><td>Mesh range</td><td>__MESH_RANGE__ m</td></tr>
  </table>
</div>

<div id="legend">
  <div class="row"><span class="dot"      style="background:#22c55e"></span>Home</div>
  <div class="row"><span class="dot"      style="background:#facc15"></span>Waypoints</div>
  <div class="row"><span class="dot"      style="background:#ef4444"></span>Land</div>
  <div class="row"><span class="line-seg" style="background:rgba(148,163,184,.45)"></span>Planned route</div>
  <div class="row" id="leg-mesh-in">
    <span class="line-seg" style="background:#22c55e"></span>Mesh link (in range)
  </div>
  <div class="row" id="leg-mesh-out">
    <span class="line-seg" style="background:#ef4444"></span>Mesh link (out of range)
  </div>
</div>

<script>
// ── Constants injected by Python ─────────────────────────────────────────────
const WAYPOINTS  = __WAYPOINTS__;
const WP_NAMES   = __WPNAMES__;
const NUM_DRONES = __NUM_DRONES__;
const MESH_RANGE = __MESH_RANGE__;

// Hide mesh legend rows in single-drone mode
if (NUM_DRONES < 2) {
  document.getElementById('mesh-row').style.display = 'none';
  document.getElementById('leg-mesh-in').style.display  = 'none';
  document.getElementById('leg-mesh-out').style.display = 'none';
}

const DRONE_COLORS = [
  '#38bdf8', '#f97316', '#4ade80', '#a855f7', '#f43f5e',
  '#eab308', '#06b6d4', '#ec4899', '#84cc16', '#8b5cf6',
];

// ── Cesium viewer ────────────────────────────────────────────────────────────
const viewer = new Cesium.Viewer('cesiumContainer', {
  imageryProvider: new Cesium.UrlTemplateImageryProvider({
    url         : 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
    minimumLevel: 0,
    maximumLevel: 18,
    credit      : '&copy; OpenStreetMap contributors',
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

// ── Base route (dashed line) ─────────────────────────────────────────────────
if (WAYPOINTS.length >= 2) {
  viewer.entities.add({
    polyline: {
      positions: Cesium.Cartesian3.fromDegreesArrayHeights(
        WAYPOINTS.flatMap(w => [w.lon, w.lat, 1100])),
      width   : 2,
      material: new Cesium.PolylineDashMaterialProperty({
        color     : Cesium.Color.fromCssColorString('#94a3b8').withAlpha(0.45),
        dashLength: 16,
      }),
    },
  });
}

// ── Waypoint markers ─────────────────────────────────────────────────────────
WAYPOINTS.forEach(w => {
  const color = w.type === 'home' ? Cesium.Color.fromCssColorString('#22c55e')
              : w.type === 'land' ? Cesium.Color.fromCssColorString('#ef4444')
              :                     Cesium.Color.fromCssColorString('#facc15');
  viewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(w.lon, w.lat, 1100),
    point   : {
      pixelSize   : w.type === 'wp' ? 10 : 14,
      color,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 1.5,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
    label: {
      text        : w.name,
      font        : '11px "Courier New"',
      fillColor   : Cesium.Color.WHITE,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style       : Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset : new Cesium.Cartesian2(0, -20),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });
});

// ── Initial camera centred on route midpoint ──────────────────────────────────
viewer.camera.flyTo({
  destination : Cesium.Cartesian3.fromDegrees(__ROUTE_LON__, __ROUTE_LAT__, 900000),
  orientation : { heading: 0, pitch: Cesium.Math.toRadians(-55), roll: 0 },
  duration    : 2,
});

// ── Per-drone entity registry ─────────────────────────────────────────────────
// droneState[id] = { cartVal, trailPts, acEntity, trailEntity, lastCount }
const droneState = {};

// ── Mesh-link entity registry ─────────────────────────────────────────────────
// meshState['i-j'] = { pts: [C3, C3], colorVal: Color, entity }
const meshState = {};

function ensureDroneEntities(id) {
  if (droneState[id]) return;
  const hex   = DRONE_COLORS[id % DRONE_COLORS.length];
  const color = Cesium.Color.fromCssColorString(hex);
  const state = {
    cartVal  : Cesium.Cartesian3.fromDegrees(__HOME_LON__, __HOME_LAT__, 0),
    trailPts : [],
    lastCount: 0,
  };

  state.acEntity = viewer.entities.add({
    position : new Cesium.CallbackProperty(() => state.cartVal, false),
    billboard: {
      image                   : buildPlaneIcon(hex),
      width: 32, height: 32,
      verticalOrigin          : Cesium.VerticalOrigin.CENTER,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
    label: {
      text        : '',
      font        : '11px "Courier New"',
      fillColor   : color,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style       : Cesium.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset : new Cesium.Cartesian2(0, 28),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    },
  });

  state.trailEntity = viewer.entities.add({
    polyline: {
      positions: new Cesium.CallbackProperty(() => state.trailPts, false),
      width    : 2.5,
      material : new Cesium.PolylineGlowMaterialProperty({
        glowPower: 0.12,
        color,
      }),
    },
  });

  droneState[id] = state;
}

function ensureMeshEntity(fromId, toId) {
  const key = `${fromId}-${toId}`;
  if (meshState[key]) return;
  const ms = {
    pts     : [Cesium.Cartesian3.ZERO, Cesium.Cartesian3.ZERO],
    colorVal: Cesium.Color.TRANSPARENT,
  };
  ms.entity = viewer.entities.add({
    polyline: {
      positions: new Cesium.CallbackProperty(() => ms.pts, false),
      width    : 1.5,
      material : new Cesium.ColorMaterialProperty(
        new Cesium.CallbackProperty(() => ms.colorVal, false)),
    },
  });
  meshState[key] = ms;
}

// ── HUD: pre-create drone rows ────────────────────────────────────────────────
(function initHudRows() {
  const tbody = document.getElementById('drone-tbody');
  for (let i = 0; i < NUM_DRONES; i++) {
    const hex = DRONE_COLORS[i % DRONE_COLORS.length];
    const row = document.createElement('tr');
    row.id = `dr-${i}`;
    row.innerHTML =
      `<td style="color:${hex};font-weight:bold">${NUM_DRONES > 1 ? 'D'+i : 'Aircraft'}</td>` +
      `<td id="ds-${i}" class="status-wait">Waiting&hellip;</td>` +
      `<td id="da-${i}">&#8212;</td>` +
      `<td id="dv-${i}">&#8212;</td>` +
      `<td id="dw-${i}">&#8212;</td>`;
    tbody.appendChild(row);
  }
})();

function updateHudRow(id, complete, p) {
  const statusEl = document.getElementById(`ds-${id}`);
  if (p === null) return;  // not yet started
  statusEl.textContent = complete ? '\u2705 Done' : '\u2708 Flying';
  statusEl.className   = complete ? 'status-done' : 'status-fly';
  document.getElementById(`da-${id}`).textContent = (p.alt||0).toFixed(0) + '\u00A0m';
  document.getElementById(`dv-${id}`).textContent = (p.spd||0).toFixed(1) + '\u00A0m/s';
  document.getElementById(`dw-${id}`).textContent = WP_NAMES[p.wp] || ('WP' + p.wp);
}

// ── Polling loop ──────────────────────────────────────────────────────────────
let initialZoomDone = false;

function poll() {
  fetch('/telemetry')
    .then(r => r.json())
    .then(data => {
      const drones    = data.drones     || {};
      const meshLinks = data.mesh_links || [];

      for (const [idStr, drone] of Object.entries(drones)) {
        const id  = parseInt(idStr);
        const pts = drone.positions || [];

        if (pts.length === 0) continue;

        ensureDroneEntities(id);
        const state = droneState[id];
        const p     = pts[pts.length - 1];
        const alt   = p.alt || 0;

        state.cartVal = Cesium.Cartesian3.fromDegrees(p.lon, p.lat, alt + 70);

        if (pts.length !== state.lastCount) {
          state.trailPts = pts.map(q =>
            Cesium.Cartesian3.fromDegrees(q.lon, q.lat, (q.alt || 0) + 70));
          state.lastCount = pts.length;
        }

        const altLabel = alt.toFixed(0) + '\u00A0m';
        state.acEntity.label.text =
          NUM_DRONES > 1 ? `D${id}\u00A0${altLabel}` : altLabel;

        updateHudRow(id, drone.complete, p);

        if (!initialZoomDone) {
          initialZoomDone = true;
          viewer.camera.flyTo({
            destination : Cesium.Cartesian3.fromDegrees(p.lon, p.lat, 300000),
            orientation : { heading: 0, pitch: Cesium.Math.toRadians(-50), roll: 0 },
            duration    : 3,
          });
        }
      }

      // Update mesh-link polylines
      for (const link of meshLinks) {
        ensureMeshEntity(link.from, link.to);
        const fs = droneState[link.from];
        const ts = droneState[link.to];
        const ms = meshState[`${link.from}-${link.to}`];
        if (fs && ts && ms) {
          ms.pts      = [fs.cartVal, ts.cartVal];
          ms.colorVal = link.in_range
            ? Cesium.Color.GREEN.withAlpha(0.55)
            : Cesium.Color.RED.withAlpha(0.65);
        }
      }
    })
    .catch(() => {});
}
setInterval(poll, 800);

// ── SVG plane icon ────────────────────────────────────────────────────────────
function buildPlaneIcon(hex) {
  hex = hex || '#38bdf8';
  const svg =
    '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64" width="64" height="64">' +
    '<g transform="rotate(-45 32 32)">' +
    '<polygon points="32,4 40,28 60,32 40,36 32,60 24,36 4,32 24,28"' +
    ' fill="' + hex + '" stroke="#000" stroke-width="2"/>' +
    '</g></svg>';
  return 'data:image/svg+xml;base64,' + btoa(svg);
}
</script>
</body>
</html>
"""


# ── HTTP server ───────────────────────────────────────────────────────────────

class TelemetryHandler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass   # suppress per-request access log

    def log_error(self, *_):
        pass   # suppress default error messages (we handle them below

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            html = (HTML
                .replace('__TITLE__',      MISSION_TITLE)
                .replace('__SPEEDUP__',    str(SPEEDUP))
                .replace('__HOME_LAT__',   str(HOME_LAT))
                .replace('__HOME_LON__',   str(HOME_LON))
                .replace('__ROUTE_LAT__',  str(round(ROUTE_LAT, 4)))
                .replace('__ROUTE_LON__',  str(round(ROUTE_LON, 4)))
                .replace('__WAYPOINTS__',  json.dumps(MISSION_WAYPOINTS))
                .replace('__WPNAMES__',    json.dumps(WP_NAMES))
                .replace('__NUM_DRONES__', str(NUM_DRONES))
                .replace('__MESH_RANGE__', str(MESH_RANGE_M))
            )
            self._send(200, 'text/html; charset=utf-8', html.encode('utf-8'))

        elif self.path == '/telemetry':
            with _swarm_lock:
                snap = {k: {'positions': list(v['positions']),
                            'complete' : v['complete']}
                        for k, v in _swarm.items()}

            # Compute mesh links (all pairwise combinations)
            drone_ids  = sorted(snap.keys())
            mesh_links = []
            for i in range(len(drone_ids)):
                for j in range(i + 1, len(drone_ids)):
                    pi = snap[drone_ids[i]]['positions']
                    pj = snap[drone_ids[j]]['positions']
                    if pi and pj:
                        a, b = pi[-1], pj[-1]
                        dist = _haversine(a['lat'], a['lon'], b['lat'], b['lon'])
                        mesh_links.append({
                            'from'    : drone_ids[i],
                            'to'      : drone_ids[j],
                            'dist_m'  : round(dist, 0),
                            'in_range': dist <= MESH_RANGE_M,
                        })

            payload = json.dumps({
                'drones'    : snap,
                'mesh_links': mesh_links,
                'mesh_range': MESH_RANGE_M,
            }).encode()
            self._send(200, 'application/json', payload,
                       [('Access-Control-Allow-Origin', '*')])

        else:
            self.send_error(404)

    def _send(self, code, ctype, body, extra_headers=None):
        try:
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            for k, v in (extra_headers or []):
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # client disconnected before response was fully sent


class _QuietHTTPServer(HTTPServer):
    """HTTPServer that silently drops broken-pipe / client-disconnect errors."""
    allow_reuse_address = True   # allow immediate rebind after previous run

    _IGNORE = ('BrokenPipeError', 'ConnectionResetError', 'ConnectionAbortedError')

    def handle_error(self, request, client_address):
        import traceback
        tb = traceback.format_exc()
        if any(name in tb for name in self._IGNORE):
            return   # client disconnected — not a real error
        print(f'[HTTP] Error from {client_address}:\n{tb}')


def start_web_server():
    # _http_server is already bound in main(); just serve requests.
    _http_server.serve_forever()
    # returns when _http_server.shutdown() is called


# ── SITL / MAVLink helpers ────────────────────────────────────────────────────

def connect(port, label='', retries=30):
    tag = f'[{label}] ' if label else ''
    print(f'{tag}Connecting to tcp:127.0.0.1:{port} ...')
    for i in range(retries):
        try:
            mav = mavutil.mavlink_connection(
                f'tcp:127.0.0.1:{port}', source_system=255)
            for _ in range(20):
                if mav.wait_heartbeat(timeout=3) and mav.target_system > 0:
                    break
            if mav.target_system   == 0: mav.target_system   = 1
            if mav.target_component == 0: mav.target_component = 1
            print(f'{tag}Connected. sysid={mav.target_system}')
            return mav
        except Exception as e:
            print(f'{tag}  attempt {i+1}/{retries}: {e}')
            time.sleep(1)
    raise RuntimeError(f'{tag}Could not connect to SITL on port {port}')


def parse_mission(filepath):
    """Parse a QGC WPL 110 mission file into a list of waypoint dicts."""
    with open(filepath) as fh:
        lines = fh.readlines()
    if not lines[0].strip().startswith('QGC WPL'):
        raise ValueError(f'{filepath} is not a QGC WPL file')
    waypoints = []
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


def upload_mission(mav, mission_file, label=''):
    """Upload mission via MAVLink protocol with stale-request deduplication."""
    from pymavlink import mavwp
    tag    = f'[{label}] ' if label else ''
    loader = mavwp.MAVWPLoader(target_system=mav.target_system,
                               target_component=mav.target_component)
    n = loader.load(mission_file)
    print(f'{tag}Uploading {n} waypoints ...')
    for i in range(n):
        wp = loader.wp(i)
        print(f'{tag}  WP{wp.seq}: cmd={wp.command} '
              f'lat={wp.x:.5f} lon={wp.y:.5f} alt={wp.z:.1f}m')

    # Clear any previously stored mission so stale WPs don't persist
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component,
                                   mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    clr = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    print(f'{tag}  Clear: {"OK" if clr and clr.type == 0 else str(clr)}')
    time.sleep(0.3)

    mav.mav.mission_count_send(mav.target_system, mav.target_component,
                               n, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    success   = False
    last_sent = -1   # highest seq we have already answered

    for _ in range(n * 6 + 20):
        msg = mav.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=5)
        if msg is None:
            print(f'{tag}  Timeout waiting for MISSION_REQUEST'); break

        if msg.get_type() == 'MISSION_ACK':
            success = (msg.type == 0)
            ack_str = 'ACCEPTED' if success else f'FAILED result={msg.type}'
            print(f'{tag}  Upload: {ack_str}')
            break

        req_seq  = msg.seq
        req_type = msg.get_type()

        # On TCP there is no packet loss — any re-request for an already-answered
        # seq is a stale queued duplicate.  Answering it rewinds the vehicle's
        # internal counter → MAV_MISSION_INVALID_SEQUENCE (13).
        if req_seq <= last_sent:
            print(f'{tag}  SKIP {req_type} seq={req_seq} (stale, last_sent={last_sent})')
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
        print(f'{tag}  → WP{wp.seq} cmd={wp.command} '
              f'lat={wp.x:.5f} lon={wp.y:.5f} alt={wp.z:.0f}m')

    if not success:
        print(f'{tag}  WARNING: upload may be incomplete')


def verify_mission(mav, label=''):
    """Download mission back from vehicle and print it."""
    tag = f'[{label}] ' if label else ''
    print(f'{tag}Verifying mission ...')
    mav.mav.mission_request_list_send(mav.target_system, mav.target_component)
    cnt = mav.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if cnt is None:
        print(f'{tag}  No MISSION_COUNT response'); return
    print(f'{tag}  Vehicle reports {cnt.count} waypoints:')
    for i in range(cnt.count):
        mav.mav.mission_request_int_send(mav.target_system, mav.target_component, i)
        wp = mav.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        if wp is None:
            print(f'{tag}  WP{i}: no response'); continue
        print(f'{tag}  WP{wp.seq}: cmd={wp.command} frame={wp.frame} '
              f'lat={wp.x/1e7:.5f} lon={wp.y/1e7:.5f} alt={wp.z:.1f}m')
    mav.mav.mission_ack_send(mav.target_system, mav.target_component,
                             0, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)


def set_mode(mav, mode_name, label=''):
    tag     = f'[{label}] ' if label else ''
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f'{tag}Mode {mode_name} not found!'); return
    mav.mav.set_mode_send(mav.target_system,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          mode_id)
    print(f'{tag}Mode → {mode_name} (id={mode_id})')


def arm(mav, label=''):
    tag = f'[{label}] ' if label else ''
    print(f'{tag}Arming ...')
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + 20
    while time.time() < deadline:
        msg = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == 0:
                print(f'{tag}Armed!'); return True
            print(f'{tag}  Arm refused (result={msg.result}), force-arming ...')
            time.sleep(1)
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0)
    print(f'{tag}Warning: arm timeout'); return False


# ── Per-drone flight monitor ──────────────────────────────────────────────────

def monitor_drone_flight(mav, n_waypoints, drone_id):
    """Record telemetry and detect mission completion for one drone."""
    label    = f'Drone {drone_id}'
    last_seq = n_waypoints - 1
    last_wp  = -1
    last_print  = 0.0
    last_record = 0.0

    print(f'\n[{label}] === FLIGHT IN PROGRESS (Speedup {SPEEDUP}×) ===')

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
            print(f'[{label}] >>> WP {msg.seq}: {WP_NAMES.get(msg.seq, f"WP {msg.seq}")}')

        elif mtype == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000
            spd = (msg.vx**2 + msg.vy**2)**0.5 / 100
            now = time.time()

            if now - last_record >= 0.5:
                with _swarm_lock:
                    _swarm[drone_id]['positions'].append({
                        'lat': round(lat, 6),
                        'lon': round(lon, 6),
                        'alt': round(alt, 1),
                        'spd': round(spd, 1),
                        'wp' : last_wp,
                        't'  : round(now, 2),
                    })
                last_record = now

            if now - last_print >= 5:
                print(f'[{label}]  {lat:.4f},{lon:.4f}  '
                      f'{alt:.0f}m  {spd:.0f} m/s  WP:{last_wp}')
                last_print = now

        elif mtype == 'STATUSTEXT':
            print(f'[{label}]  MSG: {msg.text}')
            if 'Disarmed' in msg.text or 'landed' in msg.text.lower():
                with _swarm_lock:
                    _swarm[drone_id]['complete'] = True
                print(f'[{label}] === FLIGHT COMPLETE (statustext) ===')
                return

        elif mtype == 'HEARTBEAT':
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed and last_wp >= last_seq - 1:
                with _swarm_lock:
                    _swarm[drone_id]['complete'] = True
                print(f'[{label}] === FLIGHT COMPLETE (disarmed) ===')
                return


# ── Per-drone thread ──────────────────────────────────────────────────────────

def run_drone(drone_id, offset_wps, home_str):
    """Launch one SITL instance, upload mission, arm and monitor flight."""
    port  = SITL_PORT + drone_id * 10
    label = f'Drone {drone_id}'

    # Stagger launches so each SITL has time to bind its port
    if drone_id > 0:
        delay = LAUNCH_DELAY * drone_id
        print(f'[{label}] Waiting {delay:.1f}s before launch ...')
        time.sleep(delay)

    mission_path = write_temp_mission(offset_wps, drone_id)

    cmd = [
        BINARY,
        '--model',    MODEL,
        '--speedup',  str(SPEEDUP),
        '--defaults', DEFAULTS,
        '--sim-address=127.0.0.1',
        f'-I{drone_id}',
        '--home',     home_str,
    ]
    print(f'\n[{label}] Starting SITL on port {port}')
    print(f'[{label}] Home: {home_str}')
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # Drain SITL stdout → log (daemon so it dies when proc finishes)
    def _drain_sitl(p, lbl):
        for raw in iter(p.stdout.readline, b''):
            line = raw.decode('utf-8', errors='replace').rstrip()
            if line:
                try:
                    print(f'  [SITL/{lbl}] {line}')
                except Exception:
                    pass
    threading.Thread(target=_drain_sitl, args=(proc, label), daemon=True).start()

    try:
        time.sleep(5)
        mav = connect(port, label=label)

        print(f'[{label}] Waiting for vehicle init ...')
        time.sleep(5)
        for _ in range(50):
            mav.recv_match(blocking=True, timeout=0.1)

        upload_mission(mav, mission_path, label=label)
        time.sleep(1)
        verify_mission(mav, label=label)
        time.sleep(1)

        set_mode(mav, 'AUTO', label=label)
        time.sleep(1)
        arm(mav, label=label)
        time.sleep(2)

        monitor_drone_flight(mav, len(offset_wps), drone_id)

    except Exception as exc:
        print(f'[{label}] ERROR: {exc}')
        with _swarm_lock:
            _swarm[drone_id]['complete'] = True
    finally:
        proc.terminate()
        print(f'[{label}] SITL terminated.')


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description='ArduPlane SITL auto-flight with live Cesium.js map '
                    '(single-drone or swarm)')
    parser.add_argument(
        'mission', nargs='?',
        default=os.path.join(_SCRIPT_DIR, 'mission.txt'),
        help='Path to QGC WPL mission file (default: mission.txt next to this script)')
    parser.add_argument(
        '--speedup', type=int, default=50,
        help='Simulation speedup factor (default: 50)')
    parser.add_argument(
        '--port', type=int, default=5760,
        help='Base SITL MAVLink TCP port; drone i uses port + i*10 (default: 5760)')
    parser.add_argument(
        '--web-port', type=int, default=8765,
        help='Cesium HTTP server port (default: 8765)')
    parser.add_argument(
        '--drones', type=int, default=1,
        metavar='N',
        help='Number of drones in swarm (default: 1)')
    parser.add_argument(
        '--mesh-range', type=int, default=300,
        metavar='M',
        help='Max mesh radio range in metres (default: 300)')
    parser.add_argument(
        '--launch-delay', type=float, default=5.0,
        metavar='D',
        help='Seconds between sequential drone launches (default: 5.0)')
    return parser.parse_args()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    args = parse_args()

    # Early checks before anything else
    if not os.path.isfile(BINARY):
        sys.exit(
            f'ERROR: SITL binary not found:\n  {BINARY}\n'
            f'Build first:  ./waf configure --board sitl && ./waf plane')

    if not os.path.isfile(args.mission):
        sys.exit(f'ERROR: Mission file not found:\n  {args.mission}')

    # Open log file — all print() from here on also goes to logs/flight_*.log
    setup_logging()

    # Expose CLI values as module-level names used throughout
    MISSION_FILE = args.mission
    SPEEDUP      = args.speedup
    SITL_PORT    = args.port
    WEB_PORT     = args.web_port
    NUM_DRONES   = max(1, args.drones)
    MESH_RANGE_M = args.mesh_range
    LAUNCH_DELAY = args.launch_delay

    # Parse mission and build all derived config
    base_wps = parse_mission(MISSION_FILE)
    build_mission_config(base_wps)

    # Generate per-drone lateral / altitude offsets
    offsets = generate_swarm_offsets(NUM_DRONES, MESH_RANGE_M)

    print(f'\nSwarm plan: {NUM_DRONES} drone(s), mesh range {MESH_RANGE_M} m')
    perp = (ROUTE_BEARING + 90.0) % 360.0
    for i, (lat_off, alt_off) in enumerate(offsets):
        print(f'  Drone {i}: lateral={lat_off:+.1f} m  alt={alt_off:+.1f} m')

    # Pre-populate swarm dict so the HTTP server can serve all slots immediately
    for i in range(NUM_DRONES):
        _swarm[i] = {'positions': [], 'complete': False}

    # Bind the HTTP server in the MAIN thread so port conflicts fail immediately
    # with a clear message rather than silently dying in a background thread.
    try:
        _http_server = _QuietHTTPServer(('127.0.0.1', WEB_PORT), TelemetryHandler)
    except OSError as e:
        sys.exit(
            f'ERROR: Cannot bind web server to port {WEB_PORT}: {e}\n'
            f'  → Another instance may be running.  '
            f'Try: --web-port {WEB_PORT + 1}')

    # Hand serve_forever() off to a non-daemon thread (non-daemon so that
    # _http_server.shutdown() in the finally block runs before interpreter exit).
    threading.Thread(target=start_web_server, daemon=False).start()
    print(f'\nCesium map → http://127.0.0.1:{WEB_PORT}/')
    threading.Timer(2.0,
        lambda: webbrowser.open(f'http://127.0.0.1:{WEB_PORT}/')).start()

    # Build per-drone missions and home strings, then launch threads
    threads = []
    for i, (lat_off, alt_off) in enumerate(offsets):
        offset_wps = make_offset_mission(base_wps, lat_off, alt_off)
        home_lat, home_lon = _offset_latlon(HOME_LAT, HOME_LON, perp, lat_off)
        home_alt = HOME_ALT + alt_off
        home_str = f'{home_lat:.7f},{home_lon:.7f},{home_alt:.1f},{ROUTE_BEARING:.1f}'
        t = threading.Thread(
            target=run_drone,
            args=(i, offset_wps, home_str),
            daemon=False)
        t.start()
        threads.append(t)

    try:
        for t in threads:
            t.join()
        print('\n=== ALL DRONES COMPLETE ===')
    except KeyboardInterrupt:
        print('\n[!] Interrupted — shutting down.')
    finally:
        if _http_server:
            _http_server.shutdown()   # unblocks serve_forever() in its thread
        if _log_file and not _log_file.closed:
            _log_file.flush()
            _log_file.close()
