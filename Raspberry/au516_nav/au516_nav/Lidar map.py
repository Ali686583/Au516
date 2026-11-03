"""
lidar_map.py : affiche en direct ce que le LiDAR voit, dans un navigateur web.

Principe :
  1. Le noeud s'abonne a /scan et convertit chaque mesure (distance, angle) en point (x, y).
  2. Il lance un mini serveur web sur le Pi (port 8000 par defaut).
  3. Depuis le PC (connecte au WiFi du robot), on ouvre http://192.168.5.1:8000
     et la page dessine les points en vue de dessus, avec le robot au centre.

Repere : x positif = devant le robot (haut de l'ecran), y positif = gauche (gauche de l'ecran).

Lancement (apres avoir lance le driver LiDAR dans un autre terminal) :
    ros2 run au516_nav lidar_map
    ros2 run au516_nav lidar_map --ros-args -p port:=8080     # pour changer de port
"""

import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


PAGE = r"""<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<title>Carte LiDAR - AU516</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  html, body { margin: 0; height: 100%; background: #111; color: #ddd;
               font-family: system-ui, Arial, sans-serif; overflow: hidden; }
  canvas { display: block; width: 100%; height: 100%; }
  #hud { position: fixed; top: 10px; left: 10px; background: rgba(0,0,0,0.6);
         padding: 8px 12px; border-radius: 6px; font-size: 14px; line-height: 1.5; }
  #hud button { background: #333; color: #ddd; border: 1px solid #666; border-radius: 4px;
                padding: 2px 10px; margin-right: 4px; cursor: pointer; font-size: 14px; }
  #hud button:hover { background: #444; }
  #status { color: #8f8; }
  #status.err { color: #f88; }
</style>
</head>
<body>
<canvas id="c"></canvas>
<div id="hud">
  <div><strong>Carte LiDAR</strong> &mdash; <span id="status">connexion…</span></div>
  <div>Points : <span id="n">0</span> &nbsp;|&nbsp; Échelle : <span id="sc">0</span> px/m</div>
  <div>
    <button id="zin">Zoom +</button><button id="zout">Zoom −</button>
    <button id="pause">Pause</button>
  </div>
  <div style="font-size:12px;color:#999">Molette = zoom. Le robot est au centre, l'avant est en haut.</div>
</div>
<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
let scale = 100;          // pixels par metre
let points = [];
let rangeMax = 12;
let paused = false;

function resize() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * dpr;
  canvas.height = canvas.clientHeight * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resize);
resize();

// Repere ecran : x (devant) vers le haut, y (gauche) vers la gauche
function toScreen(x, y) {
  const cx = canvas.clientWidth / 2, cy = canvas.clientHeight / 2;
  return [cx - y * scale, cy - x * scale];
}

function drawGrid() {
  const cx = canvas.clientWidth / 2, cy = canvas.clientHeight / 2;
  const step = scale >= 150 ? 0.5 : (scale >= 40 ? 1 : 2);
  ctx.lineWidth = 1;
  ctx.font = '12px system-ui, Arial';
  for (let r = step; r * scale < Math.max(cx, cy) * 1.5 && r <= rangeMax; r += step) {
    ctx.strokeStyle = '#2a2a2a';
    ctx.beginPath(); ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = '#666';
    ctx.fillText(r + ' m', cx + 4, cy - r * scale - 3);
  }
  ctx.strokeStyle = '#333';
  ctx.beginPath();
  ctx.moveTo(cx, 0); ctx.lineTo(cx, canvas.clientHeight);
  ctx.moveTo(0, cy); ctx.lineTo(canvas.clientWidth, cy);
  ctx.stroke();
  ctx.fillStyle = '#777';
  ctx.fillText('AVANT', cx + 6, 16);
  ctx.fillText('GAUCHE', 6, cy - 6);
  ctx.fillText('DROITE', canvas.clientWidth - 56, cy - 6);
}

function drawRobot() {
  const cx = canvas.clientWidth / 2, cy = canvas.clientHeight / 2;
  const w = 0.10 * scale, l = 0.15 * scale;   // petit gabarit indicatif
  ctx.fillStyle = '#4da3ff';
  ctx.beginPath();
  ctx.moveTo(cx, cy - l);
  ctx.lineTo(cx - w, cy + l * 0.6);
  ctx.lineTo(cx + w, cy + l * 0.6);
  ctx.closePath();
  ctx.fill();
}

function drawPoints() {
  for (const p of points) {
    const x = p[0], y = p[1];
    const d = Math.hypot(x, y);
    const hue = Math.max(0, 120 - (d / 4) * 120);   // rouge (proche) -> vert (loin)
    ctx.fillStyle = 'hsl(' + hue + ', 90%, 55%)';
    const s = toScreen(x, y);
    ctx.fillRect(s[0] - 1.5, s[1] - 1.5, 3, 3);
  }
}

function draw() {
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  drawGrid();
  drawPoints();
  drawRobot();
  document.getElementById('n').textContent = points.length;
  document.getElementById('sc').textContent = Math.round(scale);
}

async function poll() {
  const st = document.getElementById('status');
  if (!paused) {
    try {
      const r = await fetch('/points', { cache: 'no-store' });
      const data = await r.json();
      points = data.points;
      rangeMax = data.range_max || rangeMax;
      st.textContent = points.length ? 'en direct' : 'en attente de /scan…';
      st.className = '';
    } catch (e) {
      st.textContent = 'connexion perdue';
      st.className = 'err';
    }
    draw();
  }
  setTimeout(poll, 80);
}

function zoom(f) { scale = Math.min(600, Math.max(10, scale * f)); draw(); }
document.getElementById('zin').onclick = () => zoom(1.25);
document.getElementById('zout').onclick = () => zoom(0.8);
document.getElementById('pause').onclick = (ev) => {
  paused = !paused;
  ev.target.textContent = paused ? 'Reprendre' : 'Pause';
};
canvas.addEventListener('wheel', (ev) => { ev.preventDefault(); zoom(ev.deltaY < 0 ? 1.1 : 0.9); }, { passive: false });

poll();
</script>
</body>
</html>
"""


def convertir_scan(msg):
    """Convertit un LaserScan en liste de points (x, y) en metres, en ignorant les mesures invalides."""
    points = []

    for i, distance in enumerate(msg.ranges):
        if not math.isfinite(distance):
            continue

        if not msg.range_min <= distance <= msg.range_max:
            continue

        angle = msg.angle_min + i * msg.angle_increment

        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        points.append((x, y))

    return points


class MapRequestHandler(BaseHTTPRequestHandler):
    """Repond a deux adresses : / (la page web) et /points (les derniers points en JSON)."""

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        elif self.path.startswith("/points"):
            points, range_max = self.server.get_data()
            body = json.dumps({
                "points": [[round(x, 3), round(y, 3)] for (x, y) in points],
                "range_max": range_max,
            }).encode("utf-8")
            self._send(200, "application/json", body)
        else:
            self._send(404, "text/plain; charset=utf-8", b"Not found")

    def _send(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        pass    # evite d'inonder le terminal avec une ligne par requete


class LidarMap(Node):
    def __init__(self):
        super().__init__("lidar_map")

        self.declare_parameter("port", 8000)
        port = self.get_parameter("port").get_parameter_value().integer_value

        self.points = []
        self.range_max = 12.0

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_cb,
            qos_profile_sensor_data
        )

        # Serveur web dans un thread a part, pour ne pas bloquer ROS 2
        self.server = ThreadingHTTPServer(("0.0.0.0", port), MapRequestHandler)
        self.server.get_data = lambda: (self.points, self.range_max)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f"Carte disponible sur http://<ip_du_pi>:{port}  (ex : http://192.168.5.1:{port})")

    def scan_cb(self, msg):
        self.points = convertir_scan(msg)
        self.range_max = float(msg.range_max)

    def stop_server(self):
        self.server.shutdown()
        self.server.server_close()


def main():
    rclpy.init()
    node = LidarMap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()