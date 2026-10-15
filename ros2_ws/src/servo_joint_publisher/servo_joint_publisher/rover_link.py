"""Rover (ESP32 sensor node) link and telemetry parsing.

The desktop app and the web server both read the rover through this module, so
they agree on what a reading means. It speaks what the boards already send, so
no firmware change is needed:

* ``src/rover.cpp`` (PlatformIO env ``rover``) uses the robot_protocol.h line
  protocol on USB serial (115200) and on UDP port 8888. On its own it sends
  only a ``HEARTBEAT:`` line on serial every 2 s, so the dashboards ask for
  readings:

  - ``PING`` gets ``ACK:PONG:Sensor_Rover``, which identifies the board.
  - ``CMD:read`` gets ``ACK:SENSORS:{json}``, with a fresh reading.

  The JSON keys are ``temp`` and ``humidity`` (DHT11), ``gas`` (MQ-5, 0-4095),
  ``accel`` and ``gyro`` ([x, y, z] in m/s² and rad/s, from the MPU6050), plus
  ``count``, ``interval_ms``, ``uptime_ms``, ``ip`` and ``name``. The firmware
  reports 0 for a sensor that has never answered; those zeros count as "no
  reading" here.
* ``src/esp32_sensor_node.cpp`` prints text blocks on serial every 2 s
  ("Temperature: 24.00 °C", "Accel X: … | Y: … | Z: …", "MQ-5 Analog: …").
* An older sensor node serves JSON at ``http://<ip>/data``, or prints
  "DHT11 temp: … C hum: …" lines. It also has a BMP280, water, flame, GPS and SD card.

robot_protocol.h answers PING itself on every robot that uses it, so a PING
can't move anything. ``CMD:`` lines reach the robot's own handler (on the Quard
Bot any of them feeds the motion watchdog), so ``CMD:read`` is only sent after
the PING reply named the rover.
"""

import json
import math
import re
import socket
import time
import urllib.error
import urllib.request

DEFAULT_HOST = "192.168.1.100"
DEFAULT_UDP_PORT = 8888
POLL_INTERVAL_S = 0.2       # CMD:read rate once the rover is identified
PROBE_INTERVAL_S = 1.0      # PING rate until then
REPLY_TIMEOUT_S = 1.0
HTTP_INTERVAL_S = 0.5       # /data rate (older sensor node)

# Sensors on the rover board that src/rover.cpp and src/esp32_sensor_node.cpp
# read: DHT11 on GPIO 4, MQ-5 on GPIO 34, MPU6050 on I2C 21/22. The other
# cards (BMP280, water, flame, GPS, SD card) belong to the older sensor node.
ROVER_SENSORS = ("dht11", "mq", "mpu6050")

# Readings before (or between) real data. None means "no reading", which the
# dashboards show as "--" instead of a made-up value.
EMPTY_READINGS = {
    "temperature": None, "temp_dht": None, "humidity": None, "temp_bmp": None, "pressure": None,
    "dht_error": None, "accel": None, "gyro": None, "pitch": None, "roll": None,
    "raw_pitch": None, "raw_roll": None, "gas": None, "water": None, "flame": None,
    "air_quality": None, "lat": None, "lng": None, "gps_valid": None, "gps_sats": None,
    "alert": None, "sd_ok": None, "uptime_ms": None,
    "imu_ok": None,             # False: the board reported no MPU6050
    "device": None,             # robot name, e.g. "Sensor_Rover"
    "rover_ip": None,           # the rover's own Wi-Fi address, as it reports it
    "sample_count": None,       # src/rover.cpp's reading counter
    "sensors_fitted": None,     # list of ROVER_SENSORS when the board is known, else None (show every card)
}

_JSON_PREFIXES = ("ACK:SENSORS:", "TELEMETRY:", "STATUS:")
_PONG_RE = re.compile(r"^ACK:PONG(?::(.*))?$", re.I)
_HEARTBEAT_RE = re.compile(r"^HEARTBEAT:NAME=([^|]*?)\s*(?:\|(.*))?$", re.I)
_NUM = r"(-?\d+(?:\.\d+)?)"


def is_rover_name(name):
    return bool(name) and re.search(r"rover", name, re.I) is not None


def air_quality(gas):
    return "Clean" if gas < 600 else ("Moderate" if gas < 1500 else "Danger")


def data_url(text):
    """http://<host>/data from the Wi-Fi field (a host, IP or URL)."""
    cur = (text or "").strip() or DEFAULT_HOST
    if not cur.startswith(("http://", "https://")):
        cur = "http://" + cur
    cur = cur.rstrip('/')
    return cur if cur.endswith('/data') else cur + "/data"


def _num(v):
    if isinstance(v, bool):
        return None
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _reading(d, keys, cast=float):
    """First present, finite value among keys in a JSON reply, else None."""
    for k in keys:
        if d.get(k) is None:
            continue
        v = _num(d[k])
        if v is None:
            return None
        return int(v) if cast is int else v
    return None


def _vec3(v):
    """[x, y, z] or {"x", "y", "z"} → (x, y, z), else None."""
    if isinstance(v, dict):
        v = [v.get(k) for k in ("x", "y", "z")]
    if isinstance(v, (list, tuple)) and len(v) == 3:
        xyz = tuple(_num(c) for c in v)
        if None not in xyz:
            return xyz
    return None


def _set_imu(st, accel, gyro, imu_offsets):
    """Store accel (m/s²) and gyro (rad/s), and the tilt the accelerometer gives."""
    if accel is not None:
        ax, ay, az = accel
        st["accel"] = {"x": round(ax, 2), "y": round(ay, 2), "z": round(az, 2)}
        if ax or ay or az:
            raw_p = round(math.degrees(math.atan2(ay, math.sqrt(ax ** 2 + az ** 2))), 1)
            raw_r = round(math.degrees(math.atan2(-ax, az)), 1)
            offsets = imu_offsets or {}
            st["raw_pitch"], st["raw_roll"] = raw_p, raw_r
            st["pitch"] = round(raw_p - float(offsets.get("pitch", 0.0)), 1)
            st["roll"] = round(raw_r - float(offsets.get("roll", 0.0)), 1)
    if gyro is not None:
        st["gyro"] = {"x": round(gyro[0], 3), "y": round(gyro[1], 3), "z": round(gyro[2], 3)}


def _log_raw(st, text):
    log = st.get("raw_log")
    if isinstance(log, list):
        log.append(text)
        del log[:-100]


def is_rover_json(d):
    """True for the JSON src/rover.cpp sends (getSensorJson)."""
    return is_rover_name(str(d.get("name", ""))) or (isinstance(d.get("accel"), list) and "temp" in d)


def apply_rover_json(st, d, imu_offsets=None):
    """Apply a src/rover.cpp reading (the JSON after ACK:SENSORS:, TELEMETRY: or STATUS:)."""
    st["sensors_fitted"] = list(ROVER_SENSORS)
    if d.get("name") is not None:
        st["device"] = str(d["name"])
    if d.get("ip"):
        st["rover_ip"] = str(d["ip"])
    for key, src in (("sample_count", "count"), ("uptime_ms", "uptime_ms")):
        v = _reading(d, (src,), int)
        if v is not None:
            st[key] = v

    # The firmware starts DHT11 at 0 °C / 0 % and keeps the last good value when
    # a read fails, so 0 and 0 together means the DHT11 has never answered.
    t, h = _num(d.get("temp")), _num(d.get("humidity"))
    if t is not None and h is not None:
        if t == 0 and h == 0:
            st["dht_error"] = True
        else:
            st["temp_dht"] = st["temperature"] = t
            st["humidity"] = h
            st["dht_error"] = False

    gas = _reading(d, ("gas",), int)
    if gas is not None:
        st["gas"] = gas
        st["air_quality"] = air_quality(gas)

    # Already in m/s² and rad/s (Adafruit_MPU6050). All zeros means no MPU6050:
    # a working one always feels gravity.
    accel, gyro = _vec3(d.get("accel")), _vec3(d.get("gyro"))
    if accel is not None and not any(accel) and (gyro is None or not any(gyro)):
        st["imu_ok"] = False
    elif accel is not None:
        st["imu_ok"] = True
        _set_imu(st, accel, gyro, imu_offsets)


def apply_legacy_json(st, d, imu_offsets=None):
    """Apply an older sensor node's /data JSON. Only fields present change.
    Returns the number of fields applied."""
    hits = 0
    for key, keys, cast in (("temp_dht", ("temp_dht", "temperature"), float),
                            ("humidity", ("humidity",), float),
                            ("temp_bmp", ("temp_bmp",), float),
                            ("pressure", ("pressure",), float),
                            ("gas", ("air", "gas"), int),
                            ("water", ("water",), int),
                            ("flame", ("flame",), int),
                            ("lat", ("lat",), float),
                            ("lng", ("lng",), float),
                            ("gps_sats", ("gps_sats",), int),
                            ("uptime_ms", ("uptime_ms",), int)):
        v = _reading(d, keys, cast)
        if v is not None:
            st[key] = v
            hits += 1
    if st["temp_dht"] is not None:
        st["temperature"] = st["temp_dht"]
    if st["gas"] is not None:
        st["air_quality"] = air_quality(st["gas"])
    for key in ("gps_valid", "sd_ok", "dht_error"):
        if key in d and d[key] is not None:
            st[key] = bool(d[key])
            hits += 1
    if d.get("alert") is not None:
        st["alert"] = str(d["alert"]).upper()
        hits += 1

    accel = [_reading(d, (k,)) for k in ("ax", "ay", "az")]
    gyro = [_reading(d, (k,)) for k in ("gx", "gy", "gz")]
    accel = None if None in accel else accel
    gyro = None if None in gyro else gyro
    if accel and any(abs(v) > 50 for v in accel):        # raw MPU6050 LSB (16384/g)
        accel = [(v / 16384.0) * 9.80665 for v in accel]
    if gyro and any(abs(v) > 50 for v in gyro):          # raw LSB (131 per deg/s)
        gyro = [math.radians(v / 131.0) for v in gyro]
    if accel or gyro:
        _set_imu(st, accel, gyro, imu_offsets)
        hits += 1
    return hits


def parse_json_text(st, raw, imu_offsets=None):
    """Apply a JSON reply (an HTTP /data page). Returns False if it isn't a JSON object."""
    try:
        d = json.loads(raw)
    except ValueError:
        return False
    if not isinstance(d, dict):
        return False
    _log_raw(st, f"[JSON] {raw[:90]}...")
    if is_rover_json(d):
        apply_rover_json(st, d, imu_offsets)
    else:
        apply_legacy_json(st, d, imu_offsets)
    return True


def parse_line(st, line, imu_offsets=None):
    """Apply one line from the rover: a serial line or a UDP reply.

    Returns (kind, name). kind is:
    - 'rover': a src/rover.cpp reading
    - 'reading': a reading in another format
    - 'pong' or 'heartbeat': name is the robot's name
    - None: anything else (boot messages, ERR: replies)
    """
    line = line.strip()
    if not line:
        return None, None
    _log_raw(st, line)

    m = _PONG_RE.match(line)
    if m:
        return 'pong', (m.group(1) or '').strip()
    m = _HEARTBEAT_RE.match(line)
    if m:
        name = m.group(1).strip()
        ip = re.search(r"IP=([\d.]+)", m.group(2) or '')
        if ip and is_rover_name(name):
            st["rover_ip"] = ip.group(1)
        return 'heartbeat', name

    body = None
    for prefix in _JSON_PREFIXES:
        if line.upper().startswith(prefix):
            body = line[len(prefix):]
            break
    if body is None and line.startswith('{'):
        body = line
    if body is not None:
        try:
            d = json.loads(body)
        except ValueError:
            return None, None
        if not isinstance(d, dict):
            return None, None
        if is_rover_json(d):
            apply_rover_json(st, d, imu_offsets)
            return 'rover', st.get("device")
        return ('reading' if apply_legacy_json(st, d, imu_offsets) else None), None

    if re.search(r"MPU6050 not found", line, re.I):
        st["imu_ok"] = False
        return None, None
    if re.search(r"MPU6050 (?:OK|initialized)", line, re.I):
        st["imu_ok"] = True
        return None, None
    return ('reading' if _parse_text(st, line, imu_offsets) else None), None


def _parse_text(st, line, imu_offsets):
    """Text readings from src/esp32_sensor_node.cpp and the older sensor node.
    Returns True if the line held one."""
    hit = False
    if "MQ-5 Analog:" in line or "SENSOR DATA" in line:
        st["sensors_fitted"] = list(ROVER_SENSORS)      # esp32_sensor_node.cpp

    # 1. DHT11
    if "DHT11: ERROR" in line:
        st["dht_error"] = True
        hit = True
    else:
        m = re.search(r"(?:Temperature:|DHT11\s+temp:)\s*" + _NUM, line, re.I)
        if m:
            st["temperature"] = st["temp_dht"] = float(m.group(1))
            st["dht_error"] = False
            hit = True
        m = re.search(r"(?:Humidity:|hum:)\s*" + _NUM, line, re.I)
        if m:
            st["humidity"] = float(m.group(1))
            st["dht_error"] = False
            hit = True

    # 2. BMP280
    m = re.search(r"BMP280\s+temp:\s*" + _NUM + r"\s*C\s+pres:\s*" + _NUM, line, re.I)
    if m:
        st["temp_bmp"], st["pressure"] = float(m.group(1)), float(m.group(2))
        hit = True

    # 3. Gas and water
    m = re.search(r"(?:MQ-5 Analog:|Gas:)\s*(\d+)", line, re.I)
    if m:
        st["gas"] = int(m.group(1))
        st["air_quality"] = air_quality(st["gas"])
        hit = True
    m = re.search(r"Water:\s*(\d+)", line, re.I)
    if m:
        st["water"] = int(m.group(1))
        hit = True

    # 4. Flame
    if "Flame:" in line:
        if "FIRE" in line:
            st["flame"], st["alert"] = 0, "FIRE"
        else:
            st["flame"] = 1
        hit = True

    # 5. MPU6050: "Accel X: 0.1 | Y: 0.2 | Z: 9.8 m/s^2" or "Accel: X=… Y=… Z=…"
    xyz = r"\s*" + _NUM + r"(?:\s*\|\s*Y:|\s+Y=)\s*" + _NUM + r"(?:\s*\|\s*Z:|\s+Z=)\s*" + _NUM
    m = re.search(r"(?:Accel X:|Accel:\s*X=)" + xyz, line, re.I)
    if m:
        accel = [float(g) for g in m.groups()]
        if "m/s" not in line and any(abs(v) > 50 for v in accel):    # raw LSB (16384/g)
            accel = [(v / 16384.0) * 9.80665 for v in accel]
        st["imu_ok"] = True
        _set_imu(st, accel, None, imu_offsets)
        hit = True
    m = re.search(r"(?:Gyro X:|Gyro:\s*X=)" + xyz, line, re.I)
    if m:
        gyro = [float(g) for g in m.groups()]
        if "rad/s" not in line and any(abs(v) > 20 for v in gyro):   # raw LSB (131 per deg/s)
            gyro = [math.radians(v / 131.0) for v in gyro]
        _set_imu(st, None, gyro, imu_offsets)
        hit = True

    # 6. GPS
    m = re.search(r"GPS:\s*" + _NUM + r",\s*" + _NUM + r"\s+sats:\s*(\d+)", line, re.I)
    if m:
        st["lat"], st["lng"] = float(m.group(1)), float(m.group(2))
        st["gps_sats"], st["gps_valid"] = int(m.group(3)), True
        hit = True
    elif re.search(r"GPS:\s*no fix", line, re.I):
        st["gps_valid"] = False
        hit = True

    # 7. Alert
    m = re.search(r"Alert:\s*(\w+)", line, re.I)
    if m:
        st["alert"] = m.group(1).upper()
        hit = True
    return hit


def describe_net_error(err):
    """Plain-language reason for a failed HTTP request (same wording as the dashboards)."""
    if isinstance(err, urllib.error.HTTPError):
        return f"HTTP {err.code} {err.reason}"
    reason = getattr(err, 'reason', err)
    if isinstance(reason, socket.gaierror):
        return "host name not found (a .local name needs mDNS; try the IP address)"
    if isinstance(reason, (socket.timeout, TimeoutError)) or 'timed out' in str(reason):
        return "no reply (timed out)"
    if isinstance(reason, ConnectionRefusedError):
        return "connection refused (board up, but its web server isn't running?)"
    if isinstance(reason, OSError) and getattr(reason, 'errno', None) in (101, 113, 10051, 10065):
        return "network unreachable (is this computer on the robot's Wi-Fi?)"
    return str(reason)


class RoverLinkError(Exception):
    """A failed poll. str() is a short reason for the link card."""


class WrongRobot(RoverLinkError):
    """Something answered, but it isn't the rover."""


class UdpRover:
    """Ask src/rover.cpp for readings over UDP: PING until it answers as the
    rover, then CMD:read. Replies come back to this socket's own port, so
    nothing else that uses port 8888 on this computer is in the way."""

    def __init__(self, host, port=DEFAULT_UDP_PORT):
        self.host, self.port = host, port
        self.addr = None        # resolved IPv4 address
        self.name = None        # robot name from ACK:PONG, once it is the rover
        self.sock = None        # opened on the first poll

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _drain(self):
        """Drop late replies to earlier requests."""
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recvfrom(4096)
        except (BlockingIOError, ConnectionResetError):
            pass
        finally:
            self.sock.setblocking(True)

    def _ask(self, cmd, want):
        """Send cmd; return the first reply from the rover that starts with want."""
        self._drain()
        self.sock.sendto((cmd + "\n").encode("ascii"), (self.addr, self.port))
        deadline = time.monotonic() + REPLY_TIMEOUT_S
        other = None
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise RoverLinkError(f"{self.host} replied {other[:60]!r}" if other
                                     else f"no reply on UDP {self.port}")
            self.sock.settimeout(left)
            try:
                data, src = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except ConnectionResetError:    # Windows: ICMP port unreachable
                raise RoverLinkError(f"nothing listening on UDP {self.port} at {self.host}")
            if src[0] != self.addr:
                continue
            text = data.decode("utf-8", errors="ignore").strip()
            if text.upper().startswith(want):
                return text
            other = text

    def poll(self):
        """One request. Returns the reply line; raises RoverLinkError."""
        if self.sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", 0))  # Windows can't recvfrom() on an unbound socket
        if self.addr is None:
            try:
                self.addr = socket.getaddrinfo(self.host, self.port, socket.AF_INET,
                                               socket.SOCK_DGRAM)[0][4][0]
            except (socket.gaierror, UnicodeError):
                raise RoverLinkError("host name not found (a .local name needs mDNS; try the IP address)")
        try:
            if self.name is None:
                reply = self._ask("PING", "ACK:PONG")
                name = (_PONG_RE.match(reply).group(1) or '').strip()
                if not is_rover_name(name):
                    raise WrongRobot(f"{self.host} answers as {name or 'another robot'}, not the rover")
                self.name = name
                return reply
            return self._ask("CMD:read", "ACK:SENSORS:")
        except WrongRobot:
            raise
        except RoverLinkError:
            # Look it up and identify it again next time: DHCP may have moved it.
            self.addr = self.name = None
            raise
        except OSError as e:
            self.addr = self.name = None
            raise RoverLinkError(describe_net_error(e))


class WifiRover:
    """The rover over Wi-Fi, from what the Wi-Fi field holds:

    - a host or IP (or host:port): the rover protocol over UDP, falling back
      to an HTTP /data page if nothing answers on UDP;
    - an http:// URL: that /data page only.
    """

    def __init__(self, text):
        text = (text or "").strip() or DEFAULT_HOST
        self.udp = None
        self.mode = None            # 'udp' or 'http' once one has answered
        if "://" in text or "/" in text:
            self.url, self.mode = data_url(text), 'http'
            return
        host, port = text, DEFAULT_UDP_PORT
        h, sep, p = text.rpartition(":")
        if sep and h and ":" not in h and p.isdigit():
            host, port = h, int(p)
        self.host, self.port = host, port
        self.udp = UdpRover(host, port)
        self.url = data_url(host)

    @property
    def label(self):
        if self.mode == 'http':
            return self.url
        if self.mode == 'udp':
            return f"{self.udp.name or 'rover'} at {self.host}:{self.port} (UDP)"
        return f"{self.host} (UDP {self.port}, or HTTP /data)"

    @property
    def interval(self):
        return POLL_INTERVAL_S if self.mode == 'udp' else HTTP_INTERVAL_S

    def close(self):
        if self.udp is not None:
            self.udp.close()

    def poll(self):
        """One request. Returns ('line', reply) for the rover protocol or
        ('json', body) for an HTTP /data page; raises RoverLinkError."""
        udp_err = None
        if self.udp is not None and self.mode != 'http':
            try:
                reply = self.udp.poll()
                self.mode = 'udp'
                return 'line', reply
            except RoverLinkError as e:
                if self.mode == 'udp' or isinstance(e, WrongRobot):
                    raise
                udp_err = e
        try:
            req = urllib.request.Request(self.url, headers={"User-Agent": "RobotDashboard/1.0"})
            with urllib.request.urlopen(req, timeout=2.0) as resp:
                body = resp.read().decode('utf-8', errors='ignore')
        except Exception as e:
            if isinstance(e, urllib.error.HTTPError) and e.code == 404:
                why = f"HTTP 404: no /data page at {self.url}"
            else:
                why = f"HTTP /data: {describe_net_error(e)}"
            raise RoverLinkError(f"{udp_err}; {why}" if udp_err else why)
        self.mode = 'http'
        return 'json', body


class SerialRover:
    """Line reader for the rover's serial port that also asks for readings.

    Until the board says who it is, it sends PING once a second. Once the board
    answers (or sends a heartbeat) as the rover, it sends CMD:read 5 times a
    second, one at a time. A board that answers as another robot, or that prints
    readings on its own (src/esp32_sensor_node.cpp), is only listened to.
    """

    def __init__(self, ser):
        self.ser = ser              # an open serial.Serial with a short timeout
        self.name = None
        self.other = None
        self.passive = False
        self._buf = b""
        self._sent = 0.0
        self._pending = False

    def _due(self, now):
        if self.name:
            if now - self._sent >= (REPLY_TIMEOUT_S if self._pending else POLL_INTERVAL_S):
                self._sent, self._pending = now, True
                return "CMD:read"
        elif not (self.other or self.passive) and now - self._sent >= PROBE_INTERVAL_S:
            self._sent = now
            return "PING"
        return None

    def read_lines(self):
        """Send whatever request is due, then return the complete lines received."""
        cmd = self._due(time.monotonic())
        if cmd:
            self.ser.write((cmd + "\n").encode("ascii"))
        chunk = self.ser.read(self.ser.in_waiting or 1)
        if not chunk:
            return []
        self._buf += chunk
        *lines, self._buf = self._buf.split(b"\n")
        if len(self._buf) > 4096:       # no newline at all: wrong baud rate
            self._buf = b""
        return [t for t in (b.decode("utf-8", errors="ignore").strip() for b in lines) if t]

    def saw(self, kind, name):
        """Feed back what parse_line made of a line. Returns a note for the
        console when it learns what the board is."""
        if kind == 'rover' or (kind in ('pong', 'heartbeat') and is_rover_name(name)):
            if kind == 'rover':
                self._pending = False
            if not self.name:
                self.name = name or "Sensor_Rover"
                return f"{self.name} (src/rover.cpp) found: asking it for readings {1 / POLL_INTERVAL_S:.0f}× a second"
        elif kind in ('pong', 'heartbeat') and not self.name and self.other is None:
            self.other = name or "another robot"
            return f"this port answers as {self.other}, not the rover"
        elif kind == 'reading' and not self.name and not self.passive:
            self.passive = True
            return "the board prints readings by itself: listening only"
        return None
