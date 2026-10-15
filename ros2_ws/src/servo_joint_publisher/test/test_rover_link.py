"""Tests for rover_link: the rover's replies, parsed the way both dashboards do.

The lines below are what src/rover.cpp and src/esp32_sensor_node.cpp print.
A fake rover on 127.0.0.1 answers PING and CMD:read like robot_protocol.h.
Run with pytest, or directly: python3 test/test_rover_link.py
"""

import os
import socket
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'servo_joint_publisher'))
import rover_link  # noqa: E402

ROVER_JSON = ('{"name":"Sensor Rover","count":42,"temp":24.0,"humidity":61.0,'
              '"accel":[0.12,-0.30,9.79],"gyro":[0.01,-0.02,0.00],"gas":812,'
              '"streaming":false,"interval_ms":2000,"ip":"192.168.1.42","uptime_ms":125000}')
# What src/rover.cpp sends before its DHT11 and MPU6050 have ever answered
ROVER_JSON_NO_SENSORS = ('{"name":"Sensor Rover","count":1,"temp":0.0,"humidity":0.0,'
                         '"accel":[0.00,0.00,0.00],"gyro":[0.00,0.00,0.00],"gas":0,'
                         '"streaming":false,"interval_ms":2000,"ip":"192.168.4.1","uptime_ms":3000}')


def fresh():
    return dict(rover_link.EMPTY_READINGS, raw_log=[])


def test_rover_reply():
    st = fresh()
    assert rover_link.parse_line(st, "ACK:SENSORS:" + ROVER_JSON) == ('rover', 'Sensor Rover')
    assert st["temp_dht"] == st["temperature"] == 24.0
    assert st["humidity"] == 61.0 and st["dht_error"] is False
    assert st["gas"] == 812 and st["air_quality"] == "Moderate"
    assert st["accel"] == {"x": 0.12, "y": -0.3, "z": 9.79}
    assert st["gyro"] == {"x": 0.01, "y": -0.02, "z": 0.0}
    assert st["imu_ok"] is True and st["raw_pitch"] is not None and st["roll"] is not None
    assert st["sample_count"] == 42 and st["uptime_ms"] == 125000
    assert st["rover_ip"] == "192.168.1.42"
    assert st["sensors_fitted"] == list(rover_link.ROVER_SENSORS)
    # Same JSON as a streamed TELEMETRY line or a STATUS reply
    for prefix in ("TELEMETRY:", "STATUS:"):
        assert rover_link.parse_line(fresh(), prefix + ROVER_JSON)[0] == 'rover'


def test_rover_zeros_are_no_reading():
    st = fresh()
    rover_link.parse_line(st, "ACK:SENSORS:" + ROVER_JSON_NO_SENSORS)
    assert st["temp_dht"] is None and st["humidity"] is None and st["dht_error"] is True
    assert st["accel"] is None and st["gyro"] is None and st["pitch"] is None
    assert st["imu_ok"] is False
    assert st["gas"] == 0          # 0 is a real MQ-5 reading


def test_imu_offsets():
    st = fresh()
    rover_link.parse_line(st, "ACK:SENSORS:" + ROVER_JSON)
    assert st["pitch"] != 0.0
    rover_link.parse_line(st, "ACK:SENSORS:" + ROVER_JSON, {"pitch": st["raw_pitch"], "roll": st["raw_roll"]})
    assert st["pitch"] == 0.0 and st["roll"] == 0.0


def test_protocol_lines():
    st = fresh()
    assert rover_link.parse_line(st, "ACK:PONG:Sensor_Rover") == ('pong', 'Sensor_Rover')
    assert rover_link.parse_line(st, "ACK:PONG") == ('pong', '')
    hb = "HEARTBEAT:NAME=Sensor_Rover | MODE=AUTO | IP=192.168.1.42 | RSSI=-61"
    assert rover_link.parse_line(st, hb) == ('heartbeat', 'Sensor_Rover')
    assert st["rover_ip"] == "192.168.1.42"
    assert rover_link.parse_line(st, "HEARTBEAT:NAME=Robot_Arm | MODE=AUTO | IP=10.0.0.9 | RSSI=AP") == ('heartbeat', 'Robot_Arm')
    assert st["rover_ip"] == "192.168.1.42"
    assert rover_link.parse_line(st, '[WARN] MPU6050 not found on SDA 21 / SCL 22') == (None, None)
    assert st["imu_ok"] is False
    assert rover_link.parse_line(st, 'ERR:UNKNOWN_TARGET:foo') == (None, None)
    # Another robot's STATUS reply carries no readings
    assert rover_link.parse_line(st, 'STATUS:{"name":"Robot_Arm","mode":"AUTO","ip":"10.0.0.9"}') == (None, None)


def test_sensor_node_text():
    """src/esp32_sensor_node.cpp's block."""
    st = fresh()
    block = ["========== SENSOR DATA ==========", "Temperature: 23.50 °C", "Humidity: 55.00 %",
             "MPU6050:", "Accel X: 0.20 | Y: -0.10 | Z: 9.70 m/s^2", "Gyro X: 0.01 | Y: 0.02 | Z: -0.03 rad/s",
             "MQ-5 Analog: 1650", "================================="]
    kinds = [rover_link.parse_line(st, line)[0] for line in block]
    assert kinds == [None, 'reading', 'reading', None, 'reading', 'reading', 'reading', None]
    assert st["temp_dht"] == 23.5 and st["humidity"] == 55.0
    assert st["accel"] == {"x": 0.2, "y": -0.1, "z": 9.7} and st["gyro"]["z"] == -0.03
    assert st["gas"] == 1650 and st["air_quality"] == "Danger"
    assert st["sensors_fitted"] == list(rover_link.ROVER_SENSORS)
    st = fresh()
    rover_link.parse_line(st, "DHT11: ERROR")
    assert st["dht_error"] is True


def test_older_node():
    st = fresh()
    for line in ("DHT11 temp: 22.0 C hum: 40.0", "BMP280 temp: 23.1 C pres: 1007.5",
                 "Gas: 300 Water: 700", "Flame: clear", "GPS: 12.971600, 77.594600 sats: 7", "Alert: OK"):
        assert rover_link.parse_line(st, line)[0] == 'reading', line
    assert (st["temp_dht"], st["humidity"], st["temp_bmp"], st["pressure"]) == (22.0, 40.0, 23.1, 1007.5)
    assert (st["gas"], st["water"], st["flame"], st["gps_sats"], st["alert"]) == (300, 700, 1, 7, "OK")
    assert st["sensors_fitted"] is None
    st = fresh()
    assert rover_link.parse_json_text(st, '{"temp_dht":21.5,"humidity":30,"ax":0,"ay":0,"az":16384,"water":5,"sd_ok":true}')
    assert st["temp_dht"] == 21.5 and st["water"] == 5 and st["sd_ok"] is True
    assert abs(st["accel"]["z"] - 9.81) < 0.01
    assert not rover_link.parse_json_text(st, "<html>not json</html>")


class FakeRover:
    """Answers like robot_protocol.h + src/rover.cpp on a local UDP port."""

    def __init__(self, name="Sensor_Rover"):
        self.name = name
        self.got = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.port = self.sock.getsockname()[1]
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                data, src = self.sock.recvfrom(512)
            except OSError:
                return
            line = data.decode().strip()
            self.got.append(line)
            if line == "PING":
                reply = "ACK:PONG:" + self.name
            elif line == "CMD:read":
                reply = "ACK:SENSORS:" + ROVER_JSON
            else:
                reply = "ERR:UNKNOWN_PREFIX:" + line
            self.sock.sendto(reply.encode(), src)

    def close(self):
        self.sock.close()


def test_wifi_udp():
    rover = FakeRover()
    link = rover_link.WifiRover(f"127.0.0.1:{rover.port}")
    try:
        st = fresh()
        kind, reply = link.poll()
        assert (kind, reply) == ('line', "ACK:PONG:Sensor_Rover") and link.mode == 'udp'
        kind, reply = link.poll()
        assert kind == 'line' and rover_link.parse_line(st, reply)[0] == 'rover'
        assert st["temp_dht"] == 24.0
        assert rover.got == ["PING", "CMD:read"]
        assert "Sensor_Rover" in link.label
    finally:
        link.close()
        rover.close()


def test_wifi_wrong_robot_gets_no_cmd():
    rover = FakeRover("Sesame_Quadruped")
    link = rover_link.WifiRover(f"127.0.0.1:{rover.port}")
    try:
        for _ in range(2):
            try:
                link.poll()
                raise AssertionError("expected WrongRobot")
            except rover_link.WrongRobot as e:
                assert "Sesame_Quadruped" in str(e)
        assert rover.got == ["PING", "PING"]       # never a CMD: line
    finally:
        link.close()
        rover.close()


def test_wifi_field_forms():
    link = rover_link.WifiRover("http://10.1.2.3/")
    assert link.mode == 'http' and link.url == "http://10.1.2.3/data" and link.udp is None
    link = rover_link.WifiRover("rover-sensors.local")
    assert (link.host, link.port, link.url) == ("rover-sensors.local", 8888, "http://rover-sensors.local/data")
    link.close()


class FakeSerial:
    """Just enough of serial.Serial for SerialRover."""

    def __init__(self, replies):
        self.replies = replies      # cmd -> reply line
        self.written = []
        self.rx = b""

    @property
    def in_waiting(self):
        return len(self.rx)

    def write(self, data):
        cmd = data.decode().strip()
        self.written.append(cmd)
        if cmd in self.replies:
            self.rx += (self.replies[cmd] + "\r\n").encode()

    def read(self, n):
        out, self.rx = self.rx[:n], self.rx[n:]
        return out


def run_serial(ser, steps):
    sr = rover_link.SerialRover(ser)
    st, notes = fresh(), []
    for _ in range(steps):
        for line in sr.read_lines():
            note = sr.saw(*rover_link.parse_line(st, line))
            if note:
                notes.append(note)
        sr._sent -= rover_link.PROBE_INTERVAL_S    # skip the wait between requests
    return sr, st, notes


def test_serial_rover():
    ser = FakeSerial({"PING": "ACK:PONG:Sensor_Rover", "CMD:read": "ACK:SENSORS:" + ROVER_JSON})
    sr, st, notes = run_serial(ser, 6)
    assert ser.written[0] == "PING" and "CMD:read" in ser.written
    assert sr.name == "Sensor_Rover" and st["temp_dht"] == 24.0
    assert len(notes) == 1 and "src/rover.cpp" in notes[0]


def test_serial_other_robot_gets_no_cmd():
    ser = FakeSerial({"PING": "ACK:PONG:Robot_Arm"})
    sr, st, notes = run_serial(ser, 6)
    assert ser.written == ["PING"] and sr.other == "Robot_Arm"


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print("ok", name)
