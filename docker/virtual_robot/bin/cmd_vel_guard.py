#!/usr/bin/env python3
"""cmd_vel_guard: laatste schakel tussen twist_mux en turtlebot3_node.

    twist_mux -> /cmd_vel_mux -> cmd_vel_guard -> /cmd_vel_out -> turtlebot3_node

1. Watchdog. twist_mux stuurt enkel door wat binnenkomt en publiceert zelf
   nooit een stop; de OpenCR houdt de laatste snelheid vast zolang
   turtlebot3_node leeft. Valt de wifi weg terwijl een pijl ingedrukt is,
   crasht Nav2 of stopt een eigen node midden in een rit, dan reed de robot
   dus door. Komt er `timeout` s niets meer binnen terwijl de robot rijdt,
   dan publiceert de guard een nulsnelheid.
2. Maximale snelheid (docent, Beheer > Robot): plafond voor lineaire en
   hoeksnelheid, voor alle bronnen (pagina, gamepad, eigen nodes, Nav2).
   Bewaard in /root/turtlebot_state/speed_limit.json (blijft na een
   herstart en een update). Enkel aan te passen via Beheer (PIN):
   status_page_server.py schrijft het bestand, de guard leest het elke
   seconde opnieuw. De parameters max_linear / max_angular / timeout zijn
   alleen-lezen: `ros2 param set` wordt geweigerd, anders kon elke student
   met ROS op zijn laptop het plafond van de docent opheffen.

3. Actieve stuurbron voor de statuspagina (t13/t40): /cmd_vel_status
   (std_msgs/String, JSON, 2 Hz): welke ingang van twist_mux nu rijdt
   (zelfde prioriteiten en timeout als twist_mux.yaml), de snelheid die de
   guard doorstuurt, het plafond en de snelheid van de sliders.
4. Snelheid van de gamepad (t76): de sliders op Rijden publiceren
   /teleop_speed (JSON {"lin", "ang"}); de guard bewaart dat in
   teleop_speed.json en zet het als scale_linear/scale_angular bij
   teleop_twist_joy_node (stick volledig = sliderwaarde), turbo (RB) = het
   plafond. Ook opnieuw als de joystick later (her)start.

5. Lichtblick-Teleop (turtlebot-vis) publiceert enkel geometry_msgs/Twist,
   de robot werkt sinds Jazzy (v1.0.0) overal met TwistStamped. De guard
   zet /cmd_vel_lichtblick (Twist) om naar /cmd_vel_web (TwistStamped):
   zelfde ingang en prioriteit als de webteleop van de statuspagina.

6. Piep als de verbinding wegvalt terwijl de robot rijdt (23-10): de robot
   verliest zijn wifi-link met de AP, of er waren webpagina's verbonden
   (rosbridge, /connected_clients, behalve 127.0.0.1) en nu 3 s geen enkele
   meer. Een route of project rijdt dan verder zonder dat iemand meekijkt:
   het uit-geluid van de OpenCR (/sound), elke 60 s opnieuw zolang de robot
   rijdt; het aan-geluid als er weer een pagina verbonden is. Aan/uit in
   Beheer (state/conn_alarm, standaard aan).

Alle stuurtopics zijn geometry_msgs/TwistStamped (zoals in de simulatie).

Gestart door twist_mux_start.sh, parameters in twist_mux.yaml.
"""
import json
import os
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from turtlebot3_msgs.srv import Sound
try:
    from rosbridge_msgs.msg import ConnectedClients
except ImportError:          # zonder rosbridge_msgs enkel de wifi-link
    ConnectedClients = None

STATE_FILE = "/root/turtlebot_state/speed_limit.json"
CONN_ALARM_FILE = "/root/turtlebot_state/conn_alarm"   # "false" = geen piep (Beheer)
WIFI_OPERSTATE = "/sys/class/net/wlan0/operstate"
CONN_LOST_AFTER = 3.0        # s zonder pagina voor het als weg telt
CONN_DRIVING_WINDOW = 10.0   # s na het laatste rijcommando telt nog als rijden
CONN_REPEAT = 60.0           # s tussen herhaalde piepjes
SOUND_OFF, SOUND_ON = 0, 1   # turtlebot3_msgs/srv/Sound (constanten staan daar enkel als commentaar)
TELEOP_FILE = "/root/turtlebot_state/teleop_speed.json"
TELEOP_DEFAULT = (0.12, 1.0)   # zoals de sliders op Rijden
TELEOP_NODE = "/teleop_twist_joy_node"
# TurtleBot3 burger: hardwaregrenzen. Lineair 0.21 i.p.v. de 0.22 uit de
# specificaties: de OpenCR begrenst op 0.033*2*pi*61/60 = 0.2108 m/s, en 61 rpm
# is 266 Dynamixel-eenheden - net boven de snelheidslimiet 265 van de XL430,
# die het commando dan weigert: bij 0.22 bleven de wielen stil (23-10, t78).
HW_MAX_LINEAR = 0.21
HW_MAX_ANGULAR = 2.84


def load_limits():
    try:
        with open(STATE_FILE) as f:
            d = json.load(f)
        # een bestand van voor 0.26.0 kan nog 0.22 bevatten
        return min(float(d["max_linear"]), HW_MAX_LINEAR), min(float(d["max_angular"]), HW_MAX_ANGULAR)
    except (OSError, ValueError, KeyError, TypeError):
        return None


def load_teleop():
    try:
        with open(TELEOP_FILE) as f:
            d = json.load(f)
        return float(d["lin"]), float(d["ang"])
    except (OSError, ValueError, KeyError, TypeError):
        return TELEOP_DEFAULT


# Zoals twist_mux.yaml: (naam, topic, prioriteit); timeout 0.5 s voor alle drie
SOURCES = [("joy", "cmd_vel_joy", 100), ("web", "cmd_vel_web", 50), ("cmd_vel", "cmd_vel", 10)]
SOURCE_TIMEOUT = 0.5


def clamp(v, limit):
    return max(-limit, min(limit, v))


class CmdVelGuard(Node):
    def __init__(self):
        super().__init__("cmd_vel_guard")
        self.declare_parameter("timeout", 1.5)
        self.declare_parameter("max_linear", HW_MAX_LINEAR)
        self.declare_parameter("max_angular", HW_MAX_ANGULAR)
        self.timeout = self.get_parameter("timeout").value
        self.max_lin = self.get_parameter("max_linear").value
        self.max_ang = self.get_parameter("max_angular").value
        self.internal = False    # True while the guard itself applies the file
        self.add_on_set_parameters_callback(self.on_params)
        self.file_mtime = None
        self.reload_file()
        self.create_timer(1.0, self.reload_file)

        self.pub = self.create_publisher(TwistStamped, "cmd_vel_out", 10)
        self.create_subscription(TwistStamped, "cmd_vel_mux", self.on_cmd, 10)
        self.last = 0.0          # time.monotonic() of the last message
        self.moving = False      # last forwarded command was not zero
        self.create_timer(0.1, self.check)

        # Actieve stuurbron: laatste bericht per ingang van twist_mux
        self.source_seen = {}
        for name, topic, _prio in SOURCES:
            self.create_subscription(TwistStamped, topic, lambda m, n=name: self.source_seen.__setitem__(n, time.monotonic()), 10)
        self.web_pub = self.create_publisher(TwistStamped, "cmd_vel_web", 10)
        self.create_subscription(Twist, "cmd_vel_lichtblick", self.on_lichtblick, 10)
        self.out = (0.0, 0.0)
        # Gamepad-snelheid uit de sliders (t76)
        self.teleop = load_teleop()
        self.teleop_pushed = None   # wat teleop_twist_joy_node nu heeft (None = onbekend)
        self.teleop_busy = False
        self.teleop_cli = self.create_client(SetParameters, TELEOP_NODE + "/set_parameters")
        self.create_subscription(String, "teleop_speed", self.on_teleop_speed, 10)
        self.create_timer(1.0, self.push_teleop)
        # Piep bij verbindingsverlies tijdens het rijden (punt 6)
        self.page_count = None          # aantal pagina's op afstand (None = nog onbekend)
        self.had_pages = False     # sinds de laatste melding minstens één pagina gezien
        self.no_pages_since = None
        self.last_moving = -1e9
        self.alarm_on = False
        self.last_alarm = 0.0
        self.sound_cli = self.create_client(Sound, "/sound")
        if ConnectedClients is not None:
            self.create_subscription(ConnectedClients, "connected_clients", self.on_clients,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0, self.conn_check)
        self.status_pub = self.create_publisher(String, "cmd_vel_status", 10)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info("cmd_vel_guard: timeout %.1f s, max %.2f m/s, %.2f rad/s"
                               % (self.timeout, self.max_lin, self.max_ang))

    def reload_file(self):
        try:
            mtime = os.stat(STATE_FILE).st_mtime
        except OSError:
            mtime = None
        if mtime == self.file_mtime:
            return
        self.file_mtime = mtime
        lin, ang = load_limits() or (HW_MAX_LINEAR, HW_MAX_ANGULAR)
        if (lin, ang) == (self.max_lin, self.max_ang):
            return
        self.internal = True
        try:
            results = self.set_parameters([rclpy.Parameter("max_linear", value=float(lin)),
                                           rclpy.Parameter("max_angular", value=float(ang))])
        finally:
            self.internal = False
        bad = [r.reason for r in results if not r.successful]
        if bad:
            self.get_logger().warn("%s: ongeldige waarden genegeerd (%s)" % (STATE_FILE, bad[0]))
        else:
            self.get_logger().info("maximale snelheid: %.2f m/s, %.2f rad/s" % (lin, ang))

    def on_params(self, params):
        if not self.internal:
            return SetParametersResult(successful=False,
                                       reason="alleen-lezen: maximale snelheid enkel via Beheer (PIN)")
        lin, ang, timeout = self.max_lin, self.max_ang, self.timeout
        for p in params:
            if p.name == "max_linear":
                lin = float(p.value)
            elif p.name == "max_angular":
                ang = float(p.value)
            elif p.name == "timeout":
                timeout = float(p.value)
        if not (0.01 <= lin <= HW_MAX_LINEAR):
            return SetParametersResult(successful=False, reason="max_linear: 0.01 tot %.2f m/s" % HW_MAX_LINEAR)
        if not (0.1 <= ang <= HW_MAX_ANGULAR):
            return SetParametersResult(successful=False, reason="max_angular: 0.1 tot %.2f rad/s" % HW_MAX_ANGULAR)
        if not (0.2 <= timeout <= 5.0):
            return SetParametersResult(successful=False, reason="timeout: 0.2 tot 5 s")
        self.max_lin, self.max_ang, self.timeout = lin, ang, timeout
        return SetParametersResult(successful=True)

    def stamped(self, twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist = twist
        return out

    def on_lichtblick(self, msg):
        self.web_pub.publish(self.stamped(msg))

    def on_cmd(self, msg):
        out = Twist()
        out.linear.x = clamp(msg.twist.linear.x, self.max_lin)
        out.angular.z = clamp(msg.twist.angular.z, self.max_ang)
        self.pub.publish(self.stamped(out))
        self.out = (out.linear.x, out.angular.z)
        self.last = time.monotonic()
        self.moving = out.linear.x != 0.0 or out.angular.z != 0.0

    def check(self):
        if self.moving and time.monotonic() - self.last > self.timeout:
            self.pub.publish(self.stamped(Twist()))
            self.out = (0.0, 0.0)
            self.moving = False
            self.get_logger().warn("geen stuurcommando meer sinds %.1f s - robot gestopt" % self.timeout)


    def on_teleop_speed(self, msg):
        try:
            d = json.loads(msg.data)
            lin = max(0.02, min(float(d["lin"]), HW_MAX_LINEAR))
            ang = max(0.2, min(float(d["ang"]), HW_MAX_ANGULAR))
        except (ValueError, KeyError, TypeError):
            return
        if (lin, ang) == self.teleop:
            return
        self.teleop = (lin, ang)
        try:
            with open(TELEOP_FILE + ".tmp", "w") as f:
                json.dump({"lin": round(lin, 3), "ang": round(ang, 3)}, f)
            os.replace(TELEOP_FILE + ".tmp", TELEOP_FILE)
        except OSError as exc:
            self.get_logger().warn("%s niet bewaard: %s" % (TELEOP_FILE, exc))
        self.push_teleop()

    def push_teleop(self):
        """Slider speed as normal and the ceiling as turbo speed to the gamepad
        driver, whenever it differs or the driver has (re)started."""
        if not self.teleop_cli.service_is_ready():
            self.teleop_pushed = None    # joystick off: push again once it is there
            return
        want = (self.teleop[0], self.teleop[1], self.max_lin, self.max_ang)
        if want == self.teleop_pushed or self.teleop_busy:
            return
        req = SetParameters.Request()
        for name, val in (("scale_linear.x", want[0]), ("scale_angular.yaw", want[1]),
                          ("scale_linear_turbo.x", want[2]), ("scale_angular_turbo.yaw", want[3])):
            req.parameters.append(ParameterMsg(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(val))))
        self.teleop_busy = True

        def done(fut):
            self.teleop_busy = False
            try:
                ok = all(r.successful for r in fut.result().results)
            except Exception:
                ok = False
            self.teleop_pushed = want    # also on failure: no retry (and warning) every second
            if ok:
                self.get_logger().info("gamepad: %.2f m/s, %.2f rad/s (turbo %.2f / %.2f)" % want)
            else:
                self.get_logger().warn("gamepad-snelheid instellen mislukt (%s)" % TELEOP_NODE)
        self.teleop_cli.call_async(req).add_done_callback(done)

    def on_clients(self, msg):
        n = sum(1 for c in msg.clients if c.ip_address not in ("127.0.0.1", "::1", "::ffff:127.0.0.1"))
        if n == 0 and self.page_count != 0:
            self.no_pages_since = time.monotonic()
        if n > 0:
            self.had_pages = True
        self.page_count = n

    @staticmethod
    def conn_alarm_enabled():
        try:
            with open(CONN_ALARM_FILE) as f:
                return f.read().strip() != "false"
        except OSError:
            return True

    @staticmethod
    def wifi_up():
        try:
            with open(WIFI_OPERSTATE) as f:
                return f.read().strip() in ("up", "unknown")
        except OSError:
            return True              # geen wlan0 (kabel?): niet als verlies tellen

    def play(self, value):
        if self.sound_cli.service_is_ready():
            req = Sound.Request()
            req.value = value
            self.sound_cli.call_async(req)

    def conn_check(self):
        try:
            now = time.monotonic()
            if self.moving:
                self.last_moving = now
            pages_gone = (self.had_pages and self.page_count == 0 and self.no_pages_since is not None
                          and now - self.no_pages_since >= CONN_LOST_AFTER)
            wifi_down = not self.wifi_up()
            lost = pages_gone or wifi_down
            driving = now - self.last_moving < CONN_DRIVING_WINDOW
            if lost and driving and self.conn_alarm_enabled():
                if not self.alarm_on or now - self.last_alarm >= CONN_REPEAT:
                    if not self.alarm_on:
                        self.get_logger().warn("verbinding weg (%s) terwijl de robot rijdt - piep"
                                               % ("wifi-link" if wifi_down else "geen webpagina meer"))
                    self.alarm_on, self.last_alarm = True, now
                    self.play(SOUND_OFF)
            elif self.alarm_on and not wifi_down and self.page_count:
                self.get_logger().info("verbinding terug")
                self.alarm_on = False
                self.had_pages = True
                self.play(SOUND_ON)
        except Exception as exc:     # nooit de guard laten vallen voor een piepje
            self.get_logger().warn("verbindingspiep: %s" % exc)

    def active_source(self):
        now = time.monotonic()
        live = [(prio, name) for name, _topic, prio in SOURCES
                if now - self.source_seen.get(name, -1e9) < SOURCE_TIMEOUT]
        return max(live)[1] if live else None

    def publish_status(self):
        fresh = time.monotonic() - self.last < self.timeout
        lin, ang = self.out if fresh else (0.0, 0.0)
        self.status_pub.publish(String(data=json.dumps({
            "source": self.active_source(), "moving": self.moving and fresh,
            "lin": round(lin, 3), "ang": round(ang, 3),
            "max_linear": self.max_lin, "max_angular": self.max_ang,
            "teleop_linear": self.teleop[0], "teleop_angular": self.teleop[1],
            "conn_alarm": self.conn_alarm_enabled()})))


def main():
    rclpy.init()
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
