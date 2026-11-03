#!/usr/bin/env python3
"""Static file server for the status page (replaces `python3 -m
http.server`). Identical, except every response carries
`Cache-Control: no-cache`, so browsers (phones especially, where a hard
refresh is awkward) always revalidate and pick up a new page/script after
an update instead of mixing a fresh index.html with a stale common.js."""

import functools
import html
import http.server
import json
import os
import sys
import urllib.parse
import urllib.request
import zipfile

import fenix_admin
import fenix_services
import map_bundle

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
PHOTO_DIR = "/root/turtlebot_state/photos"
WIFI_PSK = "/root/turtlebot_state/wifi_psk"   # setup_turtlebot.sh, uit de SD-kaart
DIRECTORY = sys.argv[2] if len(sys.argv) > 2 else "/root/turtlebot3_ws/status_page"
# Docenten-PIN voor Beheer (fenix_admin.py): tokens in memory
ADMIN = fenix_admin.Sessions()
# Beheer-acties na de PIN: doorgegeven aan launch_control_node (AdminCommands,
# enkel op 127.0.0.1), behalve de maximale snelheid (bestand voor cmd_vel_guard)
LAUNCH_CONTROL_ADMIN = "http://127.0.0.1:8091/"
LAUNCH_CONTROL_ACTIONS = ("update", "rollback", "install_version", "list_versions", "desktop", "usb_on", "usb_off",
                          "usb_options", "monitor_options", "camera_settings")
SPEED_LIMIT = "/root/turtlebot_state/speed_limit.json"
CONN_ALARM = "/root/turtlebot_state/conn_alarm"     # cmd_vel_guard: piep bij verbindingsverlies
IMAGE_CHANNEL = "/root/turtlebot_state/image_channel"   # updatekanaal (setup_turtlebot.sh op de host)
HW_MAX_LINEAR, HW_MAX_ANGULAR = 0.21, 2.84   # zoals cmd_vel_guard.py


def set_speed_limit(args):
    lin, ang = float(args["max_linear"]), float(args["max_angular"])
    if not 0.01 <= lin <= HW_MAX_LINEAR:
        return {"success": False, "message": "max. lineair: 0.01 tot %.2f m/s" % HW_MAX_LINEAR}
    if not 0.1 <= ang <= HW_MAX_ANGULAR:
        return {"success": False, "message": "max. draaien: 0.1 tot %.2f rad/s" % HW_MAX_ANGULAR}
    tmp = SPEED_LIMIT + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"max_linear": round(lin, 3), "max_angular": round(ang, 3)}, f)
    os.replace(tmp, SPEED_LIMIT)
    return {"success": True, "message": "maximale snelheid %.2f m/s, %.2f rad/s" % (lin, ang)}


def admin_action(action, args):
    if action == "service_log":       # Status > Diensten (t79)
        return fenix_services.log_tail(str(args.get("name", "")))
    if action == "service_restart":
        return fenix_services.restart(str(args.get("name", "")))
    if action == "image_channel":     # Beheer > Software: welke tag Software bijwerken ophaalt
        ch = str(args.get("channel", ""))
        if ch not in ("humble", "jazzy"):
            return {"success": False, "message": "onbekend kanaal: %s" % ch}
        with open(IMAGE_CHANNEL + ".tmp", "w") as f:
            f.write(ch + "\n")
        os.replace(IMAGE_CHANNEL + ".tmp", IMAGE_CHANNEL)
        return {"success": True, "message": "updatekanaal %s - geldt bij de volgende Software bijwerken"
                % ("Jazzy (:jazzy)" if ch == "jazzy" else "Humble (:raspios-zenoh)")}
    if action == "conn_alarm":        # Beheer > Robot: piep als de verbinding wegvalt tijdens het rijden
        on = bool(args.get("on", True))
        with open(CONN_ALARM, "w") as f:
            f.write("true" if on else "false")
        return {"success": True, "message": "piep bij verbindingsverlies " + ("aan" if on else "uit")}
    if action == "speed_limit":
        try:
            return set_speed_limit(args)
        except (KeyError, TypeError, ValueError):
            return {"success": False, "message": "ongeldige waarden"}
    if action not in LAUNCH_CONTROL_ACTIONS:
        return {"success": False, "message": "onbekende opdracht: %s" % action}
    req = urllib.request.Request(LAUNCH_CONTROL_ADMIN, method="POST",
                                 data=json.dumps({"action": action, "args": args}).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=25) as r:
            return json.loads(r.read())
    except (OSError, ValueError) as exc:
        return {"success": False, "message": "launch_control niet bereikbaar (%s)" % exc}


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    # Map bundles (map_bundle.py): nav.html "Exporteer" / "Importeer"
    def _json(self, code, obj, filename=None):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        if filename:
            self.send_header("Content-Disposition", 'attachment; filename="%s"' % filename)
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        if url.path == "/api/admin/status":   # Beheer: PIN set? token still valid?
            token = urllib.parse.parse_qs(url.query).get("token", [""])[0]
            return self._json(200, {"pin_set": fenix_admin.pin_set(), "unlocked": ADMIN.valid(token)})
        if url.path == "/api/admin/wifi":     # Info: wifi-QR, enkel na de PIN (bevat het wachtwoord)
            token = urllib.parse.parse_qs(url.query).get("token", [""])[0]
            if not ADMIN.valid(token):
                return self._json(401, {"error": "eerst ontgrendelen (Beheer)"})
            nr = int(os.environ.get("TURTLEBOT_NR", "0") or 0)
            try:
                with open(WIFI_PSK) as f:
                    psk = f.read().strip()
            except OSError:
                psk = ""
            return self._json(200, {"ssid": "TB-AP-%02d" % nr, "psk": psk})
        if url.path == "/api/projects":   # projects.html (projects_lib.py, project.json)
            import projects_lib
            return self._json(200, sorted(projects_lib.projects().values(), key=lambda m: m.get("title", "")))
        if url.path == "/api/photos" or url.path.startswith("/photos/"):   # route point photos (nav_web_node)
            return self._photos(url.path)
        if url.path == "/api/map_file":   # Mappen: kaart + zones zonder Navigatie (t83)
            try:
                return self._json(200, map_bundle.map_grid(urllib.parse.parse_qs(url.query).get("name", [""])[0]))
            except (ValueError, OSError, IndexError) as exc:
                return self._json(404, {"error": str(exc)})
        if url.path == "/api/map_export":
            name = urllib.parse.parse_qs(url.query).get("name", [""])[0]
            try:
                data = map_bundle.export_zip(name)
            except (ValueError, OSError) as exc:
                return self._json(404, {"error": str(exc)})
            self.send_response(200)
            self.send_header("Content-Type", "application/zip")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Content-Disposition", 'attachment; filename="%s.zip"' % name)
            self.end_headers()
            self.wfile.write(data)
            return None
        return super().do_GET()

    def _photos(self, path):
        if path == "/api/photos":
            try:
                names = sorted(os.listdir(PHOTO_DIR), reverse=True)
            except OSError:
                names = []
            body = ("<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width'>"
                    "<title>Routefoto's</title><body style='font-family:sans-serif;background:#111;color:#ddd'>"
                    "<h3>Routefoto's (%d, nieuwste eerst)</h3>" % len(names) +
                    "".join("<div style='display:inline-block;margin:4px'><a href='/photos/%s'><img src='/photos/%s' "
                            "width=240 loading=lazy></a><br><small>%s</small></div>"
                            % (html.escape(n, True), html.escape(n, True), html.escape(n)) for n in names))
            data = body.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return None
        name = urllib.parse.unquote(path[len("/photos/"):])
        if "/" in name or name.startswith(".") or not os.path.isfile(os.path.join(PHOTO_DIR, name)):
            return self._json(404, {"error": "foto niet gevonden"})
        with open(os.path.join(PHOTO_DIR, name), "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", "image/png" if name.endswith(".png") else "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)
        return None

    def _body_json(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(min(n, 10000)) or b"{}")

    def do_POST(self):
        url = urllib.parse.urlparse(self.path)
        if url.path in ("/api/admin/unlock", "/api/admin/lock"):
            try:
                body = self._body_json()
            except ValueError:
                return self._json(400, {"error": "bad request"})
            if url.path.endswith("/lock"):
                ADMIN.lock_token(str(body.get("token", "")))
                return self._json(200, {"ok": True})
            code, out = ADMIN.unlock(self.client_address[0], str(body.get("pin", "")))
            return self._json(code, out)
        if url.path == "/api/admin/action":   # Beheer: {token, action, args}
            try:
                body = self._body_json()
            except ValueError:
                return self._json(400, {"success": False, "message": "ongeldige opdracht"})
            if not ADMIN.valid(str(body.get("token", ""))):
                return self._json(401, {"success": False, "message": "PIN vereist (of verlopen) - ontgrendel Beheer opnieuw"})
            args = body.get("args") if isinstance(body.get("args"), dict) else {}
            return self._json(200, admin_action(str(body.get("action", "")), args))
        if url.path != "/api/map_import":
            return self._json(404, {"error": "not found"})
        name = urllib.parse.parse_qs(url.query).get("name", [None])[0]
        try:
            n = int(self.headers.get("Content-Length", 0))
            if n > 20 * 1024 * 1024:
                raise ValueError("bestand te groot")
            bundle = map_bundle.bundle_from_bytes(self.rfile.read(n))
            msg = map_bundle.import_bundle(bundle, name)
        except (ValueError, KeyError, TypeError, OSError, zipfile.BadZipFile) as exc:
            return self._json(400, {"error": str(exc)})
        return self._json(200, {"message": msg})


if __name__ == "__main__":
    handler = functools.partial(NoCacheHandler, directory=DIRECTORY)
    http.server.ThreadingHTTPServer(("0.0.0.0", PORT), handler).serve_forever()
