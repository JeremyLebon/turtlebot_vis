"""Map bundles: a map with its zones and routes in one JSON document, to copy
maps between robots - downloaded/uploaded on nav.html (status_page_server.py)
or through the fleet monitor's map library (launch_control_node.py).

    {"format": "turtlebot-map/1", "name", "created", "from",
     "yaml": "<map yaml text>", "pgm_b64": "<map image>",
     "zones": [...], "routes": {...}}

Importing never overwrites silently: an existing map of the same name is
first renamed to <name>_vorige (then _vorige2, ...), and the map Navigation
runs with is refused.
"""
import base64
import glob
import io
import json
import os
import re
import socket
import struct
import time
import zipfile
import zlib

MAP_DIR = "/root/turtlebot_state/maps"
FORMAT = "turtlebot-map/1"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
# what belongs to a map (also launch_control_node.MAP_SUFFIXES)
SUFFIXES = (".yaml", ".pgm", ".zones.json", ".routes.json", ".orig.pgm",
            ".zones.bak.json", ".edited.pgm")


def list_maps():
    return sorted(os.path.splitext(os.path.basename(p))[0]
                  for p in glob.glob(os.path.join(MAP_DIR, "*.yaml")))


def running_map():
    """Name of the map Navigation runs with (from its command line), or None."""
    for cmdline in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            with open(cmdline, "rb") as f:
                args = f.read().split(b"\0")
        except OSError:
            continue
        if any(b"navigation2.launch.py" in a for a in args):
            for a in args:
                if a.startswith(b"map:="):
                    return os.path.splitext(os.path.basename(a[5:].decode()))[0]
    return None


def _image_path(yaml_text, name):
    m = re.search(r"(?m)^image:\s*(\S+)", yaml_text)
    img = m.group(1) if m else name + ".pgm"
    return img if os.path.isabs(img) else os.path.join(MAP_DIR, img)


def _read_json(path, key, default):
    try:
        with open(path) as f:
            return json.load(f).get(key, default)
    except (OSError, ValueError):
        return default


def export_bundle(name):
    if name not in list_maps():
        raise ValueError("kaart '%s' bestaat niet" % name)
    with open(os.path.join(MAP_DIR, name + ".yaml")) as f:
        yaml_text = f.read()
    with open(_image_path(yaml_text, name), "rb") as f:
        pgm = f.read()
    return {"format": FORMAT, "name": name, "created": time.time(), "from": socket.gethostname(),
            "yaml": yaml_text, "pgm_b64": base64.b64encode(pgm).decode(),
            "zones": _read_json(os.path.join(MAP_DIR, name + ".zones.json"), "zones", []),
            "routes": _read_json(os.path.join(MAP_DIR, name + ".routes.json"), "routes", {})}


def _rename_map(src, dst):
    for suf in SUFFIXES:
        path = os.path.join(MAP_DIR, src + suf)
        if not os.path.exists(path):
            continue
        target = os.path.join(MAP_DIR, dst + suf)
        if suf == ".yaml":
            with open(path) as f:
                text = re.sub(r"(?m)^image:.*$", "image: " + dst + ".pgm", f.read())
            with open(target, "w") as f:
                f.write(text)
            os.remove(path)
        else:
            os.rename(path, target)


def import_bundle(bundle, name=None):
    """Write the bundle as map <name> (default: its own name) -> message."""
    if not isinstance(bundle, dict) or bundle.get("format") != FORMAT:
        raise ValueError("geen kaartbundel (turtlebot-map/1)")
    name = (name or bundle.get("name") or "").strip()
    if not NAME_RE.match(name):
        raise ValueError("ongeldige naam '%s' (enkel letters, cijfers, - en _)" % name)
    if name == running_map():
        raise ValueError("kaart '%s' is in gebruik door Navigatie - stop Navigatie of kies een andere naam" % name)
    pgm = base64.b64decode(bundle["pgm_b64"])
    if not pgm.startswith(b"P5"):
        raise ValueError("kaartafbeelding is geen PGM")
    os.makedirs(MAP_DIR, exist_ok=True)
    note = ""
    if name in list_maps():
        backup, i = name + "_vorige", 2
        while backup in list_maps():
            backup, i = "%s_vorige%d" % (name, i), i + 1
        _rename_map(name, backup)
        note = " (de bestaande kaart heet nu '%s')" % backup
    yaml_text = re.sub(r"(?m)^image:.*$", "image: " + name + ".pgm", bundle["yaml"])
    with open(os.path.join(MAP_DIR, name + ".pgm"), "wb") as f:
        f.write(pgm)
    with open(os.path.join(MAP_DIR, name + ".yaml"), "w") as f:
        f.write(yaml_text)
    for suf, key, default in ((".zones.json", "zones", []), (".routes.json", "routes", {})):
        data = bundle.get(key) or default
        if data:
            with open(os.path.join(MAP_DIR, name + suf), "w") as f:
                json.dump({key: data}, f)
    n_z, n_r = len(bundle.get("zones") or []), len(bundle.get("routes") or {})
    return "kaart '%s' geimporteerd (%d zones, %d routes, van %s)%s" % (
        name, n_z, n_r, bundle.get("from", "?"), note)


# ---- zip: the ordinary map files (also usable with a map_server elsewhere) ----
def export_zip(name):
    """<name>.yaml, <name>.pgm (+ .zones.json, .routes.json) in a zip."""
    b = export_bundle(name)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr(name + ".yaml", re.sub(r"(?m)^image:.*$", "image: " + name + ".pgm", b["yaml"]))
        pgm = base64.b64decode(b["pgm_b64"])
        z.writestr(name + ".pgm", pgm)
        try:   # extra: to look at / use in a report (not needed for importing)
            z.writestr(name + ".png", pgm_to_png(pgm))
        except ValueError:
            pass
        if b["zones"]:
            z.writestr(name + ".zones.json", json.dumps({"zones": b["zones"]}, indent=1))
        if b["routes"]:
            z.writestr(name + ".routes.json", json.dumps({"routes": b["routes"]}, indent=1))
    return buf.getvalue()


def bundle_from_zip(data):
    """Bundle dict from a zip with a map yaml + its pgm (zones/routes optional)."""
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        names = z.namelist()
        yamls = [n for n in names if n.endswith(".yaml")]
        if not yamls:
            raise ValueError("geen .yaml in de zip")
        yaml_name = yamls[0]
        yaml_text = z.read(yaml_name).decode()
        m = re.search(r"(?m)^image:\s*(\S+)", yaml_text)
        base = os.path.dirname(yaml_name)
        img = os.path.join(base, os.path.basename(m.group(1))) if m else yaml_name[:-5] + ".pgm"
        if img not in names:
            raise ValueError("kaartafbeelding %s niet in de zip" % img)
        stem = yaml_name[:-5]

        def opt(suffix, key, default):
            return json.loads(z.read(stem + suffix)).get(key, default) if stem + suffix in names else default
        return {"format": FORMAT, "name": os.path.basename(stem), "from": "zip",
                "yaml": yaml_text, "pgm_b64": base64.b64encode(z.read(img)).decode(),
                "zones": opt(".zones.json", "zones", []), "routes": opt(".routes.json", "routes", {})}


def bundle_from_bytes(data):
    """A zip or a .tbmap.json -> bundle dict."""
    if data[:2] == b"PK":
        return bundle_from_zip(data)
    return json.loads(data)


def _parse_pgm(pgm):
    """Binary 8-bit PGM (P5) -> (width, height, pixels), top row first."""
    parts, pos = [], 0
    while len(parts) < 4:   # magic, width, height, maxval (comments skipped)
        while pgm[pos:pos + 1].isspace():
            pos += 1
        if pgm[pos:pos + 1] == b"#":
            pos = pgm.index(b"\n", pos) + 1
            continue
        end = pos
        while not pgm[end:end + 1].isspace():
            end += 1
        parts.append(pgm[pos:end])
        pos = end
    if parts[0] != b"P5" or int(parts[3]) > 255:
        raise ValueError("geen 8-bit P5-PGM")
    w, h = int(parts[1]), int(parts[2])
    return w, h, pgm[pos + 1:pos + 1 + w * h]


def pgm_to_png(pgm):
    """Binary 8-bit PGM (P5) -> grayscale PNG, standard library only."""
    w, h, pixels = _parse_pgm(pgm)
    raw = b"".join(b"\x00" + pixels[y * w:(y + 1) * w] for y in range(h))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)) +
            chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def _yaml_value(text, key, default):
    m = re.search(r"(?m)^%s:\s*(.+?)\s*$" % key, text)
    return m.group(1) if m else default


def map_grid(name):
    """A saved map as nav_msgs/OccupancyGrid-like JSON for nav.html, read from
    the files - to look at and edit maps and zones without Navigation (t83).
    Same conversion as map_server (trinary): occupancy p = (255 - pixel) / 255
    (or pixel / 255 with negate), > occupied_thresh -> 100, < free_thresh ->
    0, else -1. Cells bottom row first, as in an OccupancyGrid; base64 of
    the int8 values (-1 -> 255)."""
    if not NAME_RE.match(name or "") or name not in list_maps():
        raise ValueError("kaart '%s' bestaat niet" % name)
    with open(os.path.join(MAP_DIR, name + ".yaml")) as f:
        yaml_text = f.read()
    with open(_image_path(yaml_text, name), "rb") as f:
        w, h, pixels = _parse_pgm(f.read())
    res = float(_yaml_value(yaml_text, "resolution", "0.05"))
    origin = [float(v) for v in _yaml_value(yaml_text, "origin", "[0, 0, 0]").strip("[]").split(",")[:3]]
    negate = _yaml_value(yaml_text, "negate", "0").strip() in ("1", "true", "True")
    occ_t = float(_yaml_value(yaml_text, "occupied_thresh", "0.65"))
    free_t = float(_yaml_value(yaml_text, "free_thresh", "0.25"))
    lut = bytearray(256)
    for px in range(256):
        p = px / 255.0 if negate else (255 - px) / 255.0
        lut[px] = 100 if p > occ_t else 0 if p < free_t else 255
    cells = b"".join(bytes(pixels[y * w:(y + 1) * w]).translate(lut) for y in range(h - 1, -1, -1))
    return {"name": name,
            "info": {"width": w, "height": h, "resolution": res,
                     "origin": {"position": {"x": origin[0], "y": origin[1], "z": 0.0}}},
            "data_b64": base64.b64encode(cells).decode(),
            "zones": _read_json(os.path.join(MAP_DIR, name + ".zones.json"), "zones", []),
            "routes": _read_json(os.path.join(MAP_DIR, name + ".routes.json"), "routes", {}),
            "edited": os.path.exists(os.path.join(MAP_DIR, name + ".orig.pgm"))}
