#!/usr/bin/env python3
"""Controleert de meetronde van sessie 4: rapport van de simulator tegen de CSV's van de student.

Gebruik:  python3 controleer_meetronde.py <map> [-v] [--csv overzicht.csv]

<map> bevat de ingediende bestanden, los of in een submap per student (zoals Toledo ze geeft):
  rapport_*_proj_*.json   rapport van de taak "Sessie 4 · Project: meetronde"
  metingen_<rnr>.csv      een rij per meetpunt (script van de student)
  route_<rnr>.csv         een rij per 0,5 s tijdens het rijden (script van de student)
Bestanden horen bij elkaar via het studentnummer (in de bestandsnaam en in het rapport).

Per student wordt nagegaan:
  rapport   controlecode, studentcode en opname kloppen (zelfde controle als controleer_rapporten.py),
            taak = meetronde, wereld = Klaslokaal
  punten    P1-P3 gehaald volgens het rapport, en de route van de student bevat P1-P3 van het rapport
  route     de posities in route_<rnr>.csv liggen op de echte route uit het rapport
  sensoren  de sensorwaarden in route_<rnr>.csv en metingen_<rnr>.csv passen bij de echte waarden
            op die plaats (de bronnen liggen per studentnummer elders: data van iemand anders past niet)
  tijd      de tijden in metingen_<rnr>.csv vallen in de meetronde van het rapport
  hulp      de student gebruikte geen hulpweergave (bronnen, veldkaart, gemeten spoor) of zette de sensoren uit

Het rapport heeft de echte positie (map-frame) en sensorwaarden elke 0,5 s. De posities van de student komen
van Nav2 (AMCL): enkele cm tot ~20 cm ernaast is normaal. Sensorwaarden worden op dezelfde plaats vergeleken
(de velden liggen vast), met ruime marges voor ruis en traagheid van de sensoren.
Geen sluitende beveiliging: de simulator draait bij de student. Wel moet wie data overneemt of verzint ook een
rapport hebben waarvan de echte rit en sensorwaarden daarbij passen.
"""
import csv
import json
import math
import re
import sys
from datetime import datetime, timedelta
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from controleer_rapporten import check as check_rapport  # noqa: E402

try:
    from zoneinfo import ZoneInfo
    TZ = ZoneInfo("Europe/Brussels")
except Exception:   # zonder tzdata: tijd van deze computer
    TZ = None

RNR = re.compile(r"[rRuUcC]\d{7,8}")
POS_MED, POS_P90 = 0.20, 0.40           # m: route van de student t.o.v. de echte route (mediaan, 90 %)
POINT_TOL = 0.35                        # m: zelfde tolerantie als de simulator voor P1-P3
NEAR = 0.30                             # m: echte meting zoeken binnen deze afstand
SENS_OK = 0.8                           # deel van de vergelijkbare waarden binnen de marge
# marge per sensor: max(absoluut, relatief × echte waarde); ruis en traagheid van de sensoren
MARGE = {"temperature": (0.8, 0.0), "humidity": (0.04, 0.0), "co2": (60.0, 0.15), "illuminance": (40.0, 0.15)}
KOL = ["t_s", "x", "y", "yaw_deg", "temperature", "humidity", "co2", "illuminance", "sound"]


def num(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as f:
        return [{k.strip().lower(): (v or "").strip() for k, v in row.items() if k} for row in csv.DictReader(f)]


def rnr_of(text):
    m = RNR.search(text)
    return m.group(0).lower() if m else None


class Truth:
    """Echte route uit het rapport: zoeken op plaats (raster van 0,5 m)."""

    def __init__(self, rows, kolommen):
        idx = {k: kolommen.index(k) for k in KOL if k in kolommen}
        self.rows = [{k: r[i] for k, i in idx.items()} for r in rows if r[idx["x"]] is not None]
        # basiswaarde per sensor (mediaan over de rit): het grootste deel van het lokaal ligt ver van de bronnen
        self.base = {}
        for k in MARGE:
            vals = sorted(r[k] for r in self.rows if r.get(k) is not None)
            if vals:
                self.base[k] = vals[len(vals) // 2]
        self.grid = {}
        for r in self.rows:
            self.grid.setdefault((math.floor(r["x"] * 2), math.floor(r["y"] * 2)), []).append(r)

    def near(self, x, y, d=NEAR):
        """Echte rijen binnen d van (x, y), dichtste eerst."""
        gx, gy = math.floor(x * 2), math.floor(y * 2)
        out = []
        for i in range(gx - 1, gx + 2):
            for j in range(gy - 1, gy + 2):
                for r in self.grid.get((i, j), []):
                    dd = math.hypot(r["x"] - x, r["y"] - y)
                    if dd <= d:
                        out.append((dd, r))
        return [r for _, r in sorted(out, key=lambda t: t[0])]

    def dist(self, x, y):
        best = math.inf
        for r in self.near(x, y, 1.0) or self.rows:
            best = min(best, math.hypot(r["x"] - x, r["y"] - y))
        return best


def sensor_match(truth, rows, xk, yk):
    """(vergeleken, binnen marge, voorbeeld van een afwijking) voor de sensorwaarden in rows.

    Enkel waarden die iets zeggen tellen mee: de echte of de ingediende waarde wijkt af van de basiswaarde
    (in de buurt van een bron). Ver van de bronnen meet elke student hetzelfde, dus dat bewijst niets."""
    n = good = 0
    bad = None
    for row in rows:
        x, y = num(row.get(xk)), num(row.get(yk))
        if x is None or y is None:
            continue
        cand = truth.near(x, y)
        if not cand:
            continue
        for k, (a, rel) in MARGE.items():
            v = num(row.get(k))
            real = [c[k] for c in cand if c.get(k) is not None]
            if v is None or not real:
                continue
            tol = max(a, rel * abs(real[0]))
            base = truth.base.get(k)
            if base is not None and abs(v - base) <= tol and all(abs(t - base) <= tol for t in real):
                continue
            n += 1
            # beste overeenkomst onder de echte waarden op die plek (sensoren zijn traag: waarde van net ervoor)
            err = min(abs(v - t) for t in real)
            if err <= tol:
                good += 1
            elif bad is None or err / tol > bad[0]:
                bad = (err / tol, f"{k} {v:g} bij ({x:.2f}, {y:.2f}), echt {real[0]:g}")
    return n, good, bad[1] if bad else ""


def check_student(rnr, reports, metingen, route, verbose):
    res = {"nr": rnr, "naam": "?", "rapport": "ontbreekt", "punten": "-", "route": "-", "sensoren": "-", "tijd": "-", "hulp": "-"}
    notes = []
    proj = []
    for p in reports:
        r = json.loads(json.loads(p.read_text(encoding="utf-8"))["inhoud"])
        if r.get("taak", {}).get("id") == "proj":
            proj.append((p, r))
    if not proj:
        notes.append("geen rapport van de meetronde" + (f" ({len(reports)} andere rapporten)" if reports else ""))
        return res, notes
    # het rapport met de meeste routeregels (een student kan meerdere pogingen indienen)
    path, r = max(proj, key=lambda t: len(t[1].get("meetronde", {}).get("route", [])))
    if len(proj) > 1:
        notes.append(f"{len(proj)} rapporten van de meetronde, gecontroleerd: {path.name}")
    c = check_rapport(path)
    res["naam"] = c["naam"]
    res["rapport"] = c["echt"] if c["echt"] != "ok" else ("ok" if c["opname"] in ("ok", "geen") else f"opname {c['opname']}")
    if c["opname"] == "geen":
        notes.append("rapport zonder opname")
    if r.get("wereld") != "Klaslokaal":
        res["rapport"] = "andere wereld"
    if (r.get("student", {}).get("nr", "").strip().lower()) != rnr:
        res["rapport"] = "ander nummer"
        notes.append(f"rapport van {r.get('student', {}).get('nr')}, bestanden van {rnr}")
    mr = r.get("meetronde", {})
    truth = Truth(mr.get("route", []), mr.get("kolommen", KOL))
    pts = mr.get("punten", [])

    # punten: gehaald volgens het rapport; en zitten P1-P3 van het rapport in de route van de student?
    gehaald = [p for p in pts if p.get("bereikt_s") is not None]
    res["punten"] = f"{len(gehaald)}/3"
    if metingen:
        for p in pts:
            rows = [m for m in metingen if m.get("point", "").upper() == p["punt"]]
            if not rows:
                notes.append(f"{p['punt']} ontbreekt in metingen_{rnr}.csv")
                res["punten"] += "" if res["punten"].endswith("!") else " !"
                continue
            gx, gy = num(rows[0].get("goal_x")), num(rows[0].get("goal_y"))
            if gx is not None and gy is not None and math.hypot(gx - p["x"], gy - p["y"]) > 0.15:
                notes.append(f"{p['punt']} in de CSV ({gx:g}, {gy:g}) ≠ rapport ({p['x']:g}, {p['y']:g}): punten van iemand anders?")
                res["punten"] += "" if res["punten"].endswith("!") else " !"

    # route: posities van de student op de echte route?
    if not truth.rows:
        notes.append("rapport zonder route (meetronde te kort?)")
    if route and truth.rows:
        ds = sorted(truth.dist(num(x["x"]), num(x["y"])) for x in route if num(x.get("x")) is not None and num(x.get("y")) is not None)
        if ds:
            med, p90 = ds[len(ds) // 2], ds[int(len(ds) * 0.9)]
            res["route"] = ("ok" if med <= POS_MED and p90 <= POS_P90 else "AFWIJKEND") + f" {med * 100:.0f}/{p90 * 100:.0f} cm"
            if verbose:
                notes.append(f"route: {len(ds)} posities, mediaan {med * 100:.1f} cm, 90 % {p90 * 100:.1f} cm van de echte route")
    elif not route:
        res["route"] = "ontbreekt"

    # sensoren: route- en meetwaarden tegen de echte waarden op dezelfde plaats
    n1, g1, b1 = sensor_match(truth, route, "x", "y") if route and truth.rows else (0, 0, "")
    n2, g2, b2 = sensor_match(truth, metingen, "nav2_x", "nav2_y") if metingen and truth.rows else (0, 0, "")
    n, g = n1 + n2, g1 + g2
    if n:
        res["sensoren"] = ("ok" if g >= SENS_OK * n else "AFWIJKEND") + f" {100 * g // n}%"
        if g < SENS_OK * n or verbose:
            notes.append(f"sensoren: {g}/{n} waarden bij een bron binnen de marge" + (f"; bv. {b1 or b2}" if (b1 or b2) else ""))
    elif route or metingen:
        res["sensoren"] = "niet te vergelijken"

    # tijd: meetrijen tijdens de meetronde? time = hh:mm:ss van het script: in de container meestal UTC (geen
    # tijdzone ingesteld), op een eigen pc lokale tijd; rapport = UTC. Een rij is goed als ze in een van beide past.
    try:
        g0 = datetime.fromisoformat(r["gestart"].replace("Z", "+00:00"))
        g1 = datetime.fromisoformat(r["beeindigd"].replace("Z", "+00:00"))
        t0, t1 = g0.astimezone(TZ), g1.astimezone(TZ)
        marge = timedelta(minutes=2)

        def past(hm, a, b):
            t = a.replace(hour=int(hm[1]), minute=int(hm[2]), second=int(hm[3] or 0), microsecond=0)
            if t < a - timedelta(hours=12):
                t += timedelta(days=1)
            return a - marge <= t <= b + marge
        buiten = []
        for m in metingen or []:
            hm = re.fullmatch(r"(\d{1,2}):(\d{2})(?::(\d{2}))?", m.get("time", ""))
            if hm and not (past(hm, g0, g1) or past(hm, t0, t1)):
                buiten.append(m.get("point", "?") + " " + m["time"])
        if metingen:
            res["tijd"] = "ok" if not buiten else f"{len(buiten)} buiten"
            if buiten:
                notes.append(f"meetrijen buiten de meetronde ({t0:%H:%M:%S}-{t1:%H:%M:%S}): " + ", ".join(buiten[:4]))
    except (KeyError, ValueError):
        pass

    h = mr.get("hulpweergave", {})
    used = [k.replace("_", " ") for k, v in h.items() if v]
    res["hulp"] = "geen" if not used else ", ".join(used)
    return res, notes


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    if "--csv" in sys.argv:
        args = [a for a in args if a != sys.argv[sys.argv.index("--csv") + 1]]
    folder = Path(args[0] if args else ".")
    files = {}
    for p in folder.rglob("*"):
        if not p.is_file():
            continue
        name = p.name.lower()
        if name.endswith(".json"):
            try:
                d = json.loads(p.read_text(encoding="utf-8"))
                nr = json.loads(d["inhoud"]).get("student", {}).get("nr", "").strip().lower()
            except (ValueError, KeyError, TypeError, UnicodeDecodeError):
                continue
            files.setdefault(nr or rnr_of(p.name) or "?", {}).setdefault("reports", []).append(p)
        elif name.endswith(".csv") and name.startswith(("metingen", "route")):
            nr = rnr_of(p.name) or rnr_of(str(p.parent))
            if nr:
                files.setdefault(nr, {})["metingen" if name.startswith("metingen") else "route"] = p
    if not files:
        print("Geen rapporten of CSV's gevonden in", folder.resolve())
        return
    rows = []
    for nr in sorted(files):
        f = files[nr]
        try:
            met = read_csv(f["metingen"]) if "metingen" in f else []
            rou = read_csv(f["route"]) if "route" in f else []
        except (OSError, csv.Error, UnicodeDecodeError) as e:
            rows.append(({"nr": nr, "naam": "?", "rapport": "?", "punten": "-", "route": "-", "sensoren": "-", "tijd": "-", "hulp": "-"}, [f"CSV onleesbaar: {e}"]))
            continue
        res, notes = check_student(nr, f.get("reports", []), met, rou, verbose)
        if "metingen" not in f:
            notes.insert(0, f"metingen_{nr}.csv ontbreekt")
        if "route" not in f:
            notes.insert(0, f"route_{nr}.csv ontbreekt")
        rows.append((res, notes))
    keys = ["naam", "nr", "rapport", "punten", "route", "sensoren", "tijd", "hulp"]
    w = {k: max(len(k), *(len(str(r[k])) for r, _ in rows)) for k in keys}
    print("  ".join(k.ljust(w[k]) for k in keys))
    for r, notes in rows:
        print("  ".join(str(r[k]).ljust(w[k]) for k in keys))
        for n in notes:
            print("    - " + n)
    if "--csv" in sys.argv:
        out = Path(sys.argv[sys.argv.index("--csv") + 1])
        with out.open("w", newline="", encoding="utf-8") as fh:
            wr = csv.writer(fh, delimiter=";")
            wr.writerow(keys + ["opmerkingen"])
            for r, notes in rows:
                wr.writerow([r[k] for k in keys] + [" | ".join(notes)])
        print("CSV geschreven:", out)


if __name__ == "__main__":
    main()
