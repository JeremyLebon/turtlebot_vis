#!/usr/bin/env python3
"""Controleert proefrapporten van TB3 Lite Sim 3D.

Gebruik:  python3 controleer_rapporten.py <map met .json-rapporten en opnames>

Per rapport wordt nagegaan:
  - controlecode: SHA-256 over de exacte rapportinhoud klopt (inhoud niet aangepast)
  - leesbaar deel ("rapport") is gelijk aan de gecontroleerde inhoud
  - studentcode klopt voor naam, nummer, wereld en datum
  - opname: bestand aanwezig en SHA-256 gelijk aan de waarde in het rapport
Daarna volgt een overzicht per student en taak, en optioneel een CSV (--csv overzicht.csv).

Let op: dit bewijst dat rapport en opname bij elkaar horen en niet achteraf zijn aangepast.
Het is geen sluitende beveiliging: de simulator draait bij de student zelf.
"""
import csv
import hashlib
import json
import sys
from pathlib import Path

SALT = "tb3-lite-sim/vives"
B32 = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789"


def student_code(naam, nr, wereld, datum):
    s = "|".join([SALT, naam.strip().lower(), nr.strip().lower(), wereld, datum])
    return "".join(B32[b % 32] for b in hashlib.sha256(s.encode()).digest()[:6])


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    inhoud, controle = data.get("inhoud", ""), data.get("controle", "")
    r = json.loads(inhoud) if inhoud else {}
    ok_hash = hashlib.sha256((SALT + "|" + inhoud).encode("utf-8")).hexdigest()[:16] == controle
    ok_same = data.get("rapport") == r
    st = r.get("student", {})
    ok_code = bool(st) and student_code(st.get("naam", ""), st.get("nr", ""), r.get("wereld", ""), st.get("datum", "")) == st.get("code")
    video = r.get("opname")
    if video:
        vpath = path.with_name(video["bestand"])
        video_status = ("ok" if sha256_file(vpath) == video["sha256"] else "ANDER BESTAND") if vpath.exists() else "ontbreekt"
    else:
        video_status = "geen"
    echt = ok_hash and ok_same and ok_code
    return {
        "bestand": path.name, "naam": st.get("naam", "?"), "nr": st.get("nr", ""), "taak": r.get("taak", {}).get("id", "?"),
        "resultaat": r.get("resultaat", "?"), "datum": st.get("datum", ""), "wereld": r.get("wereld", ""),
        "echt": "ok" if echt else "AANGEPAST" if not ok_hash or not ok_same else "CODE FOUT",
        "opname": video_status,
        "mislukt": "; ".join(c["controle"] for c in r.get("controles", []) if not c.get("ok") and not c.get("info")),
    }


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    folder = Path(args[0] if args else ".")
    rows = [check(p) for p in sorted(folder.glob("*.json")) if '"tb3-lite-sim-rapport' in p.read_text(encoding="utf-8", errors="ignore")]
    if not rows:
        print("Geen rapporten gevonden in", folder.resolve())
        return
    w = {k: max(len(k), *(len(str(r[k])) for r in rows)) for k in ["naam", "nr", "taak", "resultaat", "echt", "opname"]}
    print("  ".join(k.ljust(w[k]) for k in w))
    for r in rows:
        print("  ".join(str(r[k]).ljust(w[k]) for k in w) + (f"   ✗ {r['mislukt']}" if r["mislukt"] else ""))
    if "--csv" in sys.argv:
        out = Path(sys.argv[sys.argv.index("--csv") + 1])
        with out.open("w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()), delimiter=";")
            wr.writeheader(); wr.writerows(rows)
        print("CSV geschreven:", out)


if __name__ == "__main__":
    main()
