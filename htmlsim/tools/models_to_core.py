#!/usr/bin/env python3
"""Zet models/models.json als MODEL_INFO in src/core.js (maten, collider, label, temperatuur). Opnieuw draaien na een
wijziging aan de modellen. De GLB's zelf sluit build.py in de pagina in."""
import json, re
from pathlib import Path
here = Path(__file__).resolve().parent.parent
man = json.loads((here / "models" / "models.json").read_text())
info = {k: {kk: v[kk] for kk in ("size", "collider", "label", "temp") if kk in v} for k, v in man.items()}
core = here / "src" / "core.js"; s = core.read_text()
line = "const MODEL_INFO = " + json.dumps(info, separators=(",", ":"), ensure_ascii=False) + ";"
s = re.sub(r"^const MODEL_INFO = .*;$", lambda m: line, s, flags=re.M) if re.search(r"^const MODEL_INFO = ", s, re.M) else s.rstrip("\n") + "\n" + line + "\n"
core.write_text(s); print(f"MODEL_INFO: {len(info)} modellen")
