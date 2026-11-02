#!/usr/bin/env python3
"""Bouwt de simulator tot één HTML-bestand.

  python3 build.py            → dist/tb3-lite-sim-3d.html   (lokaal openen, met rosbridge)
  python3 build.py --artifact → dist/artifact.html          (zonder doctype/head/body, voor publicatie als artifact)
  python3 build.py --local    → dist/index.html             (turtlebot-vis: three.js en Rapier uit ./vendor, geen internet)
"""
import datetime, json, re, subprocess, sys
from pathlib import Path
here = Path(__file__).parent
src = lambda n: (here / "src" / n).read_text(encoding="utf-8")
html = src("template.html").replace("/*__CSS__*/", src("style.css")).replace("/*__CORE__*/", src("core.js")).replace("/*__TASKS__*/", src("tasks.js"))


def last_update():
    """Datum van de laatste commit in htmlsim/, vandaag bij niet-gecommitte wijzigingen
    in src/. Zonder git (docker build: COPY htmlsim zonder .git): de builddatum."""
    try:
        git = lambda *a: subprocess.run(["git", *a], cwd=here, capture_output=True, text=True, check=True).stdout.strip()
        if not git("status", "--porcelain", "--", "src", "package.json"):
            return datetime.date.fromisoformat(git("log", "-1", "--format=%cs", "--", "."))
    except (OSError, subprocess.CalledProcessError, ValueError):
        pass
    return datetime.date.today()


# Titelbalk: versie uit package.json (ophogen bij elke update voor de studenten)
html = html.replace("__VERSIE__", json.loads((here / "package.json").read_text())["version"])
html = html.replace("__DATUM__", last_update().strftime("%d-%m-%Y"))
(here / "dist").mkdir(exist_ok=True)
if "--local" in sys.argv:
    # turtlebot-vis: zonder internet (robotwifi), three.js en Rapier uit ./vendor
    # (de Dockerfile haalt ze bij het bouwen), lettertypes van het systeem
    html = html.replace("https://cdn.jsdelivr.net/npm/three@0.160.0/", "./vendor/three/")
    html = html.replace("https://cdn.jsdelivr.net/npm/@dimforge/rapier3d-compat@0.14.0/", "./vendor/rapier/")
    html = re.sub(r'<link[^>]*fonts\.(googleapis|gstatic)\.com[^>]*>\s*', "", html)
    assert "cdn.jsdelivr" not in html and "googleapis" not in html, "nog een CDN-verwijzing"
    out = here / "dist" / "index.html"
elif "--artifact" in sys.argv:
    for pat in [r"<!doctype html>\s*", r"<html[^>]*>\s*", r"</html>\s*", r"<head>\s*", r"</head>\s*", r"<body>\s*", r"</body>\s*",
                r'<meta charset="utf-8">\s*', r'<meta name="viewport"[^>]*>\s*']:
        html = re.sub(pat, "", html, flags=re.I)
    out = here / "dist" / "artifact.html"
else:
    out = here / "dist" / "tb3-lite-sim-3d.html"
out.write_text(html, encoding="utf-8")
print(f"geschreven: {out} ({out.stat().st_size // 1024} kB)")
