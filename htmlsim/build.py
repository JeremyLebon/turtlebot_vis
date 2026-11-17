#!/usr/bin/env python3
"""Bouwt de simulator en de robot-editor, elk tot één HTML-bestand.

  python3 build.py            → dist/tb3-lite-sim-3d.html + dist/robot-editor.html   (lokaal openen)
  python3 build.py --artifact → dist/artifact.html          (simulator zonder doctype/head/body)
  python3 build.py --local    → dist/index.html + dist/editor.html   (turtlebot-vis: three.js en Rapier uit ./vendor)
  python3 build.py --site     → dist/site/index.html + dist/site/editor.html   (CDN, voor een artifact met twee pagina's)
"""
import datetime, json, re, subprocess, sys
from pathlib import Path
here = Path(__file__).parent
src = lambda n: (here / "src" / n).read_text(encoding="utf-8")


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


local, site = "--local" in sys.argv, "--site" in sys.argv
SIM_FILE, EDITOR_FILE = ("index.html", "editor.html") if (local or site) else ("tb3-lite-sim-3d.html", "robot-editor.html")
version = json.loads((here / "package.json").read_text())["version"]
date = last_update().strftime("%d-%m-%Y")


def finish(html):
    # Titelbalk: versie uit package.json (ophogen bij elke update voor de studenten)
    html = html.replace("__VERSIE__", version).replace("__DATUM__", date)
    html = html.replace("__EDITOR_URL__", EDITOR_FILE).replace("__SIM_URL__", SIM_FILE)
    if local:
        # turtlebot-vis: zonder internet (robotwifi), three.js en Rapier uit ./vendor
        # (de Dockerfile haalt ze bij het bouwen), lettertypes van het systeem
        html = html.replace("https://cdn.jsdelivr.net/npm/three@0.160.0/", "./vendor/three/")
        html = html.replace("https://cdn.jsdelivr.net/npm/@dimforge/rapier3d-compat@0.14.0/", "./vendor/rapier/")
        html = re.sub(r'<link[^>]*fonts\.(googleapis|gstatic)\.com[^>]*>\s*', "", html)
        assert "cdn.jsdelivr" not in html and "googleapis" not in html, "nog een CDN-verwijzing"
    return html


sim = src("template.html").replace("/*__CSS__*/", src("style.css")).replace("/*__CORE__*/", src("core.js")).replace("/*__TASKS__*/", src("tasks.js"))
editor = (src("editor.html").replace("/*__CSS__*/", src("style.css")).replace("/*__EDCSS__*/", src("editor.css"))
          .replace("/*__EDITOR__*/", src("editor.js").replace("/*__CORE__*/", src("core.js"))))
sim, editor = finish(sim), finish(editor)
out_dir = here / "dist" / ("site" if site else "")
out_dir.mkdir(parents=True, exist_ok=True)
if "--artifact" in sys.argv:
    for pat in [r"<!doctype html>\s*", r"<html[^>]*>\s*", r"</html>\s*", r"<head>\s*", r"</head>\s*", r"<body>\s*", r"</body>\s*",
                r'<meta charset="utf-8">\s*', r'<meta name="viewport"[^>]*>\s*']:
        sim = re.sub(pat, "", sim, flags=re.I)
    outs = [(out_dir / "artifact.html", sim)]
else:
    outs = [(out_dir / SIM_FILE, sim), (out_dir / EDITOR_FILE, editor)]
for out, html in outs:
    out.write_text(html, encoding="utf-8")
    print(f"geschreven: {out} ({out.stat().st_size // 1024} kB)")
