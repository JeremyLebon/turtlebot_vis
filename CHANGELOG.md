# Changelog - turtlebot-vis

Versienummer in `VERSION` (MAJOR.MINOR.PATCH): PATCH = enkel fixes, MINOR =
nieuwe functies, MAJOR = grote herindeling. Verhogen vóór elke image-build
(`nobel86/turtlebot-rpi5-vis:jazzy`). De build zet het met de datum en de
commit in `TB_VIS_VERSION` ("v1.0.0 · 2026-11-11 abc1234"), zichtbaar
bovenaan `check` (bij elke start).

Eigen versies, los daarvan:
- **simulator** (htmlsim): `htmlsim/package.json`, in de titelbalk van de simulator;
- **virtuele robot**: de robotsoftware uit turtlebot_docker
  (`docker/virtual_robot/VERSION`, `tools/sync_virtual_robot.sh`), op de
  Fenix-pagina bij Status.

## 1.0.13 - 2026-11-12
- **`check` herkent een beschadigde image**: vergelijkt de grootte van alle
  bibliotheken met die bij het bouwen (<0,2 s). Een afgebroken pull liet bij
  een student afgekapte bestanden achter (twist_mux "file too short", SLAM
  tekende niets); een gewone `--pull` herstelt dat niet. Rode regel *Image*
  met de oplossing (`docker image rm` en opnieuw pullen).

## 1.0.12 - 2026-11-12
- **Poort van de webpagina's instelbaar** (`VIS_PORT`, standaard 8080): bezet
  een ander programma op Windows 8080, zet dan `VIS_PORT=8081` in `.env` (of
  `VIS_PORT=8081 ./start.sh`). Lichtblick, de simulator (`/sim/`),
  `/geheugen.html`, `check` en de banner volgen die poort. Na het wijzigen:
  `./start.sh` (maakt de container opnieuw aan).

## 1.0.11 - 2026-11-12
- **Python-bibliotheken voor het project van sessie 4** zitten in de image:
  pandas, reportlab, seaborn (en python3-venv). matplotlib, numpy en OpenCV
  zaten er al. Geen venv of pip meer nodig voor analyse en PDF-verslag.

## 1.0.10 - 2026-11-11
- **Virtuele robot 1.9.10**: coördinaten onder de muis op de kaart (x en y in
  het frame `map` naast de cursor, zonder een doel te sturen).

## 1.0.9 - 2026-11-11
- **Simulator 1.4.4**: HUD-regel **belasting**: CPU van de laptop (uit
  `/system_info` van de virtuele robot), sim-tijd t.o.v. echte tijd en fps;
  rood bij > 85 % CPU of als de simulator achterop raakt (< 0,9x).
  Costmaps en paden staan standaard uit en worden enkel opgehaald als het
  vinkje aan staat (rosbridge zette ze om naar JSON, ook als niemand keek).
- **Virtuele robot 1.9.9**: korte tekst bij **Echte fout** voor een oude kaart.

## 1.0.8 - 2026-11-11
- **Virtuele robot 1.9.8**: Navigatiestatus toont planner/controller en het
  doel; lokalisatie wordt "goed" tot 30 cm / 20° (σ); nieuwe rij **Echte
  fout** (AMCL tegenover de echte positie uit de simulator) voor kaarten die
  vanaf nu in de simulator bewaard worden.

## 1.0.7 - 2026-11-11
- **Virtuele robot 1.9.7: Nav2 verdraagt haperingen tot 1 s** (was 0,1-0,2 s):
  op een drukke laptop verloor de controller de robotpositie ("Unable to
  transform robot pose") en reed de robot door obstakels.
- **Simulator 1.4.3: ~40 % minder berichten naar rosbridge.** `/imu` 50 -> 20 Hz,
  `/ground_truth` en `/joint_states` 30 -> 10 Hz, `/joy` 20 -> 1 Hz zonder
  gamepad. `/clock`, `/odom`, `/tf` en `/scan` blijven. rosbridge (Python, één
  kern) liep op een Windows-laptop vol en gooide `/tf` en `/clock` weg.

## 1.0.6 - 2026-11-11
- **Simulator 1.4.2: TurtleBot3-wereld niet meer symmetrisch.** Zonder de
  schans (weg sinds 1.3.1) was de wereld puntsymmetrisch (zeshoek, 3 x 3
  palen, twee dozen in spiegelbeeld): een positie en dezelfde positie 180°
  gedraaid rond het midden gaven dezelfde lidarscan. AMCL kon zich dan
  vastzetten op het spiegelbeeld en Nav2 reed "naar de andere kant". Nu een
  vaste kist tegen de bovenste wand en een kort muurstuk linksonder. Oude
  kaarten van deze wereld missen die twee: beter een nieuwe kaart maken.

## 1.0.5 - 2026-11-11
- **Virtuele robot 1.9.6**: laag **Particles** (AMCL) op Navigatie en MPPI
  als standaard-controller op de pagina (zelfde als de echte robot 1.9.6).

## 1.0.4 - 2026-11-11
- **Simulator 1.4.1: zijbalk wegschuiven.** Knop *Zijbalk* in de titelbalk
  van de simulator: de 3D-wereld krijgt de hele breedte; nog eens klikken =
  terug. De keuze blijft bewaard.

## 1.0.3 - 2026-11-11
- **`./start.sh` in dezelfde tabel als `check`**: eerste blok *Opstarten* met
  Scripts (versie, git pull), Image, Container en de gestarte onderdelen;
  tijdens het werk één regel voortgang. Met een oudere image als lijst.

## 1.0.2 - 2026-11-11
- **`check` als tabel**: kolommen Onderdeel, Adres en Info, met een kop per
  blok; lange uitleg loopt door in de kolom Info (past ook in 80 kolommen).
- **`./start.sh --pull` doet ook `git pull`** (de scripts), vóór de image. Is
  `start.sh` zelf veranderd, dan start meteen de nieuwe versie. Lukt `git
  pull` niet (eigen wijzigingen, geen internet), dan een melding en verder
  met de huidige scripts.

## 1.0.1 - 2026-11-11
- **`check` en `./start.sh` leesbaarder**: blokken *Open in de browser*,
  *Verbinding*, *Robot* en *Deze laptop*; eerst de link, dan de uitleg (op een
  eigen regel); onderaan het besluit en de commando's. *Simulator* staat er nog
  maar één keer (versie en of hij open is).
- **`./start.sh`**: geen banner meer vóór en na `check` (zelfde info), geen
  regels van docker compose (enkel bij de eerste download), geen
  XAUTH-waarschuwing.
- **Banner** van de shell (bij `docker exec` en bij een andere robot): links
  en commando's onder elkaar.

## 1.0.0 - 2026-11-11
Eerste genummerde versie.
- **Versienummer** bovenaan `check`; de virtuele robot toont de versie van de
  robotsoftware op Fenix > Status (`v1.9.3 · 2026-11-10 85ae1c8 (virtuele robot)`).
- **`check` (ook bij `./start.sh`)** toont de versies van de simulator en de
  virtuele robot; de regels *Bridge* en *Bridges* zijn samengevoegd (een
  dubbele bridge komt als waarschuwing *Dubbel*).
- **127.0.0.1 in plaats van localhost** in de simulator, Fenix, Lichtblick, de
  banner en `check`: onder WSL lukte na een wifi-onderbreking tot ~400 s geen
  nieuwe verbinding met `localhost`.
- **Simulator 1.4.0**: wereld *Klaslokaal* (9 × 7 m) met tafels en stoelen.
- **Virtuele robot 1.9.3**: Navigatie wacht op de beginpositie, camerabeeld op
  Rijden valt niet meer weg, de balk boven de kaart bedekt de kaart niet meer.
