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
