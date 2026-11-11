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

## 1.0.0 - 2026-11-11
Eerste genummerde versie.
- **Versienummer** bovenaan `check`; de virtuele robot toont de versie van de
  robotsoftware op Fenix > Status (`v1.9.3 · 2026-11-10 85ae1c8 (virtuele robot)`).
- **127.0.0.1 in plaats van localhost** in de simulator, Fenix, Lichtblick, de
  banner en `check`: onder WSL lukte na een wifi-onderbreking tot ~400 s geen
  nieuwe verbinding met `localhost`.
- **Simulator 1.4.0**: wereld *Klaslokaal* (9 × 7 m) met tafels en stoelen.
- **Virtuele robot 1.9.3**: Navigatie wacht op de beginpositie, camerabeeld op
  Rijden valt niet meer weg, de balk boven de kaart bedekt de kaart niet meer.
