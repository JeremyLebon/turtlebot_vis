# Turtlebot Vis

Visualisatie-client (rviz2/rqt) voor op de studenten-laptop, om een
turtlebot van op afstand te bekijken/besturen.

## Op de laptop (Ubuntu of WSL op Windows)

De container praat via **Zenoh** met de robot: een rechtstreekse
TCP-verbinding naar de Zenoh-router van de robot (`10.0.<nr>.10:7447`),
zonder de multicast-discovery van DDS - lichter op de wifi en het werkt ook
door WSL2 heen.

### Vereisten

- Docker Desktop met WSL2-integratie ingeschakeld, of Docker rechtstreeks
  in WSL2 geïnstalleerd.
- WSLg (komt standaard mee met recente Windows 11/WSL2) voor
  GUI-doorsturing van rviz2/rqt.

### Setup

```bash
git clone --recurse-submodules https://github.com/JeremyLebon/turtlebot_vis.git
cd turtlebot_vis
```

### Starten

De image staat al op Docker Hub (`nobel86/turtlebot-rpi5-vis:zenoh`): eerst
pullen, **met internet** (ca. 4 GB):

```bash
docker compose pull
```

Daarna, met de laptop op de wifi van de robot (`TB-AP-<nr>`):

```bash
./start.sh
```

Dat start de container op de achtergrond, voert **`check`** uit (robot
gevonden, Zenoh-router bereikbaar, aantal topics, komt er echt data binnen,
zelfde ROS-versie als de robot, Lichtblick, bridge, display voor rviz2,
`ros2_ws`) en opent een shell in de container. `exit` verlaat de shell, de
container blijft draaien. Opties: `./start.sh --no-shell` (enkel starten en
check), `./start.sh --pull` (eerst de nieuwste image ophalen).

Nog een terminal in de container: **`./connect.sh`** (start de container
eerst als hij niet draait). Eén commando: `./connect.sh ros2 topic list`.

Met de hand kan ook: `docker compose up -d` en `docker exec -it turtlebot-vis
bash` - de eerste shell doet de check dan vanzelf. Opnieuw controleren:
`check` in de container.

**Welke robot** zoekt de container zelf (`docker/robot_select.sh`): de
access points zijn van elkaar afgeschermd, dus vanaf `TB-AP-<nr>` antwoordt
alleen robot `<nr>` op `10.0.<nr>.10:7447`. Die keuze (`ROS_DOMAIN_ID=<nr>`,
Zenoh-router `10.0.<nr>.10`) komt in `/tmp/robot.env` en elke shell neemt ze
over; bij het openen staat er bv. `TurtleBot 9 (ROS_DOMAIN_ID=9, ...)`.

- Andere wifi gekozen? De container merkt dat de robot weg is en zoekt
  opnieuw; open shells passen zich aan bij de volgende prompt (draaiende
  programma's zoals rviz2 opnieuw starten).
- In de container: `robot` = opnieuw zoeken, `robot 5` = vast robot 5.
- Meer dan één robot bereikbaar (bv. docent op een gerouteerd netwerk): kies
  met `robot <nr>` of zet `ROBOT_NR=<nr>` in `.env`.

In de container:

```bash
ros2 topic list          # zou de topics van de robot moeten tonen
rviz2                    # of: ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

### HTML-simulator (thuis, huiswerk)

```bash
./start.sh --htmlsim
```

Start in de container een eigen Zenoh-router en rosbridge; surf naar
<http://localhost:8080/sim/>. De **TB3 Lite Sim 3D** (TurtleBot3 in de browser,
met de automatische tests van de huiswerkopdrachten) verbindt vanzelf. Je
nodes draai je in de container (`./connect.sh`), zoals op de robot. Geen
internet nodig. Terug naar de echte robot: `robot` in de container. Gazebo in
de container: `./start.sh --sim`. Broncode en tests van de simulator:
`htmlsim/` (zie `htmlsim/README.md`).

### Lichtblick (visualisatie in de browser)

De container start ook **Lichtblick** (open-source web-versie van Foxglove
Studio) en **foxglove_bridge**: surf vanuit Windows naar
<http://localhost:8080>. De pagina verbindt vanzelf met de bridge
(`ws://localhost:8765`) en opent een layout met 3D (kaart, scan, robot, pad,
doel zetten), camera en teleop (`/cmd_vel_web`). Handig als het rviz2-venster
onder WSLg niet verschijnt. De bridge haalt de topics via Zenoh, net als
rviz2: geen extra belasting op de robot. Logs: `/tmp/vis_services/` in de
container. "Connection failed" in Lichtblick: kijk in
`/tmp/vis_services/foxglove_bridge.log` - wacht de bridge nog op de robot,
dan is de robot niet bereikbaar (verkeerde wifi, robot uit, verkeerd vast
nummer).

### Problemen oplossen

- Eerst `check` in de container: die zegt meestal wat er mis is.
- `ros2 topic list` toont niets: welke robot meldt de shell bij het openen
  (`TurtleBot <nr>`)? Geen of de verkeerde: laptop op de juiste wifi
  `TB-AP-<nr>`, robot aan? Dan `robot` (opnieuw zoeken). `ROS_DOMAIN_ID`
  moet exact het robotnummer zijn (onderdeel van Zenoh's
  topic-key-namespacing) - de automatische keuze zorgt daarvoor. Draait de
  bringup op de robot? Anders `ros2 daemon stop` en opnieuw proberen.
- Geen GUI-venster (rviz2/rqt): gebruik Lichtblick (hierboven), of probeer
  `LIBGL_ALWAYS_SOFTWARE=1 rviz2` (tekenen via de CPU i.p.v. de GPU).
- WSLg nakijken: controleer of WSLg werkt met een simpele
  test buiten Docker (bv. `xeyes` of een andere Linux GUI-app rechtstreeks
  in WSL), vóór je het in de container probeert.
- De compose gebruikt `network_mode: host`. Op Docker Desktop (Windows)
  moet host-networking expliciet aanstaan: Docker Desktop instellingen ->
  Resources -> Network -> "Enable host networking". Staat dit uit, dan
  lijkt de container te starten maar is poort 7447 van buiten de container
  niet bereikbaar zoals verwacht.

## Camera kalibreren (intrinsiek)

Nodig voor alles wat afstanden uit het camerabeeld haalt (AprilTag, lidar op
het camerabeeld). Eenmalig per robot en per resolutie:

1. Print een dambord (bv. 9x7 vakjes = 8x6 binnenhoeken, vakjes van 25 mm -
   meet na!) en kleef het op iets vlaks.
2. Laptop verbonden met de robot, `turtlebot-vis` draait, en X11 werkt
   (de kalibratie opent een venster).
3. `./calibrate_camera.sh 8x6 0.025`
4. Beweeg het dambord tot X, Y, Size en Skew groen zijn, dan CALIBRATE en
   COMMIT. De robot bewaart de kalibratie zelf; de Systeem-pagina toont
   "Intrinsiek gekalibreerd".
