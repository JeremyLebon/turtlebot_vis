# Turtlebot Vis

Visualisatie-client (rviz2/rqt) voor op de studenten-laptop, om een
turtlebot van op afstand te bekijken/besturen.

## Op WSL (Windows) - Zenoh branch

Deze branch (`zenoh`) is aangepast om via Zenoh te verbinden i.p.v.
CycloneDDS. Reden: DDS werkt op zich ook door WSL2 heen, maar de
multicast-discovery-mechanica van DDS geeft merkbaar veel netwerk-overhead
(zeker relevant op een gedeelde/beperkte wifi-verbinding met meerdere
robots/studenten tegelijk). Zenoh in client-modus (rechtstreekse unicast
TCP-verbinding naar de robot's Zenoh-router, geen multicast-discovery
nodig) is hier lichter. De robot moet uiteraard al op de `zenoh`/
`raspios-migration`-branch draaien (met een actieve Zenoh-router op poort
7447) - zie `turtlebot_setup/docs/zenoh-migration.md`.

### Vereisten

- Docker Desktop met WSL2-integratie ingeschakeld, of Docker rechtstreeks
  in WSL2 geïnstalleerd.
- WSLg (komt standaard mee met recente Windows 11/WSL2) voor
  GUI-doorsturing van rviz2/rqt.

### Setup

```bash
git clone -b zenoh --recurse-submodules https://github.com/JeremyLebon/turtlebot_vis.git
cd turtlebot_vis
```

Pas `.env` aan: zet `ROBOT_ZENOH_IP` op het IP van de turtlebot waarmee je
verbindt (en `ROS_DOMAIN_ID` indien nodig - moet niet per se matchen met
de robot, maar overzichtelijker als het wel matcht).

### Starten

De image staat al op Docker Hub (`nobel86/turtlebot-rpi5-vis:zenoh`), dus
pullen is genoeg - lokaal bouwen (`docker compose build`) kan ook, maar is
niet nodig:

```bash
docker compose pull
docker compose up -d
docker exec -it turtlebot-vis bash
```

In de container:

```bash
source /root/turtlebot3_ws/install/setup.bash
ros2 topic list          # zou de topics van de robot moeten tonen
rviz2                    # of: ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

### Problemen oplossen

- `ros2 topic list` toont niets: meest voorkomende oorzaak is een
  **`ROS_DOMAIN_ID`-mismatch** met de robot - dit moet exact matchen,
  ook al is de Zenoh-verbinding zelf oké (domain ID is onderdeel van
  Zenoh's topic-key-namespacing, zelfde isolatie-effect als bij DDS).
  Controleer daarnaast `echo $ZENOH_CONFIG_OVERRIDE` in de container (moet
  het juiste IP tonen), en of de robot's Zenoh-router effectief draait en
  bereikbaar is (`ping <ROBOT_ZENOH_IP>` vanuit WSL, en dat je op hetzelfde
  netwerk/dezelfde robot-AP zit).
- Geen GUI-venster (rviz2/rqt): controleer of WSLg werkt met een simpele
  test buiten Docker (bv. `xeyes` of een andere Linux GUI-app rechtstreeks
  in WSL), vóór je het in de container probeert.
- De compose gebruikt `network_mode: host`. Op Docker Desktop (Windows)
  moet host-networking expliciet aanstaan: Docker Desktop instellingen ->
  Resources -> Network -> "Enable host networking". Staat dit uit, dan
  lijkt de container te starten maar is poort 7447 van buiten de container
  niet bereikbaar zoals verwacht.

### Terugvallen op CycloneDDS

Zet in `docker-compose.yaml` de `RMW_IMPLEMENTATION`-regel om naar
`rmw_cyclonedds_cpp` (staat als commentaar klaar) - werkt ook via WSL2,
maar verwacht dan meer netwerk-verkeer dan met Zenoh.
