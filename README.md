# Turtlebot Vis

Visualisatie-client (rviz2/rqt) voor op de studenten-laptop, om een
turtlebot van op afstand te bekijken/besturen.

## Op WSL (Windows) - Zenoh branch

Deze branch (`zenoh`) is aangepast om via Zenoh te verbinden i.p.v.
CycloneDDS. Reden: DDS-multicast-discovery werkt niet door WSL2's virtuele
netwerk heen (zie `turtlebot_docker/README.md`), maar Zenoh in
client-modus (rechtstreekse unicast TCP-verbinding naar de robot's
Zenoh-router) wel. De robot moet uiteraard al op de `zenoh`/
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

### Bouwen en starten

```bash
docker compose build
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

- `ros2 topic list` toont niets: controleer `echo $ZENOH_CONFIG_OVERRIDE`
  in de container (moet het juiste IP tonen), en of de robot's
  Zenoh-router effectief draait en bereikbaar is
  (`ping <ROBOT_ZENOH_IP>` vanuit WSL, en dat je op hetzelfde netwerk/
  dezelfde robot-AP zit).
- Geen GUI-venster (rviz2/rqt): controleer of WSLg werkt met een simpele
  test buiten Docker (bv. `xeyes` of een andere Linux GUI-app rechtstreeks
  in WSL), vóór je het in de container probeert.

### Terugvallen op CycloneDDS

Zet in `docker-compose.yaml` de `RMW_IMPLEMENTATION`-regel om naar
`rmw_cyclonedds_cpp` (staat als commentaar klaar) - werkt enkel als je
niet via WSL2 verbindt (dus rechtstreeks Linux, of de robot op hetzelfde
fysieke netwerk zonder WSL ertussen).
