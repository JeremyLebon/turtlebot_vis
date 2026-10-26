# TB3 Lite Sim 3D

TurtleBot3-simulator die volledig in de browser draait: three.js voor het beeld, Rapier (WebAssembly) voor de fysica,
rosbridge v2 voor de koppeling met ROS 2. Lichter dan Gazebo, geen GPU of installatie nodig bij de student.

## In turtlebot-vis (standaard)

```bash
./start.sh --htmlsim        # in de map turtlebot_vis, of in de container: robot htmlsim
```
Dan in de browser <http://localhost:8080/sim/>: de simulator verbindt vanzelf met rosbridge in de container
(`ws://localhost:9090`). Werkt zonder internet (three.js en Rapier zitten in de image, `python3 build.py --local`).
Je eigen nodes draai je in de container (`./connect.sh`), zoals bij de echte robot. Terug naar de robot: `robot`.

## Gebruiken (los, zonder turtlebot-vis)

1. `dist/tb3-lite-sim-3d.html` lokaal openen in Chrome of Edge (internet nodig voor three.js, Rapier en lettertypen via CDN).
2. In de ROS 2-container (poort 9090 open):
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```
3. In de pagina op **Verbinden** klikken. `/cmd_vel` staat standaard op **automatisch**: de simulator herkent via rosapi
   of je script `Twist` (Humble, labo-robot) of `TwistStamped` (Jazzy) publiceert.
4. **Klaar voor de echte robot?** controleert wat op de labo-robot (Humble, Zenoh) anders is: type van `/cmd_vel`,
   nodes die simulator-topics lezen (`/ground_truth`, `/sim/battery_state`, `/student_id`) en gebruik van `/clock`.

Een gepubliceerde versie (artifact, `dist/artifact.html`) werkt ook, maar dan blokkeert de browser de verbinding naar
`localhost`: rijden, werelden, URDF laden en opnemen kunnen, ROS-koppeling niet.

## Wat erin zit

| Onderdeel | Details |
|---|---|
| Robot | Ingebouwde Burger (primitieven, met camera en 3D-lidar) of eigen URDF met STL/DAE/OBJ-meshes; mini-xacro (`arg`, `property`, `${}`) |
| Fysica | Rapier 200 Hz, wielen op scharnieren met snelheidsmotor + versnellingsbegrenzing, rem bij stilstand, joints uit URDF (revolute/prismatic/continuous) met PI-positieregeling |
| Sensoren | `/scan` (LDS-01/LDS-02/eigen bereik, ruis, geen echo = 0.0 of inf), `/imu`, `/odom` (perfect of wielencoders), `/joint_states`, `/camera/image_raw/compressed` + `camera_info`, `/points` (16 lagen) |
| ROS | `/clock` (sim-tijd), `/tf`, `/tf_static`, `/ground_truth`, `/student_id`; leest `/cmd_vel`, `/joint_commands`, `/rosout`, `/map`, costmaps, `/plan`, `/local_plan`, `/robot_description` |
| Nav2/SLAM | kaart en costmaps op de vloer via `map → odom`, Startpositie (`/initialpose`) en Doel (`/goal_pose`) aanklikken |
| Werelden | TurtleBot3-wereld, Fabriekshal (16 × 12 m), Lege kamer; eigen werelden als JSON (zie `worlds/`) |
| Labo-robot | `/cmd_vel`-type automatisch herkend (Twist/TwistStamped) met melding bij een verkeerd type; batterij-topic kiesbaar (`/sim/battery_state` of `/battery_state`); controle "Klaar voor de echte robot?" |
| Opdrachten | automatische tests voor taak 1 (obstakelalarm + 4 sectoren), taak 2 (batterij), taak 3 (90° + vierkant); proefrapport met controlecode, opname met testpaneel |
| Identificatie | student + code (SHA-256 over naam/nr/wereld/datum), watermerk op vloer en camerabeeld |

## Docent: rapporten controleren

```bash
python3 tools/controleer_rapporten.py ingeleverd/ --csv overzicht.csv
```
Controleert per rapport de controlecode, de studentcode en de SHA-256 van de opname.

## Ontwikkelen

```bash
npm install            # alleen nodig voor de tests
python3 build.py       # src/ → dist/tb3-lite-sim-3d.html   (--artifact voor de artifact-variant)
npm test               # fysica- en logicatests in Node
npm run test:e2e -- t1 # volledige browsertest met nep-rosbridge (headless Chromium)
TWIST=1 COMPAT=1 npm run test:e2e -- t3   # studentscript als Twist (Humble) + compatibiliteitscontrole
```

Bronbestanden in `src/`: `template.html` (HTML + paneel + 3D-weergave + rosbridge), `core.js` (fysica, URDF, sensoren,
werelden; zonder DOM, dus testbaar in Node), `tasks.js` (opdrachten, opname, rapport), `style.css`.
Zie `CLAUDE.md` voor de architectuur en de valkuilen.
