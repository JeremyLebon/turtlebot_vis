# CLAUDE.md — TB3 Lite Sim 3D

Context voor Claude Code. Antwoorden en UI-teksten in het Nederlands (Vlaams), technisch en direct.
Doelpubliek: studenten Mobiele Automatisering / ROS 2 (VIVES), docent bouwt eigen labo-tools.

## Build en tests

- `python3 build.py` voegt `src/template.html` + `src/style.css` + `src/core.js` + `src/tasks.js` samen tot
  `dist/tb3-lite-sim-3d.html` (placeholders `/*__CSS__*/`, `/*__CORE__*/`, `/*__TASKS__*/`).
  `--artifact` geeft `dist/artifact.html` zonder doctype/head/body.
- Na elke wijziging: `python3 build.py`, dan syntaxcheck van het module-script:
  `python3 -c "import re;s=open('dist/tb3-lite-sim-3d.html').read();open('/tmp/m.mjs','w').write(re.findall(r'<script type=\"module\">(.*?)</script>',s,re.S)[0])" && node --check /tmp/m.mjs`
- `npm test` (tests/test_core.mjs): fysica in Node. Draai dit na elke wijziging in `src/core.js`.
- `npm run test:e2e -- <taak> [--bad]` (`TWIST=1`: studentscript publiceert Twist; `COMPAT=1`: compatibiliteitscontrole): echte browser (headless Chromium via @sparticuz/chromium, swiftshader)
  met nep-rosbridge op poort 9090 en gesimuleerde studentnodes. CDN-verzoeken worden uit `node_modules` bediend.
  Headless loopt de simulatie ~4× trager dan echte tijd; tests moeten daarom op sim-tijd werken.

## Architectuur

- **Assenstelsel:** overal ROS-conventie (x vooruit, y links, z omhoog), ook in three.js (`DEFAULT_UP = (0,0,1)`).
  Rapier- en three-cilinders liggen langs y → `QX90` / `rotateX(π/2)` om naar URDF-z te gaan.
- **core.js** heeft geen DOM. `createSim(RAPIER)` geeft wereld, robot, sensoren. `parseURDF(text, DOMParser)` krijgt
  de parser mee, zodat Node (xmldom) kan testen.
- **Robot uit URDF:** links met fixed joints worden één star lichaam. Alle lichamen hebben bij de spawn dezelfde
  oriëntatie (rootframe); `linkOff[link]` geeft de link t.o.v. zijn lichaam. Reden: Rapier's revolute/prismatic
  JointData heeft maar één as voor beide lichamen, dus een joint-origin met rotatie werkt alleen zo.
  Massa en inertia uit `<inertial>` via `setAdditionalMassProperties` (colliders density 0, enkel diagonaal).
  Robot botst niet met zichzelf (collision groups), lidar-stralen zien alleen de wereld.
- **Aandrijving:** aandrijfwielen worden automatisch herkend (continuous, as ≈ ±y, links/rechts). Wiel-colliders zijn
  bollen (cilinders geven schuurwrijving bij draaien ter plaatse). Snelheidsmotor (factor 500) + versnellings-
  begrenzing 2 m/s². Overige revolute/prismatic joints: positiemotor (20000/300) + integrerende term.
- **Rem bij stilstand** (step in core.js): bij nulcommando en uitgelopen wielen worden de horizontale snelheid en de
  pose (x, y, yaw) vastgehouden. Zonder rem kruipt de robot ~2°/40 s door trillingen van de bolcontacten. Niet weghalen.
- **Contacten:** `contact_natural_frequency = 120`, 8 solver-iteraties. De standaard (30 Hz) laat robots mm's zakken.
- **Odometrie:** `params.odomSource` "truth" (standaard, zoals de Gazebo diff-drive-plugin) of "wheels" (encoders,
  met straalfout L/R). `/ground_truth` = echte pose t.o.v. de startpose, frame `sim_world`.
- **Tijd:** `/clock` uit de simulatie; alle time-outs (cmd_vel 0,5 s, versheid 0,3 s) in sim-tijd, niet in wandtijd.
  Bij een verborgen tabblad stopt `requestAnimationFrame` en dus ook de sim-tijd.
- **Scan "geen echo":** 0.0 (echte LDS) of inf (Gazebo). inf gaat als `Infinity` in de JSON (rosbridge/Python accepteert dat).
- **Kaartlagen:** OccupancyGrid als DataTexture, geplaatst via `map → odom` uit `/tf` (zonder throttle subscriben).
  Lagen op 8/14/20 mm met polygonOffset; camera-near 0,05 (anders z-fighting met de vloer).
- **Werelden:** data-gedreven (`loadWorld(def)`), elementen `wall/box/cyl/rack/conveyor/pallet/movable/mark`.
  Dozen 35 cm hoog: lidar van de Burger zit op 18 cm, lagere objecten scheert de scan voorbij.
- **Opdrachten (tasks.js):** log van de studentnode via `/rosout` (rcl_interfaces/Log), match op berichttekst
  (niet de nodenaam: "battery_warning" bevat "warn"). Taak 1 kijkt naar **links** (`T1_SECTOR` = 90° ± 15°, sinds 28-10;
  de voorsector staat uitgewerkt in het labo) en koppelt zich enkel aan een node die over obstakels logt (`acceptNode`). Taak 2 publiceert zelf `/battery_state` (het paneel pauzeert dan) en controleert via
  `/rosapi/publishers` dat er geen andere publisher is. Taak 3 meet met de grondwaarheid; vierkant: hoeken = stukken met
  yaw-verandering, zijden = afstand tussen draaipunten (robuust voor haperende commando's).
- **/cmd_vel-type (tasks.js, onderaan):** rosbridge abonneert een topic met één type; een ander type komt in ROS 2
  gewoon niet aan. In "automatisch" abonneert de simulator pas als `/rosapi/publishers` een publisher toont (zonder
  rosbridge/rosapi) en neemt het type van `/rosapi/topic_type`. Valt de publisher weg, dan wordt het abonnement
  opgeheven: anders geeft rosapi het type van ons eigen abonnement terug (rosapi geeft enkel het eerste type).
  Poll elke 0,7 s (wandtijd). Handmatig type: melding als het herkende type afwijkt of er na 4 s niets binnenkomt.
  Zonder rosapi: na 3 mislukte calls TwistStamped. `cmdState.lastMsg` wordt gezet in `ws.onmessage`.
- **Labo-robot:** de echte robots draaien Jazzy over Zenoh (`/cmd_vel` = TwistStamped, batterij op `/battery_state`); tot de overschakeling nog Humble (Twist).
  Taak 2 gebruikt `batTopic()` = `/battery_state` (28-10: `/sim/battery_state` en `homework/simulation_sensors.py` zijn weg).
  `compatCheck()` meldt nodes die `/ground_truth`, `/student_id` of `/clock` lezen.
- **Rapport:** `{rapport, inhoud, controle}`; `controle = sha256(SALT + "|" + inhoud)[:16]`, `inhoud` is de exacte
  JSON-string (zodat Python dezelfde bytes hasht). Opname-SHA-256 staat in het rapport. SALT `tb3-lite-sim/vives`
  staat ook in `tools/controleer_rapporten.py`; beide samen aanpassen. SHA-256 heeft een JS-terugval voor
  pagina's buiten een secure context.
- **Sensoren zoals de echte TB3 (template, na `/*__TASKS__*/`):** `/sensor_state` 10 Hz (encoders = wielhoek × 4096/2π, knoppen SW1/SW2),
  `/battery_state` 1 Hz (3S-LiPo-curve `LIPO`, zakt met de gereden afstand of met de hand), service `/sound` via
  `advertise_service` (piep met WebAudio). Onder 11,0 V: `currentCmd()` geeft 0 (motoren uit) en piepen. Tijdens Taak 2 met
  batterij-topic `/battery_state` publiceert de simulator die niet zelf. Geen `NaN` in berichten: JSON maakt er `null` van en
  rosbridge weigert het bericht.
- **Velden in de wereld:** `fields: [{kind: gas|heat|humidity|light, x, y, strength, sigma}]` in de wereld-JSON (ook in `WORLDS` in core.js);
  som van Gauss-bellen + basiswaarde (400 ppm, 20 °C, 150 lux) + ruis op `/sim/gas_value`, `/sim/temperature`, `/sim/humidity`, `/sim/illuminance`
  (1 Hz, plaats = grondwaarheid). Bronnen in 3D standaard verborgen.
- **Gamepad → `/joy`:** knoppen in de pagina of een echte gamepad (Gamepad-API, standaard-indeling) naar de indeling van
  joy_node met de F710 (XInput): 8 assen (links/op = +, triggers rust 1.0), 11 knoppen (A=0 … RB=5), 20 Hz sim-tijd.
  Met LB rijdt de robot met de sticks (zoals teleop_twist_joy, RB = turbo); voorrang na toetsenbord/knoppen in beeld.
  `setPointerCapture` na het registreren van de knop en in try (gooit bij nagebootste klikken).
- **LDS-02-preset:** `LIDAR.nRange = [220, 230]`: elke scan een willekeurig aantal punten, zoals de echte lidar.
- **Geen toestelherkenning:** toestel-ID en browser-/computervingerafdruk zijn eruit (GDPR, 23-10); niet terugzetten.
- **turtlebot-vis:** `build.py --local` → `dist/index.html` met three.js en Rapier uit `./vendor` (Dockerfile van turtlebot-vis), geserveerd op `http://localhost:8080/sim/`; daar verbindt de pagina vanzelf (`robot htmlsim` start rosbridge).
- **Opname:** samengesteld canvas (WebGL + HUD + camera + testpaneel + balk) via `captureStream`, of `getDisplayMedia`
  voor het hele scherm. `drawComposite()` direct na `renderer.render` (anders leeg WebGL-beeld).

## Valkuilen

- Geen `prevRobot`, `keys`, enz. gebruiken vóór hun declaratie op module-niveau tijdens het opstarten (TDZ):
  functies die bij het laden lopen (applyTheme, buildWorldMeshes) mogen alleen al gedeclareerde `const`s gebruiken.
- `deg` bestaat al in template.html (stringformatter); in tasks.js heet de omzetting `r2d`.
- Artifact-CSP: alleen scripts van cdnjs/jsdelivr/unpkg, geen WebSocket naar localhost.
- Rapier-versie vastgepind op 0.14.0 (API: `timeOfImpact ?? toi`, `configureMotorModel`, geen frame1/frame2 bij joints).
