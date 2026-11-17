# CLAUDE.md — TB3 Lite Sim 3D

Context voor Claude Code. Antwoorden en UI-teksten in het Nederlands (Vlaams), technisch en direct.
Doelpubliek: studenten Mobiele Automatisering / ROS 2 (VIVES), docent bouwt eigen labo-tools.

## Build en tests

- `python3 build.py` voegt `src/template.html` + `src/style.css` + `src/core.js` + `src/tasks.js` samen tot
  `dist/tb3-lite-sim-3d.html` (placeholders `/*__CSS__*/`, `/*__CORE__*/`, `/*__TASKS__*/`), en
  `src/editor.html` + `editor.css` + `editor.js` (met core.js) tot `dist/robot-editor.html`.
  `--artifact` geeft `dist/artifact.html` zonder doctype/head/body. `--local` → `index.html` + `editor.html` (vendor),
  `--site` → `dist/site/index.html` + `editor.html` (CDN, artifact met twee pagina's). `__SIM_URL__`/`__EDITOR_URL__`
  worden per variant ingevuld.
- Titelbalk "TurtleBot Sim v<versie> · <datum>": versie uit `package.json` (ophogen bij een update voor de studenten),
  datum = laatste commit in htmlsim/ (vandaag bij niet-gecommitte wijzigingen, builddatum in de Docker-build zonder git).
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
- **Aandrijftypes (`cfg.drive`, analyzeModel):** `diff` (1 wiel per kant), `skid` (meer wielen per kant: alle wielen van een
  kant krijgen dezelfde snelheid; ook rupsen = verborgen wielen; slipt bij draaien, ~60 % van de gevraagde hoek, zoals echt),
  `mecanum` (wielen met "mecanum" in de jointnaam: wielen zonder wrijving, het lichaam krijgt de gevraagde vx/vy/ω met
  versnellingsbegrenzing via setLinvel; `/cmd_vel` linear.y, toetsen Shift + ← →), `ackermann` (revolute om z met een wiel
  eronder = stuurjoint, role `steer`, ForceBased-motor; δ = atan(L·w/v), binnenwiel stuurt meer, achterwielen met
  bochtdifferentieel; draait niet ter plaatse). **Vering:** prismatic met "suspension"/"shock"/"spring" in de naam = role
  `spring` (ForceBased positiemotor naar 0: k uit `<dynamics spring_stiffness>`, c uit `damping`, anders 3 cm inzakken en
  60 % kritische demping); de tussenlink moet zwaar genoeg zijn (±3 kg), anders haalt de wielmotor zijn snelheid niet
  (solver met een lichte schakel tussen romp en wiel). Preset `rover`. **Rocker-bogie:** revolute met "rocker" = role
  `rocker` (ForceBased; `rockerDiff()` elke stap: elke rocker naar eigen hoek − gemiddelde → som 0 = differentieel),
  revolute met "bogie" = `passive` (vrij, enkel limieten). Preset `rockerbogie` (6 wielen, skid). Ingebouwde robots: `ROBOT_PRESETS` + `presetURDF(k)` in core.js (eigen
  ontwerp, primitieven), keuzelijst `#presetSel`. `cfg.leftWheel`/`rightWheel` blijven het eerste wiel (encoders, odometrie).
- **Rijknoppen in beeld = joystick:** indrukken en slepen op `#pad`; richting en grootte t.o.v. de STOP-knop (dode zone 25 %),
  ook schuin (vooruit + draaien, nodig voor Ackermann). Mecanum: knop `#padStrafe` of Shift → links/rechts = zijwaarts.
  Optie `#padMode` (Simulatie, `localStorage tb3sim.padMode`): knoppen, joystick (`#joy` in `#pad`, traploos, veert terug) of
  verborgen (klassen `mode-joy`/`mode-off` op `#pad`, zodat `testing` en `hidden` van `#pad` blijven werken).
  Numpad 7 9 1 3 = schuin (`CODEMAP`, op `e.code`). Keuzelijsten (wereld, robot) verliezen de focus na een keuze, anders
  veranderen de pijltjes de lijst i.p.v. de robot te sturen.
- **Rem bij stilstand** (step in core.js): bij nulcommando en uitgelopen wielen worden de horizontale snelheid en de
  pose (x, y, yaw) vastgehouden. Zonder rem kruipt de robot ~2°/40 s door trillingen van de bolcontacten. Niet weghalen.
- **Contacten:** `contact_natural_frequency = 120`, 8 solver-iteraties. De standaard (30 Hz) laat robots mm's zakken.
- **Odometrie:** `params.odomSource` "truth" (standaard, zoals de Gazebo diff-drive-plugin) of "wheels" (encoders,
  met straalfout L/R). `/ground_truth` = echte pose t.o.v. de startpose, frame `sim_world`.
- **Tijd:** `/clock` uit de simulatie; alle time-outs (cmd_vel 0,5 s, versheid 0,3 s) in sim-tijd, niet in wandtijd.
  Bij een verborgen tabblad stopt `requestAnimationFrame`; dan tikt een worker-timer (`hiddenTick`, 20 ms) de simulatie
  verder zonder renderen (camera pauzeert). Zonder die timer bevroor de sim-tijd zodra de Fenix-pagina of Lichtblick
  in een ander tabblad openging: SLAM en Nav2 hingen, de robot bleef staan.
- **Scan "geen echo":** 0.0 (echte LDS) of inf (Gazebo). inf gaat als `Infinity` in de JSON (rosbridge/Python accepteert dat).
- **Kaartlagen:** OccupancyGrid als DataTexture, geplaatst via `map → odom` uit `/tf` (zonder throttle subscriben).
  Lagen op 8/14/20 mm met polygonOffset; camera-near 0,05 (anders z-fighting met de vloer).
- **Werelden:** data-gedreven (`loadWorld(def)`), elementen `wall/box/cyl/rack/conveyor/pallet/table/chair/movable/mark`,
  buiten `tree/orchard/fence/sphere` (static kind `ball` voor de kruin; `fence` = palen + doorzichtig paneel via `def.opacity`).
  Wereldvelden `ground`/`sky` (kleur) en `geo` {lat, lon, alt} (GPS). Dozen 35 cm hoog: lidar van de Burger zit op 18 cm,
  lagere objecten scheert de scan voorbij. `worlds/*.json` en `WORLDS` in core.js samen aanpassen.
- **Extra sensoren (htmlsim-next, Robot > Sensoren, `.xSens`, standaard uit, `localStorage tb3sim.x.<id>`):** GPS
  `/gps/fix` 5 Hz (wereld = ENU: x oost, y noord, vanaf `geo`), sonar `/range/front|left|right` 10 Hz (`sim.rangeCone`,
  3 × 11 stralen in een kegel van 30°, zonder de vloer; Infinity = geen echo), bumper `/bumper` 20 Hz (`sim.bumper()`,
  contacten van het hoofdlichaam behalve de vloer; ook in `/sensor_state.bumper`), magnetometer `/imu/mag` 20 Hz.
  Dieptecamera (`xDepth`): `depthCam` op de plaats van de robotcamera, `scene.overrideMaterial = MeshDepthMaterial(RGBADepthPacking)`,
  witte achtergrond = geen meting (NaN), 160×120 32FC1 + 80×60 punten in het optische frame. Radar (`xRadar`, `sim.radarScan`): 121 × 3
  stralen, treffers op hetzelfde object samengevoegd, Doppler = (v_object − v_robot)·richting, PointCloud2 x y z velocity rcs.
  Gekleurde 3D-lidar (`l3Color`): `scan3d` geeft per punt de kleur van het object (`bodyColor`); in het bericht sRGB, in three.js lineair.
  Lijnsensor (`xLine`): 5 punten, reflectie = luminantie van de bovenste markering of de grond (`floorRefl`), 0 boven 5 cm.
  Cliff (`xCliff`): `sim.downRange` vanaf 4 hoeken. UWB (`xUwb`): ankers = wereldelement `uwb` (`worldInfo.uwb`), NLOS via
  `sim.lineOfSight` (+0,3–0,7 m, meer ruis); `/uwb/anchors` in het frame van `/ground_truth`. Stereo (`xStereo`): twee klonen van
  `robotCam` op ±3 cm, rechts `P[3] = -f·B`. Thermisch/segmentatie (`renderFlat`): elke mesh tijdelijk een ander materiaal; thermisch = ShaderMaterial
  die per pixel eigen temperatuur (`temp` in de wereld-JSON, robot 32, vloer 20, rest 21) + de heat-velden van `curFields()`
  rekent (zelfde formule als `fieldValue`/de temperatuursensor), 16 bits in rood/groen; segmentatie = label-id in rood.
  InstancedMesh zonder instanceColor; labels uit
  `mesh.userData.lab` = soort wereldelement (`statics[].item`, gezet in loadWorld via `curItem`). ArUco: wereldelement `aruco`
  (DICT_4X4_50 uit OpenCV in `ARUCO_4X4`), vlak met canvastextuur op de voorkant (`arucoPlane`); OpenCV herkent ze in het
  camerabeeld (test). 3D-lidar: `L3_PRESETS` + `sim.sweep3d` per stap (bewegingsvervorming, tijd per punt), `/points` als
  PointXYZIRT (Velodyne: x y z intensity f32, ring u16 op 16, time f32 op 18, step 22; met kleur rgb op 22). IMU: `#imuHz`.
  Frames uit `XMOUNT` (t.o.v. de rootlink) in `/tf_static`. Tests: `node tests/e2e_extra.cjs` (9092), `node tests/e2e_sensors2.cjs` (9093).
- **Beeldverwerking van de student (geen AI in de simulator):** abonnement op `/detections` (vision_msgs/Detection2DArray, pixels
  van het camerabeeld; vision_msgs 4.x `center.position` en ouder `center.x`) → kaders op `#camPreview` na elke capture
  (`drawVisionOverlay`, ook in de opname), en `/student/segmentation` (mono8, label-id's van `/segmentation/labels`) →
  half doorzichtig masker + IoU tegen de echte segmentatie van hetzelfde beeld (`segGT`, op header.stamp; gemiddelde over
  de klassen die de student voorspelt). Zolang er een masker binnenkomt rendert de simulator de segmentatie ook zonder
  vinkje. Voorbeeld: `examples/aruco_detections.py` (OpenCV ArUco → detecties + masker, OpenCV 4.6 en ≥ 4.7).
  AI-voorbeeld: `examples/yolox_detections.py` (YOLOX-S int8 uit de OpenCV Model Zoo, Apache-2.0, cv2.dnn op de CPU, werkt in
  OpenCV 4.6 ondanks "≥ 4.8" in de zoo; ~0,1 s per beeld; `--test foto.jpg` zonder ROS). Model in de image op
  `/opt/models/yolox` (Dockerfile, vastgepinde commit + sha256), anders ~/ros2_ws/models of eenmalig downloaden. Herkent
  op de primitieve werelden niets (getest: 0 detecties in het Klaslokaal), op foto's en het beeld van de echte robot wel.
- **3D-modellen (`models/`, CC0, `models/CREDITS.md`):** 33 GLB's, genormaliseerd (ware grootte, voorkant +x, onderkant 0,
  y-omhoog zoals glTF). `models/models.json` = maten (x lengte, y breedte, z hoogte), collider (box/cyl), segmentatielabel, COCO-naam,
  temperatuur. `tools/models_to_core.py` zet het als `MODEL_INFO` in core.js (wereldelement `model`: name x y [z yaw scale]);
  `build.py` sluit de GLB's als base64 in (`/*__MODELS__*/` → `MODEL_GLB`), de pagina parset ze één keer (GLTFLoader) en kloont
  per plaatsing (+90° om x). GLTFLoader + utils/BufferGeometryUtils in de vendor-lijst van de Dockerfile. Werelden Kantoor en
  Ziekenhuis (kamernummers als ArUco 20-23, 30-32, gele lijn in de gang); auto's/personen op het Bedrijfsterrein en in de
  Fabriekshal. Personen = Quaternius (CC0, via poly.pizza, idle-pose gebakken met echte verhoudingen; Kenney-personages
  hadden een hoofd van 1/4 van de lengte → vervangen). Meubels/auto's: Poly Haven-scans en Poly Pizza (CC0), basiskleur
  max. 512 px JPEG, per as geschaald; enkel `bestelwagen` en `defibrillator` zijn nog Kenney.
  Bomen (`tree`/`orchard`, `kind` loof|berk|den) en maïs: collider blijft stam + bol, in 3D getekend met `boom_<kind>` /
  `gewas_mais` als InstancedMesh per deelmesh (`instancedModel`, async; `worldGen` tegen een oude wereld). Wisselen 3D ↔
  eenvoudig = `buildWorldMeshes()` opnieuw. Kool en aardappel blijven procedureel (model te zwaar of viel uit elkaar). YOLOX herkent ze (person 0,89-0,95), auto's (car/truck), stoelen en
  laptops. Labels 20-34 in `SEG_LABELS` (persoon, voertuig, … defibrillator). Keuzelijst **Objecten** (`modelDetail`,
  localStorage `tb3sim.modelDetail`): `3d` of `simple` = `simpleModel()` (blokken/cilinders, zelfde maten, label en
  temperatuur, GLB wordt dan niet geparst); `rebuildModels()` wisselt live via `modelGroups`.
- **Zelf trainen:** `examples/dataset_opnemen.py` (beeld + `/segmentation/labels_map` met dezelfde stamp, 160 × 120),
  `examples/seg_train.py` (PyTorch, klein U-Net 16-32-64, export ONNX opset 11), `examples/seg_infer.py` (cv2.dnn →
  `/student/segmentation`, htmlsim toont masker + IoU).
- **Terrein (`terrain` in de wereld):** analytische hoogte (`terrainFn`: tilt, gauss-heuvels, golving) → Rapier-heightfield (ligt
  in xz: +90° om x, rij i naar −y, kolom j naar +x). `groundZ(x, y)` zet statics, dozen, palen en de robot op de grond; vloer en
  markeringen worden gedrapeerd (`drape`), geen raster. Kaartlagen, veldkaart en spoor blijven vlak op z = 0.
  `crops` = smalle doos per 2 m (collider) + InstancedMesh-planten. De wielmotor heeft een integrerende term (anders -30 %
  snelheid op 15 % helling); op vlakke grond 90° draaien op tijd: 87,7° i.p.v. 86,0°.
- **Opdrachten (tasks.js):** log van de studentnode via `/rosout` (rcl_interfaces/Log), match op berichttekst
  (niet de nodenaam: "battery_warning" bevat "warn"). `/rosout` met `queue_length: 100` (bij 1 laat rosbridge een info-regel vallen
  als dezelfde callback meteen ook een warn logt). Taak 1 kijkt naar **links** (`T1_SECTOR` = 90° ± 15°, sinds 28-10;
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
- **Velden in de wereld:** `fields: [{kind: gas|heat|humidity|light|sound, x, y, strength, sigma}]` in de wereld-JSON
  (ook in `WORLDS` in core.js, beide samen aanpassen). Omgevingssensoren op **`/sensors/...`, dezelfde topics, types,
  basiswaarden, tijdconstanten en ruis als `sim_sensors_node.py` op de robot** (turtlebot_docker, sessie 4): temperature,
  humidity (0-1), pressure, illuminance, gas/co2 (= gas-veld), gas/tvoc (0,5 × gas), gas/co (0,02 × gas), sound (dB, log-som),
  summary (JSON, voor de Fenix-pagina). 2 Hz, eerste-orde-traagheid in sim-tijd. De oude `/sim/gas_value`, `/sim/temperature`,
  `/sim/humidity`, `/sim/illuminance` lopen nog mee. In de virtuele robot start de knop Sensoren geen sim_sensors_node
  (FENIX_SIM): deze simulator is de bron. Test: `node tests/e2e_sensors.cjs`. Bronnen in 3D standaard verborgen.
- **Sessie 4 (projectplan, template na publishStudent):** enkel in de wereld `PLAN_WORLD` ("Klaslokaal") en met een
  studentnummer: `makePlan()` → seed = sha256(STU_SALT|plan|nr|wereld) → mulberry32 → 3 punten uit `PLAN_CAND` (≥ 1,5 m
  uit elkaar, richting k·45°) en 5 bronnen (één per soort, ≥ 1 m van de punten, ≥ 1,5 m onderling, sterkte ± 30 %,
  sigma ± 20 % rond de wereld). `curFields()` vervangt `sim.worldInfo.fields` voor sensoren, 3D-bollen en veldkaart.
  `PLAN_CAND` = 40 vrije plekken (≥ 45 cm van alle colliders tot 1,2 m hoog, bereikbaar vanaf de spawn), eenmalig
  berekend met Rapier-intersecties op de wereld; opnieuw berekenen als het Klaslokaal verandert. Coördinaten voor de
  student via `worldToMap()` = `tinv(worldMap())` (odomOrigin + map → odom uit /tf; `toMap` = startpose is enkel de
  terugval tijdens het opstarten). Taak `proj` (tasks.js): geen vaste test, logt elke 0,5 s sim-tijd
  pose + `sensVals`, P1-P3 ≤ 35 cm (Nav2-tolerantie 25 cm + AMCL-fout), hulpweergaven; rapport krijgt `meetronde` via `def.extra`. Test:
  `node tests/e2e_project.cjs` (nep-rosbridge op 9091, hook `window.__tb3test` enkel met `?test`).
- **Controle meetronde (`tools/controleer_meetronde.py`):** rapport (taak proj) tegen `metingen_<rnr>.csv` en
  `route_<rnr>.csv` (kolommen uit de GitBook sessie 4). Vergelijkt op plaats, niet op tijd (`t` van de student =
  wandtijd sinds de start van zijn script, `t_s` = sim-tijd). Gebruikt `check()` uit controleer_rapporten.py.
  Kolomnamen in de GitBook veranderen = dit script mee aanpassen.
- **Kaartankers (knop Beginpositie zetten):** bij "Kaart opslaan" (`/web/nav_cmd` zones_adopt, of een nieuwe naam in
  `/launch_status.maps` tijdens SLAM) bewaart `saveAnchor` `worldMap()` in `localStorage tb3sim.mapAnchors`
  ("<wereld>|<kaart>"). Draait Navigatie (`/launch_status.navigation`, `map_running`) met een kaart met anker, dan gebruikt
  `worldToMap` het anker i.p.v. /tf; de knop publiceert `/initialpose` = echte pose in dat frame. Cartographer legt de
  oorsprong op de pose bij de start van SLAM (niet op odom). Zelfde gegevens als `<kaart>.simworld.json` van nav_web_node.
- **HUD inklapbaar:** knop `#hudToggle` (`hud.hidden`, `localStorage tb3sim.hudClosed`); zonder keuze ingeklapt als
  `#stage` < 760 px breed. De HUD wordt ook ingeklapt elke frame gevuld: `drawComposite()` leest `textContent`
  (niet `innerText`, dat is leeg bij een verborgen element), zodat de opname de HUD altijd toont.
- **Wereld onthouden:** keuze in de lijst → `localStorage tb3sim.world`, hersteld bij het laden (niet bij de lege
  kamer van de tests, die gaat via `switchWorld` zonder de lijst).
- **Gamepad → `/joy`:** knoppen in de pagina of een echte gamepad (Gamepad-API, standaard-indeling) naar de indeling van
  joy_node met de F710 (XInput): 8 assen (links/op = +, triggers rust 1.0), 11 knoppen (A=0 … RB=5), 20 Hz sim-tijd.
  Met LB rijdt de robot met de sticks (zoals teleop_twist_joy, RB = turbo); voorrang na toetsenbord/knoppen in beeld.
  `setPointerCapture` na het registreren van de knop en in try (gooit bij nagebootste klikken).
- **LDS-02-preset:** `LIDAR.nRange = [220, 230]`: elke scan een willekeurig aantal punten, zoals de echte lidar.
- **Geen toestelherkenning:** toestel-ID en browser-/computervingerafdruk zijn eruit (GDPR, 23-10); niet terugzetten.
- **Virtuele robot (`fenix start` bij `robot htmlsim`, of `./start.sh --fenix`; Fenix-pagina op :8090):** de simulator abonneert ook `/cmd_vel_out` (uitgang van twist_mux + cmd_vel_guard van de
  robotsoftware in turtlebot-vis). Komt daar iets binnen (laatste 10 s), dan rijdt hij enkel daarop, zoals de motoren van de echte robot;
  `/cmd_vel` gaat dan via twist_mux (prioriteiten, snelheidslimiet); een Twist-script rijdt nog rechtstreeks (gaat niet door twist_mux).
  De opdrachten blijven `/cmd_vel` meten. Knop Fenix-pagina zodra :8090 antwoordt.
- **turtlebot-vis:** `build.py --local` → `dist/index.html` met three.js en Rapier uit `./vendor` (Dockerfile van turtlebot-vis), geserveerd op `http://localhost:8080/sim/`; daar verbindt de pagina vanzelf (`robot htmlsim` start rosbridge).
- **Robot-editor (template, vóór de hoofdlus; `ED`):** `sim.setFixedJoint(naam, origin)` (core.js) zet `joint.origin`
  en verschuift de links eronder binnen hun star lichaam (`linkOff` + `colliders` via `setTranslationWrtParent`), zonder
  herbouw; massa en grondhoogte blijven (sensoren zijn licht). Geeft `null` als er een bewegende joint onder hangt
  (die frames staan niet in de lijst). Gizmo = three `TransformControls` op `linkNodes[child]`, laag 1, raycaster
  `layers.enableAll()`; node-pose → origin t.o.v. de ouder via `an.linkT`. `/tf_static` opnieuw bij loslaten (en
  hoogstens elke 250 ms tijdens het slepen). Sensor toevoegen = link + fixed joint in `model` (camera met optisch frame
  `OPTICAL_Q`), `cfg[slot]` en herbouw. Export: `patchURDF(ED.text, edits, added)` werkt op de oorspronkelijke tekst
  (na `preXacro`) en verandert enkel `<origin>` van gewijzigde joints. `ED.orig` = origins bij het laden
  (`edNewModel` in `loadModelText`/`loadBuiltin`). Hulpweergaven (assen, collisions, scanring) hangen aan de
  linkknopen en verdwijnen met `robotG.clear()`; `edDecorate()` aan het einde van `buildRobotVisuals`.
  Test: `node tests/e2e_editor.cjs [map]` (nep-rosbridge op 9092). In de vendorlijst van de Dockerfile staat
  `TransformControls.js`: een nieuwe three-addon daar ook toevoegen.
- **Aparte robot-editor (`src/editor.*`, `dist/robot-editor.html`):** robot bouwen of aanpassen. Model = object van
  `parseURDF`; elke wijziging via `commit()` (geschiedenis 100 stappen, `localStorage tb3editor.autosave`, scène
  volledig herbouwd). Scène per link: `frame` (= joint.origin, gizmo-doel bij verplaatsen) → `motion` (voorbeeldstand
  van de joint) → vormen + kinderen. "Enkel de vorm"/schalen: gizmo op de visual-mesh; collision volgt als die gelijk
  was aan de visual. Export `toURDF(model, extras)` (core.js) schrijft de URDF volledig opnieuw; inertia met
  `inertial.auto` uit de vorm; onbekende blokken (gazebo, ros2_control, transmission) blijven als ruwe XML in `extras`.
  Bouwblokken in `PARTS` (wielen: continuous, as +y, cilinder rpy π/2 0 0, zoals de sim ze herkent; zwenkwiel heet
  caster… = zonder wrijving). Controle gebruikt `analyzeModel` en de laagste punten in de scène.
  Koppeling met de simulator: editor → sim via `BroadcastChannel("tb3sim")` {type:"urdf"} met `ack` terug; zonder
  antwoord binnen 0,8 s `localStorage tb3sim.fromEditor` + nieuw tabblad `index#editor-robot`. Sim → editor:
  `tb3sim.toEditor` + `editor#from-sim` (knop Robot-editor in de titelbalk en onder Robot). In een iframe (artifact)
  navigeren beide pagina's in hetzelfde kader i.p.v. een nieuw tabblad (`window.self !== window.top`). Test: `node tests/e2e_robot_editor.cjs [map]`
  (bouwt een robot, sleept met de muis aan de gizmo, rijdt de export in Rapier, stuurt naar een open sim).
  htmlsim-next: bouwblokken `skid4`, `mecanum4` (jointnamen met mecanum), `ackermann` (achteras + twee stuurjoints om z
  met elk een wiel), `suspension` (prismatic …_suspension_joint, 3 kg, tussen wiel en ouder; veer/demper → `<dynamics>`),
  ingebouwde robots (`ROBOT_PRESETS`) in het menu Nieuw / openen; de controle toont `cfg.drive`, sturing, vering, steunpunten.
- **Opname:** samengesteld canvas (WebGL + HUD + camera + testpaneel + balk) via `captureStream`, of `getDisplayMedia`
  voor het hele scherm. `drawComposite()` direct na `renderer.render` (anders leeg WebGL-beeld).
  Enkel MP4/H.264 (`pickMime`, 07-11): Toledo speelt .webm niet af. Chrome/Edge ≥ 126 kunnen dat, Firefox en
  Chromium zonder propriëtaire codecs niet: dan weigert de opname en start een test met opname niet.

## Valkuilen

- Scrolzoom is eigen code (`controls.enableZoom = false`): OrbitControls 0.160 deelt door `devicePixelRatio | 0`,
  0 bij browserzoom < 100 %, dan sprong de camera per scrolstap naar min/max (of verdween alles). Grenzen 0,3-40 m.

- Geen `prevRobot`, `keys`, enz. gebruiken vóór hun declaratie op module-niveau tijdens het opstarten (TDZ):
  functies die bij het laden lopen (applyTheme, buildWorldMeshes) mogen alleen al gedeclareerde `const`s gebruiken.
- `deg` bestaat al in template.html (stringformatter); in tasks.js heet de omzetting `r2d`.
- Artifact-CSP: alleen scripts van cdnjs/jsdelivr/unpkg, geen WebSocket naar localhost.
- Rapier-versie vastgepind op 0.14.0 (API: `timeOfImpact ?? toi`, `configureMotorModel`, geen frame1/frame2 bij joints).
