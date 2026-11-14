// Fysica- en logicatests zonder browser:  node tests/test_core.mjs
// Laadt src/core.js in Node (Rapier + xmldom) en controleert rijgedrag, rem, odometrie, URDF-arm en werelden.
import fs from "fs"; import { createRequire } from "module"; import { fileURLToPath } from "url"; import path from "path";
const require = createRequire(import.meta.url), here = path.dirname(fileURLToPath(import.meta.url));
const R = require("@dimforge/rapier3d-compat"), { DOMParser } = require("@xmldom/xmldom");
const core = fs.readFileSync(path.join(here, "../src/core.js"), "utf8");
const M = new Function("R", core + ";return {createSim,parseURDF,analyzeModel,BURGER_URDF,WORLDS,ROBOT_PRESETS,presetURDF};")();
await R.init();
let fails = 0; const ok = (name, cond, info = "") => { console.log(`${cond ? "✓" : "✗"} ${name}${info ? "  — " + info : ""}`); if (!cond) fails++; };
const burger = () => { const s = M.createSim(R); s.loadWorld(M.WORLDS.empty); const m = M.parseURDF(M.BURGER_URDF, DOMParser); s.buildRobot(m, M.analyzeModel(m).cfg, 0, 0, 0); return s; };
const run = (s, c, t) => { for (let i = 0; i < t / s.DT; i++) s.step(c); };

{ const s = burger(); run(s, { v: 0.2, w: 0 }, 3); const p = s.truthPose(); ok("Burger rijdt 3 s aan 0,2 m/s", p.x > 0.5 && p.x < 0.62, `x = ${p.x.toFixed(3)} m`); }
{ const s = burger(); run(s, { v: 0, w: 1 }, Math.PI); ok("Burger draait ter plaatse ~π rad", Math.abs(s.truthPose().th) > 2.8, `θ = ${s.truthPose().th.toFixed(3)}`); }
{ const s = burger(); run(s, { v: 0, w: 0.5 }, Math.PI); run(s, { v: 0, w: 0 }, 1); const a = s.truthPose(); run(s, { v: 0, w: 0 }, 40); const b = s.truthPose();
  ok("Geen kruip in rust (rem)", Math.abs(b.th - a.th) < 1e-3 && Math.hypot(b.x - a.x, b.y - a.y) < 1e-4, `Δθ ${((b.th - a.th) * 57.3).toFixed(4)}°`); }
{ const s = burger(); s.params.odomSource = "wheels"; s.params.odomErr = 0; run(s, { v: 0.2, w: 0 }, 3); run(s, { v: 0, w: 1 }, 3);
  const p = s.truthPose(); ok("Wielodometrie volgt de werkelijkheid (± 3 cm)", Math.hypot(p.x - s.odom.x, p.y - s.odom.y) < 0.03); }
{ const s = burger(); s.LIDAR.rmax = 2; const pr = s.addProbe(-0.032 + 0.29, 0); run(s, { v: 0, w: 0 }, 0.05);
  const sc = s.scan("base_scan"), fr = [...sc.ranges.slice(0, 15), ...sc.ranges.slice(-15)].filter(r => r > 0.12);
  ok("Testpaal op 0,25 m zichtbaar voor de lidar", fr.length > 5 && Math.abs(Math.min(...fr) - 0.25) < 0.04, `min ${Math.min(...fr).toFixed(3)} m`); s.removeProbe(pr); }
{ const ARM = `<robot name="arm"><link name="world"/><joint name="fix" type="fixed"><parent link="world"/><child link="base"/></joint>
  <link name="base"><collision><origin xyz="0 0 0.05"/><geometry><box size="0.2 0.2 0.1"/></geometry></collision><inertial><mass value="5"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="swing" type="revolute"><parent link="base"/><child link="boom"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-1.5" upper="1.5" effort="100" velocity="1"/></joint>
  <link name="boom"><collision><origin xyz="0.25 0 0.05"/><geometry><box size="0.5 0.05 0.05"/></geometry></collision><inertial><origin xyz="0.25 0 0.05"/><mass value="1"/><inertia ixx="0.001" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="lift" type="revolute"><parent link="boom"/><child link="stick"/><origin xyz="0.5 0 0.05" rpy="0 0 0.3"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="100" velocity="1"/></joint>
  <link name="stick"><inertial><origin xyz="0.15 0 0"/><mass value="0.5"/><inertia ixx="0.001" iyy="0.005" izz="0.005" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>`;
  const s = M.createSim(R); s.loadWorld(M.WORLDS.empty); const m = M.parseURDF(ARM, DOMParser), r = s.buildRobot(m, M.analyzeModel(m).cfg, 0, 0, 0);
  r.byName.swing.target = 0.8; r.byName.lift.target = -0.5; run(s, { v: 0, w: 0 }, 4);
  ok("URDF-arm haalt doelhoeken onder zwaartekracht", Math.abs(r.byName.swing.pos - 0.8) < 0.01 && Math.abs(r.byName.lift.pos + 0.5) < 0.01, `swing ${r.byName.swing.pos.toFixed(3)}, lift ${r.byName.lift.pos.toFixed(3)}`); }
for (const k of Object.keys(M.WORLDS)) { const s = M.createSim(R); const wi = s.loadWorld(M.WORLDS[k]); const m = M.parseURDF(M.BURGER_URDF, DOMParser); s.buildRobot(m, M.analyzeModel(m).cfg, ...wi.spawn);
  s.LIDAR.rmax = 8;   // LDS-02-bereik: in de buitenwerelden staat binnen 3,5 m soms niets
  run(s, { v: 0.2, w: 0 }, 1); ok(`Wereld "${M.WORLDS[k].name}" laadt en scant`, s.scan("base_scan").pts.length > 60, `${s.statics.length} vaste objecten`);
  if (M.WORLDS[k].terrain) { const p = s.truthPose(), g = s.groundZ(p.x, p.y); ok(`  robot staat op het terrein`, Math.abs(p.z - g) < 0.05, `z ${p.z.toFixed(3)}, grond ${g.toFixed(3)}`); } }
// heuvel op: de Burger rijdt het pad op in Heuvelland (helling tot ~20 %)
{ const s = M.createSim(R); s.loadWorld(M.WORLDS.hills); const m = M.parseURDF(M.BURGER_URDF, DOMParser);
  s.buildRobot(m, M.analyzeModel(m).cfg, 1.0, 1.0, Math.PI / 4); const z0 = s.truthPose().z;   // onderaan de top op (5, 5)
  run(s, { v: 0.2, w: 0 }, 12); const p = s.truthPose();
  ok("Burger rijdt een heuvel op", p.z - z0 > 0.3 && Math.hypot(p.x - 1, p.y - 1) > 2.0, `${Math.hypot(p.x - 1, p.y - 1).toFixed(2)} m gereden, ${(p.z - z0).toFixed(2)} m gestegen`); }
// ingebouwde robots per aandrijftype
const preset = k => { const s = M.createSim(R); s.loadWorld({ name: "groot", floor: [20, 20], items: [] }); const m = M.parseURDF(M.presetURDF(k), DOMParser), cfg = M.analyzeModel(m).cfg;
  Object.assign(cfg, { vmax: M.ROBOT_PRESETS[k].vmax, wmax: M.ROBOT_PRESETS[k].wmax }); s.buildRobot(m, cfg, 0, 0, 0); return s; };
for (const [k, drive] of [["skid", "skid"], ["tracked", "skid"], ["mecanum", "mecanum"], ["ackermann", "ackermann"]]) {
  let s = preset(k); ok(`${k}: aandrijving ${drive} herkend`, s.robot.cfg.drive === drive, `${s.robot.cfg.drive}, ${s.robot.cfg.leftWheels.length}+${s.robot.cfg.rightWheels.length} wielen`);
  run(s, { v: 0.4, w: 0 }, 3); let p = s.truthPose();
  ok(`${k}: rijdt rechtdoor 3 s aan 0,4 m/s`, p.x > 0.95 && p.x < 1.25 && Math.abs(p.y) < 0.06 && Math.abs(p.th) < 0.05, `x ${p.x.toFixed(3)} y ${p.y.toFixed(3)} θ ${(p.th * 57.3).toFixed(1)}°`);
  if (k !== "ackermann") { s = preset(k); run(s, { v: 0, w: 1.0 }, 2); p = s.truthPose();
    // skid-steer en rupsen slippen bij draaien (zoals echt; Gazebo: wheel_separation_multiplier): > 1 rad van de 2
    ok(`${k}: draait ter plaatse`, Math.abs(p.th) > 1.0 && Math.hypot(p.x, p.y) < 0.1, `θ ${(p.th * 57.3).toFixed(0)}° (2 rad gevraagd), verschoven ${(Math.hypot(p.x, p.y) * 100).toFixed(1)} cm`); }
}
{ const s = preset("mecanum"); run(s, { v: 0, w: 0, vy: 0.3 }, 3); const p = s.truthPose();
  ok("mecanum: rijdt zijwaarts", p.y > 0.75 && Math.abs(p.x) < 0.05 && Math.abs(p.th) < 0.03, `x ${p.x.toFixed(3)} y ${p.y.toFixed(3)} θ ${(p.th * 57.3).toFixed(1)}°`); }
{ const s = preset("ackermann"), L = s.robot.cfg.steer.L; run(s, { v: 0.5, w: 0.5 }, 16); const xs = [], ys = [];
  for (let i = 0; i < 60; i++) { run(s, { v: 0.5, w: 0.5 }, 0.2); const p = s.truthPose(); xs.push(p.x); ys.push(p.y); }
  const cx = (Math.max(...xs) + Math.min(...xs)) / 2, cy = (Math.max(...ys) + Math.min(...ys)) / 2, r = xs.reduce((a, x, i) => a + Math.hypot(x - cx, ys[i] - cy), 0) / xs.length;
  ok("ackermann: cirkel met straal v/w = 1 m", Math.abs(r - 1.0) < 0.2, `straal ${r.toFixed(2)} m (wielbasis ${L} m)`);
  const s2 = preset("ackermann"); run(s2, { v: 0, w: 1 }, 2); const p2 = s2.truthPose(); ok("ackermann: draait niet ter plaatse", Math.abs(p2.th) < 0.05 && Math.hypot(p2.x, p2.y) < 0.02, `θ ${(p2.th * 57.3).toFixed(1)}°`); }
// 3D-lidar-presets: verdeeld over de omwenteling, tijd per punt, ringen; rozet verandert per scan
for (const k of ["vlp16", "mid360"]) {
  const s = burger(), P = s.L3_PRESETS[k], st = {}; s.addBox(1.5, 0, 0, 1.0, 1.0); let scan = null, steps = 0;
  while (!scan && steps < 1000) { s.step({ v: 0, w: 0 }); scan = s.sweep3d("lidar_3d_link", P, st); steps++; }
  const n = scan.pts.length / 5, ts = [], rings = new Set();
  for (let i = 0; i < n; i++) { ts.push(scan.pts[i * 5 + 4]); rings.add(scan.pts[i * 5 + 3]); }
  const mono = ts.every((t, i) => !i || t >= ts[i - 1] - 1e-9);
  ok(`3D-lidar ${k}: scan over ${steps} stappen, tijd per punt`, steps === Math.round(200 / P.hz) && mono && ts[ts.length - 1] < 1 / P.hz && n > 50,
    `${n} punten, t ${ts[0].toFixed(3)}–${ts[ts.length - 1].toFixed(3)} s, ${rings.size} ringen`);
  if (k === "mid360") { let s2 = null; while (!s2) { s.step({ v: 0, w: 0 }); s2 = s.sweep3d("lidar_3d_link", P, st); }
    ok("MID-360: volgende scan een ander patroon", s2.pts.length !== scan.pts.length || s2.pts[0] !== scan.pts[0]); }
}
{ // bewegingsvervorming: tijdens het draaien liggen de punten van een muur niet op één rechte in het sensorframe van het begin
  const s = burger(), P = s.L3_PRESETS.vlp16, st = {}; let scan = null;
  run(s, { v: 0, w: 1.5 }, 1);
  while (!scan) { s.step({ v: 0, w: 1.5 }); scan = s.sweep3d("lidar_3d_link", P, st); }
  ok("3D-lidar meet tijdens de beweging (tijdveld voor deskew)", scan.pts.length > 100); }
{ // cliff: robot aan de rand van de vloer van de lege kamer (vloer tot 6 + 2 m)
  const s = M.createSim(R); s.loadWorld({ name: "rand", floor: [4, 4], items: [] }); const m = M.parseURDF(M.BURGER_URDF, DOMParser);
  s.buildRobot(m, M.analyzeModel(m).cfg, 0, 0, 0); const T = s.linkWorld("base_footprint");
  const near = s.downRange({ x: T.p.x, y: T.p.y, z: T.p.z + 0.02 }, 0.5), edge = s.downRange({ x: 4.05, y: 0, z: 0.02 }, 0.5);
  ok("cliff: vloer onder de robot, niets voorbij de rand", near < 0.04 && !isFinite(edge), `onder ${near.toFixed(3)} m, voorbij de rand ${edge}`); }
{ // UWB-zichtlijn: muur ertussen
  const s = M.createSim(R); s.loadWorld({ name: "uwb", floor: [6, 6], items: [{ type: "wall", x0: 0, y0: -1, x1: 0, y1: 1, h: 1 }, { type: "uwb", x: 2, y: 0, z: 0.5 }] });
  s.world.step();   // query-pipeline bijwerken (normaal gebeurt dat in step)
  const free = s.lineOfSight({ x: -2, y: 2, z: 0.5 }, { x: 2, y: 2, z: 0.5 }), blocked = s.lineOfSight({ x: -2, y: 0, z: 0.5 }, { x: 2, y: 0, z: 0.5 });
  ok("UWB: zichtlijn vrij / muur ertussen", !isFinite(free) && blocked > 1.9 && blocked < 2.1 && s.worldInfo.uwb.length === 1, `geblokkeerd op ${blocked.toFixed(2)} m`); }
console.log(fails ? `\n${fails} test(s) mislukt` : "\nAlle tests geslaagd"); process.exit(fails ? 1 : 0);
