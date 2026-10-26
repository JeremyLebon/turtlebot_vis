// Fysica- en logicatests zonder browser:  node tests/test_core.mjs
// Laadt src/core.js in Node (Rapier + xmldom) en controleert rijgedrag, rem, odometrie, URDF-arm en werelden.
import fs from "fs"; import { createRequire } from "module"; import { fileURLToPath } from "url"; import path from "path";
const require = createRequire(import.meta.url), here = path.dirname(fileURLToPath(import.meta.url));
const R = require("@dimforge/rapier3d-compat"), { DOMParser } = require("@xmldom/xmldom");
const core = fs.readFileSync(path.join(here, "../src/core.js"), "utf8");
const M = new Function("R", core + ";return {createSim,parseURDF,analyzeModel,BURGER_URDF,WORLDS};")();
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
  run(s, { v: 0.2, w: 0 }, 1); ok(`Wereld "${M.WORLDS[k].name}" laadt en scant`, s.scan("base_scan").pts.length > 60, `${s.statics.length} vaste objecten`); }
console.log(fails ? `\n${fails} test(s) mislukt` : "\nAlle tests geslaagd"); process.exit(fails ? 1 : 0);
