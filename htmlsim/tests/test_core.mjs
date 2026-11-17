// Fysica- en logicatests zonder browser:  node tests/test_core.mjs
// Laadt src/core.js in Node (Rapier + xmldom) en controleert rijgedrag, rem, odometrie, URDF-arm en werelden.
import fs from "fs"; import { createRequire } from "module"; import { fileURLToPath } from "url"; import path from "path";
const require = createRequire(import.meta.url), here = path.dirname(fileURLToPath(import.meta.url));
const R = require("@dimforge/rapier3d-compat"), { DOMParser, XMLSerializer } = require("@xmldom/xmldom");
const core = fs.readFileSync(path.join(here, "../src/core.js"), "utf8");
const M = new Function("R", core + ";return {createSim,parseURDF,analyzeModel,BURGER_URDF,WORLDS,rpyOf,qrpy,patchURDF,SENSOR_GEOM,toURDF};")();
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
// ---- robot-editor ----
{ let worst = 0;
  for (let i = 0; i < 200; i++) { const a = [(Math.random() - 0.5) * 6, (Math.random() - 0.5) * 3, (Math.random() - 0.5) * 6], b = M.rpyOf(M.qrpy(...a)), qa = M.qrpy(...a), qb = M.qrpy(...b);
    worst = Math.max(worst, 1 - Math.abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w)); }
  ok("rpyOf is de inverse van qrpy (200 willekeurige hoeken)", worst < 1e-9, `max afwijking ${worst.toExponential(1)}`); }
{ const s = burger(); run(s, { v: 0, w: 0 }, 0.5); const p0 = s.truthPose(), z0 = s.linkWorld("base_scan").p.z;
  const sub = s.setFixedJoint("scan_joint", { p: { x: 0.05, y: 0, z: 0.30 }, q: M.qrpy(0, 0, Math.PI / 2) });
  const T = s.linkWorld("base_scan"); run(s, { v: 0, w: 0 }, 1); const p1 = s.truthPose();
  ok("setFixedJoint verplaatst de lidar live", sub && sub[0] === "base_scan" && Math.abs(T.p.z - z0 - 0.128) < 0.002, `Δz ${(T.p.z - z0).toFixed(3)} m`);
  ok("setFixedJoint: lidar gedraaid (yaw 90°)", Math.abs(Math.atan2(2 * (T.q.w * T.q.z), 1 - 2 * T.q.z * T.q.z) - Math.PI / 2) < 0.01);
  ok("robot blijft staan na het verplaatsen", Math.hypot(p1.x - p0.x, p1.y - p0.y) < 1e-3);
  const co = s.robot.colls.find(o => o.ln === "base_scan").coll.translation();
  ok("collider van de lidar schuift mee", Math.abs(co.z - (T.p.z - 0.005)) < 0.01, `z ${co.z.toFixed(3)} m`);
  s.setFixedJoint("scan_joint", { p: { x: 0.05, y: 0, z: 0.15 }, q: M.qrpy(0, 0, Math.PI / 2) });   // op 30 cm scant hij over de paal
  s.LIDAR.rmax = 2; const pr = s.addProbe(p1.x + 0.05 + 0.29, p1.y); run(s, { v: 0, w: 0 }, 0.05);
  const sc = s.scan("base_scan"), n = sc.ranges.length, side = sc.ranges.slice(Math.round(n * 0.75) - 8, Math.round(n * 0.75) + 8).filter(r => r > 0.12 && isFinite(r));
  ok("gedraaide lidar ziet de paal rechts (−90° in zijn frame)", side.length > 3 && Math.abs(Math.min(...side) - 0.25) < 0.05, `min ${side.length ? Math.min(...side).toFixed(3) : "–"} m`); s.removeProbe(pr); }
{ const s = burger(); ok("setFixedJoint weigert onbekende of bewegende joints", s.setFixedJoint("wheel_left_joint", { p: { x: 0, y: 0, z: 0 }, q: M.qrpy(0, 0, 0) }) === null && s.setFixedJoint("bestaat_niet", { p: { x: 0, y: 0, z: 0 }, q: M.qrpy(0, 0, 0) }) === null); }
{ const txt = M.patchURDF(M.BURGER_URDF, { scan_joint: { p: { x: 0.01, y: -0.02, z: 0.25 }, q: M.qrpy(0, 0.1, 0) } },
    [{ link: "camera2_link", joint: "camera2_joint", parent: "base_link", origin: { p: { x: 0.07, y: 0, z: 0.1 }, q: M.qrpy(0, 0.3, 0) }, geom: M.SENSOR_GEOM.camera, optical: "camera2_optical_frame" }], DOMParser, XMLSerializer);
  const m = M.parseURDF(txt, DOMParser), sj = m.joints.find(j => j.name === "scan_joint"), cj = m.joints.find(j => j.name === "camera2_joint");
  ok("patchURDF schrijft de nieuwe origin", sj && Math.abs(sj.origin.p.z - 0.25) < 1e-9 && Math.abs(M.rpyOf(sj.origin.q)[1] - 0.1) < 1e-4, /<joint name="scan_joint"[^>]*>[\s\S]*?<origin[^>]*>/.exec(txt)?.[0].replace(/\s+/g, " "));
  ok("patchURDF voegt camera + optisch frame toe, één rootlink", cj && cj.parent === "base_link" && m.links.camera2_optical_frame && m.root === "base_footprint");
  ok("patchURDF laat de rest staan", Object.keys(m.links).length === Object.keys(M.parseURDF(M.BURGER_URDF, DOMParser).links).length + 2 && /material name="sensor"/.test(txt)); }
{ const a = M.parseURDF(M.BURGER_URDF, DOMParser), txt = M.toURDF(a, ['<gazebo><plugin name="x"/></gazebo>']), b = M.parseURDF(txt, DOMParser);
  const sameJ = a.joints.every(j => { const k = b.joints.find(x => x.name === j.name); return k && k.type === j.type && k.parent === j.parent && Math.abs(k.origin.p.z - j.origin.p.z) < 1e-6 && Math.abs(k.axis.y - j.axis.y) < 1e-9; });
  const ca = M.analyzeModel(a).cfg, cb = M.analyzeModel(b).cfg;
  ok("toURDF → parseURDF geeft dezelfde Burger", sameJ && Object.keys(a.links).length === Object.keys(b.links).length && cb.leftWheel === ca.leftWheel && Math.abs(cb.wheelSep - ca.wheelSep) < 1e-6 && /<gazebo>/.test(txt), `${b.joints.length} joints`);
  const m2 = M.parseURDF(txt, DOMParser); m2.links.base_link.inertial = { mass: 2, auto: true }; const t2 = M.toURDF(m2), ie = M.parseURDF(t2, DOMParser).links.base_link.inertial;
  ok("toURDF: automatische inertia uit de collision", ie && ie.mass === 2 && ie.I.zz > 0 && ie.I.xx > 0, `ixx ${ie && ie.I.xx}`);
  const s = M.createSim(R); s.loadWorld(M.WORLDS.empty); s.buildRobot(b, cb, 0, 0, 0); run(s, { v: 0.2, w: 0 }, 2);
  ok("geëxporteerde robot rijdt in de sim", s.truthPose().x > 0.3, `x ${s.truthPose().x.toFixed(3)}`); }
for (const k of Object.keys(M.WORLDS)) { const s = M.createSim(R); const wi = s.loadWorld(M.WORLDS[k]); const m = M.parseURDF(M.BURGER_URDF, DOMParser); s.buildRobot(m, M.analyzeModel(m).cfg, ...wi.spawn);
  run(s, { v: 0.2, w: 0 }, 1); ok(`Wereld "${M.WORLDS[k].name}" laadt en scant`, s.scan("base_scan").pts.length > 60, `${s.statics.length} vaste objecten`); }
console.log(fails ? `\n${fails} test(s) mislukt` : "\nAlle tests geslaagd"); process.exit(fails ? 1 : 0);
