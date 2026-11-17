import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";

const $ = id => document.getElementById(id);

/*__CORE__*/

/* ================= Robot-editor (aparte pagina) =================
   Model = het object van parseURDF (links, joints, root). Elke wijziging: commit() → geschiedenis,
   scène herbouwen, panelen, autosave. De simulator krijgt de robot als URDF-tekst (toURDF) via
   BroadcastChannel "tb3sim" (open tabblad) of localStorage "tb3sim.fromEditor" + #editor-robot. */
const SIM_URL = "__SIM_URL__";
const KEY_AUTO = "tb3editor.autosave", KEY_TO_SIM = "tb3sim.fromEditor", KEY_FROM_SIM = "tb3sim.toEditor";
const store = { get: k => { try { return localStorage.getItem(k); } catch { return null; } }, set: (k, v) => { try { localStorage.setItem(k, v); } catch {} } };
let bc = null; try { bc = new BroadcastChannel("tb3sim"); } catch {}

let M = null, extras = [], sel = "", selVis = 0;
const preview = {};                                 // jointnaam → voorbeeldstand (niet bewaard)
const hist = { list: [], i: -1 };
const files = new Map(), meshCache = new Map();
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const r1 = v => +(+v).toFixed(1), r2 = v => +(+v).toFixed(2);
const clone = o => JSON.parse(JSON.stringify(o));
const tq = q => ({ x: q.x, y: q.y, z: q.z, w: q.w });

/* ---------- 3D ---------- */
const stage = $("stage");
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
renderer.shadowMap.enabled = true;
stage.prepend(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
camera.up.set(0, 0, 1); camera.position.set(0.55, -0.6, 0.45);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0, 0, 0.08); orbit.enableDamping = true; orbit.minDistance = 0.08; orbit.maxDistance = 8;
scene.add(new THREE.HemisphereLight(0xffffff, 0x445048, 1.15));
const sun = new THREE.DirectionalLight(0xffffff, 1.5); sun.position.set(1.5, -1, 3); sun.castShadow = true;
Object.assign(sun.shadow.camera, { left: -1, right: 1, top: 1, bottom: -1, near: 0.1, far: 6 }); sun.shadow.mapSize.set(2048, 2048);
scene.add(sun);
const tok = n => new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue("--" + n).trim() || "#888");
const floor = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), new THREE.ShadowMaterial({ opacity: 0.18 })); floor.receiveShadow = true; scene.add(floor);
let grid = null;
function applyTheme() {
  scene.background = tok("sky");
  if (grid) scene.remove(grid);
  grid = new THREE.GridHelper(2, 40, tok("grid"), tok("grid")); grid.rotation.x = Math.PI / 2; grid.position.z = 0.0005; scene.add(grid);
}
applyTheme(); matchMedia("(prefers-color-scheme: dark)").addEventListener("change", applyTheme);
const robotRoot = new THREE.Group(); scene.add(robotRoot);
let nodes = {};                                     // link → { frame, motion, vis: [mesh], coll: [mesh] }
const MAT = {
  def: () => new THREE.MeshStandardMaterial({ color: 0x9aa3a0, roughness: 0.6 }),
  coll: new THREE.MeshBasicMaterial({ color: 0xff8a00, wireframe: true, depthTest: false, transparent: true, opacity: 0.6 }),
};
const selBox = new THREE.BoxHelper(undefined, 0x2f9cff); selBox.material.depthTest = false; selBox.renderOrder = 10; selBox.visible = false; scene.add(selBox);

function geomObject(g) {
  if (g.type === "box") return new THREE.BoxGeometry(...g.size);
  if (g.type === "sphere") return new THREE.SphereGeometry(g.radius, 28, 18);
  if (g.type === "cylinder") { const c = new THREE.CylinderGeometry(g.radius, g.radius, g.length, 36); c.rotateX(Math.PI / 2); return c; }
  return null;
}
function shapeMesh(el, mat) {
  const g = el.geom; let o;
  const geo = geomObject(g);
  if (geo) o = new THREE.Mesh(geo, mat);
  else {                                            // mesh-bestand
    const src = meshCache.get(g.filename);
    if (src) { o = src.clone(true); o.scale.set(...(g.scale || [1, 1, 1])); o.traverse(m => { if (m.isMesh) m.material = mat; }); }
    else { o = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.04, 0.04), new THREE.MeshBasicMaterial({ color: 0x888888, wireframe: true })); o.userData.missing = true; }
  }
  o.position.set(el.origin.p.x, el.origin.p.y, el.origin.p.z); o.quaternion.set(el.origin.q.x, el.origin.q.y, el.origin.q.z, el.origin.q.w);
  o.traverse(m => { if (m.isMesh) { m.castShadow = true; } });
  return o;
}
function buildScene() {
  robotRoot.clear(); nodes = {};
  const kids = {}; for (const j of M.joints) (kids[j.parent] ||= []).push(j);
  const add = (ln, parentObj, j) => {
    const frame = new THREE.Object3D(), motion = new THREE.Object3D();
    if (j) { frame.position.set(j.origin.p.x, j.origin.p.y, j.origin.p.z); frame.quaternion.set(j.origin.q.x, j.origin.q.y, j.origin.q.z, j.origin.q.w); }
    frame.add(motion); parentObj.add(frame);
    if (j && j.type !== "fixed") {
      const v = preview[j.name] || 0, ax = new THREE.Vector3(j.axis.x, j.axis.y, j.axis.z);
      if (j.type === "prismatic") motion.position.copy(ax.multiplyScalar(v)); else motion.quaternion.setFromAxisAngle(ax.normalize(), v);
    }
    const L = M.links[ln], n = nodes[ln] = { frame, motion, vis: [], coll: [], axes: null };
    L.visuals.forEach((v, i) => {
      const c = v.color, mat = c ? new THREE.MeshStandardMaterial({ color: new THREE.Color(c[0], c[1], c[2]), roughness: 0.6, transparent: c[3] < 1, opacity: c[3] }) : MAT.def();
      const o = shapeMesh(v, mat); o.traverse(m => { m.userData.link = ln; m.userData.vi = i; }); motion.add(o); n.vis.push(o);
    });
    if ($("showColl").checked) L.collisions.forEach(c => { const o = shapeMesh(c, MAT.coll); o.traverse(m => { m.castShadow = false; m.renderOrder = 5; m.raycast = () => {}; }); motion.add(o); n.coll.push(o); });
    if ($("showFrames").checked) { n.axes = new THREE.AxesHelper(ln === sel ? 0.07 : 0.03); n.axes.material.depthTest = false; n.axes.renderOrder = 8; motion.add(n.axes); }
    if (ln === sel && L.inertial && L.inertial.mass > 0) {   // zwaartepunt van de geselecteerde link
      const c = L.inertial.auto ? combinedInertia(L.collisions.length ? L.collisions : L.visuals, L.inertial.mass).origin.p : (L.inertial.origin || T0()).p;
      const b = new THREE.Mesh(new THREE.SphereGeometry(0.006, 16, 10), new THREE.MeshBasicMaterial({ color: 0xd63384, depthTest: false }));
      b.position.set(c.x, c.y, c.z); b.renderOrder = 12; b.raycast = () => {}; motion.add(b);
    }
    for (const k of kids[ln] || []) add(k.child, motion, k);
  };
  add(M.root, robotRoot, null);
  groundRobot();
}
function robotCOM() {                              // zwaartepunt van de hele robot (wereldcoördinaten van de scène)
  robotRoot.updateMatrixWorld(true); let m = 0; const c = new THREE.Vector3();
  for (const [ln, n] of Object.entries(nodes)) {
    const L = M.links[ln], ie = L.inertial; if (!ie || !(ie.mass > 0)) continue;
    const o = ie.auto ? combinedInertia(L.collisions.length ? L.collisions : L.visuals, ie.mass).origin.p : (ie.origin || T0()).p;
    c.add(new THREE.Vector3(o.x, o.y, o.z).applyMatrix4(n.motion.matrixWorld).multiplyScalar(ie.mass)); m += ie.mass;
  }
  return m > 0 ? { m, c: c.divideScalar(m) } : null;
}
function groundRobot() {                            // laagste punt op de vloer (enkel voor de weergave)
  robotRoot.position.z = 0; robotRoot.updateMatrixWorld(true);
  const box = new THREE.Box3(); for (const n of Object.values(nodes)) for (const o of n.vis) box.expandByObject(o);
  robotRoot.position.z = isFinite(box.min.z) ? -box.min.z : 0; robotRoot.updateMatrixWorld(true);
}
function resize() {
  const w = stage.clientWidth, h = stage.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / Math.max(1, h); camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(stage); resize();

/* ---------- gizmo en selectie ---------- */
const tc = new TransformControls(camera, renderer.domElement); tc.setSpace("local"); tc.setSize(0.8); scene.add(tc);
let gizmo = "translate";
function setSnap() {
  const v = +$("snap").value, rot = { 5: 5, 10: 15, 1: 1, 0: 0 }[v];
  tc.setTranslationSnap(v ? v / 1000 : null); tc.setRotationSnap(rot ? rot * D2R : null); tc.setScaleSnap(v ? 0.05 : null);
}
$("snap").onchange = setSnap; setSnap();
const shapeMode = () => $("shapeOnly").checked || gizmo === "scale" || sel === M.root;
function attachGizmo() {
  tc.detach();
  const n = nodes[sel]; if (!n) return;
  if (shapeMode()) { const o = n.vis[selVis]; if (o) tc.attach(o); }
  else tc.attach(n.frame);
  tc.setMode(gizmo);
  tc.setSpace(gizmo === "scale" ? "local" : "local");
}
function updateSelBox() {
  const n = nodes[sel];
  if (!n || !n.vis.length) { selBox.visible = false; return; }
  selBox.setFromObject(n.vis.length > 1 && !shapeMode() ? n.motion : n.vis[Math.min(selVis, n.vis.length - 1)]); selBox.visible = true;
}
tc.addEventListener("dragging-changed", e => { orbit.enabled = !e.value; });
tc.addEventListener("objectChange", () => { if (tc.dragging) { readGizmo(false); } });
tc.addEventListener("mouseUp", () => { if (tc.object) readGizmo(true); });
function readGizmo(commitNow) {
  const o = tc.object, L = M.links[sel]; if (!o || !L) return;
  if (o === nodes[sel].frame) {
    const j = jointOf(sel); if (!j) return;
    j.origin = { p: V(o.position.x, o.position.y, o.position.z), q: tq(o.quaternion) };
  } else {
    const v = L.visuals[selVis]; if (!v) return;
    const before = clone(v), c = L.collisions[selVis], linked = c && sameShape(c, before);
    if (gizmo === "scale") {
      const s = o.scale, g = v.geom;
      if (commitNow) {
        if (g.type === "box") g.size = [g.size[0] * s.x, g.size[1] * s.y, g.size[2] * s.z].map(x => Math.max(0.002, x));
        else if (g.type === "cylinder") { g.radius = Math.max(0.002, g.radius * (Math.abs(s.x - 1) > Math.abs(s.y - 1) ? s.x : s.y)); g.length = Math.max(0.002, g.length * s.z); }
        else if (g.type === "sphere") g.radius = Math.max(0.002, g.radius * Math.max(s.x, s.y, s.z));
        else if (g.type === "mesh") g.scale = [s.x, s.y, s.z];   // de mesh had g.scale al als schaal
      }
    } else v.origin = { p: V(o.position.x, o.position.y, o.position.z), q: tq(o.quaternion) };
    if (linked && commitNow) { c.origin = clone(v.origin); c.geom = clone(v.geom); }
  }
  if (commitNow) commit(); else { showProps(true); updateSelBox(); }
}
const sameShape = (a, b) => JSON.stringify(a.geom) === JSON.stringify(b.geom) && JSON.stringify(a.origin) === JSON.stringify(b.origin);

const ray = new THREE.Raycaster(); let downAt = null, hover = null;
const ndc = e => { const r = renderer.domElement.getBoundingClientRect(); return new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1); };
function pick(e) {
  ray.setFromCamera(ndc(e), camera);
  const objs = []; for (const n of Object.values(nodes)) for (const o of n.vis) objs.push(o);
  const hit = ray.intersectObjects(objs, true).find(h => h.object.userData.link != null);
  return hit ? hit.object.userData : null;
}
renderer.domElement.addEventListener("pointerdown", e => { downAt = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener("pointerup", e => {
  if (!downAt || tc.dragging || tc.axis) { downAt = null; return; }
  const moved = Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]); downAt = null;
  if (moved > 4) return;
  const p = pick(e);
  if (p) select(p.link, p.vi); else select("");
});
renderer.domElement.addEventListener("pointermove", e => {
  if (e.buttons) return;
  const p = pick(e), h = p ? p.link : null;
  if (h !== hover) {
    for (const ln of [hover, h]) if (ln && nodes[ln]) for (const o of nodes[ln].vis) o.traverse(m => { if (m.isMesh && m.material.emissive) m.material.emissive.setHex(ln === h ? 0x1d3b57 : 0); });
    hover = h; renderer.domElement.style.cursor = h ? "pointer" : "grab";
  }
});
renderer.domElement.addEventListener("dblclick", () => focusSel());
function focusSel() {
  const n = nodes[sel] || nodes[M.root]; if (!n) return;
  const b = new THREE.Box3().setFromObject(n.motion), c = b.getCenter(new THREE.Vector3()), r = Math.max(0.08, b.getSize(new THREE.Vector3()).length());
  const off = camera.position.clone().sub(orbit.target).setLength(r * 1.8);
  orbit.target.copy(c); camera.position.copy(c).add(off); orbit.update();
}

/* ---------- modelbewerkingen ---------- */
const jointOf = ln => M.joints.find(j => j.child === ln);
const childrenOf = ln => M.joints.filter(j => j.parent === ln).map(j => j.child);
function subtree(ln) { const out = [ln]; for (let i = 0; i < out.length; i++) out.push(...childrenOf(out[i])); return out; }
function uniq(base) { if (!M.links[base] && !M.joints.some(j => j.name === base)) return base; let i = 2; const b = base.replace(/_link$/, ""); while (M.links[`${b}${i}_link`]) i++; return `${b}${i}_link`; }
const jname = ln => { let n = ln.replace(/_link$/, "") + "_joint", i = 2; while (M.joints.some(j => j.name === n)) n = ln.replace(/_link$/, "") + `_joint${i++}`; return n; };
const T = (x = 0, y = 0, z = 0, r = 0, p = 0, yw = 0) => ({ p: V(x, y, z), q: qrpy(r, p, yw) });
const shape = (geom, origin = T(), color = null) => ({ origin, geom, color });
const GREY = [0.62, 0.65, 0.64, 1], DARK = [0.16, 0.17, 0.19, 1], WHEEL = [0.12, 0.12, 0.12, 1];
function addLink(name, parent, type, origin, visuals, mass, extra = {}) {
  name = uniq(name);
  M.links[name] = { name, inertial: mass ? { mass, auto: true } : null, visuals, collisions: extra.noColl ? [] : clone(visuals).map(v => ({ origin: v.origin, geom: v.geom })) };
  M.joints.push({ name: jname(name), type, parent, child: name, origin, axis: extra.axis || V(1, 0, 0), limit: extra.limit || null });
  return name;
}
const PARTS = {
  box: p => addLink("plate_link", p, "fixed", T(0, 0, 0.05), [shape({ type: "box", size: [0.12, 0.1, 0.008] }, T(), GREY)], 0.2),
  cylinder: p => addLink("cylinder_link", p, "fixed", T(0, 0, 0.05), [shape({ type: "cylinder", radius: 0.03, length: 0.06 }, T(0, 0, 0.03), GREY)], 0.1),
  sphere: p => addLink("sphere_link", p, "fixed", T(0, 0, 0.05), [shape({ type: "sphere", radius: 0.025 }, T(), GREY)], 0.05),
  wheel: (p, side = 1) => addLink(side > 0 ? "wheel_left_link" : "wheel_right_link", p, "continuous", T(0, side * 0.08, 0.023),
    [shape({ type: "cylinder", radius: 0.033, length: 0.018 }, T(0, 0, 0, Math.PI / 2), WHEEL)], 0.1, { axis: V(0, 1, 0) }),
  wheels: p => { PARTS.wheel(p, -1); return PARTS.wheel(p, 1); },
  caster: p => addLink("caster_back_link", p, "fixed", T(-0.08, 0, -0.0), [shape({ type: "sphere", radius: 0.01 }, T(), DARK)], 0.01),
  lidar: p => addLink("base_scan", p, "fixed", T(-0.03, 0, 0.12), [shape(SENSOR_GEOM.lidar, T(0, 0, -0.005), DARK)], 0.1),
  lidar3d: p => addLink("lidar_3d_link", p, "fixed", T(0, 0, 0.2), [shape(SENSOR_GEOM.lidar3d, T(), DARK)], 0.2),
  imu: p => addLink("imu_link", p, "fixed", T(0, 0, 0.03), [shape(SENSOR_GEOM.imu, T(), [0.1, 0.4, 0.2, 1])], 0, { noColl: true }),
  camera: p => {
    const c = addLink("camera_link", p, "fixed", T(0.06, 0, 0.1), [shape(SENSOR_GEOM.camera, T(-0.006), DARK)], 0.03);
    const o = c.replace(/_link$/, "") + "_optical_frame";
    M.links[o] = { name: o, inertial: null, visuals: [], collisions: [] };
    M.joints.push({ name: o + "_joint", type: "fixed", parent: c, child: o, origin: { p: V(), q: OPTICAL_Q }, axis: V(1, 0, 0), limit: null });
    return c;
  },
  arm: p => addLink("arm_link", p, "revolute", T(0, 0, 0.05), [shape({ type: "box", size: [0.2, 0.03, 0.03] }, T(0.1), [0.85, 0.55, 0.15, 1])], 0.2,
    { axis: V(0, 1, 0), limit: { lower: -Math.PI / 2, upper: Math.PI / 2, effort: 10, velocity: 1 } }),
  turn: p => addLink("turntable_link", p, "revolute", T(0, 0, 0.05), [shape({ type: "cylinder", radius: 0.05, length: 0.02 }, T(0, 0, 0.01), [0.3, 0.45, 0.7, 1])], 0.2,
    { axis: V(0, 0, 1), limit: { lower: -Math.PI, upper: Math.PI, effort: 10, velocity: 1 } }),
};
document.querySelectorAll("#palette button").forEach(b => b.onclick = () => {
  const parent = sel && M.links[sel] && !/optical/.test(sel) ? sel : (M.links.base_link ? "base_link" : M.root);
  const ln = PARTS[b.dataset.add](parent);
  sel = ln; selVis = 0; $("shapeOnly").checked = false; setGizmo("translate");
  commit(`${b.textContent.trim()} toegevoegd op ${parent}`);
});
function removeLink(ln) {
  if (ln === M.root) return;
  const gone = new Set(subtree(ln));
  M.joints = M.joints.filter(j => !gone.has(j.child)); for (const l of gone) delete M.links[l];
  sel = ""; commit(`${ln} verwijderd${gone.size > 1 ? ` (met ${gone.size - 1} link(s) eronder)` : ""}`);
}
function renameLink(old, nu) {
  nu = nu.trim().replace(/\s+/g, "_"); if (!nu || nu === old) return;
  if (M.links[nu]) { msg(`Naam ${nu} bestaat al.`, true); return; }
  M.links[nu] = { ...M.links[old], name: nu }; delete M.links[old];
  for (const j of M.joints) { if (j.parent === old) j.parent = nu; if (j.child === old) { j.child = nu; if (j.name === old.replace(/_link$/, "") + "_joint") j.name = jname(nu); } }
  if (M.root === old) M.root = nu;
  sel = nu; commit();
}
function reparent(ln, np) {                         // wereldpose behouden
  const j = jointOf(ln); if (!j || subtree(ln).includes(np)) return;
  robotRoot.updateMatrixWorld(true);
  const m = new THREE.Matrix4().copy(nodes[np].motion.matrixWorld).invert().multiply(nodes[ln].frame.matrixWorld);
  const p = new THREE.Vector3(), q = new THREE.Quaternion(), s = new THREE.Vector3(); m.decompose(p, q, s);
  j.parent = np; j.origin = { p: V(p.x, p.y, p.z), q: tq(q) }; commit();
}
const swapLR = n => n.replace(/left/g, "\u0000").replace(/right/g, "left").replace(/\u0000/g, "right").replace(/_l(_|$)/, "_\u0001$1").replace(/_r(_|$)/, "_l$1").replace(/\u0001/, "r");
const mirT = t => ({ p: V(t.p.x, -t.p.y, t.p.z), q: { x: -t.q.x, y: t.q.y, z: -t.q.z, w: t.q.w } });
function copySubtree(ln, mirror) {
  const names = {}, sub = subtree(ln);
  for (const l of sub) { let n = mirror ? swapLR(l) : l; if (n === l || M.links[n]) n = uniq(mirror ? l.replace(/_link$/, "") + "_spiegel_link" : l); names[l] = n; M.links[n] = { ...clone(M.links[l]), name: n }; }
  for (const l of sub) {
    const L = M.links[names[l]];
    if (mirror) { L.visuals.forEach(v => { v.origin = mirT(v.origin); }); L.collisions.forEach(c => { c.origin = mirT(c.origin); }); }
  }
  for (const l of sub) {
    const j = jointOf(l), nj = clone(j);
    nj.child = names[l]; nj.parent = names[j.parent] || j.parent;
    nj.name = mirror && swapLR(j.name) !== j.name && !M.joints.some(x => x.name === swapLR(j.name)) ? swapLR(j.name) : jname(names[l]);
    if (l === ln) nj.origin = mirror ? mirT(j.origin) : { p: V(j.origin.p.x, j.origin.p.y - 0.04, j.origin.p.z), q: j.origin.q };
    else if (mirror) nj.origin = mirT(j.origin);
    M.joints.push(nj);
  }
  sel = names[ln]; commit(mirror ? `Gespiegeld: ${ln} → ${names[ln]}` : `Gekopieerd: ${names[ln]}`);
}

/* ---------- geschiedenis, autosave ---------- */
const snapshot = () => JSON.stringify({ M, extras });
function commit(note) {
  hist.list = hist.list.slice(0, hist.i + 1); hist.list.push(snapshot()); if (hist.list.length > 100) hist.list.shift(); hist.i = hist.list.length - 1;
  store.set(KEY_AUTO, hist.list[hist.i]);
  refresh(); if (note) msg(note);
}
function restore(s) { const o = JSON.parse(s); M = o.M; extras = o.extras || []; if (!M.links[sel]) sel = ""; refresh(); store.set(KEY_AUTO, s); }
function undo() { if (hist.i > 0) { hist.i--; restore(hist.list[hist.i]); msg("Ongedaan gemaakt."); } }
function redo() { if (hist.i < hist.list.length - 1) { hist.i++; restore(hist.list[hist.i]); msg("Opnieuw."); } }
$("undoBtn").onclick = undo; $("redoBtn").onclick = redo;
function refresh() {
  $("robotName").value = M.name || "robot";
  buildScene(); renderTree(); showProps(); runChecks(); attachGizmo(); updateSelBox();
  $("undoBtn").disabled = hist.i <= 0; $("redoBtn").disabled = hist.i >= hist.list.length - 1;
  if (!$("urdfOut").hidden) $("urdfOut").value = exportText();
}
function select(ln, vi = 0) { sel = ln; selVis = vi || 0; refresh(); }

/* ---------- boom ---------- */
const JT = { fixed: "vast", continuous: "draait", revolute: "scharnier", prismatic: "schuift" };
function roleOf(ln, cfg) {
  const j = jointOf(ln);
  if (cfg.leftWheel && j && j.name === cfg.leftWheel) return "aandrijfwiel L";
  if (cfg.rightWheel && j && j.name === cfg.rightWheel) return "aandrijfwiel R";
  if (/caster/i.test(ln)) return "zwenkwiel";
  for (const [k, t] of [["lidar", "2D-lidar"], ["lidar3d", "3D-lidar"], ["camera", "camera"], ["imu", "IMU"]]) if (cfg[k] === ln) return t;
  return "";
}
function renderTree() {
  const cfg = analyzeModel(M).cfg, out = [];
  const walk = (ln, d) => {
    const j = jointOf(ln), role = roleOf(ln, cfg);
    out.push(`<li role="treeitem" data-l="${ln}" class="${ln === sel ? "sel" : ""}" style="padding-left:${6 + d * 14}px" aria-selected="${ln === sel}">${d ? "└ " : ""}${ln}` +
      (j && j.type !== "fixed" ? ` <span class="jt">${JT[j.type]}</span>` : "") + (role ? ` <span class="role">${role}</span>` : "") + `</li>`);
    for (const c of childrenOf(ln)) walk(c, d + 1);
  };
  walk(M.root, 0);
  $("tree").innerHTML = out.join("");
  $("tree").querySelectorAll("li").forEach(li => li.onclick = () => select(li.dataset.l));
  $("addHint").textContent = `Komt op ${sel && !/optical/.test(sel) ? sel : (M.links.base_link ? "base_link" : M.root)}. Kies eerst een link om er iets op te zetten.`;
}

/* ---------- eigenschappen ---------- */
const num = (id, v, step = 1, extra = "") => `<input type="number" id="${id}" value="${v}" step="${step}" ${extra}>`;
function showProps(liveOnly) {
  const L = M.links[sel];
  if (!L) {
    $("propTitle").textContent = "Robot";
    $("props").innerHTML = `<p class="hint">Klik op een onderdeel in het 3D-beeld of in de opbouw. Begin met <b>Wielen (paar)</b> en een <b>Zwenkwiel</b> op <code>base_link</code>, en zet er een <b>2D-lidar</b> op voor SLAM en Nav2.</p>
      <div class="pgrid"><label>Links</label><span class="full">${Object.keys(M.links).length}</span><label>Joints</label><span class="full">${M.joints.length} (${M.joints.filter(j => j.type !== "fixed").length} bewegend)</span>
      <label>Massa</label><span class="full">${r2(Object.values(M.links).reduce((s, l) => s + (l.inertial ? l.inertial.mass : 0), 0))} kg</span></div>`;
    return;
  }
  const j = jointOf(L.name), v = L.visuals[selVis];
  if (liveOnly) {                                   // tijdens slepen: enkel de getallen bijwerken
    const o = j ? j.origin : null;
    if (o && $("pX")) { const rpy = rpyOf(o.q); [["pX", o.p.x * 1000], ["pY", o.p.y * 1000], ["pZ", o.p.z * 1000], ["pR", rpy[0] * R2D], ["pP", rpy[1] * R2D], ["pW", rpy[2] * R2D]].forEach(([id, x]) => { $(id).value = r1(x); }); }
    if (v && $("vX")) { const rpy = rpyOf(v.origin.q); [["vX", v.origin.p.x * 1000], ["vY", v.origin.p.y * 1000], ["vZ", v.origin.p.z * 1000], ["vR", rpy[0] * R2D], ["vP", rpy[1] * R2D], ["vW", rpy[2] * R2D]].forEach(([id, x]) => { $(id).value = r1(x); }); }
    return;
  }
  $("propTitle").textContent = L.name;
  const h = [`<div class="pgrid">`, `<label for="pName">Naam</label><input class="full" id="pName" value="${L.name}" spellcheck="false">`];
  if (j) {
    const others = Object.keys(M.links).filter(n => !subtree(L.name).includes(n));
    h.push(`<label for="pParent">Op</label><select class="full" id="pParent">${others.map(n => `<option ${n === j.parent ? "selected" : ""}>${n}</option>`).join("")}</select>`);
    h.push(`<label for="pType">Joint</label><select class="full" id="pType">${Object.entries(JT).map(([k, t]) => `<option value="${k}" ${k === j.type ? "selected" : ""}>${k} · ${t}</option>`).join("")}</select>`);
    if (j.type !== "fixed") {
      const ax = j.axis, axs = ["1 0 0", "0 1 0", "0 0 1", "-1 0 0", "0 -1 0", "0 0 -1"], cur = [ax.x, ax.y, ax.z].map(a => Math.round(a)).join(" ");
      h.push(`<label for="pAxis">As</label><select class="full" id="pAxis">${axs.map(a => `<option value="${a}" ${a === cur ? "selected" : ""}>${{ "1 0 0": "x", "0 1 0": "y", "0 0 1": "z", "-1 0 0": "−x", "0 -1 0": "−y", "0 0 -1": "−z" }[a]}</option>`).join("")}${axs.includes(cur) ? "" : `<option selected value="${cur}">${cur}</option>`}</select>`);
      if (j.type !== "continuous") {
        const pr = j.type === "prismatic", L2 = j.limit || { lower: -1.57, upper: 1.57 }, f = pr ? 1000 : R2D;
        h.push(`<label for="pLo">Min</label>${num("pLo", r1(L2.lower * f))}<label for="pHi">Max ${pr ? "mm" : "°"}</label>${num("pHi", r1(L2.upper * f))}`);
      }
      const pr = j.type === "prismatic", lo = j.type === "continuous" ? -Math.PI : (j.limit ? j.limit.lower : -1), hi = j.type === "continuous" ? Math.PI : (j.limit ? j.limit.upper : 1);
      {
        const pr = j.type === "prismatic", Lm = j.limit || {}, wheel = j.type === "continuous" && /wheel|wiel/i.test(j.name + j.child);
        h.push(`<label for="pEff">Max ${pr ? "kracht N" : "koppel N·m"}</label>${num("pEff", Lm.effort || "", "any", 'min="0" placeholder="geen"')}<label for="pVel">Max ${pr ? "m/s" : "rad/s"}</label>${num("pVel", Lm.velocity || "", "any", 'min="0" placeholder="geen"')}`);
        if (!(j.type === "prismatic" && /suspension|shock|spring/i.test(j.name)))
          h.push(`<label for="pDmp">Demping</label>${num("pDmp", j.damping ?? "", "any", `min="0" placeholder="0"`)}<label for="pFri">Wrijving</label>${num("pFri", j.friction ?? "", "any", 'min="0" placeholder="0"')}`);
        h.push(`<span class="hint" style="grid-column:1/-1">${wheel ? "Aandrijfwiel: snelheid en versnelling komen uit de rijregeling (zoals de diff-drive-plugin van Gazebo); deze waarden gaan wel mee in de URDF." : j.type === "continuous" ? "Vrij draaiend: demping (N·m·s/rad) en wrijving (N·m) remmen in de simulator." : "Simulator: max snelheid begrenst de beweging, max koppel maakt de motor zo sterk (een te zwakke motor zakt door)."}</span>`);
      }
      h.push(`<label for="pPrev">Testen</label><input class="full" type="range" id="pPrev" min="${lo}" max="${hi}" step="${pr ? 0.001 : 0.01}" value="${preview[j.name] || 0}">`);
    }
    const rpy = rpyOf(j.origin.q);
    h.push(`<p class="psub">Plaats t.o.v. ${j.parent}</p>`,
      `<label for="pX">x mm</label>${num("pX", r1(j.origin.p.x * 1000))}<label for="pR">roll °</label>${num("pR", r1(rpy[0] * R2D))}`,
      `<label for="pY">y mm</label>${num("pY", r1(j.origin.p.y * 1000))}<label for="pP">pitch °</label>${num("pP", r1(rpy[1] * R2D))}`,
      `<label for="pZ">z mm</label>${num("pZ", r1(j.origin.p.z * 1000))}<label for="pW">yaw °</label>${num("pW", r1(rpy[2] * R2D))}`);
  }
  // vorm
  h.push(`<p class="psub">Vorm${L.visuals.length > 1 ? ` ${selVis + 1} van ${L.visuals.length}` : ""}</p>`);
  if (L.visuals.length > 1) h.push(`<label for="pVis">Vorm</label><select class="full" id="pVis">${L.visuals.map((x, i) => `<option value="${i}" ${i === selVis ? "selected" : ""}>${i + 1}: ${x.geom.type}</option>`).join("")}</select>`);
  if (v) {
    const g = v.geom, c = v.color || GREY, col = "#" + c.slice(0, 3).map(x => Math.round(x * 255).toString(16).padStart(2, "0")).join("");
    h.push(`<label for="vType">Soort</label><select class="full" id="vType">${["box", "cylinder", "sphere"].map(t => `<option ${t === g.type ? "selected" : ""} value="${t}">${{ box: "doos", cylinder: "cilinder", sphere: "bol" }[t]}</option>`).join("")}${g.type === "mesh" ? `<option selected value="mesh">mesh: ${g.filename.split("/").pop()}</option>` : ""}</select>`);
    if (g.type === "box") h.push(`<label for="gX">lengte x</label>${num("gX", r1(g.size[0] * 1000))}<label for="gY">breedte y</label>${num("gY", r1(g.size[1] * 1000))}<label for="gZ">hoogte z</label>${num("gZ", r1(g.size[2] * 1000))}<span></span><span class="hint">mm</span>`);
    if (g.type === "cylinder") h.push(`<label for="gR">straal</label>${num("gR", r1(g.radius * 1000))}<label for="gL">lengte</label>${num("gL", r1(g.length * 1000))}`);
    if (g.type === "sphere") h.push(`<label for="gR">straal mm</label>${num("gR", r1(g.radius * 1000))}`);
    const rv = rpyOf(v.origin.q);
    h.push(`<label for="vX">dx mm</label>${num("vX", r1(v.origin.p.x * 1000))}<label for="vR">roll °</label>${num("vR", r1(rv[0] * R2D))}`,
      `<label for="vY">dy mm</label>${num("vY", r1(v.origin.p.y * 1000))}<label for="vP">pitch °</label>${num("vP", r1(rv[1] * R2D))}`,
      `<label for="vZ">dz mm</label>${num("vZ", r1(v.origin.p.z * 1000))}<label for="vW">yaw °</label>${num("vW", r1(rv[2] * R2D))}`,
      `<label for="vCol">Kleur</label><input type="color" id="vCol" value="${col}"><label class="chk" style="grid-column:3/5"><input type="checkbox" id="vColl" ${L.collisions[selVis] && sameShape(L.collisions[selVis], v) ? "checked" : ""}> botst (collision)</label>`);
  }
  h.push(`<div class="row full" style="grid-column:1/-1"><button class="ghost" id="vAdd">Vorm erbij</button>${v ? `<button class="ghost" id="vDel">Vorm weg</button>` : ""}</div>`);
  // massa, inertie, wrijving
  const ie = L.inertial, f5 = v => +(+v).toPrecision(4);
  h.push(`<p class="psub">Massa en inertie</p><label for="mKg">kg</label>${num("mKg", ie ? ie.mass : 0, 0.01, 'min="0"')}`);
  if (ie && ie.mass > 0) {
    h.push(`<label for="mMode">Inertie</label><select id="mMode"><option value="auto" ${ie.auto ? "selected" : ""}>automatisch</option><option value="man" ${!ie.auto ? "selected" : ""}>handmatig</option></select>`);
    const cur = ie.auto ? combinedInertia(L.collisions.length ? L.collisions : L.visuals, ie.mass) : ie;
    if (ie.auto) h.push(`<span class="hint full" style="grid-column:1/-1">Uit ${L.collisions.length ? "de collisions" : "de vormen"} samen (massa volgens volume): zwaartepunt ${["x", "y", "z"].map(k => r1(cur.origin.p[k] * 1000)).join(" / ")} mm · ixx ${f5(cur.I.xx)} · iyy ${f5(cur.I.yy)} · izz ${f5(cur.I.zz)} kg·m²</span>`);
    else h.push(`<label for="cX">zw. x mm</label>${num("cX", r1(cur.origin.p.x * 1000))}<label for="iXX">ixx</label>${num("iXX", f5(cur.I.xx), "any")}`,
      `<label for="cY">zw. y mm</label>${num("cY", r1(cur.origin.p.y * 1000))}<label for="iYY">iyy</label>${num("iYY", f5(cur.I.yy), "any")}`,
      `<label for="cZ">zw. z mm</label>${num("cZ", r1(cur.origin.p.z * 1000))}<label for="iZZ">izz</label>${num("iZZ", f5(cur.I.zz), "any")}`,
      `<span class="hint" style="grid-column:1/-1">kg·m², t.o.v. het zwaartepunt; ixy/ixz/iyz blijven ${cur.I.xy || cur.I.xz || cur.I.yz ? "zoals ze waren" : "0"} (de simulator gebruikt enkel de diagonaal)</span>`);
  }
  const muDef = /caster/i.test(L.name) ? 0 : (j && j.type === "continuous" ? 1.0 : 0.4);
  h.push(`<label for="mMu">Wrijving µ</label>${num("mMu", L.mu ?? "", 0.05, `min="0" placeholder="auto"`)}<span class="hint" style="grid-column:3/5">${L.mu != null ? "→ &lt;gazebo&gt;&lt;mu1&gt;" : "leeg: wiel 1,0 · zwenkwiel 0 · rest 0,4"}</span>`);
  h.push(`</div>`);
  h.push(`<div class="row">${j ? `<button class="ghost" id="aDup">Kopiëren</button><button class="ghost" id="aMir">Spiegelen L↔R</button><button class="ghost" id="aDel">Verwijderen</button>` : `<span class="hint">Rootlink: ligt vast in de oorsprong.</span>`}</div>`);
  $("props").innerHTML = h.join("");
  // koppelingen
  const val = id => { const x = +$(id).value; return isFinite(x) ? x : 0; };
  $("pName").onchange = () => renameLink(L.name, $("pName").value);
  if (j) {
    $("pParent").onchange = () => reparent(L.name, $("pParent").value);
    $("pType").onchange = () => {
      const t = $("pType").value; j.type = t;
      if ((t === "revolute" || t === "prismatic") && !j.limit) j.limit = t === "revolute" ? { lower: -Math.PI / 2, upper: Math.PI / 2, effort: 10, velocity: 1 } : { lower: 0, upper: 0.1, effort: 50, velocity: 0.1 };
      if (t !== "fixed" && Math.abs(vlen(j.axis) - 1) > 0.01) j.axis = V(0, 0, 1);
      delete preview[j.name]; commit();
    };
    if ($("pAxis")) $("pAxis").onchange = () => { const [x, y, z] = $("pAxis").value.split(" ").map(Number); j.axis = V(x, y, z); commit(); };
    if ($("pLo")) $("pLo").onchange = $("pHi").onchange = () => { const f = j.type === "prismatic" ? 1000 : R2D; j.limit = { ...(j.limit || { effort: 10, velocity: 1 }), lower: val("pLo") / f, upper: val("pHi") / f }; commit(); };
    if ($("pEff")) $("pEff").onchange = $("pVel").onchange = () => {
      const e = $("pEff").value === "" ? 0 : Math.max(0, +$("pEff").value), v = $("pVel").value === "" ? 0 : Math.max(0, +$("pVel").value);
      j.limit = { lower: 0, upper: 0, ...(j.limit || {}), effort: e, velocity: v }; commit();
    };
    if ($("pDmp")) $("pDmp").onchange = $("pFri").onchange = () => {
      const d = $("pDmp").value, f = $("pFri").value; j.damping = d === "" ? undefined : Math.max(0, +d); j.friction = f === "" ? undefined : Math.max(0, +f); commit();
    };
    if ($("pPrev")) $("pPrev").oninput = () => { preview[j.name] = +$("pPrev").value; buildScene(); attachGizmo(); updateSelBox(); };
    ["pX", "pY", "pZ", "pR", "pP", "pW"].forEach(id => { $(id).onchange = () => { j.origin = { p: V(val("pX") / 1000, val("pY") / 1000, val("pZ") / 1000), q: qrpy(val("pR") * D2R, val("pP") * D2R, val("pW") * D2R) }; commit(); }; });
    $("aDup").onclick = () => copySubtree(L.name, false); $("aMir").onclick = () => copySubtree(L.name, true); $("aDel").onclick = () => removeLink(L.name);
  }
  if ($("pVis")) $("pVis").onchange = () => { selVis = +$("pVis").value; refresh(); };
  if (v) {
    const c = () => L.collisions[selVis], linked = () => $("vColl").checked;
    const sync = () => { if (linked()) { L.collisions[selVis] = { origin: clone(v.origin), geom: clone(v.geom) }; } };
    $("vType").onchange = () => {
      const t = $("vType").value, g = v.geom, s = g.type === "box" ? Math.max(...g.size) / 2 : g.radius || 0.03;
      v.geom = t === "box" ? { type: "box", size: [2 * s, 2 * s, 2 * s] } : t === "cylinder" ? { type: "cylinder", radius: s, length: 2 * s } : t === "sphere" ? { type: "sphere", radius: s } : g;
      sync(); commit();
    };
    for (const id of ["gX", "gY", "gZ", "gR", "gL"]) if ($(id)) $(id).onchange = () => {
      const g = v.geom, mm = x => Math.max(0.001, val(x) / 1000);
      if (g.type === "box") g.size = [mm("gX"), mm("gY"), mm("gZ")]; else { g.radius = mm("gR"); if (g.type === "cylinder") g.length = mm("gL"); }
      sync(); commit();
    };
    ["vX", "vY", "vZ", "vR", "vP", "vW"].forEach(id => { $(id).onchange = () => { v.origin = { p: V(val("vX") / 1000, val("vY") / 1000, val("vZ") / 1000), q: qrpy(val("vR") * D2R, val("vP") * D2R, val("vW") * D2R) }; sync(); commit(); }; });
    $("vCol").oninput = () => { const h2 = $("vCol").value; v.color = [1, 3, 5].map(i => parseInt(h2.slice(i, i + 2), 16) / 255).concat(v.color ? v.color[3] : 1); const n = nodes[L.name]; if (n && n.vis[selVis]) n.vis[selVis].traverse(m => { if (m.isMesh && m.material.color) m.material.color.setRGB(v.color[0], v.color[1], v.color[2]); }); };
    $("vCol").onchange = () => commit();
    $("vColl").onchange = () => { if (linked()) sync(); else L.collisions.splice(selVis, 1); commit(); };
    $("vDel").onclick = () => { L.visuals.splice(selVis, 1); if (c() && L.collisions.length > L.visuals.length) L.collisions.splice(selVis, 1); selVis = 0; commit(); };
  }
  $("vAdd").onclick = () => { const s = shape({ type: "box", size: [0.04, 0.04, 0.04] }, T(0, 0, 0.02), GREY); L.visuals.push(s); L.collisions.push({ origin: clone(s.origin), geom: clone(s.geom) }); selVis = L.visuals.length - 1; $("shapeOnly").checked = true; commit(); };
  $("mKg").onchange = () => {
    const m = val("mKg"), old = L.inertial;
    if (!(m > 0)) L.inertial = null;
    else if (old && !old.auto && old.I && old.mass > 0) {   // handmatig/uit de URDF: inertie mee schalen met de massa
      const f = m / old.mass; L.inertial = { ...old, mass: m, I: Object.fromEntries(Object.entries(old.I).map(([k, v]) => [k, v * f])) };
    } else L.inertial = { ...(old || {}), mass: m, auto: old ? old.auto !== false : true };
    commit();
  };
  if ($("mMode")) $("mMode").onchange = () => {
    if ($("mMode").value === "auto") L.inertial = { mass: L.inertial.mass, auto: true };
    else { const c = combinedInertia(L.collisions.length ? L.collisions : L.visuals, L.inertial.mass); L.inertial = { mass: L.inertial.mass, auto: false, origin: L.inertial.origin && !L.inertial.auto ? L.inertial.origin : c.origin, I: L.inertial.I && !L.inertial.auto ? L.inertial.I : c.I }; }
    commit();
  };
  if ($("cX")) ["cX", "cY", "cZ", "iXX", "iYY", "iZZ"].forEach(id => { $(id).onchange = () => {
    const ie2 = L.inertial; ie2.origin = { p: V(val("cX") / 1000, val("cY") / 1000, val("cZ") / 1000), q: ie2.origin ? ie2.origin.q : Q0() };
    ie2.I = { ...(ie2.I || {}), xx: Math.max(0, val("iXX")), yy: Math.max(0, val("iYY")), zz: Math.max(0, val("iZZ")) }; commit(); }; });
  $("mMu").onchange = () => { const v = $("mMu").value; if (v === "") { delete L.mu; delete L.mu2; } else { L.mu = Math.max(0, +v); L.mu2 = L.mu; } commit(); };
}

/* ---------- controle ---------- */
function runChecks() {
  const out = [], an = analyzeModel(M), cfg = an.cfg, add = (k, t, ln) => out.push(`<li class="${k}"${ln ? ` data-link="${ln}"` : ""}>${t}</li>`);
  robotRoot.updateMatrixWorld(true);
  const lowZ = ln => { const b = new THREE.Box3(); const n = nodes[ln]; if (!n) return Infinity; for (const o of n.vis) b.expandByObject(o); return b.min.z; };
  if (cfg.leftWheel && cfg.rightWheel) {
    add("ok", `Aandrijving herkend: ${cfg.leftWheel} + ${cfg.rightWheel} · straal ${r1(cfg.wheelRadius * 1000)} mm · spoor ${r1(cfg.wheelSep * 1000)} mm`);
    for (const w of [cfg.leftWheel, cfg.rightWheel]) { const jj = M.joints.find(j => j.name === w); if (!M.links[jj.child].collisions.length) add("bad", `${jj.child} heeft geen collision: het wiel raakt de vloer niet`, jj.child); }
    const touch = Object.keys(nodes).filter(ln => lowZ(ln) < 0.004), wheelsL = new Set([cfg.leftWheel, cfg.rightWheel].map(n => M.joints.find(j => j.name === n).child));
    const others = touch.filter(ln => !wheelsL.has(ln) && M.links[ln].collisions.length);
    if (!others.length) add("warn", "Enkel de twee wielen raken de vloer: de robot kantelt. Voeg een zwenkwiel toe.");
    for (const ln of others.filter(ln => !/caster/i.test(ln))) add("warn", `${ln} raakt de vloer en sleept mee. Is het een zwenkwiel? Noem het dan caster…, dan rolt het zonder wrijving.`, ln);
    if (others.some(ln => /caster/i.test(ln))) add("ok", "Zwenkwiel raakt de vloer.");
    const wz = Math.abs(lowZ([...wheelsL][0]) - lowZ([...wheelsL][1])); if (wz > 0.003) add("warn", `Wielen staan niet even hoog (${r1(wz * 1000)} mm verschil).`);
  } else add("warn", "Geen aandrijfwielen herkend (twee continuous joints met as langs y, links en rechts). De simulator zet de basis dan vast aan de wereld, zoals een arm op een sokkel.");
  if (cfg.lidar) {
    const z = new THREE.Vector3(); nodes[cfg.lidar] && nodes[cfg.lidar].motion.getWorldPosition(z);
    if (z.z > 0.34) add("warn", `2D-lidar op ${r1(z.z * 1000)} mm: boven de dozen (35 cm) in de werelden van de simulator.`, cfg.lidar);
    else add("ok", `2D-lidar ${cfg.lidar} op ${r1(z.z * 1000)} mm hoogte.`);
    const up = new THREE.Vector3(0, 0, 1).applyQuaternion(nodes[cfg.lidar].motion.getWorldQuaternion(new THREE.Quaternion()));
    if (up.z < 0.999) add("warn", `Scanvlak van de lidar helt ${r1(Math.acos(Math.min(1, Math.abs(up.z))) * R2D)}°.`, cfg.lidar);
  } else add("info", "Geen 2D-lidar: SLAM en Nav2 hebben er een nodig (link met lidar, laser of scan in de naam).");
  const com = robotCOM();
  if (com) add("info", `Totale massa ${r2(com.m)} kg · zwaartepunt x ${r1(com.c.x * 1000)} · y ${r1(com.c.y * 1000)} · z ${r1(com.c.z * 1000)} mm (boven de vloer).`);
  if (cfg.camera) add("ok", `Camera ${cfg.camera}.`); if (cfg.imu) add("ok", `IMU ${cfg.imu}.`);
  for (const [ln, L] of Object.entries(M.links)) if (L.collisions.length && !(L.inertial && L.inertial.mass > 0) && jointOf(ln) && jointOf(ln).type !== "fixed") add("warn", `${ln} beweegt maar heeft geen massa.`, ln);
  if (!M.links.base_link) add("info", "Geen base_link: Nav2 en de meeste ROS-pakketten verwachten base_link (en base_footprint).");
  const missing = Object.values(nodes).some(n => n.vis.some(o => o.userData.missing));
  if (missing) add("warn", "Sommige meshes ontbreken (grijze draadkubus). Open de map met de meshes via Nieuw / openen.");
  $("checks").innerHTML = out.join("");
  $("checks").querySelectorAll("li[data-link]").forEach(li => li.onclick = () => select(li.dataset.link));
}

/* ---------- laden, opslaan, simulator ---------- */
function setModel(m, ex = [], note = "") {
  M = m; extras = ex; sel = ""; selVis = 0; for (const k in preview) delete preview[k];
  for (const L of Object.values(M.links)) if (L.inertial) L.inertial.auto = L.inertial.auto ?? false;
  hist.list = []; hist.i = -1; commit(note); focusSel();
}
function emptyRobot() {
  const m = { name: "mijn_robot", root: "base_footprint", links: { base_footprint: { name: "base_footprint", inertial: null, visuals: [], collisions: [] } }, joints: [] };
  M = m;
  addLink("base_link", "base_footprint", "fixed", T(0, 0, 0.01), [shape({ type: "box", size: [0.2, 0.16, 0.008] }, T(0, 0, 0.004), [0.25, 0.27, 0.29, 1])], 0.8);
  return m;
}
async function loadText(text, label) {
  try {
    const m = parseURDF(text, DOMParser);
    const doc = new DOMParser().parseFromString(preXacro(text), "text/xml"), robotEl = doc.getElementsByTagName("robot")[0], ser = new XMLSerializer();
    const ex = Array.from(robotEl.childNodes).filter(n => n.nodeType === 1 && !["link", "joint", "material"].includes(n.nodeName)).map(n => {
      if (n.nodeName === "gazebo" && n.getAttribute("reference")) {   // mu1/mu2 staan in het model (parseURDF), niet dubbel bewaren
        for (const c of Array.from(n.childNodes)) if (c.nodeType === 1 && /^mu[12]$/.test(c.nodeName)) n.removeChild(c);
        if (!Array.from(n.childNodes).some(c => c.nodeType === 1)) return null;
      }
      return ser.serializeToString(n);
    }).filter(Boolean);
    const want = new Set(); for (const l of Object.values(m.links)) for (const e of [...l.visuals, ...l.collisions]) if (e.geom.type === "mesh") want.add(e.geom.filename);
    let miss = 0; for (const f of want) if (!(await loadMesh(f))) miss++;
    setModel(m, ex, `${label}: ${Object.keys(m.links).length} links${want.size ? `, ${want.size - miss}/${want.size} meshes` : ""}${ex.length ? `, ${ex.length} extra blok(ken) (gazebo, ros2_control…) blijven behouden` : ""}.`);
  } catch (e) { msg("Kon de URDF niet lezen: " + e.message, true); }
}
function resolveFile(filename) {
  const f = filename.replace(/^(package|file|model):\/\//, "").replace(/^\/+/, ""), keys = [...files.keys()];
  let hit = keys.find(k => k.endsWith(f) || f.endsWith(k));
  if (!hit) { const base = f.split("/").pop().toLowerCase(); hit = keys.find(k => k.split("/").pop().toLowerCase() === base); }
  return hit ? files.get(hit) : null;
}
async function loadMesh(filename) {
  if (meshCache.has(filename)) return meshCache.get(filename);
  const file = resolveFile(filename); if (!file) { return null; }
  const ext = file.name.split(".").pop().toLowerCase(); let obj = null;
  try {
    if (ext === "stl") obj = new THREE.Mesh(new STLLoader().parse(await file.arrayBuffer()));
    else if (ext === "dae") { obj = new ColladaLoader().parse(await file.text(), "").scene; obj.rotation.set(0, 0, 0); }
    else if (ext === "obj") obj = new OBJLoader().parse(await file.text());
  } catch { obj = null; }
  if (obj) meshCache.set(filename, obj); return obj;
}
const exportText = () => toURDF(clone(M), extras);
function msg(t, err) { $("msg").textContent = t; $("msg").className = "msg" + (err ? " err" : ""); }
$("robotName").onchange = () => { M.name = $("robotName").value.trim().replace(/\s+/g, "_") || "robot"; commit(); };
$("newBtn").onclick = e => { e.stopPropagation(); const p = $("newMenu"); p.hidden = !p.hidden; $("newBtn").setAttribute("aria-expanded", String(!p.hidden)); $("newFromSim").disabled = !store.get(KEY_FROM_SIM); };
document.addEventListener("click", e => { if (!$("newMenu").contains(e.target)) $("newMenu").hidden = true; });
document.querySelectorAll("#newMenu [data-new]").forEach(b => b.onclick = () => {
  $("newMenu").hidden = true;
  if (b.dataset.new === "empty") setModel(emptyRobot(), [], "Lege robot: een plaat op base_footprint. Voeg wielen, een zwenkwiel en sensoren toe.");
  if (b.dataset.new === "burger") loadText(BURGER_URDF, "TurtleBot3 Burger");
  if (b.dataset.new === "sim") { const o = JSON.parse(store.get(KEY_FROM_SIM) || "null"); if (o) loadText(o.urdf, "Uit de simulator"); }
});
$("openUrdf").onchange = async e => { const f = e.target.files[0]; $("newMenu").hidden = true; if (f) loadText(await f.text(), f.name); };
$("openDir").onchange = async e => {
  $("newMenu").hidden = true;
  for (const f of e.target.files) files.set((f.webkitRelativePath || f.name).replace(/\\/g, "/"), f);
  meshCache.clear();
  const urdfs = [...files.keys()].filter(k => /\.urdf$/i.test(k)), pref = urdfs.find(k => /burger/i.test(k)) || urdfs[0];
  if (pref) loadText(await files.get(pref).text(), pref.split("/").pop()); else msg(`${files.size} bestanden ingelezen, geen .urdf gevonden. Een .xacro eerst omzetten: xacro robot.urdf.xacro > robot.urdf`, true);
};
async function saveBlob(blob, name) {             // als artifact via de downloads-capability
  const dl = window.claude?.use ? await window.claude.use("downloads").catch(() => null) : null;
  if (dl) { try { await dl.save({ filename: name, data: blob }); } catch (e) { if (e?.code !== "declined") msg("Downloaden lukte niet: " + (e?.message || e?.code), true); } return; }
  const a = document.createElement("a"); a.href = URL.createObjectURL(blob); a.download = name; document.body.appendChild(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(a.href), 4000);
}
$("dlBtn").onclick = () => saveBlob(new Blob([exportText()], { type: "application/xml" }), `${M.name || "robot"}.urdf`);
$("showUrdf").onclick = () => { const t = $("urdfOut"); t.hidden = !t.hidden; $("showUrdf").textContent = t.hidden ? "Tonen" : "Verbergen"; if (!t.hidden) t.value = exportText(); };
$("copyBtn").onclick = async () => { try { await navigator.clipboard.writeText(exportText()); msg("URDF gekopieerd."); } catch { $("urdfOut").hidden = false; $("urdfOut").value = exportText(); $("urdfOut").select(); msg("Selecteer en kopieer de tekst."); } };
let simAck = 0;
if (bc) bc.onmessage = e => { if (e.data && e.data.type === "ack") simAck = Date.now(); };
const inFrame = window.self !== window.top;          // artifact: alles in hetzelfde kader, geen nieuwe tabbladen
$("backSim").onclick = () => { if (inFrame) location.href = SIM_URL; else { const a = document.createElement("a"); a.href = SIM_URL; a.target = "_blank"; document.body.appendChild(a); a.click(); a.remove(); } };
$("toSimBtn").onclick = () => {
  const urdf = exportText(), t = Date.now();
  store.set(KEY_TO_SIM, JSON.stringify({ urdf, t, name: M.name }));
  if (inFrame) { location.href = SIM_URL + "#editor-robot"; return; }
  if (bc) bc.postMessage({ type: "urdf", urdf, t });
  let tries = 0;
  const check = () => {
    if (simAck >= t) { msg("Robot staat in de simulator (ander tabblad)."); return; }
    if (bc && ++tries < 8) { setTimeout(check, 100); return; }   // tot 0,8 s (daarna verloopt de klik voor de pop-upblokkering) wachten op een open simulator
    const a = document.createElement("a"); a.href = SIM_URL + "#editor-robot"; a.target = "_blank"; a.rel = "opener"; document.body.appendChild(a); a.click(); a.remove();
    msg("Simulator geopend in een nieuw tabblad met deze robot.");
  };
  setTimeout(check, 100);
};

/* ---------- toetsen ---------- */
function setGizmo(g) { gizmo = g; document.querySelectorAll(".edtools .seg button").forEach(b => b.classList.toggle("on", b.dataset.g === g)); attachGizmo(); updateSelBox(); }
document.querySelectorAll(".edtools .seg button").forEach(b => b.onclick = () => setGizmo(b.dataset.g));
$("shapeOnly").onchange = () => { attachGizmo(); updateSelBox(); };
$("showColl").onchange = $("showFrames").onchange = () => { buildScene(); attachGizmo(); updateSelBox(); };
document.addEventListener("keydown", e => {
  if (/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)) return;
  const k = e.key.toLowerCase();
  if ((e.ctrlKey || e.metaKey) && k === "z") { e.preventDefault(); e.shiftKey ? redo() : undo(); return; }
  if ((e.ctrlKey || e.metaKey) && k === "y") { e.preventDefault(); redo(); return; }
  if ((e.ctrlKey || e.metaKey) && k === "d" && sel && jointOf(sel)) { e.preventDefault(); copySubtree(sel, false); return; }
  if (k === "g") setGizmo("translate"); else if (k === "r") setGizmo("rotate"); else if (k === "s") setGizmo("scale");
  else if (k === "f") focusSel(); else if (k === "escape") select("");
  else if ((k === "delete" || k === "backspace") && sel && sel !== M.root) removeLink(sel);
});

/* ---------- start ---------- */
function tick() { orbit.update(); if (selBox.visible) updateSelBox(); renderer.render(scene, camera); requestAnimationFrame(tick); }
$("loading").remove();
const fromSim = location.hash === "#from-sim" && JSON.parse(store.get(KEY_FROM_SIM) || "null");
const auto = store.get(KEY_AUTO);
if (fromSim) loadText(fromSim.urdf, "Uit de simulator");
else if (auto) { try { const o = JSON.parse(auto); setModel(o.M, o.extras, "Vorige sessie hersteld (automatisch bewaard in deze browser)."); } catch { setModel(emptyRobot()); } }
else loadText(BURGER_URDF, "TurtleBot3 Burger");
if (/[?&]test\b/.test(location.search)) window.__edtest = { get M() { return M; }, select, commit, exportText, PARTS, nodes: () => nodes, tc, readGizmo, setGizmo, undo, copySubtree, reparent };
requestAnimationFrame(tick);
