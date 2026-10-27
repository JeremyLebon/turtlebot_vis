// SIM-CORE-START
/* ===== Wiskunde: transform = { p:{x,y,z}, q:{x,y,z,w} } ===== */
const V = (x = 0, y = 0, z = 0) => ({ x, y, z });
const Q0 = () => ({ x: 0, y: 0, z: 0, w: 1 });
const vadd = (a, b) => V(a.x + b.x, a.y + b.y, a.z + b.z);
const vsub = (a, b) => V(a.x - b.x, a.y - b.y, a.z - b.z);
const vscale = (a, s) => V(a.x * s, a.y * s, a.z * s);
const vdot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
const vlen = a => Math.hypot(a.x, a.y, a.z);
const vnorm = a => { const l = vlen(a) || 1; return vscale(a, 1 / l); };
function qmul(a, b) {
  return { w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
}
const qconj = q => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w });
function qrot(q, v) { const r = qmul(qmul(q, { x: v.x, y: v.y, z: v.z, w: 0 }), qconj(q)); return V(r.x, r.y, r.z); }
const qaxis = (ax, a) => { const s = Math.sin(a / 2), n = vnorm(ax); return { x: n.x * s, y: n.y * s, z: n.z * s, w: Math.cos(a / 2) }; };
const qyaw = a => qaxis(V(0, 0, 1), a);
function qrpy(r, p, y) { return qmul(qyaw(y), qmul(qaxis(V(0, 1, 0), p), qaxis(V(1, 0, 0), r))); }   // URDF: Rz·Ry·Rx
const yawOf = q => Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
const T0 = () => ({ p: V(), q: Q0() });
const tmul = (a, b) => ({ p: vadd(a.p, qrot(a.q, b.p)), q: qmul(a.q, b.q) });
const QX90 = qaxis(V(1, 0, 0), Math.PI / 2);          // Rapier/three-cilinder (y-as) → URDF-cilinder (z-as)
const gauss = () => { let u = 0, v = 0; while (!u) u = Math.random(); while (!v) v = Math.random(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };

/* ===== Mini-xacro: arg, property, $(arg), $(find), ${expressie}. Macro's en includes niet. ===== */
function preXacro(text) {
  text = text.replace(/<!--[\s\S]*?-->/g, "");
  const attrs = tag => Object.fromEntries([...tag.matchAll(/(\w+)\s*=\s*"([^"]*)"/g)].map(m => [m[1], m[2]]));
  const args = {}, props = {};
  for (const m of text.matchAll(/<xacro:arg\b[^>]*\/>/g)) { const a = attrs(m[0]); args[a.name] = a.default ?? ""; }
  const subArgs = s => s.replace(/\$\(arg\s+([^)]+)\)/g, (m, n) => args[n.trim()] ?? "").replace(/\$\(find\s+([^)]+)\)/g, (m, pk) => "package://" + pk.trim());
  const evalExpr = e => {
    e = e.trim();
    if (e in props) return props[e];
    if (!/^[\w\s.+\-*/()]+$/.test(e)) throw new Error("xacro-expressie niet ondersteund: ${" + e + "}");
    const names = Object.keys(props).filter(k => /^[A-Za-z_]\w*$/.test(k));
    const f = new Function(...names, "pi", "radians", "degrees", "return (" + e + ");");
    const v = f(...names.map(n => (props[n] !== "" && !isNaN(+props[n])) ? +props[n] : props[n]), Math.PI, x => x * Math.PI / 180, x => x * 180 / Math.PI);
    return String(v);
  };
  const subExpr = s => s.replace(/\$\{([^}]*)\}/g, (m, e) => evalExpr(e));
  for (const m of text.matchAll(/<xacro:property\b[^>]*\/>/g)) { const a = attrs(m[0]); props[a.name] = subExpr(subArgs(a.value ?? "")); }
  text = text.replace(/<xacro:(arg|property)\b[^>]*\/>/g, "");
  text = subExpr(subArgs(text));
  if (/<xacro:/.test(text)) throw new Error("Deze URDF gebruikt xacro-macro's of includes. Zet hem eerst om: xacro robot.urdf.xacro > robot.urdf");
  return text;
}

/* ===== URDF-parser (DOMParser wordt meegegeven, zodat dit ook buiten de browser test) ===== */
function parseURDF(text, DOMParserImpl) {
  text = preXacro(text);
  const doc = new DOMParserImpl().parseFromString(text, "text/xml");
  const robotEl = doc.getElementsByTagName("robot")[0];
  if (!robotEl) throw new Error("Geen <robot>-element gevonden. Is dit een URDF (geen xacro)?");
  const kids = (el, tag) => Array.from(el.childNodes).filter(n => n.nodeType === 1 && n.nodeName === tag);
  const kid = (el, tag) => kids(el, tag)[0];
  const nums = (s, d) => (s == null || s === "" ? d : s.trim().split(/\s+/).map(Number));
  const origin = el => {
    const o = el && kid(el, "origin"); if (!o) return T0();
    const [x, y, z] = nums(o.getAttribute("xyz"), [0, 0, 0]), [r, p, yw] = nums(o.getAttribute("rpy"), [0, 0, 0]);
    return { p: V(x, y, z), q: qrpy(r, p, yw) };
  };
  const materials = {};
  const readMat = m => { const c = m && kid(m, "color"); return c ? nums(c.getAttribute("rgba"), [0.7, 0.7, 0.7, 1]) : null; };
  for (const m of kids(robotEl, "material")) materials[m.getAttribute("name")] = readMat(m);
  const geom = el => {
    const g = kid(el, "geometry"); if (!g) return null;
    const b = kid(g, "box"), c = kid(g, "cylinder"), s = kid(g, "sphere"), m = kid(g, "mesh");
    if (b) return { type: "box", size: nums(b.getAttribute("size"), [0.1, 0.1, 0.1]) };
    if (c) return { type: "cylinder", radius: +c.getAttribute("radius"), length: +c.getAttribute("length") };
    if (s) return { type: "sphere", radius: +s.getAttribute("radius") };
    if (m) { const sc = nums(m.getAttribute("scale"), [1, 1, 1]); return { type: "mesh", filename: m.getAttribute("filename"), scale: sc.length === 1 ? [sc[0], sc[0], sc[0]] : sc }; }
    return null;
  };
  const links = {};
  for (const l of kids(robotEl, "link")) {
    const name = l.getAttribute("name"), ie = kid(l, "inertial");
    let inertial = null;
    if (ie) {
      const m = kid(ie, "mass"), I = kid(ie, "inertia"), a = k => I ? +(I.getAttribute(k) || 0) : 0;
      inertial = { mass: m ? +m.getAttribute("value") : 0, origin: origin(ie),
        I: { xx: a("ixx"), yy: a("iyy"), zz: a("izz"), xy: a("ixy"), xz: a("ixz"), yz: a("iyz") } };
    }
    const visuals = kids(l, "visual").map(v => {
      const me = kid(v, "material");
      let color = me ? (readMat(me) || materials[me.getAttribute("name")] || null) : null;
      return { origin: origin(v), geom: geom(v), color };
    }).filter(v => v.geom);
    const collisions = kids(l, "collision").map(c => ({ origin: origin(c), geom: geom(c) })).filter(c => c.geom);
    links[name] = { name, inertial, visuals, collisions };
  }
  const joints = kids(robotEl, "joint").map(j => {
    const lim = kid(j, "limit"), ax = kid(j, "axis");
    return { name: j.getAttribute("name"), type: j.getAttribute("type"),
      parent: kid(j, "parent").getAttribute("link"), child: kid(j, "child").getAttribute("link"),
      origin: origin(j), axis: vnorm(V(...nums(ax && ax.getAttribute("xyz"), [1, 0, 0]))),
      limit: lim ? { lower: +(lim.getAttribute("lower") || 0), upper: +(lim.getAttribute("upper") || 0),
        velocity: +(lim.getAttribute("velocity") || 0), effort: +(lim.getAttribute("effort") || 0) } : null };
  });
  const children = new Set(joints.map(j => j.child));
  const roots = Object.keys(links).filter(n => !children.has(n));
  if (roots.length !== 1) throw new Error("De URDF moet precies één rootlink hebben (gevonden: " + roots.join(", ") + ")");
  return { name: robotEl.getAttribute("name") || "robot", links, joints, root: roots[0] };
}

/* ===== Nulstand-kinematica en automatische configuratie ===== */
function analyzeModel(model) {
  const linkT = { [model.root]: T0() }, parentJoint = {}, order = [model.root];
  const byParent = {};
  for (const j of model.joints) (byParent[j.parent] ||= []).push(j);
  for (let i = 0; i < order.length; i++) for (const j of byParent[order[i]] || []) {
    linkT[j.child] = tmul(linkT[j.parent], j.origin); parentJoint[j.child] = j; order.push(j.child);
  }
  const lname = n => n.toLowerCase();
  const find = re => order.find(n => re.test(lname(n))) || "";
  // aandrijfwielen: continuous joints met as langs ±y, links en rechts van het midden
  const cont = model.joints.filter(j => j.type === "continuous").map(j => ({ j, ax: qrot(linkT[j.child].q, j.axis), p: linkT[j.child].p }))
    .filter(o => Math.abs(o.ax.y) > 0.9 && !/caster/i.test(o.j.name));
  const left = cont.filter(o => o.p.y > 0.01).sort((a, b) => b.p.y - a.p.y)[0];
  const right = cont.filter(o => o.p.y < -0.01).sort((a, b) => a.p.y - b.p.y)[0];
  const wheelR = o => {
    const l = model.links[o.j.child], g = (l.collisions[0] || l.visuals[0] || {}).geom;
    return g && (g.type === "cylinder" || g.type === "sphere") ? g.radius : 0.05;
  };
  const cfg = {
    leftWheel: left ? left.j.name : "", rightWheel: right ? right.j.name : "",
    wheelRadius: left ? wheelR(left) : 0.033, wheelSep: left && right ? +(left.p.y - right.p.y).toFixed(4) : 0.16,
    lidar: find(/(base_scan|laser|lidar|scan)/), lidar3d: find(/(lidar_3d|velodyne|points|ouster|os1|3d_lidar)/),
    imu: find(/imu/), camera: order.find(n => /camera/.test(lname(n)) && !/optical/.test(lname(n))) || "",
    vmax: 0.22, wmax: 2.84,
  };
  if (cfg.lidar3d === cfg.lidar) cfg.lidar3d = "";
  cfg.fixedBase = !(cfg.leftWheel && cfg.rightWheel) || model.root === "world";
  const movable = model.joints.filter(j => j.type === "revolute" || j.type === "prismatic" || j.type === "continuous");
  return { linkT, parentJoint, order, cfg, movable };
}

/* ===== Simulator ===== */
function createSim(R) {
  const DT = 1 / 200;
  const G_WORLD = (0x0001 << 16) | 0xffff, G_ROBOT = (0x0002 << 16) | 0x0001, G_RAY = (0xffff << 16) | 0x0001;
  const LIDAR = { n: 360, rmin: 0.12, rmax: 3.5 };
  const L3 = { rings: 16, vfov: 30, hres: 2, rmax: 15, rmin: 0.2 };
  const world = new R.World({ x: 0, y: 0, z: -9.81 });
  world.timestep = DT;
  // stijvere contacten: standaard (30 Hz) laat kleine robots zichtbaar in de vloer zakken
  world.integrationParameters.numSolverIterations = 8;
  world.integrationParameters.contact_natural_frequency = 120;
  const statics = [], movables = [];
  const params = { odomErr: 0, lidarSigma: 0.01, imuNoise: true, ballWheels: true, wheelAcc: 2.0, odomSource: "truth" };
  let simTime = 0, robot = null;

  /* ---- wereld (data-gedreven, zie WORLDS onderaan en het JSON-formaat) ---- */
  const markings = [];
  let worldInfo = { name: "", floor: [12, 12], spawn: [0, 0, 0], items: [] };
  const rad = d => (d || 0) * Math.PI / 180;
  function addStatic(kind, desc, pos, rot, size, color, def) {
    const body = world.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(pos.x, pos.y, pos.z).setRotation(rot));
    world.createCollider(desc.setCollisionGroups(G_WORLD).setFriction(0.8), body);
    statics.push({ kind, body, size, color, def });
  }
  // box: x,y,[z = op de vloer],sx,sy,sz,[yaw,pitch,roll in graden],[color]
  function sBox(d) {
    const z = d.z ?? d.sz / 2, q = qrpy(rad(d.roll), rad(d.pitch), rad(d.yaw));
    addStatic("box", R.ColliderDesc.cuboid(d.sx / 2, d.sy / 2, d.sz / 2), V(d.x, d.y, z), q, V(d.sx, d.sy, d.sz), d.color, { type: "box", ...d });
  }
  function sCyl(d) {
    const h = d.h, body = world.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(d.x, d.y, d.z ?? h / 2));
    world.createCollider(R.ColliderDesc.cylinder(h / 2, d.r).setRotation(QX90).setCollisionGroups(G_WORLD).setFriction(0.8), body);
    statics.push({ kind: "cyl", body, size: { r: d.r, h }, color: d.color, def: { type: "cyl", ...d } });
  }
  function sWall(d) {
    const t = d.t ?? 0.1, h = d.h ?? 0.25, L = Math.hypot(d.x1 - d.x0, d.y1 - d.y0) + t;
    addStatic("box", R.ColliderDesc.cuboid(L / 2, t / 2, h / 2), V((d.x0 + d.x1) / 2, (d.y0 + d.y1) / 2, h / 2),
      qyaw(Math.atan2(d.y1 - d.y0, d.x1 - d.x0)), V(L, t, h), d.color, { type: "wall", ...d });
  }
  // stelling: staanders + liggers + legborden; de 2D-lidar ziet op 18 cm enkel de staanders
  function sRack(d) {
    const bays = d.bays ?? 3, bw = d.bay ?? 2.7, dep = d.depth ?? 1.0, H = d.h ?? 2.0, lv = d.levels ?? [0.9, 1.8];
    const L = bays * bw, c = Math.cos(rad(d.yaw)), sn = Math.sin(rad(d.yaw));
    const P = (u, v) => ({ x: d.x + c * u - sn * v, y: d.y + sn * u + c * v });
    for (let i = 0; i <= bays; i++) for (const v of [-dep / 2, dep / 2]) { const p = P(-L / 2 + i * bw, v); sBox({ x: p.x, y: p.y, sx: 0.08, sy: 0.08, sz: H, yaw: d.yaw, color: "#e0691b" }); }
    for (const z of lv) {
      for (const v of [-dep / 2, dep / 2]) { const p = P(0, v); sBox({ x: p.x, y: p.y, z, sx: L, sy: 0.05, sz: 0.1, yaw: d.yaw, color: "#2d5fb3" }); }
      const p = P(0, 0); sBox({ x: p.x, y: p.y, z: z + 0.06, sx: L, sy: dep, sz: 0.02, yaw: d.yaw, color: "#9aa3a8" });
      for (let i = 0; i < bays; i++) if ((i + Math.round(z * 10)) % 3 !== 2) {                        // pallets met lading op de legborden
        const q = P(-L / 2 + (i + 0.5) * bw, 0); sBox({ x: q.x, y: q.y, z: z + 0.07 + 0.35, sx: bw * 0.8, sy: dep * 0.85, sz: 0.7, yaw: d.yaw, color: "#c9a26b" });
      }
    }
  }
  // transportband: band op hoogte, poten eronder (op lidarhoogte enkel de poten)
  function sConveyor(d) {
    const L = d.l, W = d.w ?? 0.6, H = d.h ?? 0.8, c = Math.cos(rad(d.yaw)), sn = Math.sin(rad(d.yaw));
    sBox({ x: d.x, y: d.y, z: H - 0.08, sx: L, sy: W, sz: 0.16, yaw: d.yaw, color: "#3d4447" });
    sBox({ x: d.x, y: d.y, z: H + 0.005, sx: L - 0.1, sy: W - 0.08, sz: 0.01, yaw: d.yaw, color: "#1c1f20" });
    const n = Math.max(2, Math.round(L / 1.5) + 1);
    for (let i = 0; i < n; i++) for (const v of [-W / 2 + 0.04, W / 2 - 0.04]) {
      const u = -L / 2 + 0.1 + i * (L - 0.2) / (n - 1);
      sBox({ x: d.x + c * u - sn * v, y: d.y + sn * u + c * v, sx: 0.05, sy: 0.05, sz: H - 0.16, yaw: d.yaw, color: "#7d878b" });
    }
  }
  function sPallet(d) {                       // europallet 1,2 × 0,8 × 0,15, optioneel met lading
    sBox({ x: d.x, y: d.y, sx: 1.2, sy: 0.8, sz: 0.15, yaw: d.yaw, color: "#b98a54" });
    if (d.load) sBox({ x: d.x, y: d.y, z: 0.15 + d.load / 2, sx: 1.1, sy: 0.72, sz: d.load, yaw: d.yaw, color: d.color ?? "#d9c39a" });
  }
  function clearWorld(floor = [12, 12]) {
    for (const st of statics) world.removeRigidBody(st.body);
    for (const m of movables) world.removeRigidBody(m.body);
    statics.length = 0; movables.length = 0; markings.length = 0;
    addStatic("floor", R.ColliderDesc.cuboid(floor[0] / 2 + 2, floor[1] / 2 + 2, 0.05), V(0, 0, -0.05), Q0(), V(floor[0], floor[1], 0.1));
  }
  function loadWorld(def) {
    worldInfo = { name: def.name || "wereld", floor: def.floor || [12, 12], spawn: def.spawn || [0, 0, 0],
      items: (def.items || []).filter(it => it.type !== "movable"), fields: (def.fields || []).map(f => ({ ...f })) };
    clearWorld(worldInfo.floor);
    const make = { box: sBox, cyl: sCyl, wall: sWall, rack: sRack, conveyor: sConveyor, pallet: sPallet,
      movable: d => addBox(d.x, d.y, rad(d.yaw), d.sx ?? 0.25, d.sz ?? 0.35, d.mass ?? 0.6, d.sy, d.color),
      mark: d => markings.push({ ...d }) };
    for (const it of def.items || []) {
      const f = make[it.type]; if (!f) throw new Error("Onbekend wereldelement: " + it.type);
      f(it);
    }
    return worldInfo;
  }
  // huidige toestand terug naar JSON: vaste elementen zoals geladen, dozen op hun huidige plek
  function exportWorld() {
    const items = worldInfo.items.map(it => ({ ...it }));
    for (const m of movables) {
      const t = m.body.translation(), q = m.body.rotation();
      items.push({ type: "movable", x: +t.x.toFixed(3), y: +t.y.toFixed(3), yaw: +(yawOf(q) * 180 / Math.PI).toFixed(1),
        sx: m.size.x, sy: m.size.y, sz: m.size.z, mass: m.mass, ...(m.color ? { color: m.color } : {}) });
    }
    return { name: worldInfo.name, floor: worldInfo.floor, spawn: worldInfo.spawn, items, ...(worldInfo.fields.length ? { fields: worldInfo.fields } : {}) };
  }
  function addBox(x, y, yaw = 0, s = 0.25, h = 0.35, mass = 0.6, sy, color) {
    const w = sy ?? s;
    const body = world.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(x, y, h / 2 + 0.001).setRotation(qyaw(yaw)).setLinearDamping(0.2).setAngularDamping(0.5));
    world.createCollider(R.ColliderDesc.cuboid(s / 2, w / 2, h / 2).setMass(mass).setFriction(0.5).setCollisionGroups(G_WORLD), body);
    movables.push({ kind: "box", body, size: V(s, w, h), mass, color });
  }
  // testpaal ("stoelpoot") voor de taaktests: kinematisch, verplaatsbaar, gezien door de lidar
  const probes = [];
  function addProbe(x, y, r = 0.04, h = 0.6) {
    const body = world.createRigidBody(R.RigidBodyDesc.kinematicPositionBased().setTranslation(x, y, h / 2));
    world.createCollider(R.ColliderDesc.cylinder(h / 2, r).setRotation(QX90).setCollisionGroups(G_WORLD), body);
    const pr = { body, r, h }; probes.push(pr); return pr;
  }
  function moveProbe(pr, x, y) { pr.body.setTranslation({ x, y, z: pr.h / 2 }, true); }
  function removeProbe(pr) { const i = probes.indexOf(pr); if (i >= 0) probes.splice(i, 1); world.removeRigidBody(pr.body); }
  const worldSample = () => loadWorld(WORLDS.turtlebot3);
  const worldEmpty = () => loadWorld(WORLDS.empty);

  /* ---- robot uit URDF ---- */
  function colliderDesc(g, isWheel) {
    if (g.type === "box") return R.ColliderDesc.cuboid(g.size[0] / 2, g.size[1] / 2, g.size[2] / 2);
    if (g.type === "sphere") return R.ColliderDesc.ball(g.radius);
    if (g.type === "cylinder") return isWheel && params.ballWheels ? R.ColliderDesc.ball(g.radius) : R.ColliderDesc.cylinder(g.length / 2, g.radius);
    if (g.type === "mesh" && g.points && g.points.length >= 12) return R.ColliderDesc.convexHull(g.points);
    return null;
  }
  // rotatie van een URDF-geometrie naar de Rapier-vorm
  const shapeRot = (g, isWheel) => (g.type === "cylinder" && !(isWheel && params.ballWheels) ? QX90 : Q0());
  function geomMinZ(T, g) {                          // laagste punt in rootframe (benadering met omhullende doos)
    let h;
    if (g.type === "box") h = g.size.map(s => s / 2);
    else if (g.type === "sphere") h = [g.radius, g.radius, g.radius];
    else if (g.type === "cylinder") h = [g.radius, g.radius, g.length / 2];
    else if (g.points) { let m = Infinity; for (let i = 0; i < g.points.length; i += 3) m = Math.min(m, vadd(T.p, qrot(T.q, V(g.points[i], g.points[i + 1], g.points[i + 2]))).z); return m; }
    else return Infinity;
    let m = Infinity;
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) m = Math.min(m, vadd(T.p, qrot(T.q, V(sx * h[0], sy * h[1], sz * h[2]))).z);
    if (g.type === "cylinder") {                    // cilinder: exacter langs de cirkel
      m = Infinity;
      for (let k = 0; k < 16; k++) for (const sz of [-1, 1]) { const a = k * Math.PI / 8; m = Math.min(m, vadd(T.p, qrot(T.q, V(g.radius * Math.cos(a), g.radius * Math.sin(a), sz * h[2]))).z); }
    }
    return m;
  }

  function removeRobot() { if (robot) { for (const b of robot.bodies) world.removeRigidBody(b.rb); robot = null; } }

  function buildRobot(model, cfg, x, y, yaw) {
    removeRobot();
    const an = analyzeModel(model), { linkT, parentJoint, order } = an;
    const wheelJ = new Set([cfg.leftWheel, cfg.rightWheel].filter(Boolean));
    // 1. links groeperen tot starre lichamen (fixed joints samenvoegen)
    const bodyOf = {}, bodies = [];
    for (const ln of order) {
      const pj = parentJoint[ln];
      if (pj && (pj.type === "fixed" || pj.type === "floating" || pj.type === "planar")) bodyOf[ln] = bodyOf[pj.parent];
      else { bodyOf[ln] = bodies.length; bodies.push({ root: ln, links: [], p: linkT[ln].p }); }
      bodies[bodyOf[ln]].links.push(ln);
    }
    const linkOff = {};                               // link-frame t.o.v. zijn lichaam (lichaam heeft rootoriëntatie)
    for (const ln of order) { const B = bodies[bodyOf[ln]]; linkOff[ln] = { p: vsub(linkT[ln].p, B.p), q: linkT[ln].q }; }
    // 2. grondhoogte zodat de laagste collider de vloer raakt
    let minZ = Infinity;
    for (const ln of order) for (const c of model.links[ln].collisions) {
      const isWheel = parentJoint[ln] && wheelJ.has(parentJoint[ln].name);
      const g = (c.geom.type === "cylinder" && isWheel && params.ballWheels) ? { type: "sphere", radius: c.geom.radius } : c.geom;
      minZ = Math.min(minZ, geomMinZ(tmul(linkT[ln], c.origin), g));
    }
    const z0 = (isFinite(minZ) ? Math.max(0, -minZ) : 0) + 0.001;
    const S = { p: V(x, y, z0), q: qyaw(yaw) };
    // 3. Rapier-lichamen, colliders en massa
    for (const [bi, B] of bodies.entries()) {
      const wp = vadd(S.p, qrot(S.q, B.p));
      const fixed = bi === 0 && cfg.fixedBase;
      const desc = (fixed ? R.RigidBodyDesc.fixed() : R.RigidBodyDesc.dynamic()).setTranslation(wp.x, wp.y, wp.z).setRotation(S.q).setCanSleep(false);
      let m = 0, com = V(), Ixx = 0, Iyy = 0, Izz = 0;
      const parts = [];
      for (const ln of B.links) {
        const ie = model.links[ln].inertial;
        if (!ie || !(ie.mass > 0)) continue;
        const Ti = tmul(linkOff[ln], ie.origin);
        parts.push({ m: ie.mass, c: Ti.p, q: Ti.q, I: ie.I }); m += ie.mass; com = vadd(com, vscale(Ti.p, ie.mass));
      }
      if (m > 0) com = vscale(com, 1 / m); else { m = 0.01; com = V(); }
      for (const pt of parts) {                       // tensor naar lichaamsassen + Steiner, enkel diagonaal
        const Rm = [qrot(pt.q, V(1, 0, 0)), qrot(pt.q, V(0, 1, 0)), qrot(pt.q, V(0, 0, 1))];
        const Il = [[pt.I.xx, pt.I.xy, pt.I.xz], [pt.I.xy, pt.I.yy, pt.I.yz], [pt.I.xz, pt.I.yz, pt.I.zz]];
        const comp = (a, b) => { let s = 0; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) s += [Rm[i].x, Rm[i].y, Rm[i].z][a] * Il[i][j] * [Rm[j].x, Rm[j].y, Rm[j].z][b]; return s; };
        const d = vsub(pt.c, com);
        Ixx += comp(0, 0) + pt.m * (d.y * d.y + d.z * d.z); Iyy += comp(1, 1) + pt.m * (d.x * d.x + d.z * d.z); Izz += comp(2, 2) + pt.m * (d.x * d.x + d.y * d.y);
      }
      const minI = Math.max(1e-6, m * 1e-4);
      if (!fixed) desc.setAdditionalMassProperties(m, com, V(Math.max(minI, Ixx), Math.max(minI, Iyy), Math.max(minI, Izz)), Q0());
      const rb = world.createRigidBody(desc);
      for (const ln of B.links) {
        const isWheel = parentJoint[ln] && wheelJ.has(parentJoint[ln].name), caster = /caster/i.test(ln);
        for (const c of model.links[ln].collisions) {
          const cd = colliderDesc(c.geom, isWheel); if (!cd) continue;
          const Tc = tmul(linkOff[ln], c.origin);
          cd.setTranslation(Tc.p.x, Tc.p.y, Tc.p.z).setRotation(qmul(Tc.q, shapeRot(c.geom, isWheel))).setDensity(0)
            .setFriction(caster ? 0 : isWheel ? 1.0 : 0.4).setCollisionGroups(G_ROBOT);
          if (caster) cd.setFrictionCombineRule(R.CoefficientCombineRule.Min);
          world.createCollider(cd, rb);
        }
      }
      B.rb = rb; B.mass = m;
    }
    // 4. joints tussen lichamen
    const joints = [];
    for (const j of model.joints) {
      if (!["revolute", "continuous", "prismatic"].includes(j.type)) continue;
      const b1 = bodies[bodyOf[j.parent]], b2 = bodies[bodyOf[j.child]];
      const pj = linkT[j.child].p, axis = vnorm(qrot(linkT[j.child].q, j.axis));
      const a1 = vsub(pj, b1.p), a2 = vsub(pj, b2.p);
      const data = j.type === "prismatic" ? R.JointData.prismatic(a1, a2, axis) : R.JointData.revolute(a1, a2, axis);
      const ij = world.createImpulseJoint(data, b1.rb, b2.rb, true);
      ij.setContactsEnabled(false);
      const lim = j.limit && j.type !== "continuous" && j.limit.upper > j.limit.lower ? j.limit : null;
      if (lim) ij.setLimits(lim.lower, lim.upper);
      const role = wheelJ.has(j.name) ? "wheel" : (j.type === "continuous" ? "passive" : "position");
      if (role !== "passive") ij.configureMotorModel(R.MotorModel.AccelerationBased);
      joints.push({ j, ij, b1, b2, a1, a2, axis, role, lim, pos: 0, vel: 0, target: lim ? Math.min(Math.max(0, lim.lower), lim.upper) : 0,
        sign: role === "wheel" ? Math.sign(axis.y) : 1 });
    }
    robot = { model, cfg, an, bodies, bodyOf, linkOff, joints, rootBody: bodies[0],
      byName: Object.fromEntries(joints.map(o => [o.j.name, o])), prevV: V(), imuAcc: V(0, 0, 9.81) };
    // 0,5 s laten uitzakken met stilstaande motoren, zodat odometrie niet start met een sprongetje
    for (let i = 0; i < 100; i++) {
      for (const o of joints) { if (o.role === "wheel") o.ij.configureMotorVelocity(0, 500); else if (o.role === "position") o.ij.configureMotorPosition(o.target, 20000, 300); }
      world.step();
    }
    for (const o of joints) { measureJoint(o); if (o.j.type === "continuous") o.pos = 0; }
    const lv0 = bodies[0].rb.linvel(); robot.prevV = V(lv0.x, lv0.y, lv0.z);
    resetOdom();
    return robot;
  }

  /* ---- toestand en meting ---- */
  const odom = { x: 0, y: 0, th: 0, v: 0, w: 0 };
  const imu = { q: Q0(), gyro: V(), acc: V(0, 0, 9.81) };
  let cmdTarget = { v: 0, w: 0 };
  function resetOdom() { Object.assign(odom, { x: 0, y: 0, th: 0, v: 0, w: 0 }); if (robot) { robot.odomRef = bodyT(robot.rootBody); for (const o of robot.joints) if (o.j.type === "continuous") o.pos = 0; } }

  function bodyT(B) { const t = B.rb.translation(), q = B.rb.rotation(); return { p: V(t.x, t.y, t.z), q: { x: q.x, y: q.y, z: q.z, w: q.w } }; }
  function linkWorld(ln) { return tmul(bodyT(robot.bodies[robot.bodyOf[ln]]), robot.linkOff[ln]); }

  function measureJoint(o) {
    const T1 = bodyT(o.b1), T2 = bodyT(o.b2), axW = qrot(T1.q, o.axis);
    if (o.j.type === "prismatic") {
      const d = vdot(qrot(qconj(T1.q), vsub(vadd(T2.p, qrot(T2.q, o.a2)), vadd(T1.p, qrot(T1.q, o.a1)))), o.axis);
      o.vel = (d - o.pos) / DT; o.pos = d; return;
    }
    const w1 = o.b1.rb.angvel(), w2 = o.b2.rb.angvel();
    o.vel = vdot(V(w2.x - w1.x, w2.y - w1.y, w2.z - w1.z), axW);
    if (o.j.type === "continuous") o.pos += o.vel * DT;
    else { const qr = qmul(qconj(T1.q), T2.q); o.pos = 2 * Math.atan2(vdot(V(qr.x, qr.y, qr.z), o.axis), qr.w); if (o.pos > Math.PI) o.pos -= 2 * Math.PI; if (o.pos < -Math.PI) o.pos += 2 * Math.PI; }
  }

  function step(cmd) {
    const c = robot.cfg;
    const v = Math.max(-c.vmax, Math.min(c.vmax, cmd.v)), w = Math.max(-c.wmax, Math.min(c.wmax, cmd.w));
    cmdTarget = { v, w };
    const wmaxWheel = c.vmax / c.wheelRadius, clampW = s => Math.max(-wmaxWheel, Math.min(wmaxWheel, s));
    for (const o of robot.joints) {
      if (o.role === "wheel") {
        const lin = o.j.name === c.leftWheel ? v - w * c.wheelSep / 2 : v + w * c.wheelSep / 2;
        // versnellingsbegrenzing per wiel (zoals een motorprofiel); sprongen veroorzaken wielslip
        const want = clampW(lin / c.wheelRadius), dmax = (params.wheelAcc / c.wheelRadius) * DT;
        o.cmdW = (o.cmdW || 0) + Math.max(-dmax, Math.min(dmax, want - (o.cmdW || 0)));
        o.ij.configureMotorVelocity(o.sign * o.cmdW, 500);
      } else if (o.role === "position") {
        // PD-motor van Rapier + integrerende term tegen doorzakken onder zwaartekracht
        o.integ = Math.max(-0.5, Math.min(0.5, (o.integ || 0) + (o.target - o.pos) * DT));
        o.ij.configureMotorPosition(o.target + 8 * o.integ, 20000, 300);
      }
    }
    // rem bij stilstand: nulcommando en wielen uitgelopen → geen horizontale drift (anders kruipt de robot door
    // kleine trillingen van de bolcontacten, ongeveer 2° per 40 s)
    const wheels = robot.joints.filter(o => o.role === "wheel");
    let braking = false;
    if (wheels.length && !c.fixedBase && v === 0 && w === 0 && wheels.every(o => Math.abs(o.cmdW || 0) < 1e-9)) {
      const rb = robot.rootBody.rb, lv = rb.linvel(), av = rb.angvel();
      if (Math.hypot(lv.x, lv.y) < 0.05 && Math.abs(av.z) < 0.3) {
        braking = true;
        rb.setLinvel({ x: 0, y: 0, z: lv.z }, true); rb.setAngvel({ x: av.x, y: av.y, z: 0 }, true);
        for (const o of wheels) o.b2.rb.setAngvel(rb.angvel(), true);
        if (!robot.hold) { const T = bodyT(robot.rootBody); robot.hold = { x: T.p.x, y: T.p.y, yaw: yawOf(T.q) }; }
      }
    }
    if (!braking) robot.hold = null;
    world.step(); simTime += DT;
    if (braking && robot.hold) {                          // positie en yaw vasthouden (roll/pitch blijven vrij)
      const rb = robot.rootBody.rb, T = bodyT(robot.rootBody), dyaw = robot.hold.yaw - yawOf(T.q);
      const dx = robot.hold.x - T.p.x, dy = robot.hold.y - T.p.y;
      if (Math.abs(dyaw) > 1e-6 || Math.hypot(dx, dy) > 1e-6) {
        const qn = qmul(qyaw(dyaw), T.q);
        for (const B of robot.bodies) {                     // hele robot rond de basis meedraaien
          const BT = bodyT(B), rel = vsub(BT.p, T.p), np = vadd(V(robot.hold.x, robot.hold.y, T.p.z), qrot(qyaw(dyaw), rel));
          B.rb.setTranslation(np, true); B.rb.setRotation(B === robot.rootBody ? qn : qmul(qyaw(dyaw), BT.q), true);
        }
        void rb;
      }
    }
    for (const o of robot.joints) measureJoint(o);

    // odometrie: "truth" = perfect uit de simulatie (zoals de Gazebo diff-drive-plugin), "wheels" = wielencoders
    const L = robot.byName[c.leftWheel], Rw = robot.byName[c.rightWheel];
    if (L && Rw && params.odomSource === "truth") {
      const T = bodyT(robot.rootBody), ref = robot.odomRef, d = vsub(T.p, ref.p), th0 = yawOf(ref.q), th = yawOf(T.q);
      odom.x = Math.cos(th0) * d.x + Math.sin(th0) * d.y; odom.y = -Math.sin(th0) * d.x + Math.cos(th0) * d.y;
      odom.th = Math.atan2(Math.sin(th - th0), Math.cos(th - th0));
      const lv = robot.rootBody.rb.linvel(), av = robot.rootBody.rb.angvel();
      odom.v = Math.cos(th) * lv.x + Math.sin(th) * lv.y; odom.w = av.z;
    } else if (L && Rw) {
      const wl = L.sign * L.vel, wr = Rw.sign * Rw.vel, e = params.odomErr / 2;
      const vo = (c.wheelRadius * (1 + e) * wl + c.wheelRadius * (1 - e) * wr) / 2, wo = (c.wheelRadius * (1 - e) * wr - c.wheelRadius * (1 + e) * wl) / c.wheelSep;
      const thm = odom.th + wo * DT / 2;
      odom.x += vo * Math.cos(thm) * DT; odom.y += vo * Math.sin(thm) * DT;
      odom.th = Math.atan2(Math.sin(odom.th + wo * DT), Math.cos(odom.th + wo * DT)); odom.v = vo; odom.w = wo;
    }
    // IMU (op het lichaam van de imu-link; hefboomarm verwaarloosd)
    if (c.imu && robot.bodyOf[c.imu] != null) {
      const B = robot.bodies[robot.bodyOf[c.imu]], Tl = linkWorld(c.imu), qi = qconj(Tl.q), lv = B.rb.linvel();
      const a = V((lv.x - robot.prevV.x) / DT, (lv.y - robot.prevV.y) / DT, (lv.z - robot.prevV.z) / DT + 9.81);
      robot.prevV = V(lv.x, lv.y, lv.z);
      const n = params.imuNoise ? 1 : 0, gb = qrot(qi, B.rb.angvel()), ab = qrot(qi, a), k = 0.2;
      imu.q = Tl.q; imu.gyro = V(gb.x + n * 0.002 * gauss(), gb.y + n * 0.002 * gauss(), gb.z + n * 0.002 * gauss());
      imu.acc = V(imu.acc.x + k * (ab.x + n * 0.02 * gauss() - imu.acc.x), imu.acc.y + k * (ab.y + n * 0.02 * gauss() - imu.acc.y), imu.acc.z + k * (ab.z + n * 0.02 * gauss() - imu.acc.z));
    }
  }

  function castFrom(o, d, rmax) { const h = world.castRay(new R.Ray(o, d), rmax, true, undefined, G_RAY); return h ? (h.timeOfImpact ?? h.toi) : 0; }
  function scan(link) {
    const T = linkWorld(link), inc = 2 * Math.PI / LIDAR.n, ranges = new Array(LIDAR.n), pts = [];
    for (let k = 0; k < LIDAR.n; k++) {
      const d = qrot(T.q, V(Math.cos(k * inc), Math.sin(k * inc), 0));
      let r = castFrom(T.p, d, LIDAR.rmax);
      if (r > 0) { r += params.lidarSigma * gauss(); if (r < LIDAR.rmin || r > LIDAR.rmax) r = 0; }
      ranges[k] = Math.round(r * 1000) / 1000;
      if (r > 0) pts.push(T.p.x + d.x * r, T.p.y + d.y * r, T.p.z + d.z * r);
    }
    return { ranges, pts, inc };
  }
  function scan3d(link) {
    const T = linkWorld(link), nh = Math.round(360 / L3.hres), sensor = [], worldPts = [];
    for (let i = 0; i < L3.rings; i++) {
      const el = (-L3.vfov / 2 + i * L3.vfov / (L3.rings - 1)) * Math.PI / 180, ce = Math.cos(el), se = Math.sin(el);
      for (let k = 0; k < nh; k++) {
        const az = k * L3.hres * Math.PI / 180, db = V(ce * Math.cos(az), ce * Math.sin(az), se), d = qrot(T.q, db);
        let r = castFrom(T.p, d, L3.rmax); if (!r) continue;
        r += params.lidarSigma * gauss(); if (r < L3.rmin) continue;
        sensor.push(db.x * r, db.y * r, db.z * r); worldPts.push(T.p.x + d.x * r, T.p.y + d.y * r, T.p.z + d.z * r);
      }
    }
    return { sensor, worldPts };
  }
  // transform parent→child van een joint bij de gemeten stand (voor /tf)
  function jointTF(o) {
    const j = o.j;
    if (j.type === "prismatic") return tmul(j.origin, { p: vscale(j.axis, o.pos), q: Q0() });
    return tmul(j.origin, { p: V(), q: qaxis(j.axis, o.pos) });
  }
  function truthPose() { const T = bodyT(robot.rootBody); return { x: T.p.x, y: T.p.y, z: T.p.z, th: yawOf(T.q), q: T.q }; }

  return { R, DT, LIDAR, L3, world, statics, movables, odom, imu, params,
    get robot() { return robot; }, get simTime() { return simTime; }, get cmdTarget() { return cmdTarget; },
    worldSample, worldEmpty, loadWorld, exportWorld, markings, probes, addProbe, moveProbe, removeProbe, get worldInfo() { return worldInfo; }, addBox, buildRobot, removeRobot, resetOdom, step, scan, scan3d, linkWorld, bodyT, jointTF, truthPose };
}

/* ===== Ingebouwde TurtleBot3 Burger (afmetingen van turtlebot3_description, primitieve vormen) ===== */
const BURGER_URDF = `<?xml version="1.0"?>
<robot name="turtlebot3_burger_lite">
  <material name="dark"><color rgba="0.17 0.18 0.18 1"/></material>
  <material name="board"><color rgba="0.12 0.42 0.23 1"/></material>
  <material name="metal"><color rgba="0.80 0.82 0.81 1"/></material>
  <material name="tire"><color rgba="0.08 0.08 0.08 1"/></material>
  <material name="sensor"><color rgba="0.23 0.24 0.24 1"/></material>
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0.010"/></joint>
  <link name="base_link">
    <visual><origin xyz="-0.032 0 0.035"/><geometry><box size="0.138 0.138 0.004"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="-0.032 0 0.075"/><geometry><box size="0.138 0.138 0.004"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="-0.032 0 0.115"/><geometry><box size="0.138 0.138 0.004"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="-0.032 0 0.155"/><geometry><box size="0.138 0.138 0.004"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="-0.032 0 0.125"/><geometry><box size="0.09 0.08 0.012"/></geometry><material name="board"/></visual>
    <visual><origin xyz="-0.032 0 0.050"/><geometry><box size="0.10 0.07 0.025"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="0.030 0.06 0.095"/><geometry><cylinder radius="0.003" length="0.12"/></geometry><material name="metal"/></visual>
    <visual><origin xyz="0.030 -0.06 0.095"/><geometry><cylinder radius="0.003" length="0.12"/></geometry><material name="metal"/></visual>
    <visual><origin xyz="-0.094 0.06 0.095"/><geometry><cylinder radius="0.003" length="0.12"/></geometry><material name="metal"/></visual>
    <visual><origin xyz="-0.094 -0.06 0.095"/><geometry><cylinder radius="0.003" length="0.12"/></geometry><material name="metal"/></visual>
    <collision><origin xyz="-0.032 0 0.090"/><geometry><box size="0.140 0.140 0.120"/></geometry></collision>
    <inertial><origin xyz="-0.030 0 0.060"/><mass value="0.82"/><inertia ixx="2.2e-3" ixy="0" ixz="0" iyy="2.1e-3" iyz="0" izz="2.0e-3"/></inertial>
  </link>
  <joint name="wheel_left_joint" type="continuous"><parent link="base_link"/><child link="wheel_left_link"/><origin xyz="0 0.08 0.023" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/></joint>
  <link name="wheel_left_link">
    <visual><geometry><cylinder radius="0.033" length="0.018"/></geometry><material name="tire"/></visual>
    <visual><geometry><box size="0.05 0.008 0.021"/></geometry><material name="metal"/></visual>
    <collision><geometry><cylinder radius="0.033" length="0.018"/></geometry></collision>
    <inertial><mass value="0.028"/><inertia ixx="1.1e-5" ixy="0" ixz="0" iyy="1.1e-5" iyz="0" izz="2.1e-5"/></inertial>
  </link>
  <joint name="wheel_right_joint" type="continuous"><parent link="base_link"/><child link="wheel_right_link"/><origin xyz="0 -0.08 0.023" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/></joint>
  <link name="wheel_right_link">
    <visual><geometry><cylinder radius="0.033" length="0.018"/></geometry><material name="tire"/></visual>
    <visual><geometry><box size="0.05 0.008 0.021"/></geometry><material name="metal"/></visual>
    <collision><geometry><cylinder radius="0.033" length="0.018"/></geometry></collision>
    <inertial><mass value="0.028"/><inertia ixx="1.1e-5" ixy="0" ixz="0" iyy="1.1e-5" iyz="0" izz="2.1e-5"/></inertial>
  </link>
  <joint name="caster_back_joint" type="fixed"><parent link="base_link"/><child link="caster_back_link"/><origin xyz="-0.081 0 -0.004"/></joint>
  <link name="caster_back_link">
    <visual><geometry><sphere radius="0.008"/></geometry><material name="metal"/></visual>
    <collision><geometry><sphere radius="0.006"/></geometry></collision>
    <inertial><mass value="0.005"/><inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/></inertial>
  </link>
  <joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="-0.032 0 0.068"/></joint>
  <link name="imu_link"/>
  <joint name="scan_joint" type="fixed"><parent link="base_link"/><child link="base_scan"/><origin xyz="-0.032 0 0.172"/></joint>
  <link name="base_scan">
    <visual><origin xyz="0 0 -0.005"/><geometry><cylinder radius="0.036" length="0.03"/></geometry><material name="sensor"/></visual>
    <collision><origin xyz="0 0 -0.005"/><geometry><cylinder radius="0.036" length="0.03"/></geometry></collision>
  </link>
  <joint name="camera_joint" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz="0.04 0 0.12"/></joint>
  <link name="camera_link">
    <visual><origin xyz="-0.008 0 0"/><geometry><box size="0.012 0.025 0.025"/></geometry><material name="sensor"/></visual>
  </link>
  <joint name="camera_rgb_optical_joint" type="fixed"><parent link="camera_link"/><child link="camera_rgb_optical_frame"/><origin rpy="-1.5708 0 -1.5708"/></joint>
  <link name="camera_rgb_optical_frame"/>
  <joint name="lidar_3d_joint" type="fixed"><parent link="base_link"/><child link="lidar_3d_link"/><origin xyz="-0.032 0 0.235"/></joint>
  <link name="lidar_3d_link">
    <visual><geometry><cylinder radius="0.04" length="0.035"/></geometry><material name="sensor"/></visual>
    <visual><origin xyz="0 0 -0.035"/><geometry><cylinder radius="0.006" length="0.04"/></geometry><material name="metal"/></visual>
  </link>
</robot>`;

/* ===== Ingebouwde werelden (zelfde formaat als een geladen .json-wereld) ===== */
const WORLDS = {"turtlebot3":{"name":"TurtleBot3-wereld","floor":[12,12],"spawn":[-2.0,-0.5,0],"items":[{"type":"wall","x0":2.6,"y0":0.0,"x1":1.3,"y1":2.252},{"type":"wall","x0":1.3,"y0":2.252,"x1":-1.3,"y1":2.252},{"type":"wall","x0":-1.3,"y0":2.252,"x1":-2.6,"y1":0.0},{"type":"wall","x0":-2.6,"y0":0.0,"x1":-1.3,"y1":-2.252},{"type":"wall","x0":-1.3,"y0":-2.252,"x1":1.3,"y1":-2.252},{"type":"wall","x0":1.3,"y0":-2.252,"x1":2.6,"y1":-0.0},{"type":"cyl","x":-1.1,"y":-1.1,"r":0.15,"h":0.25},{"type":"cyl","x":-1.1,"y":0.0,"r":0.15,"h":0.25},{"type":"cyl","x":-1.1,"y":1.1,"r":0.15,"h":0.25},{"type":"cyl","x":0.0,"y":-1.1,"r":0.15,"h":0.25},{"type":"cyl","x":0.0,"y":0.0,"r":0.15,"h":0.25},{"type":"cyl","x":0.0,"y":1.1,"r":0.15,"h":0.25},{"type":"cyl","x":1.1,"y":-1.1,"r":0.15,"h":0.25},{"type":"cyl","x":1.1,"y":0.0,"r":0.15,"h":0.25},{"type":"cyl","x":1.1,"y":1.1,"r":0.15,"h":0.25},{"type":"box","x":0.55,"y":1.65,"z":0.0326,"sx":0.7,"sy":0.5,"sz":0.02,"yaw":90,"pitch":-6},{"type":"movable","x":0.55,"y":-0.55},{"type":"movable","x":-0.55,"y":0.55,"yaw":34}],"fields":[{"kind":"gas","x":0.55,"y":-0.55,"strength":900,"sigma":0.5},{"kind":"heat","x":-1.1,"y":0.55,"strength":25,"sigma":0.4},{"kind":"light","x":0.0,"y":1.65,"strength":500,"sigma":0.8}]},"factory":{"name":"Fabriekshal","floor":[16,12],"spawn":[-6.8,0,0],"items":[{"type":"wall","x0":-8,"y0":-6,"x1":8,"y1":-6,"t":0.2,"h":1.0,"color":"#b8bfba"},{"type":"wall","x0":8,"y0":-6,"x1":8,"y1":6,"t":0.2,"h":1.0,"color":"#b8bfba"},{"type":"wall","x0":8,"y0":6,"x1":-8,"y1":6,"t":0.2,"h":1.0,"color":"#b8bfba"},{"type":"wall","x0":-8,"y0":6,"x1":-8,"y1":-6,"t":0.2,"h":1.0,"color":"#b8bfba"},{"type":"box","x":-5.3,"y":-1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":-5.3,"y":1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":0,"y":-1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":0,"y":1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":5.3,"y":-1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":5.3,"y":1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"rack","x":-3.55,"y":5.25,"bays":3,"depth":1.0,"h":2.0},{"type":"rack","x":-3.55,"y":2.85,"bays":3,"depth":1.0,"h":2.0},{"type":"pallet","x":-6.8,"y":-4.8,"load":0.9},{"type":"pallet","x":-5.4,"y":-4.8,"load":1.2},{"type":"pallet","x":-6.8,"y":-3.6,"load":0.6,"color":"#c4d3db"},{"type":"pallet","x":-5.4,"y":-3.6},{"type":"box","x":3.0,"y":4.6,"sx":1.8,"sy":1.4,"sz":1.9,"color":"#d5dad7"},{"type":"box","x":6.2,"y":4.6,"sx":1.8,"sy":1.4,"sz":1.9,"color":"#d5dad7"},{"type":"box","x":4.6,"y":5.6,"sx":0.6,"sy":0.4,"sz":1.8,"color":"#5c6b73"},{"type":"wall","x0":3.2,"y0":-4.8,"x1":7.2,"y1":-4.8,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"wall","x0":7.2,"y0":-4.8,"x1":7.2,"y1":-1.8,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"wall","x0":7.2,"y0":-1.8,"x1":3.2,"y1":-1.8,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"wall","x0":3.2,"y0":-1.8,"x1":3.2,"y1":-2.9,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"wall","x0":3.2,"y0":-3.7,"x1":3.2,"y1":-4.8,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"cyl","x":5.6,"y":-3.3,"r":0.3,"h":0.8,"color":"#f07c1e"},{"type":"box","x":4.4,"y":-4.0,"sx":1.0,"sy":0.8,"sz":0.75,"color":"#8e989c"},{"type":"conveyor","x":0.3,"y":-4.4,"l":5.0,"w":0.6,"h":0.8},{"type":"box","x":-1.5,"y":-2.5,"sx":1.6,"sy":0.7,"sz":0.9,"color":"#a07a52"},{"type":"box","x":-7.72,"y":0,"sx":0.3,"sy":0.5,"sz":0.3,"color":"#2b2f31"},{"type":"mark","x":0.2,"y":0.95,"sx":14.4,"sy":0.08,"color":"#f2c230"},{"type":"mark","x":0.2,"y":-0.95,"sx":14.4,"sy":0.08,"color":"#f2c230"},{"type":"mark","x":-7.3,"y":0,"sx":0.8,"sy":0.8,"color":"#3fae5a"},{"type":"mark","x":-6.1,"y":-2.75,"sx":2.9,"sy":0.06,"color":"#f2c230"},{"type":"mark","x":-6.1,"y":-5.6,"sx":2.9,"sy":0.06,"color":"#f2c230"},{"type":"mark","x":-7.55,"y":-4.18,"sx":0.06,"sy":2.9,"color":"#f2c230"},{"type":"mark","x":-4.65,"y":-4.18,"sx":0.06,"sy":2.9,"color":"#f2c230"},{"type":"movable","x":2.5,"y":0.3,"sx":0.4,"sy":0.3,"sz":0.3,"color":"#c79a5b"},{"type":"movable","x":-1.0,"y":-0.4,"sx":0.4,"sy":0.3,"sz":0.3,"color":"#c79a5b"}],"fields":[{"kind":"gas","x":2.6,"y":-3.8,"strength":1200,"sigma":1.4},{"kind":"heat","x":5.6,"y":-3.3,"strength":35,"sigma":0.9},{"kind":"light","x":-5.3,"y":0,"strength":600,"sigma":2.2},{"kind":"light","x":0,"y":0,"strength":600,"sigma":2.2},{"kind":"light","x":5.3,"y":0,"strength":600,"sigma":2.2}]},"empty":{"name":"Lege kamer","floor":[12,12],"spawn":[0,0,0],"items":[{"type":"wall","x0":-2.5,"y0":-2.5,"x1":2.5,"y1":-2.5},{"type":"wall","x0":2.5,"y0":-2.5,"x1":2.5,"y1":2.5},{"type":"wall","x0":2.5,"y0":2.5,"x1":-2.5,"y1":2.5},{"type":"wall","x0":-2.5,"y0":2.5,"x1":-2.5,"y1":-2.5}]}};
// SIM-CORE-END
