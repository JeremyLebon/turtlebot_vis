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
function rpyOf(q) {                                     // inverse van qrpy: [roll, pitch, yaw] zoals in <origin rpy>
  const sp = 2 * (q.w * q.y - q.z * q.x), p = Math.abs(sp) >= 1 ? Math.sign(sp) * Math.PI / 2 : Math.asin(sp);
  return [Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)), p, Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))];
}
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
    const lim = kid(j, "limit"), ax = kid(j, "axis"), dyn = kid(j, "dynamics"), dnum = k => dyn && dyn.getAttribute(k) != null ? +dyn.getAttribute(k) : undefined;
    // dynamics: spring_stiffness (N/m) en damping (N·s/m) zoals sdformat/Gazebo ze uit een URDF leest (vering)
    return { name: j.getAttribute("name"), type: j.getAttribute("type"), springK: dnum("spring_stiffness"), springC: dnum("damping"),
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

/* ===== Robot-editor: URDF met aangepaste/toegevoegde sensorframes =====
   Werkt op de oorspronkelijke tekst (na mini-xacro), zodat gazebo-tags, materialen en meshes blijven staan.
   edits: { jointnaam: {p, q} }, added: [{ link, joint, parent, origin, geom, optical }] */
const SENSOR_GEOM = {
  camera: { type: "box", size: [0.012, 0.025, 0.025] }, lidar: { type: "cylinder", radius: 0.036, length: 0.03 },
  lidar3d: { type: "cylinder", radius: 0.04, length: 0.035 }, imu: { type: "box", size: [0.02, 0.02, 0.005] },
};
const OPTICAL_Q = { x: -0.5, y: 0.5, z: -0.5, w: 0.5 };            // rpy -π/2 0 -π/2: z vooruit, x rechts, y omlaag
const fmtN = v => { const r = +(+v).toFixed(5); return String(Object.is(r, -0) ? 0 : r); };
const originAttrs = T => ({ xyz: [T.p.x, T.p.y, T.p.z].map(fmtN).join(" "), rpy: rpyOf(T.q).map(fmtN).join(" ") });
function patchURDF(text, edits, added, DOMParserImpl, XMLSerializerImpl) {
  const doc = new DOMParserImpl().parseFromString(preXacro(text), "text/xml");
  const robotEl = doc.getElementsByTagName("robot")[0];
  const kids = (el, tag) => Array.from(el.childNodes).filter(n => n.nodeType === 1 && n.nodeName === tag);
  const setOrigin = (el, T) => {
    let o = kids(el, "origin")[0];
    if (!o) { o = doc.createElement("origin"); el.insertBefore(o, el.firstChild); }
    const a = originAttrs(T); o.setAttribute("xyz", a.xyz); o.setAttribute("rpy", a.rpy);
  };
  for (const j of kids(robotEl, "joint")) { const T = edits[j.getAttribute("name")]; if (T) setOrigin(j, T); }
  const geomEl = g => {
    const e = doc.createElement("geometry"), c = doc.createElement(g.type);
    if (g.type === "box") c.setAttribute("size", g.size.map(fmtN).join(" "));
    else { c.setAttribute("radius", fmtN(g.radius)); if (g.length != null) c.setAttribute("length", fmtN(g.length)); }
    e.appendChild(c); return e;
  };
  const nl = () => doc.createTextNode("\n  ");
  for (const a of added || []) {
    robotEl.appendChild(doc.createComment(` sensor toegevoegd in de simulator `)); robotEl.appendChild(nl());
    const jt = doc.createElement("joint"); jt.setAttribute("name", a.joint); jt.setAttribute("type", "fixed");
    const pa = doc.createElement("parent"); pa.setAttribute("link", a.parent); const ch = doc.createElement("child"); ch.setAttribute("link", a.link);
    jt.appendChild(pa); jt.appendChild(ch); setOrigin(jt, edits[a.joint] || a.origin); robotEl.appendChild(jt); robotEl.appendChild(nl());
    const lk = doc.createElement("link"); lk.setAttribute("name", a.link);
    if (a.geom) {
      for (const tag of ["visual", "collision"]) { const v = doc.createElement(tag); v.appendChild(geomEl(a.geom)); lk.appendChild(v); }
    }
    robotEl.appendChild(lk); robotEl.appendChild(nl());
    if (a.optical) {
      const oj = doc.createElement("joint"); oj.setAttribute("name", a.optical + "_joint"); oj.setAttribute("type", "fixed");
      const op = doc.createElement("parent"); op.setAttribute("link", a.link); const oc = doc.createElement("child"); oc.setAttribute("link", a.optical);
      oj.appendChild(op); oj.appendChild(oc); setOrigin(oj, { p: V(), q: OPTICAL_Q }); robotEl.appendChild(oj); robotEl.appendChild(nl());
      const ol = doc.createElement("link"); ol.setAttribute("name", a.optical); robotEl.appendChild(ol); robotEl.appendChild(doc.createTextNode("\n"));
    }
  }
  let out = new XMLSerializerImpl().serializeToString(doc);
  if (!/^<\?xml/.test(out)) out = '<?xml version="1.0"?>\n' + out;
  return out;
}

/* ===== Robot-editor: model → URDF (volledig opnieuw geschreven, voor de aparte editorpagina) =====
   model zoals parseURDF het geeft. Inertia met inertial.auto wordt uit de eerste collision (of visual) berekend.
   extras: ruwe XML-stukken (gazebo, transmission, ros2_control) die de editor niet kent, ongewijzigd terug. */
function inertiaOf(g, m) {
  if (!g) return { xx: 1e-5, yy: 1e-5, zz: 1e-5 };
  if (g.type === "box") { const [x, y, z] = g.size; return { xx: m * (y * y + z * z) / 12, yy: m * (x * x + z * z) / 12, zz: m * (x * x + y * y) / 12 }; }
  if (g.type === "cylinder") { const r = g.radius, h = g.length, a = m * (3 * r * r + h * h) / 12; return { xx: a, yy: a, zz: m * r * r / 2 }; }
  if (g.type === "sphere") { const a = 0.4 * m * g.radius * g.radius; return { xx: a, yy: a, zz: a }; }
  return { xx: m * 1e-3, yy: m * 1e-3, zz: m * 1e-3 };
}
function autoInertial(link) {
  const ie = link.inertial; if (!ie || !ie.auto) return ie;
  const src = link.collisions[0] || link.visuals[0], I = inertiaOf(src && src.geom, ie.mass);
  ie.origin = src ? { p: { ...src.origin.p }, q: src.origin.q } : T0();
  ie.I = { xx: I.xx, yy: I.yy, zz: I.zz, xy: 0, xz: 0, yz: 0 };
  return ie;
}
function toURDF(model, extras = []) {
  const esc = v => String(v).replace(/&/g, "&amp;").replace(/"/g, "&quot;").replace(/</g, "&lt;");
  const org = (T, ind) => { if (!T) return ""; const a = originAttrs(T); return a.xyz === "0 0 0" && a.rpy === "0 0 0" ? "" : `${ind}<origin xyz="${a.xyz}" rpy="${a.rpy}"/>\n`; };
  const geo = (g, ind) => {
    let e;
    if (g.type === "box") e = `<box size="${g.size.map(fmtN).join(" ")}"/>`;
    else if (g.type === "cylinder") e = `<cylinder radius="${fmtN(g.radius)}" length="${fmtN(g.length)}"/>`;
    else if (g.type === "sphere") e = `<sphere radius="${fmtN(g.radius)}"/>`;
    else e = `<mesh filename="${esc(g.filename)}"${g.scale && g.scale.some(v => v !== 1) ? ` scale="${g.scale.map(fmtN).join(" ")}"` : ""}/>`;
    return `${ind}<geometry>${e}</geometry>\n`;
  };
  const hex = c => c.slice(0, 3).map(v => Math.round(Math.max(0, Math.min(1, v)) * 255).toString(16).padStart(2, "0")).join("");
  const out = [`<?xml version="1.0"?>`, `<!-- gemaakt met de robot-editor van TurtleBot Sim -->`, `<robot name="${esc(model.name || "robot")}">`];
  const order = [model.root], kidsOf = {};
  for (const j of model.joints) (kidsOf[j.parent] ||= []).push(j);
  for (let i = 0; i < order.length; i++) for (const j of kidsOf[order[i]] || []) order.push(j.child);
  for (const n of Object.keys(model.links)) if (!order.includes(n)) order.push(n);
  const jointTo = Object.fromEntries(model.joints.map(j => [j.child, j]));
  for (const ln of order) {
    const j = jointTo[ln];
    if (j) {
      out.push(`  <joint name="${esc(j.name)}" type="${j.type}">`, `    <parent link="${esc(j.parent)}"/>`, `    <child link="${esc(j.child)}"/>`);
      const o = org(j.origin, "    "); if (o) out.push(o.trimEnd());
      if (j.type !== "fixed") out.push(`    <axis xyz="${[j.axis.x, j.axis.y, j.axis.z].map(fmtN).join(" ")}"/>`);
      if (j.type === "revolute" || j.type === "prismatic") { const L = j.limit || { lower: -1.57, upper: 1.57, effort: 10, velocity: 1 };
        out.push(`    <limit lower="${fmtN(L.lower)}" upper="${fmtN(L.upper)}" effort="${fmtN(L.effort || 10)}" velocity="${fmtN(L.velocity || 1)}"/>`); }
      if (j.springK != null || j.springC != null)   // vering: zoals sdformat/Gazebo het uit een URDF leest
        out.push(`    <dynamics${j.springC != null ? ` damping="${fmtN(j.springC)}"` : ""}${j.springK != null ? ` spring_stiffness="${fmtN(j.springK)}"` : ""}/>`);
      out.push(`  </joint>`);
    }
    const l = model.links[ln], ie = autoInertial(l), body = [];
    if (ie && ie.mass > 0) {
      body.push(`    <inertial>`); const o = org(ie.origin, "      "); if (o) body.push(o.trimEnd());
      body.push(`      <mass value="${fmtN(ie.mass)}"/>`, `      <inertia ixx="${+ie.I.xx.toPrecision(4)}" ixy="${+(ie.I.xy || 0).toPrecision(4)}" ixz="${+(ie.I.xz || 0).toPrecision(4)}" iyy="${+ie.I.yy.toPrecision(4)}" iyz="${+(ie.I.yz || 0).toPrecision(4)}" izz="${+ie.I.zz.toPrecision(4)}"/>`, `    </inertial>`);
    }
    for (const v of l.visuals) {
      body.push(`    <visual>`); const o = org(v.origin, "      "); if (o) body.push(o.trimEnd()); body.push(geo(v.geom, "      ").trimEnd());
      if (v.color) body.push(`      <material name="c_${hex(v.color)}"><color rgba="${v.color.map(fmtN).join(" ")}"/></material>`);
      body.push(`    </visual>`);
    }
    for (const c of l.collisions) { body.push(`    <collision>`); const o = org(c.origin, "      "); if (o) body.push(o.trimEnd()); body.push(geo(c.geom, "      ").trimEnd(), `    </collision>`); }
    out.push(body.length ? `  <link name="${esc(ln)}">\n${body.join("\n")}\n  </link>` : `  <link name="${esc(ln)}"/>`);
  }
  for (const x of extras) out.push("  " + x.trim());
  out.push(`</robot>`, "");
  return out.join("\n");
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
  // stuurbare wielen (Ackermann): revolute om z met een continuous wiel eronder; die wielen worden niet aangedreven
  const steerJ = model.joints.filter(j => j.type === "revolute" && Math.abs(qrot(linkT[j.child].q, j.axis).z) > 0.9 && (byParent[j.child] || []).some(k => k.type === "continuous"));
  const steered = new Set(steerJ.flatMap(j => (byParent[j.child] || []).filter(k => k.type === "continuous").map(k => k.name)));
  const cont = model.joints.filter(j => j.type === "continuous" && !steered.has(j.name)).map(j => ({ j, ax: qrot(linkT[j.child].q, j.axis), p: linkT[j.child].p }))
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
  // aandrijftype: diff (2 wielen), skid (meer wielen per kant, ook rupsen), mecanum, ackermann
  const lefts = cont.filter(o => o.p.y > 0.01), rights = cont.filter(o => o.p.y < -0.01);
  cfg.leftWheels = lefts.map(o => o.j.name); cfg.rightWheels = rights.map(o => o.j.name);
  cfg.wheelXY = Object.fromEntries(cont.map(o => [o.j.name, [o.p.x, o.p.y]]));
  cfg.drive = cont.some(o => /mecanum/i.test(o.j.name)) ? "mecanum" : steerJ.length && lefts.length && rights.length ? "ackermann"
    : lefts.length > 1 || rights.length > 1 ? "skid" : "diff";
  if (cfg.drive === "ackermann") {
    const xr = cont.reduce((a, o) => a + o.p.x, 0) / cont.length, xf = steerJ.reduce((a, j) => a + linkT[j.child].p.x, 0) / steerJ.length;
    cfg.steer = { joints: steerJ.map(j => ({ name: j.name, y: linkT[j.child].p.y })), L: +(xf - xr).toFixed(4),
      max: Math.min(...steerJ.map(j => j.limit && j.limit.upper > 0 ? j.limit.upper : 0.6)) };
  }
  cfg.fixedBase = !(cfg.leftWheel && cfg.rightWheel) || model.root === "world";
  const movable = model.joints.filter(j => j.type === "revolute" || j.type === "prismatic" || j.type === "continuous");
  return { linkT, parentJoint, order, cfg, movable };
}

/* ===== Simulator ===== */
function createSim(R) {
  const DT = 1 / 200;
  const G_WORLD = (0x0001 << 16) | 0xffff, G_ROBOT = (0x0002 << 16) | 0x0001, G_RAY = (0xffff << 16) | 0x0001;
  const LIDAR = { n: 360, rmin: 0.12, rmax: 3.5, nRange: null };   // nRange [a, b]: elke scan a..b punten (zoals de LDS-02)
  const L3 = { rings: 16, vfov: 30, hres: 2, rmax: 15, rmin: 0.2 };
  // 3D-lidar-presets: draaiend (spin: rings × 360/hres kolommen) of rozet (Livox: n punten, elke scan een ander patroon).
  // De scan wordt over de omwenteling verdeeld (sweep3d per stap), dus met bewegingsvervorming en tijd per punt.
  const L3_PRESETS = {
    tb3: { label: "TB3-mast (16 lagen, ±15°, 5 Hz)", rings: 16, vmin: -15, vmax: 15, hres: 2, hz: 5, rmin: 0.2, rmax: 15 },
    vlp16: { label: "Velodyne VLP-16 (16 × 900, 10 Hz)", rings: 16, vmin: -15, vmax: 15, hres: 0.4, hz: 10, rmin: 0.3, rmax: 100 },
    os1_64: { label: "Ouster OS1-64 (64 × 512, 10 Hz, zwaar)", rings: 64, vmin: -22.5, vmax: 22.5, hres: 360 / 512, hz: 10, rmin: 0.3, rmax: 120 },
    mid360: { label: "Livox MID-360 (rozet, 20 000 punten, 10 Hz)", pattern: "rosette", n: 20000, rings: 4, vmin: -7, vmax: 52, hz: 10, rmin: 0.1, rmax: 40 },
  };
  const world = new R.World({ x: 0, y: 0, z: -9.81 });
  world.timestep = DT;
  // stijvere contacten: standaard (30 Hz) laat kleine robots zichtbaar in de vloer zakken
  world.integrationParameters.numSolverIterations = 8;
  world.integrationParameters.contact_natural_frequency = 120;
  const statics = [], movables = [];
  let colorMap = null;
  let curItem = "";                             // soort wereldelement dat nu gebouwd wordt (labels van de segmentatiecamera)
  let terrain = null;                           // hoogteveld (wereld met "terrain"), anders vlakke vloer op z = 0
  const groundZ = (x, y) => terrain ? terrain.fn(x, y) : 0;                          // body-handle → kleur (gekleurde 3D-lidar), opnieuw bij een andere wereld of doos
  const params = { odomErr: 0, lidarSigma: 0.01, imuNoise: true, ballWheels: true, wheelAcc: 2.0, odomSource: "truth" };
  let simTime = 0, robot = null;

  /* ---- wereld (data-gedreven, zie WORLDS onderaan en het JSON-formaat) ---- */
  const markings = [];
  let worldInfo = { name: "", floor: [12, 12], spawn: [0, 0, 0], items: [] };
  const rad = d => (d || 0) * Math.PI / 180;
  function addStatic(kind, desc, pos, rot, size, color, def) {
    const z = pos.z + (kind === "floor" ? 0 : groundZ(pos.x, pos.y));          // op het terrein (vlak: + 0)
    const body = world.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(pos.x, pos.y, z).setRotation(rot));
    world.createCollider(desc.setCollisionGroups(G_WORLD).setFriction(0.8), body);
    statics.push({ kind, body, size, color, def, item: curItem });
  }
  // box: x,y,[z = op de vloer],sx,sy,sz,[yaw,pitch,roll in graden],[color]
  function sBox(d) {
    const z = d.z ?? d.sz / 2, q = qrpy(rad(d.roll), rad(d.pitch), rad(d.yaw));
    addStatic("box", R.ColliderDesc.cuboid(d.sx / 2, d.sy / 2, d.sz / 2), V(d.x, d.y, z), q, V(d.sx, d.sy, d.sz), d.color, { type: "box", ...d });
  }
  function sCyl(d) {
    const h = d.h, body = world.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(d.x, d.y, (d.z ?? h / 2) + groundZ(d.x, d.y)));
    world.createCollider(R.ColliderDesc.cylinder(h / 2, d.r).setRotation(QX90).setCollisionGroups(G_WORLD).setFriction(0.8), body);
    statics.push({ kind: "cyl", body, size: { r: d.r, h }, color: d.color, def: { type: "cyl", ...d }, item: curItem });
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
  // tafel: blad op hoogte h, 4 poten; l langs x (yaw draait), de 2D-lidar ziet op 18 cm enkel de poten
  function sTable(d) {
    const L = d.l ?? 1.6, W = d.w ?? 0.8, H = d.h ?? 0.75, c = Math.cos(rad(d.yaw)), sn = Math.sin(rad(d.yaw));
    sBox({ x: d.x, y: d.y, z: H - 0.015, sx: L, sy: W, sz: 0.03, yaw: d.yaw, color: d.color ?? "#c9b18a" });
    for (const u of [-L / 2 + 0.05, L / 2 - 0.05]) for (const v of [-W / 2 + 0.05, W / 2 - 0.05])
      sBox({ x: d.x + c * u - sn * v, y: d.y + sn * u + c * v, sx: 0.04, sy: 0.04, sz: H - 0.03, yaw: d.yaw, color: "#4a5056" });
  }
  // stoel: zitting 42 x 42 cm op 45 cm, rugleuning aan de -x-kant (kijkt naar +x, yaw draait), 4 dunne poten
  function sChair(d) {
    const S = 0.42, Z = 0.45, c = Math.cos(rad(d.yaw)), sn = Math.sin(rad(d.yaw));
    const P = (u, v) => ({ x: d.x + c * u - sn * v, y: d.y + sn * u + c * v });
    for (const u of [-S / 2 + 0.03, S / 2 - 0.03]) for (const v of [-S / 2 + 0.03, S / 2 - 0.03]) {
      const p = P(u, v); sCyl({ x: p.x, y: p.y, r: 0.012, h: Z - 0.02, color: "#4a5056" });
    }
    sBox({ x: d.x, y: d.y, z: Z - 0.01, sx: S, sy: S, sz: 0.02, yaw: d.yaw, color: d.color ?? "#2f6f9f" });
    const b = P(-S / 2 + 0.01, 0); sBox({ x: b.x, y: b.y, z: Z + 0.2, sx: 0.02, sy: S, sz: 0.36, yaw: d.yaw, color: d.color ?? "#2f6f9f" });
  }
  function sBall(d) {                         // bol (kruin van een boom): botst, enkel de 3D-lidar en camera zien hem
    const body = world.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(d.x, d.y, (d.z ?? d.r) + groundZ(d.x, d.y)));
    world.createCollider(R.ColliderDesc.ball(d.r).setCollisionGroups(G_WORLD).setFriction(0.8), body);
    statics.push({ kind: "ball", body, size: { r: d.r }, color: d.color, def: { type: "sphere", ...d }, item: curItem });
  }
  // boom: stam (de 2D-lidar ziet enkel die) + kruin als bol; h = hoogte van de stam tot het begin van de kruin.
  // kind: loof | berk | den = het 3D-model in beeld (template, boom_<kind>), zelfde hoogte en kruinbreedte als de bol
  function sTree(d) {
    const r = d.r ?? 0.07, h = d.h ?? 0.9, c = d.crown ?? 0.8, kind = d.kind ?? "loof";
    sCyl({ x: d.x, y: d.y, r, h: h + c * 0.6, color: d.trunk ?? "#6b4a2f", tree: { kind, H: h + c * 1.85, w: 2 * c } });
    sBall({ x: d.x, y: d.y, z: h + c * 0.85, r: c, color: d.color ?? "#4f8a3a", crown: true });
  }
  // boomgaard: rows rijen van n bomen, rijafstand row, plantafstand gap, vanaf de hoek (x, y) langs yaw;
  // jitter (m) verschuift elke boom een beetje, vast per boom (zelfde wereld = zelfde bomen)
  function sOrchard(d) {
    const rows = d.rows ?? 4, n = d.n ?? 8, row = d.row ?? 3.5, gap = d.gap ?? 1.5, j = d.jitter ?? 0.08;
    const c = Math.cos(rad(d.yaw)), sn = Math.sin(rad(d.yaw));
    let seed = Math.round((d.x * 73.1 + d.y * 37.7) * 1000) >>> 0;
    const rnd = () => { seed = (seed + 0x6d2b79f5) >>> 0; let t = seed; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
    for (let i = 0; i < rows; i++) for (let k = 0; k < n; k++) {
      const u = k * gap + (rnd() - 0.5) * 2 * j, v = i * row + (rnd() - 0.5) * 2 * j, s = 0.85 + 0.3 * rnd();
      if (d.missing && rnd() < d.missing) continue;              // ontbrekende bomen (gaten in de rij)
      sTree({ x: d.x + c * u - sn * v, y: d.y + sn * u + c * v, h: (d.h ?? 0.9) * s, crown: (d.crown ?? 0.8) * s, r: d.r, color: d.color, trunk: d.trunk, kind: d.kind });
    }
  }
  // hek (bouwhek, gaas): palen om de gap m en een doorzichtig paneel; botst als een muur, de lidar ziet het als een muur
  function sFence(d) {
    const h = d.h ?? 1.2, gap = d.gap ?? 2.5, L = Math.hypot(d.x1 - d.x0, d.y1 - d.y0), n = Math.max(1, Math.round(L / gap));
    for (let i = 0; i <= n; i++) sCyl({ x: d.x0 + (d.x1 - d.x0) * i / n, y: d.y0 + (d.y1 - d.y0) * i / n, r: 0.03, h, color: d.post ?? "#8b9196" });
    const t = 0.02;
    addStatic("box", R.ColliderDesc.cuboid(L / 2, t / 2, (h - 0.05) / 2), V((d.x0 + d.x1) / 2, (d.y0 + d.y1) / 2, 0.05 + (h - 0.05) / 2),
      qyaw(Math.atan2(d.y1 - d.y0, d.x1 - d.x0)), V(L, t, h - 0.05), d.color ?? "#9fb1bd", { type: "fence", ...d, opacity: d.opacity ?? 0.35 });
  }
  // gewasrijen (maïs, kool, ...): rows rijen van l m, rijafstand row, vanaf (x, y) langs yaw. Botsen als een smalle doos per
  // stuk van 2 m (w breed, h hoog, volgt het terrein); in beeld planten om de gap m (plant: mais | kool | aardappel)
  function sCrops(d) {
    const rows = d.rows ?? 10, L = d.l ?? 20, row = d.row ?? 0.75, w = d.w ?? 0.3, h = d.h ?? 0.6, seg = 2;
    const c = Math.cos(rad(d.yaw)), sn = Math.sin(rad(d.yaw)), ns = Math.max(1, Math.round(L / seg)), sl = L / ns;
    for (let i = 0; i < rows; i++) for (let k = 0; k < ns; k++) {
      const u = (k + 0.5) * sl, v = i * row, x = d.x + c * u - sn * v, y = d.y + sn * u + c * v;
      addStatic("crop", R.ColliderDesc.cuboid(sl / 2, w / 2, h / 2), V(x, y, h / 2), qyaw(rad(d.yaw)), V(sl, w, h), d.color ?? "#5e9a3c",
        { type: "crops", plant: d.plant ?? "mais", gap: d.gap ?? 0.25, seed: i * 1000 + k });
    }
  }
  // terrein: base + tilt[0]·x + tilt[1]·y + heuvels (gauss: x y h s|sx,sy) + golving (amp, len); res = rastergrootte (m)
  function terrainFn(t) {
    const hills = t.hills || [], tilt = t.tilt || [0, 0], wv = t.wave;
    return (x, y) => {
      let z = (t.base || 0) + tilt[0] * x + tilt[1] * y;
      for (const h of hills) { const sx = h.sx ?? h.s ?? 2, sy = h.sy ?? h.s ?? 2; z += h.h * Math.exp(-((x - h.x) ** 2 / (2 * sx * sx) + (y - h.y) ** 2 / (2 * sy * sy))); }
      if (wv) z += wv.amp * Math.sin(x * 2 * Math.PI / wv.len + 1.3) * Math.cos(y * 2 * Math.PI / (wv.len * 1.37) + 0.4);
      return z;
    };
  }
  // ArUco-marker (DICT_4X4_50, id 0-49) op een dun bord: x y [z = hoogte van het midden] yaw (normaal), size = zwarte rand
  // inbegrepen; het bord heeft een witte marge van 25 %. Beeld: template (canvastextuur op de voorkant, langs +x na yaw).
  function sAruco(d) {
    const sz = d.size ?? 0.2, m = sz * 1.25, z = d.z ?? m / 2 + 0.02;
    addStatic("aruco", R.ColliderDesc.cuboid(0.005, m / 2, m / 2), V(d.x, d.y, z), qyaw(rad(d.yaw)), V(0.01, m, m), "#ffffff", { type: "aruco", ...d });
    if (z - m / 2 > 0.05 && !d.wall) sCyl({ x: d.x - 0.03 * Math.cos(rad(d.yaw)), y: d.y - 0.03 * Math.sin(rad(d.yaw)), r: 0.015, h: z - m / 2, color: "#8b9196" });
  }
  // UWB-anker: kastje op hoogte z (standaard 2 m) op een paal; de posities staan ook in worldInfo.uwb
  function sUwb(d) {
    const z = d.z ?? 2.0;
    sBox({ x: d.x, y: d.y, z, sx: 0.09, sy: 0.09, sz: 0.06, color: "#e2622d" });
    if (!d.wall) sCyl({ x: d.x, y: d.y, r: 0.02, h: z - 0.03, color: "#8b9196" });
  }
  // lijn op de vloer (lijnvolgen): pts [[x, y], ...], w (breedte, standaard 2,5 cm), closed; stukken + ronde-ish hoeken
  function sLine(d) {
    const w = d.w ?? 0.025, pts = d.pts.slice(); if (d.closed) pts.push(pts[0]);
    for (let i = 0; i + 1 < pts.length; i++) {
      const [x0, y0] = pts[i], [x1, y1] = pts[i + 1], L = Math.hypot(x1 - x0, y1 - y0);
      markings.push({ x: (x0 + x1) / 2, y: (y0 + y1) / 2, sx: L + w, sy: w, yaw: Math.atan2(y1 - y0, x1 - x0) * 180 / Math.PI, color: d.color ?? "#16181a" });
    }
  }
  // 3D-model (models/, MODEL_INFO): name, x, y, [z = hoogte van de onderkant, bv. op een bureau], yaw, scale. Botst als
  // doos of cilinder met de maten van het model; beeld: template (GLB). Labels en temperatuur voor de camera's.
  function sModel(d) {
    const I = MODEL_INFO[d.name]; if (!I) throw new Error("Onbekend model: " + d.name);
    const k = d.scale ?? 1, [sx, sy, sz] = I.size.map(v => v * k), z = (d.z ?? 0) + sz / 2;
    const desc = I.collider === "cyl" ? R.ColliderDesc.cylinder(sz / 2, Math.max(sx, sy) * 0.4) : R.ColliderDesc.cuboid(sx / 2, sy / 2, sz / 2);
    const rot = I.collider === "cyl" ? qmul(qyaw(rad(d.yaw)), QX90) : qyaw(rad(d.yaw));
    addStatic("model", desc, V(d.x, d.y, z), rot, V(sx, sy, sz), null, { type: "model", ...d });
  }
  function clearWorld(floor = [12, 12], terr = null) {
    for (const st of statics) world.removeRigidBody(st.body);
    for (const m of movables) world.removeRigidBody(m.body);
    statics.length = 0; movables.length = 0; markings.length = 0; colorMap = null; terrain = null;
    if (!terr) { addStatic("floor", R.ColliderDesc.cuboid(floor[0] / 2 + 2, floor[1] / 2 + 2, 0.05), V(0, 0, -0.05), Q0(), V(floor[0], floor[1], 0.1)); return; }
    // Rapier-hoogteveld ligt in het xz-vlak (y omhoog): +90° om x → hoogte langs z, rij i loopt naar -y, kolom j naar +x
    const res = terr.res ?? 0.4, sx = floor[0] + 4, sy = floor[1] + 4, nc = Math.ceil(sx / res), nr = Math.ceil(sy / res), fn = terrainFn(terr);
    const H = new Float32Array((nr + 1) * (nc + 1));
    for (let j = 0; j <= nc; j++) for (let i = 0; i <= nr; i++) H[i + j * (nr + 1)] = fn(-sx / 2 + j * sx / nc, sy / 2 - i * sy / nr);
    terrain = { fn, res, sx, sy };
    addStatic("floor", R.ColliderDesc.heightfield(nr, nc, H, V(sx, 1, sy)), V(0, 0, 0), { x: Math.SQRT1_2, y: 0, z: 0, w: Math.SQRT1_2 }, V(floor[0], floor[1], 0.1));
  }
  function loadWorld(def) {
    worldInfo = { name: def.name || "wereld", floor: def.floor || [12, 12], spawn: def.spawn || [0, 0, 0],
      items: (def.items || []).filter(it => it.type !== "movable"), fields: (def.fields || []).map(f => ({ ...f })),
      ground: def.ground || "", sky: def.sky || "", geo: def.geo || null, terrain: def.terrain || null };
    clearWorld(worldInfo.floor, worldInfo.terrain);
    const make = { box: sBox, cyl: sCyl, wall: sWall, rack: sRack, conveyor: sConveyor, pallet: sPallet, table: sTable, chair: sChair,
      sphere: sBall, tree: sTree, orchard: sOrchard, fence: sFence, crops: sCrops, aruco: sAruco, uwb: sUwb, line: sLine, model: sModel,
      movable: d => addBox(d.x, d.y, rad(d.yaw), d.sx ?? 0.25, d.sz ?? 0.35, d.mass ?? 0.6, d.sy, d.color),
      mark: d => markings.push({ ...d }) };
    for (const it of def.items || []) {
      const f = make[it.type]; if (!f) throw new Error("Onbekend wereldelement: " + it.type);
      curItem = it.type; f(it); curItem = "";
    }
    worldInfo.uwb = (def.items || []).filter(it => it.type === "uwb").map((it, i) => ({ id: it.id ?? i, x: it.x, y: it.y, z: (it.z ?? 2.0) + groundZ(it.x, it.y) }));
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
    return { name: worldInfo.name, floor: worldInfo.floor, spawn: worldInfo.spawn, ...(worldInfo.ground ? { ground: worldInfo.ground } : {}),
      ...(worldInfo.sky ? { sky: worldInfo.sky } : {}), ...(worldInfo.geo ? { geo: worldInfo.geo } : {}), ...(worldInfo.terrain ? { terrain: worldInfo.terrain } : {}), items, ...(worldInfo.fields.length ? { fields: worldInfo.fields } : {}) };
  }
  function addBox(x, y, yaw = 0, s = 0.25, h = 0.35, mass = 0.6, sy, color) {
    const w = sy ?? s;
    const body = world.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(x, y, h / 2 + 0.001 + groundZ(x, y)).setRotation(qyaw(yaw)).setLinearDamping(0.2).setAngularDamping(0.5));
    world.createCollider(R.ColliderDesc.cuboid(s / 2, w / 2, h / 2).setMass(mass).setFriction(0.5).setCollisionGroups(G_WORLD), body);
    movables.push({ kind: "box", body, size: V(s, w, h), mass, color }); colorMap = null;
  }
  // testpaal ("stoelpoot") voor de taaktests: kinematisch, verplaatsbaar, gezien door de lidar
  const probes = [];
  function addProbe(x, y, r = 0.04, h = 0.6) {
    const body = world.createRigidBody(R.RigidBodyDesc.kinematicPositionBased().setTranslation(x, y, h / 2 + groundZ(x, y)));
    world.createCollider(R.ColliderDesc.cylinder(h / 2, r).setRotation(QX90).setCollisionGroups(G_WORLD), body);
    const pr = { body, r, h }; probes.push(pr); return pr;
  }
  function moveProbe(pr, x, y) { pr.body.setTranslation({ x, y, z: pr.h / 2 + groundZ(x, y) }, true); }
  function removeBox(m) { const i = movables.indexOf(m); if (i >= 0) movables.splice(i, 1); world.removeRigidBody(m.body); colorMap = null; }
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
    const wheelJ = new Set([cfg.leftWheel, cfg.rightWheel, ...(cfg.leftWheels || []), ...(cfg.rightWheels || [])].filter(Boolean));
    const steerJ = new Set(cfg.drive === "ackermann" && cfg.steer ? cfg.steer.joints.map(s => s.name) : []);
    const mecanum = cfg.drive === "mecanum";
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
    const S = { p: V(x, y, z0 + groundZ(x, y)), q: qyaw(yaw) };
    // 3. Rapier-lichamen, colliders en massa
    const colls = [];                                 // per collider de link, voor setFixedJoint
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
            .setFriction(caster || (isWheel && mecanum) ? 0 : isWheel ? 1.0 : 0.4).setCollisionGroups(G_ROBOT);
          if (caster || (isWheel && mecanum)) cd.setFrictionCombineRule(R.CoefficientCombineRule.Min);   // mecanum: het lichaam wordt aangestuurd
          colls.push({ ln, c, isWheel, coll: world.createCollider(cd, rb) });
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
      // vering: prismatic met "suspension"/"shock"/"spring" in de naam = veer + demper (positiemotor naar 0, ForceBased)
      // rocker-bogie: revolute met "rocker" = differentieel (links + rechts samen 0, de romp houdt het gemiddelde),
      // "bogie" = vrij scharnier (geen motor, enkel limieten)
      const role = wheelJ.has(j.name) ? "wheel" : steerJ.has(j.name) ? "steer" : j.type === "prismatic" && /suspension|shock|spring/i.test(j.name) ? "spring"
        : j.type === "revolute" && /rocker/i.test(j.name) ? "rocker" : j.type === "revolute" && /bogie/i.test(j.name) ? "passive"
        : (j.type === "continuous" ? "passive" : "position");
      // stuurgewrichten zijn licht: met AccelerationBased haalt de sturing haar hoek niet tegen de banden in
      if (role !== "passive") ij.configureMotorModel(role === "steer" || role === "spring" || role === "rocker" ? R.MotorModel.ForceBased : R.MotorModel.AccelerationBased);
      joints.push({ j, ij, b1, b2, a1, a2, axis, role, lim, pos: 0, vel: 0, target: lim ? Math.min(Math.max(0, lim.lower), lim.upper) : 0,
        sign: role === "wheel" ? Math.sign(axis.y) : 1 });
    }
    // veren: zakken 3 cm in onder het eigen gewicht, demping 60 % van kritisch (k en c per veer, in N/m en N·s/m)
    const springs = joints.filter(o => o.role === "spring");
    if (springs.length) {
      const mTot = bodies.reduce((a, B) => a + (B.mass || 0), 0), mPer = mTot / springs.length, k = mPer * 9.81 / 0.03, c = 2 * 0.6 * Math.sqrt(k * mPer);
      for (const o of springs) { o.k = o.j.springK ?? k; o.c = o.j.springC ?? c; o.ij.configureMotorPosition(0, o.k, o.c); }
    }
    const rockers = joints.filter(o => o.role === "rocker");
    robot = { rockers, model, cfg, an, bodies, bodyOf, linkOff, joints, colls, rootBody: bodies[0], leftSet: new Set([cfg.leftWheel, ...(cfg.leftWheels || [])]), steerAngle: 0,
      byName: Object.fromEntries(joints.map(o => [o.j.name, o])), prevV: V(), imuAcc: V(0, 0, 9.81) };
    // 0,5 s laten uitzakken met stilstaande motoren, zodat odometrie niet start met een sprongetje
    for (let i = 0; i < 100; i++) {
      for (const o of joints) { if (o.role === "wheel") o.ij.configureMotorVelocity(0, 500); else if (o.role === "position") o.ij.configureMotorPosition(o.target, 20000, 300); else if (o.role === "steer") o.ij.configureMotorPosition(0, STEER_K, STEER_D); }
      rockerDiff(); world.step();
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

  /* ---- robot-editor: vaste joint (sensorframe) live verplaatsen, zonder herbouw ----
     Past joint.origin aan en verschuift de links eronder binnen hun star lichaam (linkOff + colliders).
     Massa en grondhoogte blijven: sensoren zijn licht. Geeft de verschoven links terug, of null als
     er een bewegende joint onder hangt (dan herbouwen). */
  function setFixedJoint(name, origin) {
    const m = robot.model, j = m.joints.find(o => o.name === name);
    if (!j || j.type !== "fixed") return null;
    const sub = [j.child];
    for (let i = 0; i < sub.length; i++) for (const k of m.joints) if (k.parent === sub[i]) { if (k.type !== "fixed") return null; sub.push(k.child); }
    j.origin = { p: V(origin.p.x, origin.p.y, origin.p.z), q: origin.q };
    const an = analyzeModel(m), B = robot.bodies[robot.bodyOf[j.child]];
    robot.an = an;
    for (const ln of sub) robot.linkOff[ln] = { p: vsub(an.linkT[ln].p, B.p), q: an.linkT[ln].q };
    const inSub = new Set(sub);
    for (const o of robot.colls) if (inSub.has(o.ln)) {
      const Tc = tmul(robot.linkOff[o.ln], o.c.origin);
      o.coll.setTranslationWrtParent(Tc.p); o.coll.setRotationWrtParent(qmul(Tc.q, shapeRot(o.c.geom, o.isWheel)));
    }
    return sub;
  }

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

  // differentieel van een rocker-bogie: elke rocker naar (eigen hoek - gemiddelde) → som 0, romp op de gemiddelde helling
  const ROCKER_K = 400, ROCKER_D = 15;                 // N·m/rad, N·m·s/rad
  function rockerDiff() {
    if (!robot || robot.rockers.length !== 2) return;
    const [a, b] = robot.rockers; measureJoint(a); measureJoint(b);
    const m = (a.pos + b.pos) / 2;
    a.ij.configureMotorPosition(a.pos - m, ROCKER_K, ROCKER_D); b.ij.configureMotorPosition(b.pos - m, ROCKER_K, ROCKER_D);
  }
  // commando met versnellingsbegrenzing (mecanum-lichaam), toestand in robot[key]
  const STEER_K = 200, STEER_D = 3;                    // stuurmotor (ForceBased): N·m/rad, N·m·s/rad
  function cmdRamp(r, key, want, dt, acc = params.wheelAcc) { r[key] = (r[key] || 0) + Math.max(-acc * dt, Math.min(acc * dt, want - (r[key] || 0))); return r[key]; }
  function step(cmd) {
    const c = robot.cfg;
    const v = Math.max(-c.vmax, Math.min(c.vmax, cmd.v)), w = Math.max(-c.wmax, Math.min(c.wmax, cmd.w));
    const vy = c.drive === "mecanum" ? Math.max(-c.vmax, Math.min(c.vmax, cmd.vy || 0)) : 0;
    cmdTarget = { v, w, vy };
    const wmaxWheel = c.vmax / c.wheelRadius * (c.drive === "mecanum" ? 2.5 : 1), clampW = s => Math.max(-wmaxWheel, Math.min(wmaxWheel, s));
    // Ackermann: stuurhoek uit v en w (w = v·tan δ / L), traag bijgestuurd; bochtstraal R voor binnen/buiten
    let Rturn = Infinity;
    if (c.drive === "ackermann" && c.steer) {
      const want = Math.abs(v) > 0.02 ? Math.max(-c.steer.max, Math.min(c.steer.max, Math.atan(c.steer.L * w / v))) : robot.steerAngle;
      robot.steerAngle += Math.max(-3 * DT, Math.min(3 * DT, want - robot.steerAngle));
      if (Math.abs(robot.steerAngle) > 1e-3) Rturn = c.steer.L / Math.tan(robot.steerAngle);
    }
    for (const o of robot.joints) {
      if (o.role === "steer") {
        const y = c.steer.joints.find(s => s.name === o.j.name).y;
        o.target = isFinite(Rturn) ? Math.atan(c.steer.L / (Rturn - y)) : 0;    // binnenwiel stuurt meer dan buitenwiel
        o.ij.configureMotorPosition(o.target, STEER_K, STEER_D);
      } else if (o.role === "wheel") {
        const [wx, wy] = c.wheelXY?.[o.j.name] || [0, robot.leftSet.has(o.j.name) ? 1 : -1], k = Math.abs(wx) + Math.abs(wy);
        // mecanum (rollen 45°): ω·r = vx ∓ vy ∓ (lx + ly)·ω; enkel voor het beeld en de encoders, het lichaam wordt apart gestuurd
        const lin = c.drive === "mecanum" ? v - Math.sign(wx) * Math.sign(wy) * vy - Math.sign(wy) * k * w
          : c.drive === "ackermann" ? (isFinite(Rturn) ? v * (Rturn - wy) / Rturn : v)
          : robot.leftSet.has(o.j.name) ? v - w * c.wheelSep / 2 : v + w * c.wheelSep / 2;
        // versnellingsbegrenzing per wiel (zoals een motorprofiel); sprongen veroorzaken wielslip
        const want = clampW(lin / c.wheelRadius), dmax = (params.wheelAcc / c.wheelRadius) * DT;
        o.cmdW = (o.cmdW || 0) + Math.max(-dmax, Math.min(dmax, want - (o.cmdW || 0)));
        // integrerende term (zoals de snelheidsregeling van een Dynamixel): bergop houdt de P-motor alleen de snelheid
        // niet (15 % helling: -30 %). Enkel bij een constant commando (na de rampe), bij nul meteen leeg.
        if (o.cmdW === 0) o.wInt = 0;
        else if (Math.abs(want - o.cmdW) < 1e-9) o.wInt = Math.max(-3, Math.min(3, (o.wInt || 0) + (o.cmdW - o.sign * o.vel) * DT));
        o.ij.configureMotorVelocity(o.sign * (o.cmdW + 4 * (o.wInt || 0)), 500);
      } else if (o.role === "position") {
        // PD-motor van Rapier + integrerende term tegen doorzakken onder zwaartekracht
        o.integ = Math.max(-0.5, Math.min(0.5, (o.integ || 0) + (o.target - o.pos) * DT));
        o.ij.configureMotorPosition(o.target + 8 * o.integ, 20000, 300);
      }
    }
    rockerDiff();
    if (c.drive === "mecanum" && !c.fixedBase) {
      const rb = robot.rootBody.rb, lv = rb.linvel(), av = rb.angvel(), th = yawOf(bodyT(robot.rootBody).q);
      const tx = cmdRamp(robot, "mx", v, DT), ty = cmdRamp(robot, "my", vy, DT), tw = cmdRamp(robot, "mw", w, DT, 4);
      rb.setLinvel({ x: Math.cos(th) * tx - Math.sin(th) * ty, y: Math.sin(th) * tx + Math.cos(th) * ty, z: lv.z }, true);
      rb.setAngvel({ x: av.x, y: av.y, z: tw }, true);
    }
    // rem bij stilstand: nulcommando en wielen uitgelopen → geen horizontale drift (anders kruipt de robot door
    // kleine trillingen van de bolcontacten, ongeveer 2° per 40 s)
    const wheels = robot.joints.filter(o => o.role === "wheel");
    let braking = false;
    if (wheels.length && !c.fixedBase && v === 0 && w === 0 && vy === 0 && wheels.every(o => Math.abs(o.cmdW || 0) < 1e-9)) {
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
      odom.v = Math.cos(th) * lv.x + Math.sin(th) * lv.y; odom.vy = -Math.sin(th) * lv.x + Math.cos(th) * lv.y; odom.w = av.z;
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
    if (LIDAR.nRange) LIDAR.n = LIDAR.nRange[0] + Math.floor(Math.random() * (LIDAR.nRange[1] - LIDAR.nRange[0] + 1));
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
  // kleur van het object dat een straal raakt (gekleurde 3D-lidar); null = standaardkleur (muren, vloer)
  function bodyColor(handle) {
    if (!colorMap) { colorMap = new Map(); for (const st of statics) colorMap.set(st.body.handle, st.kind === "floor" ? "floor" : st.color || null); for (const m of movables) colorMap.set(m.body.handle, m.color || "box"); }
    return colorMap.get(handle) ?? null;
  }
  function scan3d(link) {
    const T = linkWorld(link), nh = Math.round(360 / L3.hres), sensor = [], worldPts = [], cols = [];
    for (let i = 0; i < L3.rings; i++) {
      const el = (-L3.vfov / 2 + i * L3.vfov / (L3.rings - 1)) * Math.PI / 180, ce = Math.cos(el), se = Math.sin(el);
      for (let k = 0; k < nh; k++) {
        const az = k * L3.hres * Math.PI / 180, db = V(ce * Math.cos(az), ce * Math.sin(az), se), d = qrot(T.q, db);
        const h = world.castRay(new R.Ray(T.p, d), L3.rmax, true, undefined, G_RAY); if (!h) continue;
        let r = h.timeOfImpact ?? h.toi; if (!r) continue;
        r += params.lidarSigma * gauss(); if (r < L3.rmin) continue;
        sensor.push(db.x * r, db.y * r, db.z * r); worldPts.push(T.p.x + d.x * r, T.p.y + d.y * r, T.p.z + d.z * r);
        cols.push(bodyColor(h.collider.parent()?.handle));
      }
    }
    return { sensor, worldPts, cols };
  }
  // 3D-lidar per simulatiestap: het deel van de omwenteling dat in deze stap valt, vanaf de pose van nu (zoals een
  // echte draaiende lidar). Geeft de volledige scan terug als de omwenteling rond is, anders null.
  // Per punt: x y z in het sensorframe op het moment van de meting, ring, tijd t.o.v. het begin van de scan, kleur.
  function sweep3d(link, P, st) {
    const period = 1 / P.hz, rad1 = Math.PI / 180;
    if (!st.pts) Object.assign(st, { t: 0, done: 0, pts: [], cols: [], scan: (st.scan || 0) + 1 });
    st.t += DT;
    const T = linkWorld(link), total = P.pattern === "rosette" ? P.n : Math.round(360 / P.hres);
    const due = Math.min(total, Math.floor(total * st.t / period));
    const shoot = (az, el, ring, k) => {
      const ce = Math.cos(el), db = V(ce * Math.cos(az), ce * Math.sin(az), Math.sin(el)), d = qrot(T.q, db);
      const h = world.castRay(new R.Ray(T.p, d), P.rmax, true, undefined, G_RAY); if (!h) return;
      let r = (h.timeOfImpact ?? h.toi) + params.lidarSigma * gauss(); if (r < P.rmin) return;
      st.pts.push(db.x * r, db.y * r, db.z * r, ring, k / total * period); st.cols.push(bodyColor(h.collider.parent()?.handle));
    };
    for (let k = st.done; k < due; k++) {
      if (P.pattern === "rosette") {                      // niet-herhalend: gulden hoek + per scan verschoven elevatie
        const f = (k * 0.6180339887 + st.scan * 0.137) % 1, az = (k * 137.50776 + st.scan * 3.7) % 360;
        const el = P.vmin + (P.vmax - P.vmin) * (0.5 - 0.5 * Math.cos(Math.PI * f));
        shoot(az * rad1, el * rad1, Math.min(P.rings - 1, Math.floor(f * P.rings)), k);
      } else {
        const az = k * P.hres * rad1;
        for (let i = 0; i < P.rings; i++) shoot(az, (P.vmin + i * (P.vmax - P.vmin) / (P.rings - 1)) * rad1, i, k);
      }
    }
    st.done = due;
    if (st.t + 1e-9 < period) return null;
    const out = { pts: st.pts, cols: st.cols, period }; st.pts = null; return out;
  }
  // zichtlijn tussen twee punten (UWB): afstand tot het eerste obstakel, Infinity = vrij
  function lineOfSight(a, b) {
    const d = vsub(b, a), L = vlen(d); if (L < 1e-6) return Infinity;
    const h = world.castRay(new R.Ray(a, vscale(d, 1 / L)), L - 0.12, true, undefined, G_RAY);
    return h ? (h.timeOfImpact ?? h.toi) : Infinity;
  }
  // afstand recht naar beneden (cliff-sensor), met de vloer; Infinity = niets binnen max
  function downRange(p, max) {
    const h = world.castRay(new R.Ray(p, V(0, 0, -1)), max, true, undefined, G_RAY);
    return h ? (h.timeOfImpact ?? h.toi) : Infinity;
  }
  // radar (zoals een 77 GHz-autoradar): waaier van stralen (res graden) over fov, enkele elevaties, zonder de vloer.
  // Opeenvolgende treffers op hetzelfde object en op ongeveer dezelfde afstand worden één detectie (max maxW stralen breed).
  // Per detectie: positie in het sensorframe, radiale snelheid (Doppler, + = weg van de radar) en een ruwe RCS (dBsm).
  function radarScan(T, o, vSensor) {
    const floor = statics.find(s => s.kind === "floor")?.body, rad1 = Math.PI / 180, n = Math.round(o.fov / o.res), hits = [];
    for (let k = 0; k <= n; k++) {
      const az = (-o.fov / 2 + k * o.res) * rad1; let best = null;
      for (const e of o.els) {
        const el = e * rad1, d = qrot(T.q, V(Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)));
        const h = world.castRay(new R.Ray(T.p, d), o.rmax, true, undefined, G_RAY, undefined, floor);
        const r = h ? (h.timeOfImpact ?? h.toi) : 0;
        if (h && r > 0.2 && (!best || r < best.r)) best = { r, az, handle: h.collider.parent()?.handle };
      }
      hits.push(best);
    }
    const groups = []; let cur = null;
    for (const h of hits) {
      if (h && cur && h.handle === cur.handle && Math.abs(h.r - cur.last) < 0.5 && cur.list.length < o.maxW) { cur.list.push(h); cur.last = h.r; continue; }
      if (cur) groups.push(cur);
      cur = h ? { handle: h.handle, list: [h], last: h.r } : null;
    }
    if (cur) groups.push(cur);
    return groups.map(g => {
      const az = g.list.reduce((a, h) => a + h.az, 0) / g.list.length + o.azSigma * rad1 * gauss();
      const r = Math.min(...g.list.map(h => h.r)) + o.rSigma * gauss();
      const ds = V(Math.cos(az), Math.sin(az), 0), dw = qrot(T.q, ds), b = world.getRigidBody(g.handle), lv = b ? b.linvel() : V();
      const v = (lv.x - vSensor.x) * dw.x + (lv.y - vSensor.y) * dw.y + (lv.z - vSensor.z) * dw.z + o.vSigma * gauss();
      const rcs = 10 * Math.log10(Math.max(0.05, g.list.length * o.res * rad1 * r * 1.5));   // breder en verder = groter vlak
      return { x: ds.x * r, y: ds.y * r, z: 0, r, az, v, rcs };
    });
  }
  // sonar/IR: kortste afstand binnen een kegel (fov in rad) langs de x-as van T, zonder de vloer (zoals een
  // gemonteerde sonar die iets omhoog kijkt); Infinity = geen echo, 0 = tegen het object
  function rangeCone(T, fov, rmax) {
    const floor = statics.find(s => s.kind === "floor")?.body;
    let best = Infinity;
    for (const el of [-0.5, 0, 0.5]) for (let i = 0; i <= 10; i++) {         // 3 × 11 stralen: een stam van 14 cm op 2 m valt er niet tussen
      const a = (i / 10 - 0.5) * fov, e = el * fov, d = qrot(T.q, V(Math.cos(e) * Math.cos(a), Math.cos(e) * Math.sin(a), Math.sin(e)));
      const h = world.castRay(new R.Ray(T.p, d), rmax, true, undefined, G_RAY, undefined, floor);
      if (h) best = Math.min(best, h.timeOfImpact ?? h.toi);
    }
    return best;
  }
  // bumper: raakt het hoofdlichaam van de robot iets anders dan de vloer? 1 = voor, 2 = achter (zoals /sensor_state)
  function bumper() {
    if (!robot) return 0;
    const rb = robot.rootBody.rb, floor = statics.find(s => s.kind === "floor")?.body, qi = qconj(bodyT(robot.rootBody).q);
    let bits = 0;
    for (let i = 0; i < rb.numColliders(); i++) {
      const c1 = rb.collider(i);
      world.contactPairsWith(c1, c2 => {
        const pb = c2.parent(); if (!pb || pb.handle === floor?.handle || !pb.isFixed() && !pb.isDynamic()) return;
        if (robot.bodies.some(B => B.rb.handle === pb.handle)) return;
        world.contactPair(c1, c2, (m, flipped) => {
          if (!m.numContacts()) return;
          const n = m.normal(), nb = qrot(qi, flipped ? vscale(V(n.x, n.y, n.z), -1) : V(n.x, n.y, n.z));
          if (Math.abs(nb.z) > 0.8) return;                           // erop of eronder: geen bumper
          bits |= nb.x >= 0 ? 1 : 2;
        });
      });
    }
    return bits;
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
    worldSample, worldEmpty, loadWorld, groundZ, exportWorld, markings, probes, addProbe, moveProbe, removeProbe, get worldInfo() { return worldInfo; }, addBox, removeBox, buildRobot, removeRobot, resetOdom, step, scan, scan3d, sweep3d, lineOfSight, downRange, L3_PRESETS, radarScan, rangeCone, bumper, linkWorld, setFixedJoint, bodyT, jointTF, truthPose };
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

/* ===== Ingebouwde robots per aandrijftype (eigen ontwerp, primitieve vormen) =====
   skid = 4 wielen (skid-steer), tracked = rupsen (4 verborgen wielen per kant), mecanum = zijwaarts rijden (joints met
   "mecanum" in de naam), ackermann = stuurbare voorwielen (revolute om z) + aangedreven achterwielen. */
const ROBOT_PRESETS = {
  skid: { label: "Buitenrobot, 4 wielen (skid-steer)", vmax: 1.0, wmax: 2.0 },
  tracked: { label: "Rupsvoertuig", vmax: 0.6, wmax: 1.5 },
  mecanum: { label: "Mecanum (ook zijwaarts)", vmax: 0.5, wmax: 2.0 },
  ackermann: { label: "Auto (Ackermann-sturing)", vmax: 1.0, wmax: 1.5 },
  rover: { label: "Buitenrobot met vering (4 wielen, skid-steer)", vmax: 1.2, wmax: 2.0 },
  rockerbogie: { label: "Rocker-bogie (6 wielen, zoals een Mars-rover)", vmax: 0.4, wmax: 1.0 },
};
function presetURDF(kind) {
  const f = x => +x.toFixed(5);
  const boxI = (m, x, y, z) => `<inertia ixx="${f(m * (y * y + z * z) / 12)}" ixy="0" ixz="0" iyy="${f(m * (x * x + z * z) / 12)}" iyz="0" izz="${f(m * (x * x + y * y) / 12)}"/>`;
  const cylI = (m, r, l) => `<inertia ixx="${f(m * (3 * r * r + l * l) / 12)}" ixy="0" ixz="0" iyy="${f(m * (3 * r * r + l * l) / 12)}" iyz="0" izz="${f(m * r * r / 2)}"/>`;
  const mats = `<material name="body"><color rgba="0.93 0.62 0.12 1"/></material><material name="dark"><color rgba="0.17 0.18 0.18 1"/></material>
  <material name="tire"><color rgba="0.08 0.08 0.08 1"/></material><material name="metal"><color rgba="0.75 0.77 0.76 1"/></material>
  <material name="blue"><color rgba="0.16 0.40 0.70 1"/></material>`;
  // wiel: joint met rpy -90° om x en as z (zoals de TurtleBot3), cilinder r × w; visual=false → enkel collider (rupsen)
  const wheel = (name, parent, x, y, z, r, w, m, visual = true) => `
  <joint name="${name}_joint" type="continuous"><parent link="${parent}"/><child link="${name}_link"/><origin xyz="${x} ${y} ${z}" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/></joint>
  <link name="${name}_link">${visual ? `
    <visual><geometry><cylinder radius="${r}" length="${w}"/></geometry><material name="tire"/></visual>
    <visual><geometry><box size="${f(r * 1.2)} ${f(r * 0.25)} ${f(w + 0.004)}"/></geometry><material name="metal"/></visual>` : ""}
    <collision><geometry><cylinder radius="${r}" length="${w}"/></geometry></collision>
    <inertial><mass value="${m}"/>${cylI(m, r, w)}</inertial>
  </link>`;
  // sensoren: lidar (base_scan), imu, camera + optisch frame
  const sensors = (lx, lz, cx, cz) => `
  <joint name="scan_joint" type="fixed"><parent link="base_link"/><child link="base_scan"/><origin xyz="${lx} 0 ${lz}"/></joint>
  <link name="base_scan"><visual><origin xyz="0 0 -0.0125"/><geometry><cylinder radius="0.035" length="0.025"/></geometry><material name="dark"/></visual></link>
  <joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 ${f(lz / 2)}"/></joint>
  <link name="imu_link"/>
  <joint name="camera_joint" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz="${cx} 0 ${cz}"/></joint>
  <link name="camera_link"><visual><origin xyz="-0.01 0 0"/><geometry><box size="0.02 0.06 0.025"/></geometry><material name="dark"/></visual></link>
  <joint name="camera_rgb_optical_joint" type="fixed"><parent link="camera_link"/><child link="camera_rgb_optical_frame"/><origin rpy="-1.5708 0 -1.5708"/></joint>
  <link name="camera_rgb_optical_frame"/>`;
  const base = (L, W, H, z, m, mat = "body", extra = "") => `
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 ${z}"/></joint>
  <link name="base_link">
    <visual><geometry><box size="${L} ${W} ${H}"/></geometry><material name="${mat}"/></visual>${extra}
    <collision><geometry><box size="${L} ${W} ${H}"/></geometry></collision>
    <inertial><mass value="${m}"/>${boxI(m, L, W, H)}</inertial>
  </link>`;
  if (kind === "skid") {                     // ~ 60 × 42 cm, wielen r 11 cm; lidar bovenop
    const r = 0.11, z = 0.13, wz = r - z;
    return `<?xml version="1.0"?><robot name="outdoor_skid_lite">${mats}${base(0.60, 0.40, 0.18, z, 20, "body",
      `\n    <visual><origin xyz="0 0 0.1"/><geometry><box size="0.5 0.3 0.02"/></geometry><material name="dark"/></visual>`)}
${wheel("front_left_wheel", "base_link", 0.2, 0.27, wz, r, 0.08, 1.5)}${wheel("rear_left_wheel", "base_link", -0.2, 0.27, wz, r, 0.08, 1.5)}
${wheel("front_right_wheel", "base_link", 0.2, -0.27, wz, r, 0.08, 1.5)}${wheel("rear_right_wheel", "base_link", -0.2, -0.27, wz, r, 0.08, 1.5)}
${sensors(0.1, 0.16, 0.3, 0.06)}</robot>`;
  }
  if (kind === "tracked") {                  // romp + 2 rupsen (enkel beeld), 4 verborgen wielen per kant
    const r = 0.07, z = 0.12, wz = r - z, tr = y => `\n    <visual><origin xyz="0 ${y} ${f(wz)}"/><geometry><box size="0.62 0.1 0.15"/></geometry><material name="tire"/></visual>`;
    let w = "";
    for (const [i, x] of [0.21, 0.07, -0.07, -0.21].entries()) w += wheel(`left_track_wheel${i}`, "base_link", x, 0.2, wz, r, 0.08, 0.6, false) + wheel(`right_track_wheel${i}`, "base_link", x, -0.2, wz, r, 0.08, 0.6, false);
    return `<?xml version="1.0"?><robot name="tracked_lite">${mats}${base(0.52, 0.28, 0.14, z, 15, "blue", tr(0.2) + tr(-0.2))}
${w}${sensors(0.05, 0.13, 0.27, 0.04)}</robot>`;
  }
  if (kind === "mecanum") {                  // 40 × 30 cm, mecanumwielen r 5 cm (rollen niet gesimuleerd)
    const r = 0.05, z = 0.08, wz = r - z;
    return `<?xml version="1.0"?><robot name="mecanum_lite">${mats}${base(0.40, 0.26, 0.08, z, 8)}
${wheel("front_left_mecanum_wheel", "base_link", 0.13, 0.17, wz, r, 0.05, 0.4)}${wheel("rear_left_mecanum_wheel", "base_link", -0.13, 0.17, wz, r, 0.05, 0.4)}
${wheel("front_right_mecanum_wheel", "base_link", 0.13, -0.17, wz, r, 0.05, 0.4)}${wheel("rear_right_mecanum_wheel", "base_link", -0.13, -0.17, wz, r, 0.05, 0.4)}
${sensors(0.05, 0.09, 0.2, 0.03)}</robot>`;
  }
  if (kind === "ackermann") {                // rc-auto ~ 50 × 24 cm, wielbasis 32 cm, sturen ±0,5 rad
    const r = 0.05, z = 0.07, wz = r - z, steer = (side, y) => `
  <joint name="front_${side}_steer_joint" type="revolute"><parent link="base_link"/><child link="front_${side}_steer_link"/><origin xyz="0.16 ${y} ${f(wz)}"/><axis xyz="0 0 1"/><limit lower="-0.5" upper="0.5" effort="5" velocity="3"/></joint>
  <link name="front_${side}_steer_link"><inertial><mass value="0.05"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial></link>${wheel(`front_${side}_wheel`, `front_${side}_steer_link`, 0, 0, 0, r, 0.04, 0.15)}`;
    return `<?xml version="1.0"?><robot name="ackermann_lite">${mats}${base(0.46, 0.18, 0.06, z, 3, "body",
      `\n    <visual><origin xyz="-0.02 0 0.055"/><geometry><box size="0.28 0.16 0.05"/></geometry><material name="dark"/></visual>`)}
${steer("left", 0.12)}${steer("right", -0.12)}
${wheel("rear_left_wheel", "base_link", -0.16, 0.12, wz, r, 0.04, 0.15)}${wheel("rear_right_wheel", "base_link", -0.16, -0.12, wz, r, 0.04, 0.15)}
${sensors(0.0, 0.1, 0.23, 0.03)}</robot>`;
  }
  if (kind === "rover") {                    // ~ 75 × 55 cm, 30 kg, wielen r 15 cm, elk wiel aan een veer (prismatic z, ±5 cm)
    const r = 0.15, z = 0.22, wz = r - z, susp = (name, x, y) => `
  <joint name="${name}_suspension_joint" type="prismatic"><parent link="base_link"/><child link="${name}_suspension_link"/><origin xyz="${x} ${y} ${f(wz)}"/><axis xyz="0 0 1"/>
    <limit lower="-0.05" upper="0.05" effort="800" velocity="2"/><dynamics damping="120" spring_stiffness="1500"/></joint>
  <link name="${name}_suspension_link">
    <visual><origin xyz="0 ${f(-Math.sign(y) * 0.05)} 0.08"/><geometry><cylinder radius="0.018" length="0.16"/></geometry><material name="metal"/></visual>
    <inertial><mass value="3"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>${wheel(`${name}_wheel`, `${name}_suspension_link`, 0, 0, 0, r, 0.1, 1.8)}`;
    return `<?xml version="1.0"?><robot name="rover_suspension_lite">${mats}${base(0.75, 0.42, 0.16, z, 18, "body",
      `\n    <visual><origin xyz="-0.05 0 0.11"/><geometry><box size="0.45 0.34 0.06"/></geometry><material name="dark"/></visual>`)}
${susp("front_left", 0.26, 0.31)}${susp("rear_left", -0.26, 0.31)}${susp("front_right", 0.26, -0.31)}${susp("rear_right", -0.26, -0.31)}
${sensors(0.15, 0.2, 0.37, 0.07)}</robot>`;
  }
  if (kind === "rockerbogie") {              // ~ 70 × 60 cm, 6 wielen r 8 cm; rocker scharniert aan de romp, bogie aan de rocker
    const r = 0.08, z = 0.3, W = 0.29, bar = (x0, z0, x1, z1) => {     // staaf tussen twee punten (in het vlak xz)
      const L = Math.hypot(x1 - x0, z1 - z0), p = -Math.atan2(z1 - z0, x1 - x0);
      return `\n    <visual><origin xyz="${f((x0 + x1) / 2)} 0 ${f((z0 + z1) / 2)}" rpy="0 ${f(p)} 0"/><geometry><box size="${f(L)} 0.025 0.025"/></geometry><material name="metal"/></visual>`; };
    const side = (s, y) => `
  <joint name="${s}_rocker_joint" type="revolute"><parent link="base_link"/><child link="${s}_rocker_link"/><origin xyz="0.02 ${y} 0"/><axis xyz="0 1 0"/><limit lower="-0.5" upper="0.5" effort="50" velocity="2"/></joint>
  <link name="${s}_rocker_link">${bar(0, 0, 0.25, -0.22)}${bar(0, 0, -0.13, -0.12)}
    <inertial><mass value="0.8"/><inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/></inertial></link>
  <joint name="${s}_bogie_joint" type="revolute"><parent link="${s}_rocker_link"/><child link="${s}_bogie_link"/><origin xyz="-0.13 0 -0.12"/><axis xyz="0 1 0"/><limit lower="-0.6" upper="0.6" effort="50" velocity="3"/></joint>
  <link name="${s}_bogie_link">${bar(0.12, -0.1, -0.14, -0.1)}${bar(0, 0, 0, -0.1)}
    <inertial><mass value="0.6"/><inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/></inertial></link>
${wheel(`front_${s}_wheel`, `${s}_rocker_link`, 0.25, 0, -0.22, r, 0.07, 0.5)}${wheel(`middle_${s}_wheel`, `${s}_bogie_link`, 0.12, 0, -0.1, r, 0.07, 0.5)}${wheel(`rear_${s}_wheel`, `${s}_bogie_link`, -0.14, 0, -0.1, r, 0.07, 0.5)}`;
    return `<?xml version="1.0"?><robot name="rocker_bogie_lite">${mats}${base(0.42, 0.4, 0.12, z, 8, "body",
      `\n    <visual><origin xyz="0.05 0 0.2"/><geometry><box size="0.12 0.12 0.28"/></geometry><material name="metal"/></visual>`)}
${side("left", W)}${side("right", -W)}
${sensors(0.05, 0.36, 0.22, 0.08)}</robot>`;
  }
  throw new Error("Onbekende robot: " + kind);
}

/* ===== Ingebouwde werelden (zelfde formaat als een geladen .json-wereld) ===== */
const WORLDS = {"turtlebot3":{"name":"TurtleBot3-wereld","floor":[12,12],"spawn":[-2,-0.5,0],"items":[{"type":"wall","x0":2.6,"y0":0,"x1":1.3,"y1":2.252},{"type":"wall","x0":1.3,"y0":2.252,"x1":-1.3,"y1":2.252},{"type":"wall","x0":-1.3,"y0":2.252,"x1":-2.6,"y1":0},{"type":"wall","x0":-2.6,"y0":0,"x1":-1.3,"y1":-2.252},{"type":"wall","x0":-1.3,"y0":-2.252,"x1":1.3,"y1":-2.252},{"type":"wall","x0":1.3,"y0":-2.252,"x1":2.6,"y1":0},{"type":"box","x":0.7,"y":2.04,"sx":0.6,"sy":0.3,"sz":0.35,"color":"#8a6a45"},{"type":"wall","x0":-1.95,"y0":-1.126,"x1":-1.69,"y1":-0.976},{"type":"cyl","x":-1.1,"y":-1.1,"r":0.15,"h":0.25},{"type":"cyl","x":-1.1,"y":0,"r":0.15,"h":0.25},{"type":"cyl","x":-1.1,"y":1.1,"r":0.15,"h":0.25},{"type":"cyl","x":0,"y":-1.1,"r":0.15,"h":0.25},{"type":"cyl","x":0,"y":0,"r":0.15,"h":0.25},{"type":"cyl","x":0,"y":1.1,"r":0.15,"h":0.25},{"type":"cyl","x":1.1,"y":-1.1,"r":0.15,"h":0.25},{"type":"cyl","x":1.1,"y":0,"r":0.15,"h":0.25},{"type":"cyl","x":1.1,"y":1.1,"r":0.15,"h":0.25},{"type":"movable","x":0.55,"y":-0.55},{"type":"movable","x":-0.55,"y":0.55,"yaw":34}],"fields":[{"kind":"gas","x":0.55,"y":-0.55,"strength":900,"sigma":0.5},{"kind":"heat","x":-1.1,"y":0.55,"strength":25,"sigma":0.4},{"kind":"light","x":0.0,"y":1.65,"strength":500,"sigma":0.8},{"kind":"humidity","x":-0.55,"y":-1.65,"strength":0.3,"sigma":0.5},{"kind":"sound","x":1.65,"y":0.55,"strength":35,"sigma":0.5}]},"factory":{"name":"Fabriekshal","floor":[16,12],"spawn":[-6.8,0,0],"items":[{"type":"wall","x0":-8,"y0":-6,"x1":8,"y1":-6,"t":0.2,"h":1.0,"color":"#b8bfba"},{"type":"wall","x0":8,"y0":-6,"x1":8,"y1":6,"t":0.2,"h":1.0,"color":"#b8bfba"},{"type":"wall","x0":8,"y0":6,"x1":-8,"y1":6,"t":0.2,"h":1.0,"color":"#b8bfba"},{"type":"wall","x0":-8,"y0":6,"x1":-8,"y1":-6,"t":0.2,"h":1.0,"color":"#b8bfba"},{"type":"box","x":-5.3,"y":-1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":-5.3,"y":1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":0,"y":-1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":0,"y":1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":5.3,"y":-1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"box","x":5.3,"y":1.4,"sx":0.35,"sy":0.35,"sz":4.0,"color":"#7c8a92"},{"type":"rack","x":-3.55,"y":5.25,"bays":3,"depth":1.0,"h":2.0},{"type":"rack","x":-3.55,"y":2.85,"bays":3,"depth":1.0,"h":2.0},{"type":"pallet","x":-6.8,"y":-4.8,"load":0.9},{"type":"pallet","x":-5.4,"y":-4.8,"load":1.2},{"type":"pallet","x":-6.8,"y":-3.6,"load":0.6,"color":"#c4d3db"},{"type":"pallet","x":-5.4,"y":-3.6},{"type":"box","x":3.0,"y":4.6,"sx":1.8,"sy":1.4,"sz":1.9,"color":"#d5dad7"},{"type":"box","x":6.2,"y":4.6,"sx":1.8,"sy":1.4,"sz":1.9,"color":"#d5dad7"},{"type":"box","x":4.6,"y":5.6,"sx":0.6,"sy":0.4,"sz":1.8,"color":"#5c6b73"},{"type":"wall","x0":3.2,"y0":-4.8,"x1":7.2,"y1":-4.8,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"wall","x0":7.2,"y0":-4.8,"x1":7.2,"y1":-1.8,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"wall","x0":7.2,"y0":-1.8,"x1":3.2,"y1":-1.8,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"wall","x0":3.2,"y0":-1.8,"x1":3.2,"y1":-2.9,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"wall","x0":3.2,"y0":-3.7,"x1":3.2,"y1":-4.8,"t":0.04,"h":1.8,"color":"#e8c21f"},{"type":"cyl","x":5.6,"y":-3.3,"r":0.3,"h":0.8,"color":"#f07c1e"},{"type":"box","x":4.4,"y":-4.0,"sx":1.0,"sy":0.8,"sz":0.75,"color":"#8e989c"},{"type":"conveyor","x":0.3,"y":-4.4,"l":5.0,"w":0.6,"h":0.8},{"type":"box","x":-1.5,"y":-2.5,"sx":1.6,"sy":0.7,"sz":0.9,"color":"#a07a52"},{"type":"box","x":-7.72,"y":0,"sx":0.3,"sy":0.5,"sz":0.3,"color":"#2b2f31"},{"type":"mark","x":0.2,"y":0.95,"sx":14.4,"sy":0.08,"color":"#f2c230"},{"type":"mark","x":0.2,"y":-0.95,"sx":14.4,"sy":0.08,"color":"#f2c230"},{"type":"mark","x":-7.3,"y":0,"sx":0.8,"sy":0.8,"color":"#3fae5a"},{"type":"mark","x":-6.1,"y":-2.75,"sx":2.9,"sy":0.06,"color":"#f2c230"},{"type":"mark","x":-6.1,"y":-5.6,"sx":2.9,"sy":0.06,"color":"#f2c230"},{"type":"mark","x":-7.55,"y":-4.18,"sx":0.06,"sy":2.9,"color":"#f2c230"},{"type":"mark","x":-4.65,"y":-4.18,"sx":0.06,"sy":2.9,"color":"#f2c230"},{"type":"movable","x":2.5,"y":0.3,"sx":0.4,"sy":0.3,"sz":0.3,"color":"#c79a5b"},{"type":"movable","x":-1.0,"y":-0.4,"sx":0.4,"sy":0.3,"sz":0.3,"color":"#c79a5b"},{"type":"aruco","id":0,"x":-7.89,"y":0,"z":0.42,"yaw":0,"size":0.2,"wall":true},{"type":"model","name":"arbeider","x":0.8,"y":-3.6,"yaw":-90},{"type":"model","name":"arbeider","x":-2.6,"y":-3.4,"yaw":-60},{"type":"model","name":"persoon_b","x":4.0,"y":2.2,"yaw":160},{"type":"model","name":"kegel","x":2.4,"y":-1.6},{"type":"model","name":"kegel","x":3.0,"y":-1.6},{"type":"model","name":"kegel","x":3.5999999999999996,"y":-1.6}],"fields":[{"kind":"gas","x":2.6,"y":-3.8,"strength":1200,"sigma":1.4},{"kind":"heat","x":5.6,"y":-3.3,"strength":35,"sigma":0.9},{"kind":"light","x":-5.3,"y":0,"strength":600,"sigma":2.2},{"kind":"light","x":0,"y":0,"strength":600,"sigma":2.2},{"kind":"light","x":5.3,"y":0,"strength":600,"sigma":2.2},{"kind":"humidity","x":6.2,"y":3.2,"strength":0.3,"sigma":1.2},{"kind":"sound","x":0.3,"y":-4.4,"strength":40,"sigma":1.5}]},"classroom":{"name":"Klaslokaal","floor":[12,10],"spawn":[-3.4,0.0,0],"items":[{"type":"wall","x0":-4.5,"y0":3.5,"x1":4.5,"y1":3.5,"t":0.12,"h":1.0,"color":"#d9d4c7"},{"type":"wall","x0":4.5,"y0":3.5,"x1":4.5,"y1":-3.5,"t":0.12,"h":1.0,"color":"#d9d4c7"},{"type":"wall","x0":4.5,"y0":-3.5,"x1":-3.0,"y1":-3.5,"t":0.12,"h":1.0,"color":"#d9d4c7"},{"type":"wall","x0":-3.9,"y0":-3.5,"x1":-4.5,"y1":-3.5,"t":0.12,"h":1.0,"color":"#d9d4c7"},{"type":"wall","x0":-4.5,"y0":-3.5,"x1":-4.5,"y1":3.5,"t":0.12,"h":1.0,"color":"#d9d4c7"},{"type":"box","x":4.41,"y":0,"z":0.75,"sx":0.03,"sy":3.0,"sz":0.5,"color":"#f4f6f5"},{"type":"table","x":1.8,"y":-1.5,"l":1.6,"w":0.8,"yaw":90},{"type":"chair","x":1.15,"y":-1.9,"yaw":0},{"type":"chair","x":1.15,"y":-1.1,"yaw":0},{"type":"table","x":1.8,"y":1.5,"l":1.6,"w":0.8,"yaw":90},{"type":"chair","x":1.15,"y":1.1,"yaw":0},{"type":"chair","x":1.15,"y":1.9,"yaw":0},{"type":"table","x":0.2,"y":-1.5,"l":1.6,"w":0.8,"yaw":90},{"type":"chair","x":-0.45,"y":-1.9,"yaw":0},{"type":"chair","x":-0.45,"y":-1.1,"yaw":0},{"type":"table","x":0.2,"y":1.5,"l":1.6,"w":0.8,"yaw":90},{"type":"chair","x":-0.45,"y":1.1,"yaw":0},{"type":"chair","x":-0.45,"y":1.9,"yaw":0},{"type":"table","x":-1.4,"y":-1.5,"l":1.6,"w":0.8,"yaw":90},{"type":"chair","x":-2.05,"y":-1.9,"yaw":0},{"type":"chair","x":-2.05,"y":-1.1,"yaw":0},{"type":"table","x":-1.4,"y":1.5,"l":1.6,"w":0.8,"yaw":90},{"type":"chair","x":-2.05,"y":1.1,"yaw":0},{"type":"chair","x":-2.05,"y":1.9,"yaw":0},{"type":"table","x":3.5,"y":-2.2,"l":1.6,"w":0.8,"yaw":0,"color":"#8d6e4c"},{"type":"chair","x":4.1,"y":-2.2,"yaw":180},{"type":"box","x":-4.2,"y":1.5,"sx":0.45,"sy":2.4,"sz":1.0,"color":"#b7bcc0"},{"type":"box","x":-2.0,"y":3.38,"sx":1.2,"sy":0.1,"sz":0.55,"z":0.4,"color":"#eef0f0"},{"type":"box","x":1.0,"y":3.38,"sx":1.2,"sy":0.1,"sz":0.55,"z":0.4,"color":"#eef0f0"},{"type":"box","x":3.2,"y":3.38,"sx":1.2,"sy":0.1,"sz":0.55,"z":0.4,"color":"#eef0f0"},{"type":"movable","x":0.9,"y":-0.55,"sx":0.3,"sy":0.2,"sz":0.4,"color":"#3b6ea5"},{"type":"movable","x":-0.7,"y":2.45,"sx":0.3,"sy":0.2,"sz":0.4,"color":"#a5463b"},{"type":"mark","x":-3.45,"y":-3.2,"sx":0.9,"sy":0.5,"color":"#3fae5a"}],"fields":[{"kind":"heat","x":-2.0,"y":3.1,"strength":22,"sigma":0.7},{"kind":"heat","x":3.2,"y":3.1,"strength":22,"sigma":0.7},{"kind":"light","x":0.0,"y":3.0,"strength":700,"sigma":1.6},{"kind":"gas","x":0.2,"y":-1.5,"strength":800,"sigma":0.8},{"kind":"sound","x":3.5,"y":-2.2,"strength":30,"sigma":1.0},{"kind":"humidity","x":-4.0,"y":-2.8,"strength":0.25,"sigma":0.6}]},"lab":{"name":"Proefhal","floor":[9,7],"spawn":[-3.1,0.2,1.5708],"ground":"#e9e7df","geo":{"lat":51.1945,"lon":3.2179,"alt":10},"items":[{"type":"wall","x0":-4.5,"y0":-3.5,"x1":4.5,"y1":-3.5,"t":0.1,"h":0.4,"color":"#cfd3d6"},{"type":"wall","x0":4.5,"y0":-3.5,"x1":4.5,"y1":3.5,"t":0.1,"h":0.4,"color":"#cfd3d6"},{"type":"wall","x0":4.5,"y0":3.5,"x1":-4.5,"y1":3.5,"t":0.1,"h":0.4,"color":"#cfd3d6"},{"type":"wall","x0":-4.5,"y0":3.5,"x1":-4.5,"y1":-3.5,"t":0.1,"h":0.4,"color":"#cfd3d6"},{"type":"line","closed":true,"w":0.025,"pts":[[2.1,1.1],[2.089,1.222],[2.058,1.339],[2.006,1.45],[1.936,1.55],[1.85,1.636],[1.75,1.706],[1.639,1.758],[1.522,1.789],[1.4,1.8],[-2.4,1.8],[-2.522,1.789],[-2.639,1.758],[-2.75,1.706],[-2.85,1.636],[-2.936,1.55],[-3.006,1.45],[-3.058,1.339],[-3.089,1.222],[-3.1,1.1],[-3.1,-0.7],[-3.089,-0.822],[-3.058,-0.939],[-3.006,-1.05],[-2.936,-1.15],[-2.85,-1.236],[-2.75,-1.306],[-2.639,-1.358],[-2.522,-1.389],[-2.4,-1.4],[1.4,-1.4],[1.522,-1.389],[1.639,-1.358],[1.75,-1.306],[1.85,-1.236],[1.936,-1.15],[2.006,-1.05],[2.058,-0.939],[2.089,-0.822],[2.1,-0.7]]},{"type":"aruco","id":0,"x":4.44,"y":0.2,"z":0.2,"yaw":180,"size":0.16,"wall":true},{"type":"aruco","id":1,"x":-0.5,"y":3.44,"z":0.2,"yaw":-90,"size":0.16,"wall":true},{"type":"aruco","id":2,"x":-4.44,"y":0.2,"z":0.2,"yaw":0,"size":0.16,"wall":true},{"type":"aruco","id":3,"x":1.0,"y":-3.44,"z":0.2,"yaw":90,"size":0.16,"wall":true},{"type":"aruco","id":10,"x":-3.6,"y":-2.6,"z":0.25,"yaw":45,"size":0.2},{"type":"uwb","id":0,"x":4.2,"y":3.2,"z":1.5},{"type":"uwb","id":1,"x":-4.2,"y":3.2,"z":1.5},{"type":"uwb","id":2,"x":-4.2,"y":-3.2,"z":1.5},{"type":"uwb","id":3,"x":4.2,"y":-3.2,"z":1.5},{"type":"box","x":3.2,"y":-2.5,"sx":1.2,"sy":0.9,"sz":0.12,"color":"#9aa3a8"},{"type":"box","x":2.1,"y":-2.5,"z":0.04,"sx":1.007,"sy":0.9,"sz":0.04,"pitch":-6.84,"color":"#b7bec2"},{"type":"movable","x":1.0,"y":0.2,"sx":0.25,"sy":0.25,"sz":0.35,"color":"#c79a5b"}],"fields":[{"kind":"heat","x":3.2,"y":-2.5,"strength":10,"sigma":0.6},{"kind":"light","x":-0.5,"y":0.2,"strength":500,"sigma":2.0}]},"office":{"name":"Kantoor","floor":[10,8],"spawn":[-4.2,-3.3,0.6],"ground":"#c7c2b8","geo":{"lat":51.194,"lon":3.217,"alt":10},"items":[{"type":"wall","x0":-5.0,"y0":-4.0,"x1":5.0,"y1":-4.0,"t":0.12,"h":2.4,"color":"#e3e1da"},{"type":"wall","x0":5.0,"y0":-4.0,"x1":5.0,"y1":4.0,"t":0.12,"h":2.4,"color":"#e3e1da"},{"type":"wall","x0":5.0,"y0":4.0,"x1":-5.0,"y1":4.0,"t":0.12,"h":2.4,"color":"#e3e1da"},{"type":"wall","x0":-5.0,"y0":4.0,"x1":-5.0,"y1":-4.0,"t":0.12,"h":2.4,"color":"#e3e1da"},{"type":"wall","x0":-1.2,"y0":4.0,"x1":-1.2,"y1":1.2,"t":0.1,"h":2.4,"color":"#e3e1da"},{"type":"model","name":"bureau","x":2.4,"y":-2.6,"yaw":180},{"type":"model","name":"bureaustoel","x":1.55,"y":-2.6,"yaw":0},{"type":"model","name":"laptop","x":2.45,"y":-2.35,"z":0.75,"yaw":180},{"type":"model","name":"bureau","x":2.4,"y":-0.9,"yaw":180},{"type":"model","name":"bureaustoel","x":1.55,"y":-0.9,"yaw":0},{"type":"model","name":"scherm","x":2.45,"y":-0.65,"z":0.75,"yaw":180},{"type":"model","name":"bureau","x":2.4,"y":0.8,"yaw":180},{"type":"model","name":"bureaustoel","x":1.55,"y":0.8,"yaw":0},{"type":"model","name":"laptop","x":2.45,"y":1.05,"z":0.75,"yaw":180},{"type":"model","name":"scherm","x":2.65,"y":-2.9,"z":0.75,"yaw":180},{"type":"model","name":"scherm","x":2.65,"y":0.5,"z":0.75,"yaw":180},{"type":"model","name":"zetel","x":-4.42,"y":2.3,"yaw":0},{"type":"model","name":"tv","x":-1.35,"y":2.6,"z":1.0,"yaw":180},{"type":"box","x":-1.4,"y":2.6,"sx":0.35,"sy":1.2,"sz":0.9,"color":"#8d6e4c"},{"type":"model","name":"plant_groot","x":4.45,"y":3.45},{"type":"model","name":"plant","x":-4.6,"y":-3.6},{"type":"model","name":"plant","x":4.6,"y":-3.6},{"type":"model","name":"boekenkast","x":-4.7,"y":-1.0,"yaw":0},{"type":"model","name":"boekenkast","x":-4.7,"y":-2.0,"yaw":0},{"type":"model","name":"stoel","x":-3.0,"y":1.9,"yaw":90},{"type":"model","name":"persoon_a","x":0.2,"y":-1.6,"yaw":20},{"type":"model","name":"persoon_b","x":-2.4,"y":0.4,"yaw":-40},{"type":"model","name":"persoon_c","x":3.5,"y":2.8,"yaw":200},{"type":"model","name":"persoon_d","x":-3.6,"y":3.0,"yaw":-60},{"type":"mark","x":-4.2,"y":-3.3,"sx":0.9,"sy":0.6,"color":"#3fae5a"}],"fields":[{"kind":"heat","x":2.4,"y":-0.9,"strength":4,"sigma":1.0},{"kind":"gas","x":-3.0,"y":2.5,"strength":700,"sigma":1.5},{"kind":"light","x":0,"y":0,"strength":450,"sigma":3},{"kind":"sound","x":-1.4,"y":2.6,"strength":25,"sigma":1.5}]},"hospital":{"name":"Ziekenhuis","floor":[16,10],"spawn":[-7.4,-0.5,0],"ground":"#d9dcd6","geo":{"lat":51.204,"lon":3.2236,"alt":8},"items":[{"type":"wall","x0":-8.0,"y0":-5.0,"x1":8.0,"y1":-5.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":8.0,"y0":-5.0,"x1":8.0,"y1":5.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":8.0,"y0":5.0,"x1":-8.0,"y1":5.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":-8.0,"y0":5.0,"x1":-8.0,"y1":-5.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":-8.0,"y0":1.0,"x1":-7.05,"y1":1.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":-5.95,"y0":1.0,"x1":-3.05,"y1":1.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":-1.95,"y0":1.0,"x1":0.95,"y1":1.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":2.05,"y0":1.0,"x1":4.95,"y1":1.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":6.05,"y0":1.0,"x1":8.0,"y1":1.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":-4,"y0":1.0,"x1":-4,"y1":5.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":0,"y0":1.0,"x1":0,"y1":5.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":4,"y0":1.0,"x1":4,"y1":5.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":-8.0,"y0":-1.0,"x1":-6.55,"y1":-1.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":-5.45,"y0":-1.0,"x1":-0.050000000000000044,"y1":-1.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":1.05,"y0":-1.0,"x1":4.95,"y1":-1.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":6.05,"y0":-1.0,"x1":8.0,"y1":-1.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":-2,"y0":-1.0,"x1":-2,"y1":-5.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"wall","x0":3,"y0":-1.0,"x1":3,"y1":-5.0,"t":0.12,"h":2.6,"color":"#eef0ee"},{"type":"model","name":"ziekenhuisbed","x":-6,"y":3.85,"yaw":-90},{"type":"model","name":"nachtkastje","x":-5.15,"y":4.6,"yaw":-90},{"type":"model","name":"wachtstoel","x":-7.1,"y":3.2,"yaw":0},{"type":"model","name":"wastafel","x":-4.36,"y":1.45,"yaw":90},{"type":"aruco","id":20,"x":-6.3,"y":0.9299999999999999,"z":0.45,"yaw":-90,"size":0.15,"wall":true},{"type":"model","name":"ziekenhuisbed","x":-2,"y":3.85,"yaw":-90},{"type":"model","name":"nachtkastje","x":-1.15,"y":4.6,"yaw":-90},{"type":"model","name":"wachtstoel","x":-3.1,"y":3.2,"yaw":0},{"type":"model","name":"wastafel","x":-0.36,"y":1.45,"yaw":90},{"type":"aruco","id":21,"x":-2.3,"y":0.9299999999999999,"z":0.45,"yaw":-90,"size":0.15,"wall":true},{"type":"model","name":"ziekenhuisbed","x":2,"y":3.85,"yaw":-90},{"type":"model","name":"nachtkastje","x":2.85,"y":4.6,"yaw":-90},{"type":"model","name":"wachtstoel","x":0.8999999999999999,"y":3.2,"yaw":0},{"type":"model","name":"wastafel","x":3.64,"y":1.45,"yaw":90},{"type":"aruco","id":22,"x":1.7,"y":0.9299999999999999,"z":0.45,"yaw":-90,"size":0.15,"wall":true},{"type":"model","name":"ziekenhuisbed","x":6,"y":3.85,"yaw":-90},{"type":"model","name":"nachtkastje","x":6.85,"y":4.6,"yaw":-90},{"type":"model","name":"wachtstoel","x":4.9,"y":3.2,"yaw":0},{"type":"model","name":"wastafel","x":7.64,"y":1.45,"yaw":90},{"type":"aruco","id":23,"x":5.7,"y":0.9299999999999999,"z":0.45,"yaw":-90,"size":0.15,"wall":true},{"type":"model","name":"persoon_e","x":-2.6,"y":2.4,"yaw":-30},{"type":"model","name":"persoon_c","x":5.45,"y":2.6,"yaw":200},{"type":"model","name":"wachtstoel","x":-7.2,"y":-3.2,"yaw":90},{"type":"model","name":"wachtstoel","x":-7.2,"y":-4.4,"yaw":90},{"type":"model","name":"wachtstoel","x":-6.5,"y":-3.2,"yaw":90},{"type":"model","name":"wachtstoel","x":-6.5,"y":-4.4,"yaw":90},{"type":"model","name":"wachtstoel","x":-5.800000000000001,"y":-3.2,"yaw":90},{"type":"model","name":"wachtstoel","x":-5.800000000000001,"y":-4.4,"yaw":90},{"type":"model","name":"plant","x":-7.55,"y":-2.4,"yaw":0},{"type":"model","name":"tv","x":-2.2,"y":-3.0,"yaw":180,"z":1.1},{"type":"model","name":"persoon_f","x":-5.0,"y":-3.6,"yaw":120},{"type":"model","name":"vuilnisbak","x":-2.4,"y":-4.6,"yaw":0},{"type":"aruco","id":30,"x":-6.6,"y":-0.9299999999999999,"z":0.45,"yaw":90,"size":0.15,"wall":true},{"type":"model","name":"bureau","x":0.5,"y":-2.6,"yaw":90},{"type":"model","name":"bureau","x":1.9,"y":-2.6,"yaw":90},{"type":"model","name":"bureaustoel","x":0.5,"y":-3.4,"yaw":90},{"type":"model","name":"bureaustoel","x":1.9,"y":-3.4,"yaw":90},{"type":"model","name":"scherm","x":0.5,"y":-2.45,"yaw":-90,"z":0.75},{"type":"model","name":"laptop","x":1.9,"y":-2.5,"yaw":-90,"z":0.75},{"type":"model","name":"persoon_b","x":1.2,"y":-4.2,"yaw":70},{"type":"model","name":"kapstok","x":2.6,"y":-4.6,"yaw":0},{"type":"aruco","id":31,"x":-0.1,"y":-0.9299999999999999,"z":0.45,"yaw":90,"size":0.15,"wall":true},{"type":"model","name":"toilet","x":4.0,"y":-4.4,"yaw":90},{"type":"model","name":"toilet","x":6.0,"y":-4.4,"yaw":90},{"type":"model","name":"wastafel","x":7.6,"y":-2.0,"yaw":180},{"type":"model","name":"vuilnisbak","x":7.6,"y":-3.0,"yaw":0},{"type":"wall","x0":5.0,"y0":-5.0,"x1":5.0,"y1":-3.6,"t":0.08,"h":2.0,"color":"#d6dbe0"},{"type":"aruco","id":32,"x":4.9,"y":-0.9299999999999999,"z":0.45,"yaw":90,"size":0.15,"wall":true},{"type":"model","name":"rolstoel","x":-3.6,"y":-0.6,"yaw":0},{"type":"model","name":"persoon_a","x":0.0,"y":0.3,"yaw":180},{"type":"model","name":"persoon_d","x":3.8,"y":-0.4,"yaw":10},{"type":"model","name":"defibrillator","x":7.86,"y":0.4,"yaw":180,"z":1.2},{"type":"model","name":"vuilnisbak","x":-7.7,"y":0.8,"yaw":0},{"type":"line","pts":[[-7.4,0],[7.4,0]],"w":0.03,"color":"#e8b50f"},{"type":"mark","x":-7.4,"y":-0.5,"sx":0.7,"sy":0.5,"color":"#3fae5a"},{"type":"uwb","id":0,"x":-7.8,"y":0.9,"z":2.3,"wall":true},{"type":"uwb","id":1,"x":7.8,"y":0.9,"z":2.3,"wall":true},{"type":"uwb","id":2,"x":7.8,"y":-0.9,"z":2.3,"wall":true},{"type":"uwb","id":3,"x":-7.8,"y":-0.9,"z":2.3,"wall":true}],"fields":[{"kind":"gas","x":-5.0,"y":-3.6,"strength":900,"sigma":1.4},{"kind":"humidity","x":5.5,"y":-3.8,"strength":0.3,"sigma":1.2},{"kind":"heat","x":2.0,"y":3.6,"strength":5,"sigma":1.0},{"kind":"sound","x":-2.2,"y":-3.0,"strength":28,"sigma":1.5},{"kind":"light","x":0,"y":0,"strength":450,"sigma":4}]},"yard":{"name":"Bedrijfsterrein","floor":[30,24],"spawn":[-13.2,0,0],"ground":"#9a9e9f","sky":"#bcd6ea","geo":{"lat":50.9528,"lon":3.1235,"alt":22},"items":[{"type":"fence","x0":-14.5,"y0":-11.5,"x1":14.5,"y1":-11.5},{"type":"fence","x0":14.5,"y0":-11.5,"x1":14.5,"y1":11.5},{"type":"fence","x0":14.5,"y0":11.5,"x1":-14.5,"y1":11.5},{"type":"fence","x0":-14.5,"y0":11.5,"x1":-14.5,"y1":1.6},{"type":"fence","x0":-14.5,"y0":-1.6,"x1":-14.5,"y1":-11.5},{"type":"box","x":4,"y":7.5,"sx":18,"sy":7,"sz":6,"color":"#c5cbcf"},{"type":"box","x":-1.5,"y":3.97,"sx":3.2,"sy":0.06,"sz":3.4,"color":"#46525a"},{"type":"mark","x":-1.5,"y":2.8,"sx":3.2,"sy":2.2,"color":"#6f7477"},{"type":"box","x":2.5,"y":3.97,"sx":3.2,"sy":0.06,"sz":3.4,"color":"#46525a"},{"type":"mark","x":2.5,"y":2.8,"sx":3.2,"sy":2.2,"color":"#6f7477"},{"type":"box","x":6.5,"y":3.97,"sx":3.2,"sy":0.06,"sz":3.4,"color":"#46525a"},{"type":"mark","x":6.5,"y":2.8,"sx":3.2,"sy":2.2,"color":"#6f7477"},{"type":"box","x":11.5,"y":3.97,"sx":1.0,"sy":0.06,"sz":2.1,"color":"#2d6aa0"},{"type":"box","x":-9,"y":8,"sx":8,"sy":5.5,"sz":4,"color":"#d6c9b0"},{"type":"box","x":-7.6,"y":5.22,"sx":1.4,"sy":0.06,"sz":2.2,"color":"#2d6aa0"},{"type":"box","x":-8.5,"y":-5.5,"sx":6.06,"sy":2.44,"sz":2.59,"yaw":0,"color":"#c8642c"},{"type":"box","x":-8.5,"y":-8.3,"sx":6.06,"sy":2.44,"sz":2.59,"yaw":0,"color":"#2f6d9a"},{"type":"box","x":-1.5,"y":-8.6,"sx":6.06,"sy":2.44,"sz":2.59,"yaw":90,"color":"#3e8a52"},{"type":"box","x":-8.5,"y":-6.9,"z":3.885,"sx":6.06,"sy":2.44,"sz":2.59,"color":"#a8a39b"},{"type":"cyl","x":-4.6,"y":-4.9,"r":0.29,"h":0.88,"color":"#d84c2a"},{"type":"cyl","x":-4.6,"y":-5.6,"r":0.29,"h":0.88,"color":"#2b5fa8"},{"type":"cyl","x":-3.9,"y":-4.9,"r":0.29,"h":0.88,"color":"#2b5fa8"},{"type":"box","x":-4.2,"y":-6.8,"sx":1.2,"sy":1.0,"sz":1.16,"color":"#e9ece8"},{"type":"pallet","x":0.2,"y":1.2,"load":1.0},{"type":"pallet","x":1.6,"y":1.2,"load":0.7},{"type":"pallet","x":4.6,"y":1.0,"load":1.2},{"type":"pallet","x":4.6,"y":-0.1},{"type":"box","x":12.5,"y":-1.5,"sx":2.2,"sy":1.2,"sz":1.6,"color":"#5b6870"},{"type":"cyl","x":13.2,"y":-1.5,"z":2.0,"r":0.1,"h":0.8,"color":"#3a3f42"},{"type":"mark","x":5.5,"y":-8.2,"sx":0.1,"sy":5.0,"color":"#f4f4f0"},{"type":"mark","x":8.1,"y":-8.2,"sx":0.1,"sy":5.0,"color":"#f4f4f0"},{"type":"mark","x":10.7,"y":-8.2,"sx":0.1,"sy":5.0,"color":"#f4f4f0"},{"type":"mark","x":13.3,"y":-8.2,"sx":0.1,"sy":5.0,"color":"#f4f4f0"},{"type":"mark","x":15.9,"y":-8.2,"sx":0.1,"sy":5.0,"color":"#f4f4f0"},{"type":"model","name":"auto_sedan","x":6.8,"y":-8.4,"yaw":90},{"type":"model","name":"auto_suv","x":9.4,"y":-8.4,"yaw":90},{"type":"model","name":"pickup","x":12.0,"y":-8.4,"yaw":90},{"type":"model","name":"persoon_a","x":-0.6,"y":2.2,"yaw":-80},{"type":"model","name":"arbeider","x":3.6,"y":1.9,"yaw":200},{"type":"model","name":"persoon_f","x":-9.5,"y":-2.2,"yaw":30},{"type":"model","name":"kegel","x":5.2,"y":1.6},{"type":"model","name":"kegel","x":6.1000000000000005,"y":1.6},{"type":"model","name":"kegel","x":7.0,"y":1.6},{"type":"model","name":"kegel","x":7.9,"y":1.6},{"type":"cyl","x":-12.8,"y":-2.2,"r":0.1,"h":1.0,"color":"#e8c21f"},{"type":"cyl","x":-12.8,"y":2.2,"r":0.1,"h":1.0,"color":"#e8c21f"},{"type":"cyl","x":-12,"y":4.2,"r":0.08,"h":6,"color":"#7e8589"},{"type":"cyl","x":-3,"y":-2.8,"r":0.08,"h":6,"color":"#7e8589"},{"type":"cyl","x":9,"y":2.6,"r":0.08,"h":6,"color":"#7e8589"},{"type":"cyl","x":0,"y":-11,"r":0.08,"h":6,"color":"#7e8589"},{"type":"mark","x":0,"y":-10.6,"sx":28.6,"sy":1.6,"color":"#6f9a4f"},{"type":"mark","x":-13.6,"y":6.5,"sx":1.6,"sy":9.6,"color":"#6f9a4f"},{"type":"tree","x":-11,"y":-10.7,"h":1.6,"crown":1.2},{"type":"tree","x":-6,"y":-10.7,"h":1.6,"crown":1.2},{"type":"tree","x":3,"y":-10.7,"h":1.6,"crown":1.2},{"type":"tree","x":8,"y":-10.7,"h":1.6,"crown":1.2},{"type":"mark","x":-2,"y":-0.9,"sx":22,"sy":0.1,"color":"#f2c230"},{"type":"mark","x":-2,"y":0.9,"sx":22,"sy":0.1,"color":"#f2c230"},{"type":"mark","x":-13.2,"y":0,"sx":1.0,"sy":1.0,"color":"#3fae5a"},{"type":"movable","x":-6.0,"y":0.2,"sx":0.4,"sy":0.3,"sz":0.35,"color":"#c79a5b"},{"type":"movable","x":3.0,"y":-2.6,"sx":0.4,"sy":0.3,"sz":0.35,"color":"#c79a5b"},{"type":"uwb","id":0,"x":-14.2,"y":11.2,"z":2.5},{"type":"uwb","id":1,"x":14.2,"y":11.2,"z":2.5},{"type":"uwb","id":2,"x":14.2,"y":-11.2,"z":2.5},{"type":"uwb","id":3,"x":-14.2,"y":-11.2,"z":2.5},{"type":"aruco","id":5,"x":-1.5,"y":3.9,"z":0.6,"yaw":-90,"size":0.4,"wall":true}],"fields":[{"kind":"heat","x":12.5,"y":-1.5,"strength":30,"sigma":1.6},{"kind":"sound","x":12.5,"y":-1.5,"strength":45,"sigma":2.5},{"kind":"gas","x":13.2,"y":-2.6,"strength":1500,"sigma":1.8},{"kind":"light","x":0,"y":-4,"strength":9000,"sigma":9},{"kind":"humidity","x":-4.3,"y":-6.8,"strength":0.3,"sigma":1.2}]},"orchard":{"name":"Boomgaard","floor":[30,26],"spawn":[-12.8,0,0],"ground":"#7fa65c","sky":"#bcd6ea","geo":{"lat":50.9411,"lon":3.0987,"alt":18},"items":[{"type":"fence","x0":-14.5,"y0":-12.5,"x1":14.5,"y1":-12.5,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"fence","x0":14.5,"y0":-12.5,"x1":14.5,"y1":12.5,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"fence","x0":14.5,"y0":12.5,"x1":-14.5,"y1":12.5,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"fence","x0":-14.5,"y0":12.5,"x1":-14.5,"y1":2.0,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"fence","x0":-14.5,"y0":-2.0,"x1":-14.5,"y1":-12.5,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"orchard","x":-8.0,"y":-8.0,"rows":5,"n":10,"row":3.5,"gap":1.8,"h":0.9,"crown":0.8,"missing":0.06},{"type":"mark","x":0.09999999999999964,"y":-8.0,"sx":18.0,"sy":0.9,"color":"#6c8d45"},{"type":"mark","x":0.09999999999999964,"y":-4.5,"sx":18.0,"sy":0.9,"color":"#6c8d45"},{"type":"mark","x":0.09999999999999964,"y":-1.0,"sx":18.0,"sy":0.9,"color":"#6c8d45"},{"type":"mark","x":0.09999999999999964,"y":2.5,"sx":18.0,"sy":0.9,"color":"#6c8d45"},{"type":"mark","x":0.09999999999999964,"y":6.0,"sx":18.0,"sy":0.9,"color":"#6c8d45"},{"type":"mark","x":0.09999999999999964,"y":-6.25,"sx":18.0,"sy":2.0,"color":"#93b971"},{"type":"mark","x":0.09999999999999964,"y":-2.75,"sx":18.0,"sy":2.0,"color":"#93b971"},{"type":"mark","x":0.09999999999999964,"y":0.75,"sx":18.0,"sy":2.0,"color":"#93b971"},{"type":"mark","x":0.09999999999999964,"y":4.25,"sx":18.0,"sy":2.0,"color":"#93b971"},{"type":"box","x":-11.5,"y":9.5,"sx":5,"sy":3.5,"sz":3,"color":"#8a5c3b"},{"type":"box","x":-11.5,"y":7.72,"sx":2.6,"sy":0.06,"sz":2.4,"color":"#3d2c20"},{"type":"box","x":-7.5,"y":10.5,"sx":1.2,"sy":1.0,"sz":0.76,"color":"#b98a54"},{"type":"box","x":-6.2,"y":10.5,"sx":1.2,"sy":1.0,"sz":0.76,"color":"#b98a54"},{"type":"box","x":-7.5,"y":9.3,"sx":1.2,"sy":1.0,"sz":0.76,"color":"#b98a54"},{"type":"box","x":13.0,"y":-6.25,"sx":1.2,"sy":1.0,"sz":0.76,"color":"#b98a54"},{"type":"box","x":13.0,"y":0.75,"sx":1.2,"sy":1.0,"sz":0.76,"color":"#b98a54"},{"type":"box","x":13.0,"y":4.25,"sx":1.2,"sy":1.0,"sz":0.76,"color":"#b98a54"},{"type":"box","x":10.5,"y":10.0,"sx":3.0,"sy":2.0,"sz":1.0,"color":"#5a4632"},{"type":"box","x":12.8,"y":-10.4,"sx":0.8,"sy":0.6,"sz":0.7,"color":"#2f6d9a"},{"type":"mark","x":0,"y":-11.6,"sx":28.6,"sy":1.2,"color":"#4f7fa6"},{"type":"mark","x":-12.8,"y":0,"sx":1.0,"sy":1.0,"color":"#3fae5a"},{"type":"model","name":"boer","x":-8.6,"y":7.6,"yaw":-120}],"fields":[{"kind":"humidity","x":0,"y":-11.6,"strength":0.35,"sigma":3.0},{"kind":"heat","x":9,"y":6,"strength":8,"sigma":4.0},{"kind":"light","x":-12,"y":-2,"strength":15000,"sigma":6},{"kind":"gas","x":10.5,"y":10.0,"strength":1200,"sigma":2.0},{"kind":"sound","x":12.8,"y":-10.4,"strength":35,"sigma":2.0}]},"field":{"name":"Akker","floor":[36,28],"spawn":[-15.5,1.0,0],"ground":"#8a7353","sky":"#bcd6ea","geo":{"lat":50.9302,"lon":3.0511,"alt":16},"terrain":{"res":0.5,"tilt":[0.012,0],"wave":{"amp":0.12,"len":14},"hills":[{"x":8,"y":8,"h":0.5,"s":6}]},"items":[{"type":"crops","plant":"mais","x":-12,"y":-10.25,"rows":14,"l":22,"row":0.75,"w":0.3,"h":1.3},{"type":"crops","plant":"kool","x":-12,"y":2.2,"rows":10,"l":22,"row":0.6,"w":0.35,"h":0.3,"color":"#7fae5a"},{"type":"mark","x":-1,"y":1.0,"sx":30,"sy":1.6,"color":"#a58a63"},{"type":"mark","x":13.5,"y":0,"sx":3.0,"sy":26,"color":"#b9b2a4"},{"type":"mark","x":-15.5,"y":0,"sx":4.0,"sy":26,"color":"#8db35e"},{"type":"mark","x":0,"y":-13.2,"sx":34,"sy":1.0,"color":"#4f7fa6"},{"type":"wall","x0":-17.5,"y0":13.4,"x1":17.5,"y1":13.4,"t":1.0,"h":1.6,"color":"#3f6b33"},{"type":"box","x":16.2,"y":-8.0,"z":0.45,"sx":2.4,"sy":1.2,"sz":0.9,"color":"#d8c27a"},{"type":"box","x":16.2,"y":-8.0,"z":1.35,"sx":2.4,"sy":1.2,"sz":0.9,"color":"#d8c27a"},{"type":"box","x":16.2,"y":-6.7,"z":0.45,"sx":2.4,"sy":1.2,"sz":0.9,"color":"#d8c27a"},{"type":"box","x":16.2,"y":-6.7,"z":1.35,"sx":2.4,"sy":1.2,"sz":0.9,"color":"#d8c27a"},{"type":"box","x":16.2,"y":-5.4,"z":0.45,"sx":2.4,"sy":1.2,"sz":0.9,"color":"#d8c27a"},{"type":"box","x":16.2,"y":-5.4,"z":1.35,"sx":2.4,"sy":1.2,"sz":0.9,"color":"#d8c27a"},{"type":"cyl","x":15.6,"y":9.0,"r":1.5,"h":6,"color":"#c9ccc9"},{"type":"box","x":15.8,"y":4.2,"sx":3.0,"sy":4.0,"sz":3.2,"color":"#8a5c3b"},{"type":"cyl","x":-14,"y":-12.2,"r":0.12,"h":8,"color":"#6b5a48"},{"type":"cyl","x":-4,"y":-12.2,"r":0.12,"h":8,"color":"#6b5a48"},{"type":"cyl","x":6,"y":-12.2,"r":0.12,"h":8,"color":"#6b5a48"},{"type":"tree","x":-16.5,"y":11.5,"h":2.2,"crown":1.6},{"type":"tree","x":11.5,"y":12.0,"h":2.0,"crown":1.4,"kind":"den"},{"type":"mark","x":-15.5,"y":1.0,"sx":1.0,"sy":1.0,"color":"#3fae5a"}],"fields":[{"kind":"humidity","x":0,"y":-13.2,"strength":0.3,"sigma":3.0},{"kind":"light","x":0,"y":6,"strength":20000,"sigma":10},{"kind":"gas","x":15.6,"y":9.0,"strength":1500,"sigma":2.5},{"kind":"heat","x":5,"y":-5,"strength":6,"sigma":5},{"kind":"sound","x":15.8,"y":4.2,"strength":30,"sigma":2.5}]},"hills":{"name":"Heuvelland","floor":[30,30],"spawn":[-12.5,0,0],"ground":"#7d9f58","sky":"#bcd6ea","geo":{"lat":50.779,"lon":2.788,"alt":60},"terrain":{"res":0.4,"wave":{"amp":0.06,"len":9},"hills":[{"x":5,"y":5,"h":1.4,"s":4},{"x":-6,"y":-5,"h":1.0,"s":3.5},{"x":-8,"y":9,"h":0.7,"s":2.5},{"x":3,"y":-7.5,"h":-0.6,"s":2.2},{"x":11,"y":-10,"h":0.8,"sx":2,"sy":5}]},"items":[{"type":"fence","x0":-14.5,"y0":-14.5,"x1":14.5,"y1":-14.5,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"fence","x0":14.5,"y0":-14.5,"x1":14.5,"y1":14.5,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"fence","x0":14.5,"y0":14.5,"x1":-14.5,"y1":14.5,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"fence","x0":-14.5,"y0":14.5,"x1":-14.5,"y1":2.0,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"fence","x0":-14.5,"y0":-2.0,"x1":-14.5,"y1":-14.5,"color":"#a9b39a","opacity":0.18,"gap":3.0},{"type":"mark","x":3,"y":-7.5,"sx":3.4,"sy":2.6,"color":"#3f74a3"},{"type":"sphere","x":-2,"y":3,"z":0.225,"r":0.45,"color":"#8b8f8c"},{"type":"sphere","x":1.5,"y":8.5,"z":0.175,"r":0.35,"color":"#8b8f8c"},{"type":"sphere","x":9,"y":-2,"z":0.25,"r":0.5,"color":"#8b8f8c"},{"type":"sphere","x":-9,"y":-9,"z":0.2,"r":0.4,"color":"#8b8f8c"},{"type":"sphere","x":10.5,"y":10,"z":0.15,"r":0.3,"color":"#8b8f8c"},{"type":"sphere","x":-4,"y":-1.5,"z":0.15,"r":0.3,"color":"#8b8f8c"},{"type":"tree","x":-11,"y":9,"h":1.4,"crown":1.1},{"type":"tree","x":-6,"y":11,"h":1.4,"crown":1.1,"kind":"den"},{"type":"tree","x":8,"y":-11,"h":1.4,"crown":1.1},{"type":"tree","x":12,"y":-6,"h":1.4,"crown":1.1,"kind":"berk"},{"type":"tree","x":-12,"y":-11,"h":1.4,"crown":1.1,"kind":"den"},{"type":"tree","x":2,"y":12,"h":1.4,"crown":1.1},{"type":"cyl","x":5,"y":5,"r":0.25,"h":1.0,"color":"#a49a8a"},{"type":"mark","x":-11,"y":0,"sx":3.6,"sy":1.2,"yaw":8,"color":"#b9b2a4"},{"type":"mark","x":-7,"y":0.5,"sx":3.6,"sy":1.2,"yaw":12,"color":"#b9b2a4"},{"type":"mark","x":-3.5,"y":2,"sx":3.6,"sy":1.2,"yaw":30,"color":"#b9b2a4"},{"type":"mark","x":-0.5,"y":4.5,"sx":3.6,"sy":1.2,"yaw":40,"color":"#b9b2a4"},{"type":"mark","x":2.5,"y":6.5,"sx":3.6,"sy":1.2,"yaw":35,"color":"#b9b2a4"},{"type":"mark","x":-12.5,"y":0,"sx":1.0,"sy":1.0,"color":"#3fae5a"}],"fields":[{"kind":"light","x":5,"y":5,"strength":15000,"sigma":5},{"kind":"humidity","x":3,"y":-7.5,"strength":0.35,"sigma":2.0},{"kind":"heat","x":-6,"y":-7,"strength":5,"sigma":3},{"kind":"sound","x":3,"y":-7.5,"strength":20,"sigma":2.0},{"kind":"gas","x":-9,"y":-9,"strength":600,"sigma":2.0}]},"empty":{"name":"Lege kamer","floor":[12,12],"spawn":[0,0,0],"items":[{"type":"wall","x0":-2.5,"y0":-2.5,"x1":2.5,"y1":-2.5},{"type":"wall","x0":2.5,"y0":-2.5,"x1":2.5,"y1":2.5},{"type":"wall","x0":2.5,"y0":2.5,"x1":-2.5,"y1":2.5},{"type":"wall","x0":-2.5,"y0":2.5,"x1":-2.5,"y1":-2.5}]}};
// SIM-CORE-END
const MODEL_INFO = {"auto_sedan":{"size":[4.7,1.8,1.45],"collider":"box","label":"voertuig","temp":25},"auto_suv":{"size":[4.6,1.85,1.7],"collider":"box","label":"voertuig","temp":25},"bed":{"size":[1.745,0.877,0.6],"collider":"box","label":"bed"},"bestelwagen":{"size":[4.074,2.222,2],"collider":"box","label":"voertuig","temp":25},"boekenkast":{"size":[0.4,0.9,1.8],"collider":"box","label":"kast"},"bureau":{"size":[0.75,1.4,0.75],"collider":"box","label":"tafel"},"bureaustoel":{"size":[0.716,0.64,1],"collider":"box","label":"stoel"},"defibrillator":{"size":[0.169,0.272,0.35],"collider":"box","label":"defibrillator"},"kapstok":{"size":[0.422,0.422,1.75],"collider":"cyl","label":"kapstok"},"kegel":{"size":[0.387,0.387,0.5],"collider":"cyl","label":"kegel"},"laptop":{"size":[0.224,0.3,0.25],"collider":"box","label":"laptop","temp":33},"nachtkastje":{"size":[0.333,0.446,0.55],"collider":"box","label":"kast"},"persoon_a":{"size":[0.635,0.595,1.8],"collider":"cyl","label":"persoon","temp":34},"persoon_b":{"size":[0.297,0.51,1.68],"collider":"cyl","label":"persoon","temp":34},"persoon_c":{"size":[0.618,0.58,1.75],"collider":"cyl","label":"persoon","temp":34},"persoon_d":{"size":[0.3,0.518,1.7],"collider":"cyl","label":"persoon","temp":34},"persoon_e":{"size":[0.638,0.608,1.82],"collider":"cyl","label":"persoon","temp":34},"persoon_f":{"size":[0.287,0.494,1.64],"collider":"cyl","label":"persoon","temp":34},"arbeider":{"size":[0.62,0.587,1.78],"collider":"cyl","label":"persoon","temp":34},"boer":{"size":[0.616,0.58,1.76],"collider":"cyl","label":"persoon","temp":34},"vrouw_jurk":{"size":[0.431,0.5,1.66],"collider":"cyl","label":"persoon","temp":34},"pickup":{"size":[5.3,1.9,1.85],"collider":"box","label":"voertuig","temp":25},"plant":{"size":[0.622,0.565,0.9],"collider":"cyl","label":"plant"},"rolstoel":{"size":[0.887,0.672,0.9],"collider":"box","label":"rolstoel"},"scherm":{"size":[0.148,0.729,0.45],"collider":"box","label":"scherm","temp":32},"stoel":{"size":[0.604,0.506,0.9],"collider":"box","label":"stoel"},"toilet":{"size":[0.712,0.504,0.8],"collider":"box","label":"toilet"},"tv":{"size":[0.198,0.972,0.6],"collider":"box","label":"scherm","temp":35},"vuilnisbak":{"size":[0.36,0.3,0.6],"collider":"cyl","label":"vuilnisbak"},"wachtstoel":{"size":[0.68,0.62,0.85],"collider":"box","label":"stoel"},"wastafel":{"size":[0.45,0.55,0.9],"collider":"box","label":"wastafel"},"zetel":{"size":[0.922,2.038,0.8],"collider":"box","label":"zetel"},"ziekenhuisbed":{"size":[1.891,0.95,0.65],"collider":"box","label":"bed"},"boom_loof":{"size":[3.514,3.815,6.0],"collider":"cyl","label":"tree"},"boom_berk":{"size":[3.808,5.552,8.0],"collider":"cyl","label":"tree"},"boom_den":{"size":[5.103,4.725,9.0],"collider":"cyl","label":"tree"},"gewas_mais":{"size":[0.882,1.172,2.0],"collider":"box","label":"crops"},"plant_groot":{"size":[0.858,0.909,1.1],"collider":"cyl","label":"plant"}};
