/* ================= opdrachten: automatische tests, opname en proefrapport ================= */
const SIM_VERSION = "2026-10-21";
const SYSTEM_NODES = /^(rosbridge|rosapi|slam_toolbox|amcl|map_server|map_saver|planner_server|controller_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|lifecycle_manager|collision_monitor|docking_server|route_server|robot_state_publisher|rviz|teleop|transform_listener|launch|_ros2cli|ros2cli|rqt|nav2|global_costmap|local_costmap)/;
const NUM_RE = /-?\d+(?:[.,]\d+)?/g;
const nums = s => (s.match(NUM_RE) || []).map(x => parseFloat(x.replace(",", ".")));
const isWarnLine = e => e.level >= 30 || /warn|waarsch|alarm|\blow\b|laag|kritiek|critical/i.test(e.msg);
const fmt = (x, d = 2) => (Math.round(x * 10 ** d) / 10 ** d).toFixed(d);
const r4 = x => Math.round(x * 1e4) / 1e4;
const r2d = r => r * 180 / Math.PI;
const wrapA = a => Math.atan2(Math.sin(a), Math.cos(a));

/* ---- logboek van /rosout ---- */
const rosLog = [];
function onRosout(m) {
  const e = { t: sim.simTime, level: m.level, name: (m.name || "").replace(/^\//, ""), msg: String(m.msg || "") };
  rosLog.push(e); if (rosLog.length > 2000) rosLog.shift();
  if (activeRun) activeRun.onLog(e);
}

/* ---- scanconventie: ongeldig als 0.0 (echte LDS) of inf (Gazebo) ---- */
let invalidMode = "inf";
const recentScans = [];                                    // [t, frontMin, ranges]
function frontMin(ranges, deg15 = 15) {
  const n = ranges.length, k = Math.round(n * deg15 / 360);
  const v = [...ranges.slice(0, k), ...ranges.slice(n - k)].filter(r => r > 0.12 && isFinite(r));
  return v.length ? Math.min(...v) : Infinity;
}
function publishScanMsg(msg) {
  recentScans.push([sim.simTime, frontMin(msg.ranges), msg.ranges]); if (recentScans.length > 40) recentScans.shift();
  if (!live()) return;
  const mode = activeRun?.invalidOverride || invalidMode;
  if (mode === "inf") {
    const body = JSON.stringify({ op: "publish", topic: "/scan", msg: { ...msg, ranges: msg.ranges.map(r => r === 0 ? 1e300 : r) } });
    ws.send(body.replace(/1e\+300/g, "Infinity"));       // rosbridge (Python json) aanvaardt Infinity
  } else ws.send(JSON.stringify({ op: "publish", topic: "/scan", msg }));
  stats["/scan"].n++;
}

/* ---- rosapi-servicecall (voor de controle op andere publishers) ---- */
const pendingCalls = new Map(); let callId = 0;
function callService(service, args, timeout = 2000) {
  return new Promise(res => {
    if (!live()) return res(null);
    const id = "c" + (++callId); pendingCalls.set(id, res);
    send({ op: "call_service", id, service, args });
    setTimeout(() => { if (pendingCalls.has(id)) { pendingCalls.delete(id); res(null); } }, timeout);
  });
}
function onServiceResponse(m) { const r = pendingCalls.get(m.id); if (r) { pendingCalls.delete(m.id); r(m.result === false ? null : m.values); } }

/* ---- opname: simulatorbeeld (samengesteld canvas) of volledig scherm ---- */
const comp = document.createElement("canvas"), compCtx = comp.getContext("2d");
const recorder = { mr: null, chunks: [], source: "", stream: null, started: 0, label: "" };
function pickMime() {
  for (const m of ["video/webm;codecs=vp9", "video/webm;codecs=vp8", "video/webm", "video/mp4"]) if (window.MediaRecorder && MediaRecorder.isTypeSupported(m)) return m;
  return "";
}
async function startRecording(source, label) {
  if (recorder.mr) return true;
  let stream;
  try {
    if (source === "screen") stream = await navigator.mediaDevices.getDisplayMedia({ video: { frameRate: 15 }, audio: false });
    else { const c = renderer.domElement, s = Math.min(1, 1280 / c.width); comp.width = Math.round(c.width * s); comp.height = Math.round(c.height * s); stream = comp.captureStream(25); }
  } catch (e) { $("taskMsg").textContent = "Opname niet gestart: " + e.message; return false; }
  const mime = pickMime(); if (!mime) { $("taskMsg").textContent = "Deze browser kan niet opnemen (MediaRecorder ontbreekt)."; return false; }
  Object.assign(recorder, { mr: new MediaRecorder(stream, { mimeType: mime, videoBitsPerSecond: 2.5e6 }), chunks: [], source, stream, started: Date.now(), label });
  recorder.mr.ondataavailable = ev => ev.data.size && recorder.chunks.push(ev.data);
  stream.getVideoTracks()[0].addEventListener("ended", () => { if (recorder.mr) $("recToggle").click(); });   // gebruiker stopt delen
  recorder.mr.start(1000); updateRecUI(); return true;
}
function stopRecording() {
  return new Promise(res => {
    const r = recorder; if (!r.mr) return res(null);
    r.mr.onstop = async () => {
      const blob = new Blob(r.chunks, { type: r.mr.mimeType }); r.stream.getTracks().forEach(t => t.stop());
      const sha = await sha256hex(await blob.arrayBuffer());
      const out = { blob, sha256: sha, bytes: blob.size, mime: blob.type, seconds: Math.round((Date.now() - r.started) / 1000), source: r.source };
      Object.assign(recorder, { mr: null, chunks: [], stream: null }); updateRecUI(); res(out);
    };
    r.mr.stop();
  });
}
function updateRecUI() { $("recToggle").textContent = recorder.mr ? "Opname stoppen" : "Opname starten"; $("recToggle").classList.toggle("rec", !!recorder.mr); }
// wordt na elke render aangeroepen: WebGL-beeld + HUD + camera + identificatiebalk op één canvas
function drawComposite() {
  if (!recorder.mr || recorder.source !== "sim") return;
  const c = compCtx, W = comp.width, H = comp.height;
  c.drawImage(renderer.domElement, 0, 0, W, H);
  c.font = "12px 'JetBrains Mono', monospace"; c.textBaseline = "top";
  const lines = $("hud").innerText.split("\n"), lw = Math.max(...lines.map(l => c.measureText(l).width)) + 16;
  c.fillStyle = "rgba(255,255,255,0.85)"; c.fillRect(10, 40, lw, lines.length * 16 + 10);
  c.fillStyle = "#17201b"; lines.forEach((l, i) => c.fillText(l, 18, 46 + i * 16));
  if (cfg && cfg.camera) { const cw = Math.min(260, W * 0.3), ch = cw * 0.75; c.drawImage($("camPreview"), W - cw - 10, 40, cw, ch); }
  drawOverlayOnCanvas(c, W, H);
  c.fillStyle = "rgba(10,14,12,0.82)"; c.fillRect(0, 0, W, 30);
  c.fillStyle = "#fff"; c.font = "600 14px Barlow, Arial, sans-serif"; c.textBaseline = "middle";
  c.fillText(`${stuLabel() || "geen student ingesteld"}  ·  ${recorder.label}  ·  ${new Date().toLocaleString("nl-BE")}  ·  sim ${sim.simTime.toFixed(1)} s`, 10, 15);
  c.fillStyle = "#ef4444"; c.beginPath(); c.arc(W - 18, 15, 6, 0, 7); c.fill();
}
async function sha256hex(buf) { return Array.from(await sha256bytes(buf), b => b.toString(16).padStart(2, "0")).join(""); }
// Lokaal: gewone download. Als artifact: via de downloads-capability (gewone links werken daar niet).
let dlCap; const getDl = () => (dlCap ??= (window.claude?.use ? window.claude.use("downloads").catch(() => null) : Promise.resolve(null)));
async function saveBlob(blob, name) {
  const dl = await getDl();
  if (dl) {
    try { await dl.save({ filename: name, data: blob }); }
    catch (e) { if (e?.code !== "declined") { const m = $("taskMsg"); if (m) m.textContent = "Downloaden lukte niet: " + (e?.message || e?.code); } }
    return;
  }
  const a = document.createElement("a"); a.href = URL.createObjectURL(blob); a.download = name; document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 4000);
}
// projectbestanden (enkel in de gepubliceerde artifact aanwezig)
getDl().then(async dl => {
  if (!dl) return;
  // artifacts serveren geen .zip: het archief staat er als base64-tekst en wordt hier terug omgezet
  const ZIP_URL = "project/tb3-lite-sim.zip.b64.txt";
  try { const r = await fetch(ZIP_URL, { method: "HEAD" }); if (!r.ok) return; } catch { return; }
  const b = $("dlProject"); b.hidden = false;
  b.onclick = async () => {
    b.disabled = true;
    try {
      const bin = atob((await (await fetch(ZIP_URL)).text()).replace(/\s+/g, "")), u8 = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
      await saveBlob(new Blob([u8], { type: "application/zip" }), "tb3-lite-sim.zip");
    } finally { b.disabled = false; }
  };
});

/* ---- testpaneel: live in beeld én in de opname ---- */
let resultRun = null, resultUntil = 0, lastOvUpdate = 0;
const sectorG = new THREE.Group(); sectorG.visible = false; scene.add(sectorG);
{
  const wedge = new THREE.Mesh(new THREE.CircleGeometry(2.0, 48, -15 * Math.PI / 180, 30 * Math.PI / 180),
    new THREE.MeshBasicMaterial({ color: 0xf2c230, transparent: true, opacity: 0.18, depthWrite: false }));
  wedge.position.z = 0.03; sectorG.add(wedge);
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.29, 0.31, 64), new THREE.MeshBasicMaterial({ color: 0xf2c230, transparent: true, opacity: 0.9, depthWrite: false }));
  ring.position.z = 0.032; sectorG.add(ring); sectorG.userData.ring = ring;
  for (const a of [-15, 15]) {
    const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0.034), new THREE.Vector3(2 * Math.cos(a * Math.PI / 180), 2 * Math.sin(a * Math.PI / 180), 0.034)]);
    sectorG.add(new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xc89a10 })));
  }
}
function showSectorHelpers(on) { sectorG.visible = on; }
function frameCameraFor(id) {
  const p = sim.truthPose(), c = Math.cos(p.th), s2 = Math.sin(p.th), at = (fx, fy, z, tx, ty) => {
    camera.position.set(p.x + c * fx - s2 * fy, p.y + s2 * fx + c * fy, z); controls.target.set(p.x + c * tx - s2 * ty, p.y + s2 * tx + c * ty, 0.05);
  };
  $("follow").checked = false;
  if (id === "t1" || id === "t1x") at(-0.75, -0.95, 1.05, 0.25, 0.05);
  else if (id === "t3") at(-0.35, -0.55, 1.15, 0, 0);
  else if (id === "t3x") at(-0.3, -1.25, 1.9, 0.25, 0.25);
  else at(0.9, -0.5, 1.4, 0.2, 0);                       // taak 2: robot van binnen de wereld bekeken
  controls.update();
}
function overlayData() {
  const r = activeRun || (performance.now() < resultUntil ? resultRun : null); if (!r) return null;
  const st = r.step, left = st.dur ? Math.max(0, st.dur - (sim.simTime - st.t0)) : 0;
  const logs = r.events.filter(e => e.type === "warn" || e.type === "log").slice(-5);
  return { title: r.def.label, step: r.result ? `Resultaat: ${r.result}` : (st.n ? `Stap ${st.i}/${st.n}: ${st.label}` : st.label) + (left ? ` (${left.toFixed(1)} s)` : ""),
    big: r.result ? "" : st.big, checks: r.checks, logs, node: r.node, result: r.result, flash: sim.simTime < r.flashUntil };
}
const SYM = { ok: "✓", fail: "✗", run: "…", wait: "○" };
function testOverlayFrame() {
  const o = overlayData(), el = $("testOv");
  if (sectorG.visible && cfg.lidar) {
    const T = sim.linkWorld(cfg.lidar); sectorG.position.set(T.p.x, T.p.y, 0); sectorG.rotation.z = yawOf(T.q);
    sectorG.userData.ring.material.color.set(o && o.flash ? 0xe5372b : 0xf2c230);
  }
  if (!o) { el.hidden = true; return; }
  el.hidden = false;
  if (performance.now() - lastOvUpdate < 120) return; lastOvUpdate = performance.now();
  el.classList.toggle("flash", !!o.flash); el.dataset.result = o.result ? (o.result === "GESLAAGD" ? "ok" : "fail") : "";
  el.innerHTML = `<b>${escapeHtml(o.title)}</b><div class="ovstep">${escapeHtml(o.step)}</div>${o.big ? `<div class="ovbig">${escapeHtml(o.big)}</div>` : ""}
    <ul>${o.checks.map(c => `<li data-s="${c.s}">${SYM[c.s]} ${escapeHtml(c.label)}</li>`).join("")}</ul>
    <div class="ovlog"><span>${o.node ? "/rosout van " + escapeHtml(o.node) : "/rosout: nog geen logregels van je node"}</span>${o.logs.map(e => `<div class="${e.type}">[${fmt(e.t, 1)} s] ${escapeHtml(e.msg ?? e.text)}</div>`).join("")}</div>`;
}
function drawOverlayOnCanvas(c, W, H) {
  const o = overlayData(); if (!o) return;
  const pad = 10, lh = 16, w = Math.min(W * 0.5, 560), lines = 4 + o.checks.length + o.logs.length + (o.big ? 1.5 : 0);
  const h = lines * lh + pad * 2, x = W - w - 10, y = H - h - 10;
  c.fillStyle = o.flash ? "rgba(80,14,10,0.88)" : "rgba(14,18,16,0.86)"; c.fillRect(x, y, w, h);
  if (o.result) { c.fillStyle = o.result === "GESLAAGD" ? "#2fa45c" : "#d23c2a"; c.fillRect(x, y, 6, h); }
  let yy = y + pad; c.textBaseline = "top";
  const txt = (s, font, col, dx = 0) => { c.font = font; c.fillStyle = col; let t = s; while (c.measureText(t).width > w - pad * 2 - dx && t.length > 4) t = t.slice(0, -2); c.fillText(t === s ? s : t + "…", x + pad + dx, yy); yy += lh; };
  txt(o.title, "600 14px Barlow, Arial, sans-serif", "#fff");
  txt(o.step, "500 13px Barlow, Arial, sans-serif", "#f2c230");
  if (o.big) { txt(o.big, "600 17px 'JetBrains Mono', monospace", "#fff"); yy += lh * 0.5; }
  const colS = { ok: "#4cc27e", fail: "#ef6d55", run: "#e3a645", wait: "#9aa59f" };
  for (const ch of o.checks) txt(`${SYM[ch.s]} ${ch.label}`, "12px 'JetBrains Mono', monospace", colS[ch.s]);
  txt(o.node ? `/rosout van ${o.node}:` : "/rosout: nog geen logregels van je node", "italic 12px Barlow, Arial, sans-serif", "#9aa59f");
  for (const e of o.logs) txt(`[${fmt(e.t, 1)} s] ${e.msg ?? e.text}`, "12px 'JetBrains Mono', monospace", e.type === "warn" ? "#ff8a73" : "#dfe7e2", 6);
}

/* ---- testrun ---- */
let activeRun = null, lastResult = null;
class Run {
  constructor(def) {
    this.def = def; this.t0 = sim.simTime; this.wall0 = new Date(); this.events = []; this.node = ""; this.done = false;
    this.checks = def.checks.map(c => ({ ...c, s: "wait", detail: "" })); this.state = {};
    this.step = { label: "voorbereiden…", i: 0, n: 0, t0: sim.simTime, dur: 0, big: "" }; this.flashUntil = -1; this.result = "";
  }
  setStep(label, i, n, dur = 0, big = "") { this.step = { label, i, n, t0: sim.simTime, dur, big }; }
  ev(type, text, msg) { this.events.push({ t: r4(sim.simTime - this.t0), type, text: String(text).slice(0, 300), ...(msg !== undefined ? { msg: String(msg).slice(0, 300) } : {}) }); renderChecks(); }
  check(id, ok, detail = "") { const c = this.checks.find(x => x.id === id); if (!c) return; c.s = ok === null ? "run" : ok ? "ok" : "fail"; c.detail = detail; renderChecks(); }
  onLog(e) {
    if (SYSTEM_NODES.test(e.name)) return;
    if (this.def.lockNode !== false && !this.node && (this.def.acceptNode ? this.def.acceptNode(e, this) : true)) { this.node = e.name; this.ev("node", "studentnode herkend: " + e.name); }
    if (this.node && e.name !== this.node) return;
    this.ev(e.level >= 30 ? "warn" : "log", `[${e.name}] ${e.msg}`, e.msg);
    if (e.level >= 30 || /obstacle|obstakel|warn|waarsch|alarm|\blow\b|laag/i.test(e.msg)) this.flashUntil = sim.simTime + 0.8;
    this.def.onLog?.(this, e);
  }
}
function renderChecks() {
  const r = activeRun || lastResult?.run; if (!r) { $("taskChecks").innerHTML = ""; return; }
  $("taskChecks").innerHTML = r.checks.map(c => `<li data-s="${c.s}"><span>${c.label}${c.optional ? " <em>(info)</em>" : ""}</span>${c.detail ? `<small>${escapeHtml(c.detail)}</small>` : ""}</li>`).join("");
}
const escapeHtml = s => String(s).replace(/[&<>"]/g, ch => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[ch]);
const sleepSim = (run, s) => new Promise(res => { const t = sim.simTime + s; const f = () => (activeRun !== run ? res(false) : sim.simTime >= t ? res(true) : setTimeout(f, 30)); f(); });

function prepareEmptyArena(x = 0, y = 0, th = 0) {
  if (sim.worldInfo.name !== WORLDS.empty.name) { $("worldSel").value = "empty"; switchWorld(WORLDS.empty, WORLDS.empty.name); }
  if (builtin === false) { /* eigen robot blijft */ }
  rebuild(x, y, th); keys.clear(); keyCmd = null; padCmd = null;
}
const lidarOrigin = () => sim.linkWorld(cfg.lidar).p;
function placeProbe(pr, d, angDeg) {                     // d = afstand lidar → oppervlak paal
  const o = lidarOrigin(), th = sim.truthPose().th + angDeg * Math.PI / 180, c = d + pr.r;
  sim.moveProbe(pr, o.x + c * Math.cos(th), o.y + c * Math.sin(th));
}

/* ---- taak 1 en uitbreiding ---- */
const T1_STEPS = [
  { id: "s1", d: 1.00, a: 0, label: "Paal op 1,00 m vóór: geen alarm", warn: 0, inv: "zero" },
  { id: "s2", d: 0.25, a: 0, label: "Paal op 0,25 m vóór: één waarschuwing, met de juiste afstand", warn: 1, dist: true, inv: "zero" },
  { id: "s3", d: 0.60, a: 0, label: "Paal terug naar 0,60 m: één keer \"free again\"", warn: 0, info: true, inv: "zero" },
  { id: "s4", d: 0.20, a: 0, label: "Opnieuw op 0,20 m: weer één waarschuwing", warn: 1, inv: "inf" },
  { id: "s5", d: 0.25, a: 90, label: "Paal links (90°): vrij, geen alarm", warn: 0, info: true, inv: "inf" },
  { id: "s6", d: 0.25, a: 10, label: "Schuin vóór (10°, binnen ±15°): alarm", warn: 1, inv: "inf" },
  { id: "s7", d: 0.25, a: 35, label: "Buiten de voorsector ±15° (35°): vrij, geen alarm", warn: 0, info: true, inv: "inf" },
];
async function runObstacle(run, steps, evalStep) {
  prepareEmptyArena();
  run.state.savedRmax = sim.LIDAR.rmax; sim.LIDAR.rmax = 2.0;   // muren op 2,5 m vallen buiten bereik → ongeldige metingen in elke scan
  const pr = run.state.probe = sim.addProbe(20, 20); addProbeMesh(pr);
  run.ev("stap", "bereik tijdelijk 2,0 m, zodat elke scan ongeldige waarden bevat");
  showSectorHelpers(true);
  run.invalidOverride = "zero";
  if (!(await sleepSim(run, 2.0))) return;
  for (const st of steps) {
    run.invalidOverride = st.inv; placeProbe(pr, st.d, st.a);
    run.ev("stap", `${st.label} [ongeldig = ${st.inv === "inf" ? "inf" : "0.0"}]`);
    run.setStep(st.label, steps.indexOf(st) + 1, steps.length, 3.0, `paal op ${fmt(st.d)} m · ${st.a}° · ongeldig = ${st.inv === "inf" ? "inf" : "0.0"}`);
    run.check(st.id, null); const t0 = sim.simTime, from = run.events.length;
    if (!(await sleepSim(run, 3.0))) return;
    evalStep(run, st, t0, run.events.slice(from).filter(e => e.type === "warn" || e.type === "log"));
  }
}
function scanMinsBetween(t0, t1) { return recentScans.filter(s => s[0] >= t0 - 0.05 && s[0] <= t1 + 0.05).map(s => s[1]).filter(isFinite); }
function cleanupObstacle(run) {
  showSectorHelpers(false);
  if (run.state.probe) { removeProbeMesh(run.state.probe); sim.removeProbe(run.state.probe); }
  if (run.state.savedRmax) sim.LIDAR.rmax = run.state.savedRmax;
}

const TASKS = {
  t1: {
    label: "Taak 1 · Obstakelalarm",
    how: "Voorsector: ±15° rond de voorkant (n uit angle_increment). Start obstacle_alarm.py in je container en klik dan op Test starten. De robot staat stil in de lege kamer; een paaltje (Ø 8 cm, als een stoelpoot) wordt op vaste plaatsen gezet. Het lidarbereik gaat tijdelijk naar 2 m, zodat elke scan ongeldige waarden bevat (eerst 0.0, daarna inf). Duurt ongeveer 25 s.",
    checks: [...T1_STEPS.map(s => ({ id: s.id, label: s.label })), { id: "edge", label: "Melding één keer per obstakel (flankdetectie)" }, { id: "filt", label: "Ongeldige metingen (0.0 en inf) gefilterd" }],
    async run(run) {
      run.state.repeat = false; run.state.filt = true;
      await runObstacle(run, T1_STEPS, (r, st, t0, lines) => {
        const warns = lines.filter(e => e.type === "warn" || /obstacle|obstakel/i.test(e.msg) && !/free|vrij/i.test(e.msg));
        const frees = lines.filter(e => /free|vrij/i.test(e.msg));
        let ok = warns.length === st.warn, det = `${warns.length} waarschuwing(en)`;
        if (warns.length > 1) r.state.repeat = true;
        if (st.warn === 0 && warns.length) {
          const n = nums(warns[0].msg).filter(x => x >= 0 && x < 10);
          if (n.length && n[0] < 0.12) { r.state.filt = false; det += ` met afstand ${n[0]} m: ongeldige waarde niet gefilterd`; }
        }
        if (st.info) { ok = ok && frees.length >= 1; det += `, ${frees.length}× "free again"`; if (frees.length > 1) r.state.repeat = true; }
        if (st.dist && warns.length) {
          const n = nums(warns[0].msg).filter(x => x > 0 && x < 5), mins = scanMinsBetween(t0, sim.simTime);
          if (!n.length) det += ", afstand niet in de melding";
          else {
            const near = mins.some(m => Math.abs(m - n[0]) <= 0.03);
            det += `, gemeld ${n[0]} m (scan gaf ${mins.length ? fmt(Math.min(...mins)) + "–" + fmt(Math.max(...mins)) : "?"} m)`;
            ok = ok && near;
          }
        }
        r.check(st.id, ok, det);
      });
      run.check("edge", !run.state.repeat, run.state.repeat ? "melding herhaalt bij elke scan: onthoud de vorige toestand in self.alarm" : "");
      run.check("filt", run.state.filt && run.checks.find(c => c.id === "s1").s === "ok",
        run.state.filt ? "" : "min() nam een ongeldige waarde mee: filter op isfinite en > 0.12 m");
    },
    cleanup: cleanupObstacle,
  },
  t1x: {
    label: "Taak 1 · Uitbreiding vier sectoren",
    how: "Start je uitgebreide obstacle_alarm.py (die per scan de vier sectoren en de hoek van het dichtste obstakel logt met get_logger().info). De paal komt achtereenvolgens vóór, links, achter en rechts. Duurt ongeveer 15 s.",
    checks: [
      { id: "front", label: "Vóór (0,50 m) juist gemeld" }, { id: "left", label: "Links (0,70 m) juist gemeld" },
      { id: "behind", label: "Achter (0,90 m) juist gemeld" }, { id: "right", label: "Rechts (0,40 m) juist gemeld" },
      { id: "empty", label: "Sectoren zonder geldige meting: inf of leeg, geen 0.0" }, { id: "angle", label: "Hoek van het dichtste obstakel gelogd (± 5°)" }],
    async run(run) {
      const SECT = [["front", /(front|voor)\w*\s*[:=]?\s*(inf|nan|none|-+|[\d.,]+)/i, 0, 0.5], ["left", /(left|links)\s*[:=]?\s*(inf|nan|none|-+|[\d.,]+)/i, 90, 0.7],
        ["behind", /(behind|back|achter)\w*\s*[:=]?\s*(inf|nan|none|-+|[\d.,]+)/i, 180, 0.9], ["right", /(right|rechts)\s*[:=]?\s*(inf|nan|none|-+|[\d.,]+)/i, 270, 0.4]];
      const parse = txt => Object.fromEntries(SECT.map(([k, re]) => { const m = txt.match(re); return [k, m ? m[2] : undefined]; }));
      const val = s => s === undefined ? undefined : /^(inf|nan|none|-+)$/i.test(s) ? Infinity : parseFloat(s.replace(",", "."));
      let emptyOk = true, emptySeen = false, angleOk = false, angleSeen = false;
      const steps = SECT.map(([k, , a, d]) => ({ id: k, d, a, label: `Paal ${k} op ${d} m`, inv: k === "behind" || k === "right" ? "inf" : "zero" }));
      await runObstacle(run, steps, (r, st, t0, lines) => {
        const parsed = lines.map(e => ({ e, p: parse(e.msg) })).filter(x => Object.values(x.p).filter(v => v !== undefined).length >= 3).slice(-5);
        if (!parsed.length) { r.check(st.id, false, "geen regel met de vier sectoren gevonden"); return; }
        const mins = scanMinsBetween(t0, sim.simTime), last = parsed[parsed.length - 1].p, got = val(last[st.id]);
        // verwachte afstand: zelfde paal, de scanruis bepaalt de exacte waarde
        r.check(st.id, Number.isFinite(got) && Math.abs(got - st.d) <= 0.05, `gemeld ${last[st.id] ?? "–"} m`);
        for (const [k] of SECT) if (k !== st.id && last[k] !== undefined) { emptySeen = true; const v = val(last[k]); if (Number.isFinite(v) && v < 0.12) emptyOk = false; }
        for (const { e } of parsed) {
          const m = e.msg.match(/(-?\d+(?:[.,]\d+)?)\s*(°|deg|graden)/i) || e.msg.match(/(?:angle|hoek)\D{0,6}(-?\d+(?:[.,]\d+)?)/i);
          if (m) { angleSeen = true; const g = parseFloat(m[1].replace(",", ".")), dd = Math.abs(wrapA((g - st.a) * Math.PI / 180)) * 180 / Math.PI; if (dd <= 5) angleOk = true; }
        }
        void mins;
      });
      run.check("empty", emptySeen && emptyOk, emptyOk ? "" : "0.0 gemeld voor een sector zonder obstakel");
      run.check("angle", angleSeen && angleOk, angleSeen ? (angleOk ? "" : "hoek wijkt meer dan 5° af") : "geen hoek in graden gevonden in de logregels");
    },
    cleanup: cleanupObstacle,
  },
  t2: {
    label: "Taak 2 · Batterijwaarschuwing",
    get how() { return "Tijdens de test publiceert de simulator zelf " + batTopic() + " met waarden net boven en onder de drempels (11,2 V en 50 %). Start je script en klik op Test starten. Gebruik self.get_logger().info() en .warn() i.p.v. print(): enkel die zijn zichtbaar via /rosout. Duurt ongeveer 15 s."; },
    checks: [
      { id: "pub", label: "Geen andere publisher op het batterij-topic" }, { id: "print", label: "Spanning en percentage van elk bericht gelogd" },
      { id: "volt", label: "Waarschuwing onder 11,2 V (ook net eronder: 11,15 V)" }, { id: "pct", label: "Waarschuwing onder 50 % (ook net eronder: 49 %)" },
      { id: "none", label: "Geen waarschuwing boven beide drempels (ook 11,25 V / 55 %)" }],
    lockNode: true,
    acceptNode: (e, run) => run.state.cur && nums(e.msg).some(x => Math.abs(x - run.state.cur.v) <= 0.051),
    async run(run) {
      const BT = batTopic(); run.state.bt = BT;
      const info = await callService("/rosapi/publishers", { topic: BT });
      const others = (info?.publishers || []).filter(n => !/rosbridge/.test(n));
      run.check("pub", !others.length, info ? (others.length ? "nog actief: " + others.join(", ") + ". Stop je eigen publisher op " + BT : "") : "rosapi niet bereikbaar, niet gecontroleerd");
      if (others.length) { finishRun("andere publisher actief"); return; }
      send({ op: "advertise", topic: BT, type: "sensor_msgs/msg/BatteryState" });
      run.state.adv = true;
      const SEQ = [[12.4, 0.90, ""], [11.0, 0.90, "volt"], [12.0, 0.30, "pct"], [11.25, 0.55, ""], [11.15, 0.55, "volt"], [11.8, 0.49, "pct"], [10.6, 0.10, "both"], [12.6, 1.00, ""]];
      if (!(await sleepSim(run, 1.0))) return;
      const res = [];
      for (const [v, p, why] of SEQ) {
        run.state.cur = { v, p };
        const st = stamp();
        send({ op: "publish", topic: BT, msg: { header: { stamp: st, frame_id: "battery" }, voltage: v, percentage: p, present: true } });
        run.ev("stap", `${BT}: ${v} V, ${Math.round(p * 100)} %`);
        run.setStep(`${BT}: ${v} V · ${Math.round(p * 100)} %`, SEQ.findIndex(x => x[0] === v && x[1] === p) + 1, SEQ.length, 1.5,
          `${v.toFixed(2)} V · ${Math.round(p * 100)} %  →  verwacht: ${why ? "WAARSCHUWING" : "geen waarschuwing"}`);
        const from = run.events.length;
        if (!(await sleepSim(run, 1.5))) return;
        const lines = run.events.slice(from).filter(e => e.type === "warn" || e.type === "log");
        const printed = lines.some(e => { const n = nums(e.msg); return n.some(x => Math.abs(x - v) <= 0.051) && n.some(x => Math.abs(x - p) <= 0.0051 || Math.abs(x - 100 * p) <= 0.51); });
        const warned = lines.some(e => e.type === "warn" || /warn|waarsch|alarm|\blow\b|laag|kritiek/i.test(e.msg));
        res.push({ v, p, why, printed, warned });
      }
      const show = x => `${x.v} V / ${Math.round(x.p * 100)} %`;
      run.check("print", res.filter(x => x.printed).length >= res.length - 1, `${res.filter(x => x.printed).length}/${res.length} berichten herkend in de logregels` + (run.node ? "" : " — geen logregels ontvangen: gebruik get_logger()"));
      const volt = res.filter(x => x.why === "volt" || x.why === "both"), pct = res.filter(x => x.why === "pct" || x.why === "both"), none = res.filter(x => !x.why);
      run.check("volt", volt.every(x => x.warned), volt.filter(x => !x.warned).map(x => "geen waarschuwing bij " + show(x)).join("; "));
      run.check("pct", pct.every(x => x.warned), pct.filter(x => !x.warned).map(x => "geen waarschuwing bij " + show(x)).join("; "));
      const wrong = none.filter(x => x.warned);
      run.check("none", !wrong.length, wrong.map(x => "onterecht bij " + show(x)).join("; ") + (wrong.some(x => x.p >= 0.9) ? " — percentage is 0–1, niet 0–100" : ""));
    },
    cleanup(run) { if (run.state.adv && !OUT[run.state.bt]) send({ op: "unadvertise", topic: run.state.bt }); },   // /battery_state blijft voor het sensorpaneel
  },
  t3: {
    label: "Taak 3 · Exact 90° draaien",
    how: "Klik op Test starten en start daarna turn_90.py. De robot staat in de lege kamer; de simulator meet de echte draaihoek (niet /odom) vanaf je eerste /cmd_vel tot de robot stilstaat. Niet met het toetsenbord rijden.",
    checks: [
      { id: "ros", label: "Bediend via /cmd_vel (geen handbediening)" }, { id: "angle", label: "Werkelijk gedraaid: 90° ± 2° naar links" },
      { id: "spot", label: "Ter plaatse gedraaid (verplaatsing < 2 cm)" }, { id: "zero", label: "Nulsnelheid gepubliceerd aan het einde" },
      { id: "odom", label: "Hoek volgens /odom", optional: true }],
    lockNode: false,
    async run(run) { await runMotion(run, false); },
  },
  t3x: {
    label: "Taak 3 · Uitbreiding vierkant",
    how: "Klik op Test starten en start daarna je vierkant-script (zijden van 0,5 m, terug naar de start). De simulator meet zijden, hoeken en de eindafwijking met de echte positie.",
    checks: [
      { id: "ros", label: "Bediend via /cmd_vel (geen handbediening)" }, { id: "sides", label: "Vier zijden van 0,50 m ± 3 cm" },
      { id: "turns", label: "Hoeken van 90° ± 3° (minstens drie)" }, { id: "zero", label: "Nulsnelheid gepubliceerd aan het einde" },
      { id: "close", label: "Afstand eindpunt tot start (werkelijk en volgens /odom)", optional: true }],
    lockNode: false,
    async run(run) { await runMotion(run, true); },
  },
};

/* ---- taak 3: bewegingen meten met de grondwaarheid ---- */
async function runMotion(run, square) {
  prepareEmptyArena(square ? -0.25 : 0, square ? -0.25 : 0, 0);
  run.ev("stap", "wachten op het eerste /cmd_vel van je script (max. 60 s)…");
  run.setStep("Start nu je script: wachten op /cmd_vel", 1, 2, 0, "wachten op het eerste /cmd_vel…");
  const S = run.state; S.manual = false; S.samples = []; S.lastRos = null;
  const waitStart = sim.simTime, pPre = sim.truthPose(), oPre = { ...sim.odom };   // robot staat geremd stil: dit is de startpose
  while (activeRun === run) {
    if (keyCmd || padCmd) S.manual = true;
    if (rosCmd.t >= 0 && sim.simTime - rosCmd.t < 0.3 && (Math.abs(rosCmd.v) > 0.005 || Math.abs(rosCmd.w) > 0.005)) break;
    if (sim.simTime - waitStart > 60) { run.check("ros", false, "geen /cmd_vel ontvangen. Draait je script? " + cmdHint()); finishRun("geen /cmd_vel"); return; }
    await sleepSim(run, 0.02);
  }
  if (activeRun !== run) return;
  const p0 = pPre, o0 = oPre, pNow = sim.truthPose(); let thAcc = wrapA(pNow.th - p0.th), thPrev = pNow.th, maxTh = thAcc, lastMove = sim.simTime, maxW = 0;
  S.samples.push([sim.simTime, p0.x, p0.y, 0, 0, 0]);
  run.ev("stap", "eerste commando ontvangen, meting loopt");
  run.setStep(square ? "Meting loopt: vierkant van 0,5 m" : "Meting loopt: 90° naar links", 2, 2);
  while (activeRun === run) {
    await sleepSim(run, 0.02);
    const p = sim.truthPose(); thAcc += wrapA(p.th - thPrev); thPrev = p.th; maxTh = Math.max(maxTh, thAcc);
    const fresh = rosCmd.t >= 0 && sim.simTime - rosCmd.t < 0.3;
    if (fresh) { S.lastRos = { v: rosCmd.v, w: rosCmd.w }; maxW = Math.max(maxW, Math.abs(rosCmd.w)); }
    if (keyCmd || padCmd) S.manual = true;
    S.samples.push([sim.simTime, p.x, p.y, thAcc, fresh ? rosCmd.v : 0, fresh ? rosCmd.w : 0]);
    run.step.big = square ? `gereden ${fmt(Math.hypot(p.x - p0.x, p.y - p0.y))} m van de start · gedraaid ${fmt(r2d(thAcc), 1)}°` : `gedraaid: ${fmt(r2d(thAcc), 1)}°  (doel 90,0° ± 2°)`;
    // stilstand = geen niet-nul commando én minder dan 2 mm / 0,2° beweging in de laatste seconde
    const cmdActive = fresh && (Math.abs(rosCmd.v) > 0.005 || Math.abs(rosCmd.w) > 0.005);
    const ref = S.samples.find(x => x[0] >= sim.simTime - 1.0) || S.samples[0];
    const moving = Math.hypot(p.x - ref[1], p.y - ref[2]) > 0.002 || Math.abs(thAcc - ref[3]) > 0.0035;
    if (moving || cmdActive) lastMove = sim.simTime;
    if (sim.simTime - lastMove > 1.5) break;
    if (sim.simTime - S.samples[0][0] > 120) { run.ev("stap", "time-out na 120 s"); break; }
  }
  if (activeRun !== run) return;
  const p1 = sim.truthPose();
  run.check("ros", !S.manual, S.manual ? "toetsenbord of rijknoppen gebruikt tijdens de meting" : `max |ω| ${fmt(maxW)} rad/s`);
  const zeroOk = S.lastRos && Math.abs(S.lastRos.v) < 1e-3 && Math.abs(S.lastRos.w) < 1e-3;
  run.check("zero", !!zeroOk, zeroOk ? "" : "laatste commando was niet nul: de echte robot zou blijven rijden (de simulator stopt na 0,5 s zonder commando)");
  run.ev("meting", `laatste /cmd_vel: v ${fmt(S.lastRos?.v ?? 0, 3)}, ω ${fmt(S.lastRos?.w ?? 0, 3)}`);
  if (!square) {
    const a = r2d(thAcc), shift = Math.hypot(p1.x - p0.x, p1.y - p0.y), ao = r2d(wrapA(sim.odom.th - o0.th));
    run.check("angle", Math.abs(a - 90) <= 2, `${fmt(a, 1)}° (overshoot max ${fmt(Math.max(0, r2d(maxTh) - a), 1)}°)`);
    run.check("spot", shift < 0.02, `${fmt(shift * 100, 1)} cm`);
    run.check("odom", true, `${fmt(ao, 1)}° (${sim.params.odomSource === "truth" ? "perfecte odometrie" : "wielencoders"})`);
    run.ev("meting", `werkelijk ${fmt(a, 2)}°, /odom ${fmt(ao, 2)}°, verplaatsing ${fmt(shift * 100, 1)} cm`);
    return;
  }
  // vierkant: hoeken = stukken waar de werkelijke yaw verandert; zijden = afstand tussen de draaipunten.
  // Robuust voor haperende commando's en voor trage uitloop van een P-regelaar.
  const sm = S.samples, N = sm.length, turning = new Array(N).fill(false);
  for (let i = 0; i < N; i++) {
    let j0 = i, j1 = i;
    while (j0 > 0 && sm[i][0] - sm[j0][0] < 0.2) j0--;
    while (j1 < N - 1 && sm[j1][0] - sm[i][0] < 0.2) j1++;
    const dt = sm[j1][0] - sm[j0][0];
    turning[i] = dt > 0 && Math.abs(sm[j1][3] - sm[j0][3]) / dt > 0.05;
  }
  const tsegs = [];
  for (let i = 0; i < N; i++) if (turning[i]) { const l = tsegs[tsegs.length - 1]; if (l && sm[i][0] - sm[l.b][0] < 0.5) l.b = i; else tsegs.push({ a: i, b: i }); }
  const median = arr => { const v = [...arr].sort((x, y) => x - y); return v.length ? v[Math.floor(v.length / 2)] : 0; };
  const plateau = (i0, i1) => median(sm.slice(Math.max(0, i0), Math.max(i0 + 1, i1)).map(x => x[3]));
  const turns = [], pts = [[sm[0][1], sm[0][2]]];
  tsegs.forEach((g, k) => {
    const before = plateau(k ? tsegs[k - 1].b + 1 : 0, g.a), after = plateau(g.b + 1, k < tsegs.length - 1 ? tsegs[k + 1].a : N);
    turns.push(r2d(after - before));
    const mid = sm.slice(g.a, g.b + 1); pts.push([mid.reduce((s2, x) => s2 + x[1], 0) / mid.length, mid.reduce((s2, x) => s2 + x[2], 0) / mid.length]);
  });
  pts.push([sm[N - 1][1], sm[N - 1][2]]);
  const sides = pts.slice(1).map((p, i) => Math.hypot(p[0] - pts[i][0], p[1] - pts[i][1]));
  const bigSides = sides.filter(l => l > 0.1), bigTurns = turns.filter(a => Math.abs(a) > 30);
  run.check("sides", bigSides.length >= 4 && bigSides.slice(0, 4).every(l => Math.abs(l - 0.5) <= 0.03), bigSides.map(l => fmt(l) + " m").join(", ") || "geen zijden herkend");
  run.check("turns", bigTurns.length >= 3 && bigTurns.every(a => Math.abs(Math.abs(a) - 90) <= 3), bigTurns.map(a => fmt(a, 1) + "°").join(", ") || "geen hoeken herkend");
  const dTrue = Math.hypot(p1.x - p0.x, p1.y - p0.y), dOdom = Math.hypot(sim.odom.x - o0.x, sim.odom.y - o0.y);
  run.check("close", true, `werkelijk ${fmt(dTrue * 100, 1)} cm, volgens /odom ${fmt(dOdom * 100, 1)} cm`);
  run.ev("meting", `zijden ${bigSides.map(l => fmt(l, 3)).join("/")} m, hoeken ${bigTurns.map(a => fmt(a, 1)).join("/")}°, eindafwijking ${fmt(dTrue * 100, 1)} cm`);
}

/* ---- starten, afronden, rapport ---- */
async function startTask() {
  const id = $("taskSel").value, def = TASKS[id];
  if (!student.name) { $("taskMsg").textContent = "Vul eerst je naam en studentnummer in bij Student."; return; }
  if (!live()) { $("taskMsg").textContent = "Verbind eerst met rosbridge."; return; }
  if (activeRun) return;
  lastResult = null; $("taskOut").hidden = true;
  const src = $("recSrc").value;
  if (src !== "none" && !recorder.mr) { const ok = await startRecording(src, def.label); if (!ok && src === "screen") return; }
  activeRun = new Run({ ...def, id });
  setTimeout(() => frameCameraFor(id), 50); $("pad").classList.add("testing");
  $("taskStart").disabled = true; $("taskStop").disabled = false; $("taskMsg").textContent = "Test loopt…"; renderChecks();
  activeRun.ev("start", `${def.label} · ${stuLabel()} · wereld ${sim.worldInfo.name}`);
  try { await def.run(activeRun); } catch (e) { activeRun?.ev("fout", e.message); console.error(e); }
  if (activeRun) finishRun("klaar");
}
async function finishRun(reason) {
  const run = activeRun; if (!run || run.done) return; run.done = true;
  try { run.def.cleanup?.(run); } catch (e) { console.error(e); }
  activeRun = null; run.invalidOverride = null;
  for (const c of run.checks) if (c.s === "wait" || c.s === "run") { c.s = "fail"; c.detail = c.detail || (reason === "gestopt" ? "test gestopt" : "niet uitgevoerd"); }
  run.ev("einde", reason);
  const passed = run.checks.filter(c => !c.optional).every(c => c.s === "ok");
  $("taskStart").disabled = false; $("taskStop").disabled = true;
  $("pad").classList.remove("testing");
  run.result = passed ? "GESLAAGD" : "NIET GESLAAGD"; resultRun = run; resultUntil = performance.now() + 8000;
  if (recorder.mr && recorder.label === run.def.label) await new Promise(r => setTimeout(r, 2000));   // eindbeeld mee in de opname
  const video = recorder.mr && recorder.label === run.def.label ? await stopRecording() : null;
  const stampName = new Date().toISOString().slice(0, 16).replace(/[-:T]/g, "");
  const base = `rapport_${(student.nr || student.name).replace(/\W+/g, "_")}_${run.def.id}_${stampName}`;
  const body = {
    formaat: "tb3-lite-sim-rapport/1", simulator: SIM_VERSION, taak: { id: run.def.id, naam: run.def.label },
    student: { naam: student.name, nr: student.nr, code: student.code, datum: student.date }, wereld: sim.worldInfo.name,
    instellingen: { odometrie: sim.params.odomSource, lidar_m: [sim.LIDAR.rmin, sim.LIDAR.rmax], geen_echo: invalidMode, ruis_mm: Math.round(sim.params.lidarSigma * 1000), cmd_vel: cmdState.subscribed || $("cmdType").value, batterij_topic: batTopic() },
    gestart: run.wall0.toISOString(), beeindigd: new Date().toISOString(), sim_tijd_s: [r4(run.t0), r4(sim.simTime)],
    studentnode: run.node || null, resultaat: passed ? "geslaagd" : "niet geslaagd",
    controles: run.checks.map(c => ({ controle: c.label, ok: c.s === "ok", detail: c.detail, ...(c.optional ? { info: true } : {}) })),
    opname: video ? { bestand: base + (video.mime.includes("mp4") ? ".mp4" : ".webm"), bron: video.source, bytes: video.bytes, seconden: video.seconds, sha256: video.sha256 } : null,
    verloop: run.events.slice(0, 400),
  };
  const inhoud = JSON.stringify(body);
  const controle = (await sha256hex(new TextEncoder().encode(STU_SALT + "|" + inhoud))).slice(0, 16);
  lastResult = { run, report: { rapport: body, inhoud, controle }, video, base };
  $("taskMsg").textContent = `${run.def.label}: ${body.resultaat.toUpperCase()} · controlecode ${controle}` + (video ? ` · opname ${video.seconds} s` : "");
  $("taskMsg").className = "msg " + (passed ? "ok" : "err");
  $("taskOut").hidden = false; $("dlVideo").hidden = !video; renderChecks();
}
function taskHow() { const d = TASKS[$("taskSel").value]; $("taskHow").textContent = d.how; }
$("taskSel").onchange = () => { if (!activeRun) { lastResult = null; renderChecks(); $("taskOut").hidden = true; $("taskMsg").textContent = ""; } taskHow(); };
$("taskStart").onclick = startTask;
$("taskStop").onclick = () => finishRun("gestopt");
$("dlReport").onclick = () => lastResult && saveBlob(new Blob([JSON.stringify(lastResult.report, null, 1)], { type: "application/json" }), lastResult.base + ".json");
$("dlVideo").onclick = () => lastResult?.video && saveBlob(lastResult.video.blob, lastResult.report.rapport.opname.bestand);
$("recToggle").onclick = async () => {
  if (recorder.mr) { const v = await stopRecording(); if (v) saveBlob(v.blob, `opname_${(student.nr || student.name || "student").replace(/\W+/g, "_")}_${new Date().toISOString().slice(0, 16).replace(/[-:T]/g, "")}.${v.mime.includes("mp4") ? "mp4" : "webm"}`); }
  else startRecording($("recSrc").value === "none" ? "sim" : $("recSrc").value, "vrije opname");
};
$("invalidAs").onchange = () => { invalidMode = $("invalidAs").value; };
taskHow();


/* ================= compatibiliteit met de echte labo-robot ================= */
/* /cmd_vel-type: rosbridge kan één topic maar met één type abonneren. In "automatisch" abonneert de simulator pas als
   er een publisher is (zonder rosbridge), en vraagt dan via rosapi het type op. Zonder publisher wordt het abonnement
   opgeheven, zodat het volgende script opnieuw herkend wordt (anders geeft rosapi het type van ons eigen abonnement). */
const CMD_TYPES = { "geometry_msgs/msg/Twist": "Twist", "geometry_msgs/msg/TwistStamped": "TwistStamped" };
const cmdState = { subscribed: "", detected: "", pubs: [], lastMsg: 0, since: 0, timer: null, busy: false, rosapi: true, fails: 0 };
const normType = t => { const m = String(t || "").match(/^([\w]+)\/(?:msg\/)?([\w]+)$/); return m ? `${m[1]}/msg/${m[2]}` : ""; };
const shortType = t => CMD_TYPES[t] || t || "?";
const batTopic = () => "/battery_state";   // zelfde topic als de labo-robot (28-10: /sim/battery_state is weg)
const otherNodes = list => (list || []).filter(n => !/rosbridge|rosapi/.test(n));
function cmdHint() {
  const sub = cmdState.subscribed;
  if (!sub) return "De simulator wacht nog op een publisher op /cmd_vel.";
  return `De simulator luistert als ${shortType(sub)}. Publiceert je script een ander type, zet dan /cmd_vel op automatisch.`;
}
function subscribeCmd(type) {
  if (cmdState.subscribed === type) return;
  if (cmdState.subscribed) send({ op: "unsubscribe", topic: "/cmd_vel" });
  send({ op: "subscribe", topic: "/cmd_vel", type, queue_length: 1 });
  cmdState.subscribed = type; cmdState.since = performance.now(); IN["/cmd_vel"][0] = type;
}
function unsubscribeCmd() {
  if (cmdState.subscribed) send({ op: "unsubscribe", topic: "/cmd_vel" });
  cmdState.subscribed = ""; IN["/cmd_vel"][0] = "";
}
function showCmdInfo(text, kind = "") { const el = $("cmdInfo"); el.className = "msg" + (kind ? " " + kind : ""); el.textContent = text; }
async function cmdProbe() {
  if (!live() || cmdState.busy) return; cmdState.busy = true;
  try {
    const mode = $("cmdType").value;
    const pr = await callService("/rosapi/publishers", { topic: "/cmd_vel" });
    if (!pr) {                                                           // rosapi niet bereikbaar
      if (++cmdState.fails >= 3 && mode === "auto" && !cmdState.subscribed) {
        subscribeCmd("geometry_msgs/msg/TwistStamped");
        showCmdInfo("rosapi niet bereikbaar: herkennen lukt niet, luistert als TwistStamped. Kies het type zelf als je script Twist publiceert.", "err");
      }
      return;
    }
    cmdState.fails = 0;
    const pubs = otherNodes(pr.publishers); cmdState.pubs = pubs;
    if (mode !== "auto") {
      subscribeCmd(mode);
      if (!pubs.length) { cmdState.detected = ""; showCmdInfo(`Luistert als ${shortType(mode)}. Nog geen publisher op /cmd_vel.`); return; }
      if (!cmdState.detected) { const tt = await callService("/rosapi/topic_type", { topic: "/cmd_vel" }); cmdState.detected = normType(tt?.type); }
      silentCheck(pubs, mode); return;
    }
    // automatisch
    if (!pubs.length) {
      if (cmdState.subscribed) unsubscribeCmd();
      cmdState.detected = "";
      showCmdInfo("Automatisch: wacht tot een script op /cmd_vel publiceert…"); return;
    }
    if (!cmdState.subscribed) {
      const tt = await callService("/rosapi/topic_type", { topic: "/cmd_vel" }), t = normType(tt?.type);
      if (!CMD_TYPES[t]) { showCmdInfo(`Onbekend type op /cmd_vel: ${tt?.type || "–"}. Verwacht Twist of TwistStamped.`, "err"); return; }
      cmdState.detected = t; subscribeCmd(t);
    }
    silentCheck(pubs, "auto");
  } finally { cmdState.busy = false; }
}
function silentCheck(pubs, mode) {
  const t = cmdState.subscribed, who = pubs.slice(0, 2).join(", ") + (pubs.length > 2 ? " …" : "");
  const quiet = performance.now() - Math.max(cmdState.lastMsg, cmdState.since) > 4000;
  const mism = mode !== "auto" && cmdState.detected && cmdState.detected !== t;
  const lab = t === "geometry_msgs/msg/TwistStamped" ? " · zelfde als de labo-robot" : " · labo-robot (Jazzy) verwacht TwistStamped";
  if (mism) showCmdInfo(`Je script publiceert waarschijnlijk ${shortType(cmdState.detected)}, de simulator luistert als ${shortType(t)}: er komt niets binnen. Zet /cmd_vel op automatisch of op ${shortType(cmdState.detected)}.`, "err");
  else if (quiet && cmdState.lastMsg < cmdState.since) showCmdInfo(`${shortType(t)} van ${who}${lab}. Nog geen berichten: publiceert je script misschien een ander type?`);
  else showCmdInfo(`${mode === "auto" ? "Herkend" : "Luistert als"}: ${shortType(t)} van ${who}${lab}.`, t === "geometry_msgs/msg/Twist" ? "ok" : "");
}
function cmdStart() {
  cmdStop(); cmdState.fails = 0; cmdState.detected = ""; cmdState.lastMsg = 0;
  cmdProbe(); cmdState.timer = setInterval(cmdProbe, 700);
}
function cmdStop() { if (cmdState.timer) clearInterval(cmdState.timer); cmdState.timer = null; cmdState.subscribed = ""; cmdState.pubs = []; }
$("cmdType").onchange = () => { if (live()) { unsubscribeCmd(); cmdStart(); } else showCmdInfo($("cmdType").value === "auto" ? "Type wordt herkend zodra je script op /cmd_vel publiceert." : `Luistert als ${shortType($("cmdType").value)} na het verbinden.`); };

/* ---- "Klaar voor de echte robot?": wat werkt in de simulator maar niet op de labo-robot (Jazzy, Zenoh) ---- */
const SIM_ONLY = { "/ground_truth": "bestaat alleen in de simulator (echte pose)", "/student_id": "bestaat alleen in de simulator" };
async function compatCheck() {
  const out = $("compatOut"); out.hidden = false;
  const items = [];
  const add = (s, text, note = "") => items.push(`<li data-s="${s}"><span>${text}</span>${note ? `<small>${note}</small>` : ""}</li>`);
  const render = () => { out.innerHTML = items.join(""); };
  if (!live()) { add("fail", "Niet verbonden met rosbridge", "Verbind eerst en start je script."); render(); return; }
  out.innerHTML = '<li data-s="run"><span>Controleren…</span></li>';
  const pr = await callService("/rosapi/publishers", { topic: "/cmd_vel" });
  if (!pr) { add("fail", "rosapi niet bereikbaar", "Start rosbridge met rosapi (standaard in rosbridge_websocket_launch.xml)."); render(); return; }
  const pubs = otherNodes(pr.publishers);
  if (!pubs.length) add("", "/cmd_vel: geen publisher", "Start je script en klik opnieuw om het type te controleren.");
  else {
    const t = cmdState.detected || cmdState.subscribed || normType((await callService("/rosapi/topic_type", { topic: "/cmd_vel" }))?.type);
    if (t === "geometry_msgs/msg/TwistStamped") add("ok", `/cmd_vel als TwistStamped (${pubs.join(", ")})`, "Zelfde als de labo-robot (Jazzy).");
    else add("fail", `/cmd_vel als ${shortType(t)} (${pubs.join(", ")})`, "De labo-robot draait Jazzy en verwacht geometry_msgs/msg/TwistStamped.");
  }
  for (const [tp, why] of Object.entries(SIM_ONLY)) {
    const r = await callService("/rosapi/subscribers", { topic: tp }), subs = otherNodes(r?.subscribers).filter(n => !SYSTEM_NODES.test(n.replace(/^\//, "")));
    if (subs.length) add("fail", `${subs.join(", ")} leest ${tp}`, why + ". Gebruik een parameter of remap voor de topicnaam.");
  }
  const ck = await callService("/rosapi/subscribers", { topic: "/clock" }), ckSubs = otherNodes(ck?.subscribers).filter(n => !SYSTEM_NODES.test(n.replace(/^\//, "")));
  if (ckSubs.length) add("", `${ckSubs.join(", ")} gebruikt simulatietijd (/clock)`, "Op de echte robot use_sim_time op false zetten.");
  if (!items.some(x => x.includes('data-s="fail"'))) add("ok", "Geen problemen gevonden voor de labo-robot", "Gecontroleerd: type van /cmd_vel en simulator-topics die je nodes lezen.");
  render();
}
$("compatBtn").onclick = compatCheck;
