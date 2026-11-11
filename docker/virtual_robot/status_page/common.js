// Shared helpers for the turtlebot status pages: rosbridge connection,
// connection-badge wiring, and the rate/age tracker used by several
// topic subscriptions. Loaded by both index.html and system.html.

function connectRos() {
  var host = window.location.hostname;
  // virtuele robot (http://localhost:8090): 127.0.0.1 i.p.v. localhost - onder WSL
  // lukte na een wifi-onderbreking tot ~400 s geen nieuwe verbinding met "localhost"
  var wsUrl = "ws://" + (host === "localhost" ? "127.0.0.1" : host) + ":9090";
  var wsUrlEl = document.getElementById("wsUrl");
  if (wsUrlEl) wsUrlEl.textContent = wsUrl;
  var nameEl = document.getElementById("robotName");
  if (nameEl) nameEl.textContent = host;

  var ros = new ROSLIB.Ros();

  function setConn(state, text) {
    var dot = document.getElementById("connDot");
    var txt = document.getElementById("connText");
    if (dot) dot.className = "dot " + state;
    if (txt) txt.textContent = text;
  }
  setConn("warn", "Verbinden...");
  ros.on("connection", function () { setConn("ok", "Verbonden"); });
  ros.on("close", function () { setConn("bad", "Verbinding verbroken"); });
  ros.on("error", function () { setConn("bad", "Fout"); });
  ros.fxUrl = wsUrl;   // for the automatic reconnect (initHeader, t80)
  ros.connect(wsUrl);
  return ros;
}

function timeAgo(ts) {
  var s = (Date.now() - ts) / 1000;
  if (s < 2) return "net nu";
  if (s < 60) return Math.round(s) + "s geleden";
  return Math.round(s / 60) + "m geleden";
}

function rateTracker(elHz, elAge) {
  var times = [];
  var hzEl = elHz ? document.getElementById(elHz) : null;
  var ageEl = elAge ? document.getElementById(elAge) : null;
  return function onMsg() {
    var now = Date.now();
    times.push(now);
    while (times.length > 10) times.shift();
    if (times.length > 1 && hzEl) {
      var dt = (times[times.length - 1] - times[0]) / 1000;
      var hz = dt > 0 ? (times.length - 1) / dt : 0;
      hzEl.textContent = hz.toFixed(1);
    }
    if (ageEl) ageEl.dataset.ts = now;
  };
}

function startAgeTicker(ids) {
  setInterval(function () {
    ids.forEach(function (id) {
      var el = document.getElementById(id);
      if (el && el.dataset.ts) el.textContent = timeAgo(parseInt(el.dataset.ts, 10));
    });
  }, 1000);
}

// Band, channel and channel width of the robot's wifi (system_info_node, 1.3.0+;
// width 1.4.0+), e.g. "5 GHz kanaal 36, 20 MHz"
function wifiChannel(d) {
  return d.wifi_band ? d.wifi_band + " kanaal " + d.wifi_channel + (d.wifi_width_mhz ? ", " + d.wifi_width_mhz + " MHz" : "") : "";
}

// Wifi signal from /system_info (wifi_signal_dbm, wifi_quality_percent).
// Thresholds: > -60 dBm good, -60..-70 fair, < -70 poor (video/teleop
// start to suffer below roughly -70 dBm).
function showWifi(el, d) {
  if (!el) return;
  var dbm = d.wifi_signal_dbm;
  if (dbm === undefined || dbm === null) {
    el.textContent = "--";
    el.style.color = "";
    return;
  }
  el.textContent = dbm + " dBm" +
    (d.wifi_quality_percent !== null && d.wifi_quality_percent !== undefined
      ? " (" + d.wifi_quality_percent + "%)" : "") + (wifiChannel(d) ? ", " + wifiChannel(d) : "");
  el.style.color = dbm > -60 ? "var(--green)" : (dbm >= -70 ? "var(--yellow)" : "var(--red)");
}

// Pi health from /system_info (cpu_temp_c, throttled = vcgencmd
// get_throttled bitmask). Returns {level: "ok"|"warn"|"bad"|"", text}.
// Undervoltage NOW is the classic cause of random crashes/SD corruption on
// a battery-powered Pi; "since boot" bits mean it happened at some point.
function piPower(d) {
  var t = d.throttled;
  if (t === undefined || t === null) return { level: "", text: "--" };
  if (t & 0x1) return { level: "bad", text: "ONDERSPANNING" };
  if (t & 0x6) return { level: "bad", text: "afgeremd (throttled)" };
  if (t & 0x10000) return { level: "warn", text: "onderspanning gehad sinds opstart" };
  if (t & 0x60000) return { level: "warn", text: "afgeremd geweest sinds opstart" };
  return { level: "ok", text: "OK" };
}

function piTemp(d) {
  var c = d.cpu_temp_c;
  if (c === undefined || c === null) return { level: "", text: "--" };
  return { level: c < 70 ? "ok" : (c < 80 ? "warn" : "bad"), text: c.toFixed(0) + " °C" };
}

// Fan from /system_info: off below ~50 °C is normal. Driven (pwm > 0) but
// 0 rpm means it isn't connected or is stuck.
function fanState(d) {
  var f = d.fan;
  if (!f) return { level: "", text: "geen ventilator-aansluiting" };
  if (f.pwm_percent > 0 && f.rpm === 0) return { level: "bad", text: "aangestuurd maar draait niet" };
  if (f.rpm === 0) return { level: "ok", text: "uit (koel genoeg)" };
  return {
    level: "ok",
    text: f.rpm + " tpm (" + f.pwm_percent + "%" +
      (f.level !== null ? ", stand " + f.level + "/" + f.max_level : "") + ")",
  };
}

function diskState(d) {
  if (d.disk_free_gb === undefined || d.disk_free_gb === null) return { level: "", text: "--" };
  var f = d.disk_free_gb;
  return {
    level: f > 2 ? "ok" : (f > 1 ? "warn" : "bad"),
    text: f.toFixed(1) + " GB vrij van " + d.disk_total_gb.toFixed(0) + " GB",
  };
}

var LEVEL_COLOR = { ok: "var(--green)", warn: "var(--yellow)", bad: "var(--red)", "": "" };

// Copy to clipboard. navigator.clipboard only exists on https/localhost -
// this page is served over plain http on the robot's IP, so fall back to
// the old select + execCommand trick.
function copyText(text) {
  if (navigator.clipboard && window.isSecureContext) {
    return navigator.clipboard.writeText(text);
  }
  var ta = document.createElement("textarea");
  ta.value = text;
  ta.style.position = "fixed";
  ta.style.opacity = "0";
  document.body.appendChild(ta);
  ta.select();
  var ok = false;
  try { ok = document.execCommand("copy"); } catch (e) {}
  document.body.removeChild(ta);
  return ok ? Promise.resolve() : Promise.reject();
}

// localStorage can be unavailable (private window, blocked storage).
function lsGet(key, fallback) {
  try { var v = localStorage.getItem(key); return v === null ? fallback : v; } catch (e) { return fallback; }
}
function lsSet(key, value) {
  try { localStorage.setItem(key, value); } catch (e) {}
}

// ---- Diagnostics from /system_info (shared by labo-check and system.html) ----

// Robot clock vs this browser's clock. Matters for ROS: rviz/TF on a laptop
// drop messages whose timestamps are too far off ("message filter dropping").
function clockState(d) {
  if (!d.time_unix) return { level: "", text: "--" };
  // Compare with the browser clock at the moment the message ARRIVED
  // (d._recvMs, set by the pages), not "now": the labo-check re-renders
  // every second with a /system_info message that can be up to 2 s old.
  var diff = d.time_unix - (d._recvMs || Date.now()) / 1000;  // + = robot ahead
  var abs = Math.abs(diff);
  var level = abs < 0.5 ? "ok" : (abs < 2 ? "warn" : "bad");
  var ntp = d.ntp || {};
  var ntpText = ntp.synced === true ? "NTP OK" + (ntp.server ? " (" + ntp.server + ")" : "")
    : (ntp.synced === false ? "NTP niet gesynchroniseerd" : "NTP onbekend");
  if (ntp.synced === false && level === "ok") level = "warn";
  return { level: level, text: (diff >= 0 ? "+" : "") + diff.toFixed(2) + " s t.o.v. deze laptop, " + ntpText };
}

// Ping to the robot's own access point: jitter was the symptom of last
// year's shared-wifi problem.
function pingState(d) {
  var p = d.ping;
  if (!p || p.avg_ms === null || p.avg_ms === undefined) {
    return { level: p && p.loss_percent === 100 ? "bad" : "", text: p && p.loss_percent === 100 ? "AP niet bereikbaar" : "--" };
  }
  var level = (p.loss_percent > 20 || p.jitter_ms > 30) ? "bad"
    : ((p.loss_percent > 0 || p.jitter_ms > 10) ? "warn" : "ok");
  return { level: level, text: p.avg_ms.toFixed(1) + " ms, jitter " + p.jitter_ms.toFixed(1) + " ms" +
    (p.loss_percent > 0 ? ", " + p.loss_percent + "% verlies" : "") };
}

var SERVICE_LABEL = { zenoh: "Zenoh", rosbridge: "rosbridge", webpagina: "web", terminal: "terminal", launch_control: "launch control", twist_mux: "twist_mux", cmd_vel_guard: "cmd_vel_guard", nav_web: "nav-web", system_info: "systeeminfo" };
function servicesState(d) {
  var s = d.services;
  if (!s) return { level: "", text: "--", down: [] };
  var down = Object.keys(s).filter(function (k) { return s[k] === false; });
  // services_supervisor.sh herstartte een gecrashte dienst (sinds de containerstart)
  var sup = d.supervisor || {};
  var restarted = Object.keys(sup).map(function (k) {
    var t = sup[k].last ? new Date(sup[k].last * 1000).toLocaleTimeString("nl-BE", { hour: "2-digit", minute: "2-digit" }) : "";
    return (SERVICE_LABEL[k] || k) + " " + sup[k].restarts + "x" + (t ? " (laatst " + t + ")" : "");
  });
  var text = down.length ? "niet actief: " + down.map(function (k) { return SERVICE_LABEL[k] || k; }).join(", ") : "alle diensten actief";
  if (restarted.length) text += " - herstart na crash: " + restarted.join(", ");
  return {
    level: down.length ? "bad" : (restarted.length ? "warn" : "ok"),
    text: text,
    down: down,
  };
}

function shutdownState(d) {
  var h = d.host;
  if (!h || h.prev_shutdown_clean === null || h.prev_shutdown_clean === undefined) return { level: "", text: "onbekend" };
  var when = h.prev_boot_end ? " (" + new Date(h.prev_boot_end).toLocaleString("nl-BE") + ")" : "";
  return h.prev_shutdown_clean
    ? { level: "ok", text: "netjes afgesloten" + when }
    : { level: "warn", text: "onverwacht - stroom weg / batterij uitgetrokken?" + when };
}

function sdErrorState(d) {
  var e = d.sd_errors;
  if (!e || e.count === null || e.count === undefined) return { level: "", text: "--" };
  return e.count ? { level: "bad", text: e.count + " fout(en) sinds opstart" } : { level: "ok", text: "geen fouten" };
}

// Internet via the AP's WAN uplink (/system_info "internet", checked every ~15 s)
function internetState(d) {
  var i = d.internet;
  if (!i) return { level: "", text: "--", short: "--" };
  if (i.online) return { level: "ok", text: "ja", short: "ja" };
  if (i.ip) return { level: "warn", text: "geen DNS", short: "geen DNS" };
  return { level: "", text: "nee", short: "nee" };
}

// Tooltip with the details behind the internet icon
function internetDetail(d) {
  var i = d.internet;
  if (!i) return "Internet: nog niet gecontroleerd";
  var when = i.checked ? new Date(i.checked * 1000).toLocaleTimeString("nl-BE") : "--";
  var lines = ["Internet via de WAN-poort van de access point"];
  if (i.online) lines.push("Verbonden - " + i.latency_ms + " ms naar 1.1.1.1, DNS OK");
  else if (i.ip) lines.push("Verbinding OK (" + i.latency_ms + " ms), maar DNS werkt niet - namen opzoeken lukt niet");
  else lines.push("Geen internet - de robot werkt gewoon, enkel updates zijn niet mogelijk");
  lines.push("Gecontroleerd om " + when + " (elke 15 s)");
  return lines.join("\n");
}

// ---- Versions / update check ----
var IMAGE_REPO_URL = "https://github.com/JeremyLebon/turtlebot_docker";

function escHtml(t) {
  return String(t).replace(/[&<>"]/g, function (c) {
    return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c];
  });
}

// <a> to a commit on GitHub, showing the short hash
function commitLink(repoUrl, hash) {
  // No links to GitHub on pages students see (2026-10-10: the setup repo is
  // public and not meant for students); just the short hash. Links come back
  // in the PIN-protected Beheer page.
  return escHtml(hash ? hash.substring(0, 7) : "--");
}

// Update hint from /system_info "updates" (checked on the robot every 30 min)
function updateState(d) {
  var u = d.updates;
  if (!u) return { level: "", text: "nog niet gecontroleerd" };
  var parts = [];
  if (u.setup_behind) parts.push("nieuwe configuratie (setup-repo)");
  if (u.image_behind) parts.push("nieuwe Docker-image");
  var when = u.checked ? " (gecontroleerd " + new Date(u.checked * 1000).toLocaleTimeString("nl-BE", { hour: "2-digit", minute: "2-digit" }) + ")" : "";
  if (parts.length) return { level: "warn", text: "Update beschikbaar: " + parts.join(" + ") + when };
  if (d.internet && !d.internet.online)
    return { level: "", text: "geen internet" };
  if (u.error && u.setup_behind === null && u.image_behind === null)
    return { level: "", text: "kon niet controleren (geen internet?)" + when };
  return { level: "ok", text: "up-to-date" + when };
}

// ---- Shared header (Fenix, all pages) ----
// Every page has an empty <header id="appHeader"></header>; initHeader(ros,
// page) builds: logo + robot + version, page tabs, status icons (battery,
// wifi, Pi, internet, labo-check, aanpassingen), terminal, update hint,
// power menu and STOP, plus a row with the running launches.
// Tooltips: data-tip (CSS, also on tap/focus for phones), no title=.
var FENIX_ICONS = {
  index: '<path d="M6 12h4M8 10v4"/><circle cx="16" cy="11" r="1"/><circle cx="18" cy="13" r="1"/><rect x="2" y="6" width="20" height="12" rx="6"/>',
  nav: '<path d="M3 6l6-3 6 3 6-3v15l-6 3-6-3-6 3z"/><path d="M9 3v15M15 6v15"/>',
  projects: '<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/><rect x="3" y="14" width="7" height="7" rx="1"/><rect x="14" y="14" width="7" height="7" rx="1"/>',
  info: '<circle cx="12" cy="12" r="9"/><path d="M12 11v5M12 8h.01"/>',
  status: '<path d="M3 12h4l3-7 4 14 3-7h4"/>',
  maps: '<path d="M3 6l6-2 6 2 6-2v14l-6 2-6-2-6 2z"/><path d="M9 4v14M15 6v14"/>',
  ros: '<circle cx="6" cy="6" r="2.5"/><circle cx="18" cy="6" r="2.5"/><circle cx="12" cy="18" r="2.5"/><path d="M8.5 6h7M7.2 8.2l3.6 7.6M16.8 8.2l-3.6 7.6"/>',
  beheer: '<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V7a4 4 0 0 1 8 0v4"/>',
  unlocked: '<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V7a4 4 0 0 1 7.5-2"/>',
  system: '<circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.7 1.7 0 0 0 .3 1.8l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-1.8-.3 1.7 1.7 0 0 0-1 1.5V21a2 2 0 1 1-4 0v-.1a1.7 1.7 0 0 0-1.1-1.5 1.7 1.7 0 0 0-1.8.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.7 1.7 0 0 0 .3-1.8 1.7 1.7 0 0 0-1.5-1H3a2 2 0 1 1 0-4h.1a1.7 1.7 0 0 0 1.5-1.1 1.7 1.7 0 0 0-.3-1.8l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.7 1.7 0 0 0 1.8.3H9a1.7 1.7 0 0 0 1-1.5V3a2 2 0 1 1 4 0v.1a1.7 1.7 0 0 0 1 1.5 1.7 1.7 0 0 0 1.8-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.7 1.7 0 0 0-.3 1.8V9a1.7 1.7 0 0 0 1.5 1H21a2 2 0 1 1 0 4h-.1a1.7 1.7 0 0 0-1.5 1z"/>',
  labo: '<path d="M9 11l3 3 8-8"/><path d="M20 12v7a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11"/>',
  terminal: '<path d="M4 17l6-6-6-6M12 19h8"/>',
  update: '<path d="M12 3v12M7 10l5 5 5-5M5 21h14"/>',
  power: '<path d="M12 3v9"/><path d="M6.3 6.3a8 8 0 1 0 11.4 0"/>',
  reboot: '<path d="M3 12a9 9 0 1 0 3-6.7L3 8"/><path d="M3 3v5h5"/>',
  stop: '<path d="M8 2h8l6 6v8l-6 6H8l-6-6V8z"/><path d="M9 9h6v6H9z"/>',
  x: '<path d="M6 6l12 12M18 6L6 18"/>',
  conn: '<path d="M9 2v6M15 2v6M6 8h12v4a6 6 0 0 1-12 0zM12 18v4"/>',
  play: '<path d="M7 4l13 8-13 8z" fill="currentColor"/>',
  square: '<rect x="6" y="6" width="12" height="12" rx="1.5" fill="currentColor"/>',
  save: '<path d="M5 3h11l3 3v15H5z"/><path d="M8 3v5h7V3M8 21v-7h8v7"/>',
  camera: '<path d="M3 7h4l2-3h6l2 3h4v13H3z"/><circle cx="12" cy="13" r="4"/>',
  sensor: '<path d="M14 14.8V4a2 2 0 0 0-4 0v10.8a4 4 0 1 0 4 0z"/><path d="M12 9v8M18 6a5 5 0 0 1 0 6M20.5 3.5a9 9 0 0 1 0 11"/>',
  gamepad: '<path d="M6 11h4M8 9v4M15 12h.01M18 10h.01"/><path d="M17.3 5H6.7a4 4 0 0 0-4 3.6L2 15a3 3 0 0 0 5.5 1.7L9 15h6l1.5 1.7A3 3 0 0 0 22 15l-.7-6.4A4 4 0 0 0 17.3 5z"/>',
  bolt: '<path d="M13 2L4 14h7l-1 8 9-12h-7z"/>',
  fwd: '<path d="M5 12h14M13 6l6 6-6 6"/>',
  turn: '<path d="M21 12a9 9 0 1 1-3-6.7"/><path d="M21 4v5h-5"/>',
  up: '<path d="M12 19V5M5 12l7-7 7 7"/>',
  down: '<path d="M12 5v14M5 12l7 7 7-7"/>',
  left: '<path d="M9 14L4 9l5-5"/><path d="M4 9h10a6 6 0 0 1 0 12h-3"/>',
  right: '<path d="M15 14l5-5-5-5"/><path d="M20 9H10a6 6 0 0 0 0 12h3"/>',
  eye: '<path d="M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>',
  crosshair: '<circle cx="12" cy="12" r="3"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/>',
  flag: '<path d="M5 21V4h11l-2 4 2 4H5"/>',
  route: '<circle cx="6" cy="18" r="2"/><circle cx="18" cy="6" r="2"/><path d="M8 18h6a4 4 0 0 0 0-8h-4a4 4 0 0 1 0-8h6"/>',
  zones: '<path d="M4 4h16v16H4z" stroke-dasharray="3 3"/><path d="M8 8l8 8M16 8l-8 8"/>',
  pencil: '<path d="M12 20h9"/><path d="M16.5 3.5a2.1 2.1 0 0 1 3 3L7 19l-4 1 1-4z"/>',
  pause: '<rect x="6" y="5" width="4" height="14" fill="currentColor"/><rect x="14" y="5" width="4" height="14" fill="currentColor"/>',
  undo: '<path d="M9 14L4 9l5-5"/><path d="M4 9h10a6 6 0 0 1 0 12h-3"/>',
  trash: '<path d="M4 7h16M10 11v6M14 11v6M6 7l1 13h10l1-13M9 7V4h6v3"/>',
  check: '<path d="M5 12l5 5L20 7"/>',
  copy: '<rect x="9" y="9" width="12" height="12" rx="2"/><path d="M5 15V5a2 2 0 0 1 2-2h10"/>',
  rename: '<path d="M4 7V4h16v3M9 20h6M12 4v16"/>',
  export: '<path d="M12 15V3M7 8l5-5 5 5M5 21h14"/>',
  import: '<path d="M12 3v12M7 10l5 5 5-5M5 21h14"/>',
  loop: '<path d="M17 2l4 4-4 4"/><path d="M3 11V9a3 3 0 0 1 3-3h15M7 22l-4-4 4-4"/><path d="M21 13v2a3 3 0 0 1-3 3H3"/>',
  wall: '<rect x="4" y="4" width="16" height="16" rx="2" fill="currentColor"/>',
  eraser: '<path d="M20 20H8l-5-5 10-10 8 8-6 6"/><path d="M6 12l6 6"/>',
  unknown: '<circle cx="12" cy="12" r="9"/><path d="M9.5 9a2.5 2.5 0 1 1 3.5 2.3c-.7.3-1 .9-1 1.7M12 17h.01"/>',
  plus: '<path d="M12 5v14M5 12h14"/>',
  minus: '<path d="M5 12h14"/>',
  fit: '<path d="M4 9V4h5M15 4h5v5M20 15v5h-5M9 20H4v-5"/>',
  ruler: '<path d="M3 17L17 3l4 4L7 21z"/><path d="M7 13l2 2M10 10l2 2M13 7l2 2"/>',
  localcm: '<rect x="6" y="6" width="12" height="12" rx="1"/><path d="M6 12h12M12 6v12"/><circle cx="12" cy="12" r="1.5" fill="currentColor"/>',
  globalcm: '<rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M3 15h18M9 3v18M15 3v18"/>',
  particles: '<path d="M5 7l2-1M10 5l2 1M16 6l2-1M6 13l2 1M12 11l2-1M17 13l2 1M8 18l2-1M14 18l2 1"/>',
  scan: '<circle cx="12" cy="12" r="2" fill="currentColor"/><path d="M4 8l1 1M20 8l-1 1M3 14h1.5M19.5 14H21M7 4.5l.8 1.3M17 4.5l-.8 1.3M12 3v1.5"/>',
  lidar: '<circle cx="12" cy="12" r="2"/><path d="M5 5a10 10 0 0 0 0 14M19 5a10 10 0 0 1 0 14M8 8a5 5 0 0 0 0 8M16 8a5 5 0 0 1 0 8"/>',
};
// Elements with data-icon="name" get that icon in front of their text, which
// goes into <span class="lbl"> (hidden in toolbars on phones; data-tip stays).
function fenixIcons(root) {
  (root || document).querySelectorAll("[data-icon]").forEach(function (el) {
    if (el.dataset.iconDone) return;
    el.dataset.iconDone = "1";
    var txt = el.innerHTML.trim();
    el.innerHTML = fxIcon(el.dataset.icon, 16) + (txt ? '<span class="lbl">' + txt + "</span>" : "");
    el.classList.add("fx-ib");
  });
}
function fxIcon(name, size, sw) {
  return '<svg width="' + (size || 18) + '" height="' + (size || 18) + '" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="' +
    (sw || 2) + '" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">' + FENIX_ICONS[name] + "</svg>";
}
var FENIX_LOGO =
  '<svg width="30" height="30" viewBox="0 0 32 32" aria-hidden="true">' +
  '<path d="M16 29C10.5 25 5.5 20 3 9.5c5 3 8.5 5.5 10.6 9.1C12.6 13 13.6 7.5 16 3c2.4 4.5 3.4 10 2.4 15.6C20.5 15 24 12.5 29 9.5 26.5 20 21.5 25 16 29z" fill="#f0883e"/>' +
  '<path d="M16 25.5c-2.4-2.5-3.4-5-2.8-8 1.2 1.5 2.1 2 2.8 2s1.6-.5 2.8-2c.6 3-.4 5.5-2.8 8z" fill="#ffd8a8"/>' +
  '<circle cx="16" cy="9.5" r="1.6" fill="#14161a"/></svg>';

var PAGES = [["index", "index.html", "Rijden", "Rijden: besturen en camera"],
             ["maps", "nav.html#mappen", "Mappen", "Mappen: kaart maken, kaarten beheren, bewerken, zones"],
             ["nav", "nav.html", "Navigatie", "Navigatie: beginpositie, doelen, routes"],
             ["projects", "projects.html", "Projecten", "Projecten: kant-en-klare demo's"],
             ["status", "system.html#status", "Status", "Status: gezondheid, labo-check, smartphone"],
             ["ros", "system.html#ros", "ROS", "ROS: nodes, logs, verbinden vanaf je laptop"],
             ["beheer", "system.html#beheer", "Beheer", "Beheer: enkel voor de docent (PIN)"]];

// ---- Docenten-PIN (Beheer) ----
// The PIN is checked by status_page_server.py (fenix_admin.py, hash in
// state/admin_pin); the page only keeps the session token. A threshold
// against accidents. Since 0.21.0 the Beheer actions themselves also need the
// token (fenixAdminAction); reboot/shutdown and STOP stay open for students.
// body.fx-admin = unlocked; elements with class admin-only only show then.
var FENIX_ADMIN_KEY = "fenixAdminToken";
var fenixAdmin = { pin_set: null, unlocked: false };
function fenixAdminToken() {
  try { return sessionStorage.getItem(FENIX_ADMIN_KEY) || ""; } catch (e) { return ""; }
}
function fenixAdminApply(j) {
  fenixAdmin = j;
  document.body.classList.toggle("fx-admin", !!j.unlocked);
  var b = document.getElementById("fxAdmin");
  if (b) b.style.display = j.unlocked ? "" : "none";
  document.dispatchEvent(new CustomEvent("fenix-admin", { detail: j }));
}
function fenixAdminRefresh(cb) {
  fetch("/api/admin/status?token=" + encodeURIComponent(fenixAdminToken()), { cache: "no-store" })
    .then(function (r) { return r.json(); })
    .then(function (j) {
      if (!j.unlocked) { try { sessionStorage.removeItem(FENIX_ADMIN_KEY); } catch (e) {} }
      fenixAdminApply(j);
      if (cb) cb(j);
    }).catch(function () { if (cb) cb(fenixAdmin); });
}
function fenixAdminUnlock(pin, cb) {
  fetch("/api/admin/unlock", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ pin: pin }) })
    .then(function (r) { return r.json().then(function (j) { return [r.status, j]; }); })
    .then(function (a) {
      if (a[0] === 200 && a[1].token) {
        try { sessionStorage.setItem(FENIX_ADMIN_KEY, a[1].token); } catch (e) {}
        fenixAdminRefresh(function () { cb(true, ""); });
      } else cb(false, a[1].error || "geweigerd");
    }).catch(function (e) { cb(false, String(e)); });
}
function fenixAdminLock() {
  var t = fenixAdminToken();
  try { sessionStorage.removeItem(FENIX_ADMIN_KEY); } catch (e) {}
  fetch("/api/admin/lock", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ token: t }) })
    .finally(function () { fenixAdminRefresh(); });
}
// Beheer action (update, rollback, desktop, USB, camera, speed_limit, ...):
// over http with the PIN token, not as a ROS service - the robot checks the
// token (status_page_server.py), so ROS on a student laptop can't do these.
// cb(success, message, full reply)
function fenixAdminAction(action, args, cb) {
  fetch("/api/admin/action", { method: "POST", cache: "no-store", headers: { "Content-Type": "application/json" },
                               body: JSON.stringify({ token: fenixAdminToken(), action: action, args: args || {} }) })
    .then(function (r) {
      if (r.status === 401) fenixAdminRefresh();
      return r.json();
    })
    .then(function (j) { if (cb) cb(!!j.success, j.message || "", j); })
    .catch(function (e) { if (cb) cb(false, "robot niet bereikbaar (" + e + ")"); });
}
// system.html is Status (#status, old links #info), ROS (#ros) and Beheer
// (#beheer): active tab follows the hash
var fenixTitlePage = "";
function fenixPageId(page) {
  if (page === "nav") return location.hash === "#mappen" ? "maps" : "nav";   // nav.html = Mappen + Navigatie
  if (page !== "system") return page;
  return location.hash === "#beheer" ? "beheer" : location.hash === "#ros" ? "ros" : "status";
}
function fenixSetActiveTab(page) {
  var id = fenixPageId(page);
  document.querySelectorAll("#pageTabs a").forEach(function (a) {
    var on = a.dataset.page === id;
    a.classList.toggle("active", on);
    if (on) a.setAttribute("aria-current", "page"); else a.removeAttribute("aria-current");
  });
  fenixTitlePage = (PAGES.filter(function (p) { return p[0] === id; })[0] || ["", "", ""])[2];
}
var HEADER_CSS =
  "body.fx-offline .dpad button,body.fx-offline #navPad button{opacity:.35;pointer-events:none}" +
  "#appHeader{display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:10px;margin-bottom:8px}" +
  ".fx-left{display:flex;flex-wrap:wrap;align-items:center;gap:10px}" +
  ".fx-brand{display:flex;align-items:center;gap:8px;color:var(--text);text-decoration:none;font-weight:800;font-size:1.1rem;letter-spacing:.01em}" +
  ".fx-bname{display:flex;flex-direction:column;line-height:1.05}" +
  ".fx-tag{font-size:.62rem;font-weight:500;font-style:italic;color:#f0883e;letter-spacing:.02em;margin-top:2px}" +
  ".fx-robot{font-size:.75rem;font-weight:700;color:#cfe5ff;background:#1f3a5c;border:1px solid #2f5f92;border-radius:999px;padding:2px 10px;cursor:default}" +
  ".fx-ver{font-size:.7rem;color:var(--muted);background:var(--card);border:1px solid var(--border);border-radius:999px;padding:2px 8px;" +
  "font-family:ui-monospace,SFMono-Regular,Menlo,monospace;cursor:default}" +
  ".tabs{display:flex;flex-wrap:wrap;gap:4px;background:var(--card);border:1px solid var(--border);border-radius:999px;padding:3px}" +
  ".tabs a{display:flex;align-items:center;gap:6px;color:var(--muted);text-decoration:none;font-size:.8rem;font-weight:600;padding:6px 12px;border-radius:999px;min-height:24px}" +
  ".tabs a:hover{color:var(--text)}.tabs a.active{background:var(--accent);color:#0b1a2b}" +
  ".fx-right{display:flex;flex-wrap:wrap;align-items:center;gap:8px}" +
  ".hdr-icons{display:flex;gap:2px;align-items:center;background:var(--card);border:1px solid var(--border);border-radius:999px;padding:2px 8px}" +
  ".hdr-icon{display:inline-flex;align-items:center;gap:4px;font-size:.78rem;font-weight:600;color:var(--muted);cursor:default;padding:6px 5px;border-radius:8px}" +
  ".hdr-icon svg{display:block}" +
  ".fx-round{position:relative;display:inline-flex;align-items:center;justify-content:center;width:40px;height:40px;border-radius:999px;" +
  "background:var(--card);border:1px solid var(--border);color:var(--text);cursor:pointer;padding:0;text-decoration:none}" +
  ".fx-round:hover{border-color:var(--accent)}" +
  ".fx-dot{position:absolute;top:6px;right:7px;width:8px;height:8px;border-radius:99px;background:var(--yellow);border:2px solid var(--card)}" +
  ".fx-stop{display:inline-flex;align-items:center;gap:8px;background:#da3633;border:0;color:#fff;font:inherit;font-weight:800;" +
  "letter-spacing:.04em;padding:0 16px;height:40px;border-radius:999px;cursor:pointer}" +
  ".fx-stop:hover{background:#f85149}" +
  ".fx-menu{position:absolute;right:0;top:46px;z-index:60;width:260px;background:var(--card);border:1px solid var(--border);" +
  "border-radius:12px;padding:6px;box-shadow:0 12px 30px #0009}" +
  ".fx-mi{display:flex;align-items:center;gap:10px;width:100%;background:none;border:0;color:var(--text);font:inherit;font-size:.85rem;" +
  "text-align:left;padding:10px;border-radius:8px;cursor:pointer}" +
  ".fx-mi:hover{background:#262a31}.fx-mi small{display:block;color:var(--muted);font-size:.75rem}" +
  ".fx-chips{display:flex;flex-wrap:wrap;align-items:center;gap:6px;margin:0 0 12px;font-size:.75rem;color:var(--muted);min-height:26px}" +
  ".fx-toasts{position:fixed;right:16px;bottom:16px;z-index:60;display:flex;flex-direction:column;gap:8px;max-width:360px;width:calc(100% - 32px)}" +
  ".fx-toast{display:flex;gap:10px;align-items:flex-start;background:var(--card, #1e2126);border:1px solid var(--border, #2c3038);" +
  "border-left:4px solid var(--yellow, #f1c40f);border-radius:8px;padding:10px 10px 10px 12px;font-size:.84rem;color:var(--text, #e6e8eb);" +
  "box-shadow:0 6px 20px rgba(0,0,0,.45)}" +
  ".fx-toast.bad{border-left-color:var(--red, #e74c3c)}.fx-toast b{display:block;margin-bottom:2px}" +
  ".fx-toast small{color:var(--muted, #8a8f98);display:block;margin-top:3px;word-break:break-word}" +
  ".fx-toast span{flex:1;min-width:0}.fx-toast button{flex:none;background:none;border:0;color:var(--muted, #8a8f98);cursor:pointer;padding:2px}" +
  ".fx-state{display:inline-flex;align-items:center;gap:6px;border-radius:999px;padding:3px 10px;font-weight:600;font-size:.78rem;" +
  "border:1px solid var(--border, #2c3038);color:var(--text, #e6edf3);background:var(--card, #161b22);margin-right:6px}" +
  ".fx-state i{width:8px;height:8px;border-radius:99px;background:var(--muted, #8b949e);flex:none}" +
  ".fx-state.manual i{background:var(--green, #3fb950)}.fx-state.auto i{background:var(--accent, #4da3ff)}" +
  ".fx-state.error{border-color:var(--red, #f85149)}.fx-state.error i{background:var(--red, #f85149)}" +
  ".fx-state.moving i{animation:fxPulse 1s ease-in-out infinite}@keyframes fxPulse{50%{opacity:.35}}" +
  ".fx-chip{display:inline-flex;align-items:center;gap:6px;background:#1f3a5c;border:1px solid #2f5f92;color:#cfe5ff;border-radius:999px;" +
  "padding:2px 3px 2px 10px;font-weight:600}" +
  ".fx-chip button{display:inline-flex;align-items:center;justify-content:center;width:22px;height:22px;border-radius:99px;border:0;" +
  "background:#2f5f92;color:#fff;cursor:pointer;padding:0}" +
  ".fx-pop{display:none;position:absolute;z-index:60;right:16px;width:min(440px,calc(100vw - 32px));background:var(--card);" +
  "border:1px solid var(--border);border-radius:10px;padding:10px 12px;box-shadow:0 8px 24px #0008;font-size:.85rem;box-sizing:border-box}" +
  ".fx-pop ul{list-style:none;margin:6px 0 0;padding:0}.fx-pop li{display:flex;gap:8px;align-items:baseline;padding:3px 0}" +
  ".fx-pop li .n{flex:none;width:130px;color:var(--muted)}.fx-pop .ld{flex:none;width:8px;height:8px;border-radius:50%;background:var(--muted);transform:translateY(-1px)}" +
  ".fx-pop .ld.ok{background:var(--green)}.fx-pop .ld.warn{background:var(--yellow)}.fx-pop .ld.bad{background:var(--red)}" +
  "body:not(.fx-admin) .admin-only{display:none!important}" +
  ".fx-powered{text-align:center;color:var(--muted);font-size:.7rem;letter-spacing:.06em;margin:18px 0 6px;text-transform:uppercase}" +
  ".fx-powered a{color:var(--muted);text-decoration:none;font-weight:700}.fx-powered a:hover{color:var(--accent)}" +
  ".fx-ib{display:inline-flex;align-items:center;gap:6px}.fx-ib svg{flex:none}" +
  "@media (max-width:760px){.toolbar .fx-ib .lbl{display:none}}" +
  "[data-tip]{position:relative}" +
  "[data-tip]:hover::after,[data-tip]:focus-visible::after{content:attr(data-tip);position:absolute;left:50%;top:calc(100% + 6px);" +
  "transform:translateX(-50%);background:#0b0d10;color:#e6e8eb;border:1px solid #3a404a;padding:5px 9px;border-radius:7px;font-size:12px;" +
  "font-weight:500;line-height:1.35;white-space:pre-line;width:max-content;max-width:280px;z-index:70;pointer-events:none;text-align:left;letter-spacing:0}" +
  ".fx-right [data-tip]:hover::after,.fx-right [data-tip]:focus-visible::after{left:auto;right:0;transform:none}" +
  ".tip-r[data-tip]:hover::after,.tip-r[data-tip]:focus-visible::after{left:calc(100% + 8px);top:50%;right:auto;transform:translateY(-50%)}" +
  ".tip-l[data-tip]:hover::after,.tip-l[data-tip]:focus-visible::after{left:auto;right:calc(100% + 8px);top:50%;transform:translateY(-50%)}" +
  // phones: row 1 logo + robot + STOP, row 2 tabs, row 3 icons + terminal + power
  "@media (max-width:760px){.fx-tag{display:none}.tabs a span{display:none}.tabs a{padding:8px 12px}.fx-ver{display:none}" +
  ".fx-left,.fx-right{display:contents}.fx-brand{order:1}.fx-robot{order:2}.fx-stop{order:3;margin-left:auto}" +
  ".tabs{order:4}.hdr-icons{order:5}#fxUpd{order:6}.fx-power{order:7}#fxTerm{display:none}}";

// Battery (3S LiPo) as the robot sees it: the OpenCR switches the motor
// power OFF below 11.0 V (turtlebot3_diagnosis.cpp: ~5 s under 11.0 V ->
// DYNAMIXEL power off + beeping, back on above 11.0 V). So 11.0 V = 0 %
// here, not the OpenCR's own 10.5 V; 12.3 V = 100 %. While driving the
// voltage sags, so warn early.
var BATT_EMPTY = 11.0, BATT_FULL = 12.3;
function battState(v) {
  if (v === null || v === undefined || isNaN(v)) return null;
  var pct = Math.max(0, Math.min(100, (v - BATT_EMPTY) / (BATT_FULL - BATT_EMPTY) * 100));
  if (v < BATT_EMPTY) return { pct: 0, level: "bad", text: "motoren uitgeschakeld door de OpenCR (onder 11,0 V) - nu opladen/wisselen" };
  if (v < 11.2) return { pct: pct, level: "bad", text: "nu opladen - onder 11,0 V schakelt de OpenCR de motoren uit" };
  if (v < 11.4) return { pct: pct, level: "warn", text: "bijna leeg - opladen" };
  return { pct: pct, level: "ok", text: "" };
}

function hdrColor(level) {
  return level === "ok" ? "var(--green)" : level === "warn" ? "var(--yellow)" : level === "bad" ? "var(--red)" : "var(--muted)";
}

// "v0.5.0 · 2026-10-10 4b362ce" -> {short: "v0.5.0", text: "v0.5.0 · 2026-10-10"}
// (no commit hash for students; older builds without vX.Y.Z show the date)
function fenixVersion(s) {
  if (!s) return { short: "--", text: "versie onbekend" };
  var m = /^(v\d+\.\d+\.\d+)\s*·\s*(\d{4}-\d\d-\d\d)/.exec(s);
  if (m) return { short: m[1], text: m[1] + " · " + m[2] };
  var d = /(\d{4}-\d\d-\d\d)/.exec(s);
  return { short: d ? d[1] : s, text: d ? "build " + d[1] : s };
}

// Labo-check rows [level, name, detail]. ctx: sys (/system_info), launch
// (/launch_status), batt {v, t}; optional ctx.fresh(key, ms) and ctx.hz(key)
// from a page that follows the sensor topics itself (index.html). Without
// them the sensor rows come from the launch status only (header on the
// other pages: no extra topic subscriptions = no extra rosbridge load).
function laboRows(ctx) {
  var rows = [];
  var now = Date.now();
  var sys = ctx.sys && now - ctx.sys._recvMs < 6000 ? ctx.sys : null;
  var ls = ctx.launch && now - ctx.launch.t < 6000 ? ctx.launch : null;
  var dev = sys && sys.devices ? sys.devices : {};
  var fresh = ctx.fresh;
  var bringup = fresh ? fresh("sensor_state") : !!(ls && ls.bringup && ls.robot_node);

  if (bringup) rows.push(["ok", "OpenCR / bringup", "draait"]);
  else if (sys && dev.opencr === false && dev.usb_power === "off") rows.push(["", "OpenCR / bringup", "USB-stroom uit (spaarstand) - gaat aan bij bringup"]);
  else if (sys && dev.opencr === false) rows.push(["bad", "OpenCR / bringup", "/dev/ttyACM0 niet gevonden (USB-kabel?)"]);
  else if (ls && ls.bringup && ls.robot_node === false) rows.push(["bad", "OpenCR / bringup", "OpenCR antwoordt niet (batterij? USB?)"]);
  else rows.push(["warn", "OpenCR / bringup", "bringup niet gestart"]);

  function sensor(key, label) {
    if (!fresh) { rows.push([bringup ? "ok" : "", label, bringup ? "met bringup (meting op Status)" : "wacht op bringup"]); return; }
    if (fresh(key)) rows.push(["ok", label, ctx.hz && ctx.hz(key) ? ctx.hz(key) + " Hz" : "OK"]);
    else if (bringup) rows.push(["bad", label, "geen data"]);
    else rows.push(["", label, "wacht op bringup"]);
  }
  sensor("scan", "Lidar");
  sensor("imu", "IMU");
  sensor("odom", "Odometrie");

  var b = ctx.batt;
  if (b && now - b.t < 6000 && b.v !== null && b.v !== undefined) {
    var bst = battState(b.v);
    rows.push([bst.level, "Batterij", b.v.toFixed(1) + " V (" + Math.round(bst.pct) + " %)" + (bst.text ? " - " + bst.text : "")]);
  } else rows.push(["", "Batterij", "wacht op bringup"]);

  if (!ls) rows.push(["", "Camera", "--"]);
  else if (!ls.camera_sensor && ls.sensors_source === "simulator")   // virtuele robot: beeld uit de HTML-simulator
    rows.push(fresh && fresh("camera", 5000) ? ["ok", "Camera", "beeld OK (simulator)"] : ["", "Camera", "beeld uit de simulator"]);
  else if (!ls.camera_sensor) rows.push(["", "Camera", "geen camera aangesloten"]);
  else if (!ls.camera_enabled) rows.push(["", "Camera", "uitgeschakeld"]);
  else if (!ls.camera) rows.push(["bad", "Camera", "camera_node draait niet"]);
  else if (typeof ctx.camOff === "function" ? ctx.camOff() : ctx.camOff) rows.push(["", "Camera", "uit op deze pagina"]);
  else if (!fresh) rows.push(["ok", "Camera", "draait (" + ls.camera_sensor + ")"]);
  else if (fresh("camera", 5000)) rows.push(["ok", "Camera", "beeld OK (" + ls.camera_sensor + ")"]);
  else rows.push(["bad", "Camera", "geen beeld"]);

  if (ls) {
    var jd = ls.joystick_device;
    if (ls.joystick) rows.push([jd ? "ok" : "bad", "Joystick", jd ? "aan (" + jd + ")" : "aan, maar ontvanger niet gevonden"]);
    else if (!jd && ls.usb_power === "off") rows.push(["", "Joystick", "uit, USB-stroom uit (spaarstand)"]);
    else rows.push(["", "Joystick", jd ? "uit (" + jd + " gedetecteerd)" : "uit, geen ontvanger"]);
  }

  if (sys) {
    rows.push([sys.zenoh_router ? "ok" : "bad", "Zenoh-router",
      sys.zenoh_router ? "poort 7447" + (sys.zenoh_clients && sys.zenoh_clients.length
        ? " - " + sys.zenoh_clients.length + " laptop(s) verbonden" : "") : "draait niet"]);
    var p = piPower(sys), t = piTemp(sys), dk = diskState(sys);
    rows.push([p.level, "Voeding Pi", p.text + (sys.ext5v_v ? " (" + sys.ext5v_v.toFixed(2) + " V)" : "")]);
    rows.push([t.level, "Temperatuur Pi", t.text]);
    var fn = fanState(sys);
    rows.push([fn.level, "Ventilator", fn.text]);
    var sde = sdErrorState(sys);
    rows.push([sde.level === "bad" ? "bad" : dk.level, "SD-kaart", dk.text + (sde.level === "bad" ? " - " + sde.text : "")]);
    var sv = servicesState(sys);
    rows.push([sv.level, "Diensten", sv.text]);
    var ck = clockState(sys);
    rows.push([ck.level, "Klok", ck.text]);
    var pg = pingState(sys);
    rows.push([pg.level, "Wifi naar AP", pg.text]);
    var inl = internetState(sys);
    rows.push([inl.level, "Internet", inl.text]);
    var sh = shutdownState(sys);
    rows.push([sh.level, "Vorige afsluiting", sh.text]);
    var dbm = sys.wifi_signal_dbm;
    if (dbm !== null && dbm !== undefined)
      rows.push([dbm > -60 ? "ok" : (dbm >= -70 ? "warn" : "bad"), "Wifi", dbm + " dBm"]);
  } else {
    rows.push(["bad", "Systeeminfo", "geen /system_info"]);
  }
  return rows;
}
function laboSummary(rows) {
  var bad = rows.filter(function (r) { return r[0] === "bad"; }).length;
  var warn = rows.filter(function (r) { return r[0] === "warn"; }).length;
  if (bad) return { level: "bad", text: bad + " probleem" + (bad > 1 ? "en" : "") };
  if (warn) return { level: "warn", text: warn + " aandachtspunt" + (warn > 1 ? "en" : "") };
  return { level: "ok", text: "Alles OK" };
}

// Page hook: index.html passes its topic freshness for a full labo-check.
var fenixLaboCtx = {};
function setLaboSource(fresh, hz, camOff) {
  fenixLaboCtx.fresh = fresh; fenixLaboCtx.hz = hz; fenixLaboCtx.camOff = camOff;
}

// "Powered by" line at the bottom of every page; empty URL = plain text
var LEBTEC_URL = "https://www.lebtec.be";
// Catchphrase under the name on the splash screen
var FENIX_TAGLINE = "Rise · Map · Navigate";

// Splash screen: logo + name + catchphrase while the page connects - on the
// first visit and on every refresh, not when switching between the tabs;
// gone on connection (min ~0.9 s, max ~2.5 s).
function fenixSplash(ros) {
  var seen = false, reload = false;
  try { seen = sessionStorage.getItem("fenixSplash") === "1"; sessionStorage.setItem("fenixSplash", "1"); } catch (e) {}
  try { reload = performance.getEntriesByType("navigation")[0].type === "reload"; } catch (e) {}
  if (seen && !reload) return;
  var sp = document.createElement("div");
  sp.id = "fxSplash";
  sp.style.cssText = "position:fixed;inset:0;z-index:100;background:#14161a;display:flex;flex-direction:column;" +
    "align-items:center;justify-content:center;gap:14px;transition:opacity .45s ease;font-family:inherit";
  sp.innerHTML = '<img src="fenix-logo.svg" alt="" width="132" height="132" style="filter:drop-shadow(0 8px 30px #f0883e55)">' +
    '<div style="font-size:2.4rem;font-weight:800;letter-spacing:.02em;color:#e6e8eb">Fenix</div>' +
    '<div style="font-size:1.05rem;font-style:italic;color:#f0883e;margin-top:-8px">' + escHtml(FENIX_TAGLINE) + "</div>" +
    '<div id="fxSplashMsg" style="color:#8a8f98;font-size:.9rem">connecting to ' + escHtml(window.location.hostname) + "...</div>" +
    '<div style="position:absolute;bottom:22px;color:#8a8f98;font-size:.75rem;letter-spacing:.06em">POWERED BY LEBTEC</div>';
  document.body.appendChild(sp);
  var t0 = Date.now(), done = false;
  function hide() {
    if (done) return;
    done = true;
    setTimeout(function () {
      sp.style.opacity = "0";
      setTimeout(function () { sp.remove(); }, 500);
    }, Math.max(0, 900 - (Date.now() - t0)));   // at least ~0.9 s, so it doesn't flash
  }
  if (ros.isConnected) hide(); else ros.on("connection", hide);
  setTimeout(hide, 2500);
  sp.addEventListener("click", hide);
}

function fenixFooter() {
  var f = document.createElement("div");
  f.className = "fx-powered";
  f.innerHTML = LEBTEC_URL
    ? 'Powered by <a href="' + escHtml(LEBTEC_URL) + '" target="_blank" rel="noopener">LEBTEC</a>'
    : "Powered by LEBTEC";
  document.body.appendChild(f);
}

function initHeader(ros, page) {
  var st = document.createElement("style");
  st.textContent = HEADER_CSS;
  document.head.appendChild(st);
  // tab icon + phone home-screen icon
  [["icon", "favicon.png"], ["apple-touch-icon", "fenix-logo-192.png"]].forEach(function (l) {
    var el = document.createElement("link"); el.rel = l[0]; el.href = l[1]; document.head.appendChild(el);
  });
  fenixSplash(ros);
  fenixFooter();

  var hdr = document.getElementById("appHeader");
  if (!hdr) return;
  hdr.innerHTML =
    '<div class="fx-left">' +
      '<a class="fx-brand" href="index.html">' + FENIX_LOGO + '<span class="fx-bname">Fenix<span class="fx-tag">' +
        escHtml(FENIX_TAGLINE) + "</span></span></a>" +
      '<span class="fx-robot" id="robotName" tabindex="0" data-tip="Robot">-</span>' +
      '<span class="fx-ver" id="fxVer" tabindex="0" data-tip="Fenix-versie">--</span>' +
      '<nav class="tabs" id="pageTabs" aria-label="Pagina\'s">' +
        PAGES.map(function (p) {
          return '<a href="' + p[1] + '" data-page="' + p[0] + '" data-tip="' + p[3] + '">' + fxIcon(p[0], 16) + "<span>" + p[2] + "</span></a>";
        }).join("") +
      "</nav>" +
    "</div>" +
    '<div class="fx-right">' +
      '<div class="hdr-icons" id="hdrIcons">' +
        '<span class="hdr-icon" id="hiConn" tabindex="0" data-tip="Verbinding met de robot: verbinden...">' + fxIcon("conn", 16) +
          '<span id="connDot" class="dot" style="display:none"></span><span id="connText" style="display:none"></span></span>' +
        '<span class="hdr-icon" id="hiBatt" tabindex="0" data-tip="Batterij: wacht op bringup">' +
          '<svg width="26" height="14" viewBox="0 0 26 14" aria-hidden="true"><rect x="1" y="1" width="21" height="12" rx="2" fill="none" stroke="currentColor" stroke-width="1.5"/>' +
          '<rect x="22.5" y="4.5" width="2.5" height="5" rx="1" fill="currentColor"/><rect id="hiBattFill" x="3" y="3" width="0" height="8" rx="1" fill="currentColor"/></svg>' +
          '<span id="hiBattTxt">--</span></span>' +
        '<span class="hdr-icon" id="hiWifi" tabindex="0" data-tip="Wifi: --"><svg width="18" height="14" viewBox="0 0 18 14" aria-hidden="true">' +
          [0, 1, 2, 3].map(function (i) {
            return '<rect class="hiWifiBar" x="' + (i * 4.6) + '" y="' + (10 - i * 3) + '" width="3.4" height="' + (4 + i * 3) + '" rx="1" fill="currentColor" opacity=".25"/>';
          }).join("") + "</svg></span>" +
        '<span class="hdr-icon" id="hiTemp" tabindex="0" data-tip="Raspberry Pi: --"><svg width="12" height="18" viewBox="0 0 12 18" aria-hidden="true">' +
          '<path d="M4 2.5a2 2 0 0 1 4 0v8.2a3.6 3.6 0 1 1-4 0z" fill="none" stroke="currentColor" stroke-width="1.5"/>' +
          '<circle cx="6" cy="13.6" r="2" fill="currentColor"/><rect id="hiTempFill" x="5.2" y="8" width="1.6" height="5" fill="currentColor"/></svg></span>' +
        '<span class="hdr-icon" id="hiInet" tabindex="0" data-tip="Internet: --"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true">' +
          '<circle cx="12" cy="12" r="9"/><path d="M3 12h18M12 3c2.5 2.7 3.8 5.7 3.8 9s-1.3 6.3-3.8 9M12 3c-2.5 2.7-3.8 5.7-3.8 9s1.3 6.3 3.8 9"/>' +
          '<line id="hiInetSlash" x1="4" y1="4" x2="20" y2="20" style="display:none"/></svg></span>' +
        '<span class="hdr-icon" id="hiLabo" tabindex="0" role="button" style="cursor:pointer" data-tip="Labo-check: --">' + fxIcon("labo", 17) + "</span>" +
        // Aanpassingen (launch_control_node "changes"): sliders icon + count
        '<span class="hdr-icon" id="hiChg" tabindex="0" role="button" data-tip="Aanpassingen: --" style="cursor:pointer">' +
          '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true">' +
          '<path d="M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12M20 18h0"/><circle cx="16" cy="6" r="2"/><circle cx="10" cy="12" r="2"/><circle cx="18" cy="18" r="2"/></svg>' +
          '<span id="hiChgTxt"></span></span>' +
      "</div>" +
      '<button class="fx-round" id="fxAdmin" style="display:none;color:var(--yellow)" aria-label="Beheer vergrendelen" ' +
        'data-tip="Beheer is ontgrendeld - klik om te vergrendelen">' + fxIcon("unlocked") + "</button>" +
      '<a class="fx-round admin-only" id="fxTerm" target="_blank" rel="noopener" aria-label="Terminal" data-tip="Terminal op de robot (nieuw tabblad) - enkel docent">' + fxIcon("terminal") + "</a>" +
      '<a class="fx-round" id="fxUpd" href="system.html#beheer" style="display:none" aria-label="Update beschikbaar" ' +
        'data-tip="Update beschikbaar - bijwerken doet de docent">' + fxIcon("update") + '<span class="fx-dot"></span></a>' +
      '<span class="fx-power" style="position:relative">' +
        '<button class="fx-round" id="fxPower" aria-label="Aan/uit" aria-haspopup="true" data-tip="Afsluiten / herstarten">' + fxIcon("power") + "</button>" +
        '<div class="fx-menu" id="fxPowerMenu" style="display:none">' +
          '<button class="fx-mi" data-power="shutdown"><span style="color:var(--red)">' + fxIcon("power") + "</span>" +
            "<span>Afsluiten<small>einde labo - wacht tot de groene led niet meer knippert</small></span></button>" +
          '<button class="fx-mi" data-power="reboot">' + fxIcon("reboot") + "<span>Herstarten<small>ongeveer 1 minuut weg</small></span></button>" +
          '<div id="fxPowerMsg" style="color:var(--muted);font-size:.78rem;padding:4px 10px"></div>' +
        "</div>" +
      "</span>" +
      '<button class="fx-stop" id="fxStop" data-tip="Alles stoppen wat rijdt - motoren stil\n(bringup, SLAM, navigatie, joystick, projecten; de camera blijft aan)">' + fxIcon("stop", 18, 2.5) + "STOP</button>" +
    "</div>";

  fenixSetActiveTab(page);
  window.addEventListener("hashchange", function () { fenixSetActiveTab(page); });
  document.getElementById("fxAdmin").addEventListener("click", function () { fenixAdminLock(); });
  fenixAdminRefresh();
  setInterval(fenixAdminRefresh, 60000);

  var chips = document.createElement("div");
  chips.className = "fx-chips";
  chips.id = "fxChips";
  hdr.parentNode.insertBefore(chips, hdr.nextSibling);

  var term = document.getElementById("fxTerm");
  term.href = "http://" + window.location.hostname + ":7681";

  // Connection icon (connectRos() ran before the header existed)
  function setConnIcon(level, text) {
    var el = document.getElementById("hiConn");
    el.style.color = hdrColor(level);
    el.setAttribute("data-tip", "Verbinding met de robot: " + text);
  }
  setConnIcon(ros.isConnected ? "ok" : "warn", ros.isConnected ? "verbonden" : "verbinden...");
  ros.on("connection", function () { setConnIcon("ok", "verbonden"); });
  ros.on("close", function () { setConnIcon("bad", "verbroken - opnieuw verbinden..."); });
  ros.on("error", function () { setConnIcon("bad", "fout"); });

  function trigger(name, cb) {
    new ROSLIB.Service({ ros: ros, name: name, serviceType: "std_srvs/srv/Trigger" })
      .callService(new ROSLIB.ServiceRequest({}), function (r) { if (cb) cb(r.success, r.message); },
                   function (err) { if (cb) cb(false, String(err)); });
  }

  // STOP: no confirmation, it's the emergency button
  var stopBtn = document.getElementById("fxStop");
  stopBtn.addEventListener("click", function () {
    stopBtn.disabled = true;
    trigger("/launch/stop_all", function () { stopBtn.disabled = false; });
  });

  // Power menu (students may shut down / reboot; updating is for the teacher)
  var menu = document.getElementById("fxPowerMenu");
  document.getElementById("fxPower").addEventListener("click", function (e) {
    e.stopPropagation();
    menu.style.display = menu.style.display === "none" ? "" : "none";
  });
  menu.addEventListener("click", function (e) {
    e.stopPropagation();
    var b = e.target.closest("[data-power]");
    if (!b) return;
    var off = b.dataset.power === "shutdown";
    if (!confirm(off ? "Robot afsluiten? Alles stopt. Wacht daarna tot de groene led niet meer knippert voor je de batterij loskoppelt."
                     : "Robot herstarten? Alles stopt; de pagina is ongeveer 1 minuut weg.")) return;
    document.getElementById("fxPowerMsg").textContent = off ? "afsluiten..." : "herstarten...";
    trigger(off ? "/system/shutdown" : "/system/reboot", function (ok, msg) {
      document.getElementById("fxPowerMsg").textContent = (ok ? "" : "Fout: ") + msg;
    });
  });

  // Battery (only published while bringup runs)
  var batt = { t: 0 };
  fenixLaboCtx.batt = batt;
  new ROSLIB.Topic({ ros: ros, name: "/battery_state", messageType: "sensor_msgs/msg/BatteryState",
                     throttle_rate: 2000, queue_length: 1 })
    .subscribe(function (m) {
      var pct = m.percentage;
      if (pct !== null && pct !== undefined && pct <= 1.0) pct *= 100;
      batt.pct = pct; batt.v = m.voltage; batt.t = Date.now();
      renderBatt();
    });
  function renderBatt() {
    var el = document.getElementById("hiBatt");
    var fresh = Date.now() - batt.t < 6000;
    var bs = fresh ? battState(batt.v) : null;
    var pct = bs ? bs.pct : null;
    var level = bs ? bs.level : "";
    el.style.color = hdrColor(level);
    document.getElementById("hiBattFill").setAttribute("width", pct === null ? 0 : (17 * pct / 100).toFixed(1));
    document.getElementById("hiBattTxt").textContent = pct === null ? "--" : Math.round(pct) + "%";
    el.setAttribute("data-tip", pct === null
      ? "Batterij: onbekend - wordt enkel gemeten als de robot draait (bringup)"
      : "Batterij " + Math.round(pct) + " % - " + batt.v.toFixed(2) + " V" + (bs.text ? "\n" + bs.text : "") +
        "\n0 % = 11,0 V: daar schakelt de OpenCR de motoren uit");
    renderAlert();
  }
  setInterval(renderBatt, 2000);

  // Red banner under the header when the robot can't drive: motor power cut
  // by the OpenCR (battery), or turtlebot3_node gone while bringup runs
  // (OpenCR not answering - it restarts by itself, respawn in robot.launch.py).
  var launchSt = { t: 0 };
  fenixLaboCtx.launch = launchSt;
  new ROSLIB.Topic({ ros: ros, name: "/launch_status", messageType: "std_msgs/msg/String", throttle_rate: 1000, queue_length: 1 })
    .subscribe(function (m) {
      beat();
      try { launchSt = JSON.parse(m.data); launchSt.t = Date.now(); } catch (e) { return; }
      fenixLaboCtx.launch = launchSt;
      renderAlert(); renderChips();
    });
  var alertEl = document.createElement("div");
  alertEl.id = "robotAlert";
  alertEl.style.cssText = "display:none;margin:0 0 12px;padding:8px 12px;border-radius:8px;background:var(--red);color:#2a0d0d;font-weight:600;font-size:.88rem";
  chips.parentNode.insertBefore(alertEl, chips.nextSibling);

  // Connection lost (t80). A closed websocket is noticed at once, but after a
  // wifi drop the socket can stay "open" for minutes (TCP timeout). So also a
  // heartbeat: /launch_status (1 Hz), /cmd_vel_status (2 Hz) and /system_info
  // all silent for HEARTBEAT_MS -> big red banner, driving buttons off
  // (body.fx-offline) and the "fx-offline" event on ros (pages stop driving).
  // Reconnect every 2 s after a close; silent for DEAD_MS on an "open" socket
  // -> close it ourselves so that reconnect starts. roslib subscribes the
  // topics again by itself (reconnect_on_close).
  var HEARTBEAT_MS = 3000, DEAD_MS = 8000, RECONNECT_MS = 2000;
  var loadedVersion = null, reloading = false;   // Fenix-versie bij het laden (zie /system_info)
  // Geluid bij verbinding weg / terug (23-10). Browsers laten audio pas toe na
  // een klik of toets: de AudioContext wordt bij de eerste interactie gemaakt.
  var everOnline = false, lostBeeped = false, audio = null;
  function unlockAudio() {
    try {
      if (!audio) audio = new (window.AudioContext || window.webkitAudioContext)();
      if (audio.state === "suspended") audio.resume();
    } catch (e) { /* geen audio: dan enkel de balk */ }
  }
  ["pointerdown", "keydown"].forEach(function (ev) { document.addEventListener(ev, unlockAudio, { passive: true }); });
  function beep(tones) {        // [[Hz, s], ...]
    if (!audio || audio.state !== "running") return;
    var t = audio.currentTime + 0.02;
    tones.forEach(function (tn) {
      var o = audio.createOscillator(), g = audio.createGain();
      o.frequency.value = tn[0]; o.connect(g); g.connect(audio.destination);
      g.gain.setValueAtTime(0.0001, t); g.gain.exponentialRampToValueAtTime(0.25, t + 0.01);
      g.gain.exponentialRampToValueAtTime(0.0001, t + tn[1]);
      o.start(t); o.stop(t + tn[1] + 0.02); t += tn[1] + 0.08;
    });
  }
  var lastBeat = Date.now(), offline = false, offlineSince = 0, forcedAt = 0, reconnectTimer = null;
  var connEl = document.createElement("div");
  connEl.id = "connAlert";
  connEl.setAttribute("role", "alert");
  connEl.style.cssText = "display:none;margin:0 0 12px;padding:12px 14px;border-radius:8px;font-weight:700;font-size:1rem";
  chips.parentNode.insertBefore(connEl, chips.nextSibling);
  function beat() {
    lastBeat = Date.now();
    everOnline = true;
    if (offline) setOffline(false);
  }
  fenixLaboCtx.beat = beat;
  function robotLabel() {
    return (fenixLaboCtx.sys && fenixLaboCtx.sys.hostname) || window.location.hostname;
  }
  function offlineText() {
    var s = Math.round((Date.now() - offlineSince) / 1000);
    return "Geen verbinding met " + robotLabel() + (s >= 2 ? " sinds " + s + " s" : "") +
      " - de robot stopt vanzelf (na 1,5 s zonder commando). Opnieuw verbinden...";
  }
  function setOffline(on) {
    if (on === offline) return;
    offline = on;
    document.body.classList.toggle("fx-offline", on);
    if (on) {
      offlineSince = Date.now();
      connEl.style.background = "var(--red)"; connEl.style.color = "#2a0d0d";
      connEl.textContent = offlineText();
      connEl.style.display = "";
      ros.emit("fx-offline");
      if (everOnline) { beep([[880, 0.15], [880, 0.15], [880, 0.15]]); lostBeeped = true; }
    } else {
      connEl.style.background = "var(--green)"; connEl.style.color = "#0d2a14";
      connEl.textContent = "Verbinding met " + robotLabel() + " hersteld";
      if (lostBeeped) { beep([[660, 0.1], [990, 0.15]]); lostBeeped = false; }
      setTimeout(function () { if (!offline) connEl.style.display = "none"; }, 3000);
    }
  }
  function scheduleReconnect() {
    if (reconnectTimer || !ros.fxUrl) return;
    reconnectTimer = setTimeout(function () {
      reconnectTimer = null;
      if (!ros.isConnected) ros.connect(ros.fxUrl);
    }, RECONNECT_MS);
  }
  ros.on("close", function () { setOffline(true); scheduleReconnect(); });
  ros.on("error", scheduleReconnect);
  setInterval(function () {
    var silent = Date.now() - lastBeat;
    if (silent > HEARTBEAT_MS) setOffline(true);
    if (ros.isConnected && silent > DEAD_MS && Date.now() - forcedAt > DEAD_MS) {
      forcedAt = Date.now();   // at most one forced close per DEAD_MS
      try { ros.close(); } catch (e) { /* the close event schedules the reconnect */ }
    }
    if (offline) connEl.textContent = offlineText();
  }, 1000);

  // Robot state + active control source (t13/t40): cmd_vel_guard tells which
  // twist_mux input drives (/cmd_vel_status), launch_status what runs.
  var velSt = { t: 0 };
  new ROSLIB.Topic({ ros: ros, name: "/cmd_vel_status", messageType: "std_msgs/msg/String", throttle_rate: 500, queue_length: 1 })
    .subscribe(function (m) {
      beat();
      try { velSt = JSON.parse(m.data); velSt.t = Date.now(); } catch (e) { return; }
      renderChips();
    });
  function robotState() {
    var lfresh = Date.now() - launchSt.t < 6000, vfresh = Date.now() - velSt.t < 3000;
    if (alertEl.textContent) return { cls: "error", text: "Fout", tip: alertEl.textContent };
    if (!lfresh) return { cls: "", text: "onbekend", tip: "Geen status van de robot (launch_control)" };
    var pr = launchSt.projects_running || {};
    var proj = Object.keys(pr).filter(function (k) { return pr[k]; }).map(function (k) { return k.split(":")[0]; })[0];
    if (!launchSt.bringup) return { cls: "", text: "Inactief", tip: "Bringup staat uit: de motoren luisteren niet" };
    if (!vfresh) return { cls: "", text: "Klaar", tip: "Stuurbron onbekend (cmd_vel_guard, oudere image?)" };
    var moving = velSt.moving ? " moving" : "";
    var speed = velSt.moving ? "\nNu " + velSt.lin.toFixed(2) + " m/s, " + velSt.ang.toFixed(2) + " rad/s" : "";
    var cap = "\nMaximum " + (+velSt.max_linear).toFixed(2) + " m/s, " + (+velSt.max_angular).toFixed(2) + " rad/s";
    if (velSt.source === "joy") return { cls: "manual" + moving, text: "Handbediend · gamepad", tip: "De gamepad stuurt (/cmd_vel_joy, gaat voor alles)" + speed + cap };
    if (velSt.source === "web") return { cls: "manual" + moving, text: "Handbediend · pagina", tip: "Een webpagina stuurt (/cmd_vel_web, gaat voor /cmd_vel)" + speed + cap };
    if (velSt.source === "cmd_vel") {
      var who = launchSt.navigation ? "Nav2" : proj ? "project " + proj : "eigen node";
      return { cls: "auto" + moving, text: "Autonoom · " + who, tip: who + " stuurt via /cmd_vel (gamepad en pagina gaan voor)" + speed + cap };
    }
    if (launchSt.navigation) return { cls: "auto", text: "Klaar · Nav2", tip: "Navigatie draait en wacht op een doel" + cap };
    if (proj) return { cls: "auto", text: "Klaar · project " + proj, tip: "Project draait, stuurt nu niet" + cap };
    return { cls: "", text: "Klaar", tip: "Bringup draait, niemand stuurt" + cap };
  }

  // Running launches as chips, each with its own stop button
  var CHIP_NAMES = { bringup: "bringup", slam: "kaart maken (SLAM)", navigation: "navigatie", camera: "camera", joystick: "joystick", sensors: "sensoren (sim)" };
  function renderChips() {
    var fresh = Date.now() - launchSt.t < 6000;
    var items = [];
    if (fresh) {
      Object.keys(CHIP_NAMES).forEach(function (k) { if (launchSt[k]) items.push({ id: k, label: CHIP_NAMES[k] }); });
      var pr = launchSt.projects_running || {};
      Object.keys(pr).forEach(function (k) { if (pr[k]) items.push({ id: "project:" + k, label: "project " + k.split(":")[0] }); });
    }
    var st = robotState();
    chips.innerHTML = '<span class="fx-state ' + st.cls + '" tabindex="0" data-tip="' + escHtml(st.tip) + '"><i></i>' + escHtml(st.text) + "</span>" +
      '<span>Actief:</span>' + (!fresh ? "<span>--</span>" : items.length ? items.map(function (c) {
      return '<span class="fx-chip">' + escHtml(c.label) + '<button data-stop="' + escHtml(c.id) + '" aria-label="Stop ' + escHtml(c.label) +
        '" data-tip="Stop ' + escHtml(c.label) + '">' + fxIcon("x", 12, 3) + "</button></span>";
    }).join("") : "<span>niets</span>");
  }
  chips.addEventListener("click", function (e) {
    var b = e.target.closest("[data-stop]");
    if (!b) return;
    b.disabled = true;
    var id = b.dataset.stop;
    if (id.indexOf("project:") === 0) {
      var p = id.slice(8).split(":");
      new ROSLIB.Topic({ ros: ros, name: "/launch/project_cmd", messageType: "std_msgs/msg/String" })
        .publish(new ROSLIB.Message({ data: JSON.stringify({ project: p[0], launch: p[1], action: "stop" }) }));
    } else trigger("/launch/stop_" + id);
  });
  renderChips();
  setInterval(renderChips, 3000);

  // Popovers: Aanpassingen + Labo-check
  function makePop(id) {
    var el = document.createElement("div");
    el.id = id; el.className = "fx-pop";
    document.body.appendChild(el);
    el.addEventListener("click", function (e) { e.stopPropagation(); });
    return el;
  }
  function togglePop(pop, anchorId, fill) {
    [chgPop, laboPop].forEach(function (p) { if (p !== pop) p.style.display = "none"; });
    if (pop.style.display === "block") { pop.style.display = "none"; return; }
    var r = document.getElementById(anchorId).getBoundingClientRect();
    pop.style.top = (r.bottom + window.scrollY + 8) + "px";
    fill();
    pop.style.display = "block";
  }
  var chgPop = makePop("chgPop"), laboPop = makePop("laboPop");
  document.addEventListener("click", function () {
    chgPop.style.display = "none"; laboPop.style.display = "none"; menu.style.display = "none";
  });

  var resetTopic = new ROSLIB.Topic({ ros: ros, name: "/system/reset", messageType: "std_msgs/msg/String" });
  function sendReset(what) { resetTopic.publish(new ROSLIB.Message({ data: JSON.stringify({ what: what }) })); }
  var chgList = [];
  function renderChanges() {
    var el = document.getElementById("hiChg");
    var fresh = Date.now() - launchSt.t < 6000;
    chgList = fresh ? (launchSt.changes || []) : [];
    el.style.color = fresh && chgList.length ? "var(--orange, #f0883e)" : hdrColor("");
    document.getElementById("hiChgTxt").textContent = fresh && chgList.length ? chgList.length : "";
    el.setAttribute("data-tip", !fresh ? "Aanpassingen: onbekend" : chgList.length
      ? "Aanpassingen t.o.v. de standaard (" + chgList.length + "):\n- " + chgList.map(function (c) { return c.text; }).join("\n- ") + "\n(klik voor details en herstellen)"
      : "Aanpassingen: alles standaard");
    if (chgPop.style.display === "block") fillChgPop();
  }
  function fillChgPop() {
    chgPop.innerHTML = "<b>Aanpassingen t.o.v. de standaard</b>" +
      (chgList.length ? chgList.map(function (c) {
        return '<div style="display:flex;gap:8px;align-items:center;justify-content:space-between;margin-top:6px">' +
          "<span>" + escHtml(c.text) + '</span><button class="chg-reset" data-id="' + escHtml(c.id) + '" style="flex:none">Herstel</button></div>';
      }).join("") + '<div style="margin-top:10px;display:flex;gap:8px;align-items:center;justify-content:space-between">' +
        '<span style="color:var(--muted);font-size:.78rem">Alles standaard = enkel instellingen; kaartbewerkingen en zones per regel (er wordt een reservekopie gemaakt)</span>' +
        '<button class="chg-reset" data-id="all" style="flex:none">Alles standaard</button></div>'
        : '<div style="margin-top:6px;color:var(--muted)">Alles staat standaard.</div>');
  }
  document.getElementById("hiChg").addEventListener("click", function (e) {
    e.stopPropagation();
    togglePop(chgPop, "hiChg", fillChgPop);
  });
  chgPop.addEventListener("click", function (e) {
    var b = e.target.closest(".chg-reset");
    if (!b) return;
    var all = b.dataset.id === "all";
    var work = /^(map_edit|zones):/.test(b.dataset.id);
    if (!confirm(all ? "Alle instellingen terugzetten naar de standaard? (kaartbewerkingen en zones blijven)"
                 : work ? "Dit wist werk van iemand (kaartbewerking of zones). Er wordt een reservekopie gemaakt op de robot. Doorgaan?"
                 : "Deze aanpassing terugzetten?")) return;
    sendReset(b.dataset.id);
    b.disabled = true; b.textContent = "...";
  });

  // Labo-check: icon colour = worst row, hover = summary, click = all rows
  function renderLabo() {
    var rows = laboRows(fenixLaboCtx);
    var s = laboSummary(rows);
    var el = document.getElementById("hiLabo");
    el.style.color = hdrColor(s.level);
    var issues = rows.filter(function (r) { return r[0] === "bad" || r[0] === "warn"; });
    el.setAttribute("data-tip", "Labo-check: " + s.text +
      (issues.length ? "\n- " + issues.map(function (r) { return r[1] + ": " + r[2]; }).join("\n- ") : "") + "\n(klik voor alle controles)");
    if (laboPop.style.display === "block") fillLaboPop(rows, s);
    return rows;
  }
  function fillLaboPop(rows, s) {
    rows = rows || laboRows(fenixLaboCtx);
    s = s || laboSummary(rows);
    laboPop.innerHTML = '<b>Labo-check</b> <span style="color:' + hdrColor(s.level) + '">' + escHtml(s.text) + "</span><ul>" +
      rows.map(function (r) {
        return '<li><span class="ld ' + r[0] + '"></span><span class="n">' + escHtml(r[1]) + "</span><span>" + escHtml(r[2]) + "</span></li>";
      }).join("") + "</ul>";
  }
  document.getElementById("hiLabo").addEventListener("click", function (e) {
    e.stopPropagation();
    togglePop(laboPop, "hiLabo", function () { fillLaboPop(); });
  });
  setInterval(renderLabo, 1000);

  // Robot alerts (t35): a card at the bottom right on every page when
  // something needs attention. Conditions (battery, Pi power, temperature,
  // services) stay until fixed or closed; a closed one comes back only after
  // it was fixed in between. ERROR/FATAL from /rosout (collected by
  // system_info_node) once each. Closed/seen keys in sessionStorage, so
  // switching tabs doesn't show them again.
  var toastBox = document.createElement("div");
  toastBox.className = "fx-toasts";
  toastBox.setAttribute("aria-live", "polite");
  document.body.appendChild(toastBox);
  var TOAST_KEY = "fenixAlertsSeen", shownToasts = {}, toastSeen = {};
  try { toastSeen = JSON.parse(sessionStorage.getItem(TOAST_KEY) || "{}"); } catch (e) { toastSeen = {}; }
  function saveSeen() { try { sessionStorage.setItem(TOAST_KEY, JSON.stringify(toastSeen)); } catch (e) {} }
  function showToast(a) {
    var el = document.createElement("div");
    el.className = "fx-toast " + a.level;
    el.innerHTML = "<span><b>" + escHtml(a.title) + "</b>" + escHtml(a.text) +
      (a.detail ? "<small>" + escHtml(a.detail) + "</small>" : "") + "</span>" +
      '<button aria-label="Sluiten" data-tip="Sluiten">' + fxIcon("x", 14, 3) + "</button>";
    el.querySelector("button").addEventListener("click", function () {
      toastSeen[a.key] = 1; saveSeen();
      el.remove(); delete shownToasts[a.key];
    });
    toastBox.appendChild(el);
    shownToasts[a.key] = { el: el, once: a.once };
    if (a.once) setTimeout(function () { el.remove(); delete shownToasts[a.key]; }, 30000);
  }
  function robotAlerts() {
    var out = [], d = fenixLaboCtx.sys, now = Date.now();
    if (Date.now() - batt.t < 6000 && batt.v >= BATT_EMPTY) {
      var bs = battState(batt.v);
      if (bs && bs.level !== "ok")
        out.push({ key: "batt-" + bs.level, level: bs.level, title: "Batterij " + batt.v.toFixed(1) + " V", text: bs.text });
    }
    if (!d || now - d._recvMs > 10000) return out;
    var pw = piPower(d);
    if (pw.level === "bad") out.push({ key: "pi-power", level: "bad", title: "Raspberry Pi: " + pw.text,
      text: "Voeding van de Pi te zwak (batterij bijna leeg of slechte kabel/regelaar) - kans op crashes" });
    var tp = piTemp(d);
    if (tp.level === "bad") out.push({ key: "pi-temp", level: "bad", title: "Raspberry Pi warm: " + tp.text,
      text: "Boven 80 °C remt de Pi zichzelf af - ventilator nakijken, minder tegelijk laten draaien" });
    var sv = servicesState(d);
    sv.down.forEach(function (k) {
      out.push({ key: "svc-" + k, level: "bad", title: "Dienst gestopt: " + (SERVICE_LABEL[k] || k),
        text: "Wordt binnen ~20 s vanzelf herstart; blijft dit, herstart dan de robot" });
    });
    var sup = d.supervisor || {};
    Object.keys(sup).forEach(function (k) {
      if (d.time_unix && sup[k].last < d.time_unix - 600) return;
      out.push({ key: "restart-" + k + "-" + sup[k].restarts, once: true, level: "warn",
        title: (SERVICE_LABEL[k] || k) + " was gecrasht",
        text: "Vanzelf herstart (" + sup[k].restarts + "x sinds de start). Details: Status > Diensten" });
    });
    // only errors from the last 2 min (robot clock) on the first look
    (d.rosout_errors || []).forEach(function (e) {
      if (d.time_unix && e.t < d.time_unix - 120) return;
      out.push({ key: "rosout-" + e.t + "-" + e.node, once: true, level: e.level === "FATAL" ? "bad" : "warn",
        title: e.level + " van " + e.node, text: "", detail: e.msg });
    });
    return out;
  }
  function renderToasts() {
    var active = {}, all = robotAlerts();
    all.forEach(function (a) { active[a.key] = 1; });
    // max 3 cards (phone); the rest waits, most serious first
    all.filter(function (a) { return !shownToasts[a.key] && !toastSeen[a.key]; })
      .sort(function (a, b) { return (b.level === "bad") - (a.level === "bad"); })
      .forEach(function (a) {
        if (Object.keys(shownToasts).length >= 3) return;
        showToast(a);
        if (a.once) { toastSeen[a.key] = 1; saveSeen(); }
      });
    // condition fixed: card away, and it may come back later
    Object.keys(shownToasts).forEach(function (k) {
      if (!active[k] && !shownToasts[k].once) { shownToasts[k].el.remove(); delete shownToasts[k]; }
    });
    var changed = false;
    Object.keys(toastSeen).forEach(function (k) {
      if (!active[k] && !/^(restart|rosout)-/.test(k)) { delete toastSeen[k]; changed = true; }
    });
    if (changed) saveSeen();
  }
  setInterval(renderToasts, 2000);

  function renderAlert() {
    renderChanges();
    var msg = "";
    var bfresh = Date.now() - batt.t < 6000, lfresh = Date.now() - launchSt.t < 6000;
    if (bfresh && batt.v < BATT_EMPTY)
      msg = "Batterij " + batt.v.toFixed(1) + " V - de OpenCR heeft de motoren uitgeschakeld (onder 11,0 V). Opladen of batterij wisselen.";
    else if (lfresh && launchSt.bringup && launchSt.robot_node === false)
      msg = "De OpenCR antwoordt niet (batterij leeg? USB?) - turtlebot3_node start vanzelf opnieuw zodra hij weer antwoordt.";
    else if (lfresh && launchSt.sim_restart && launchSt.sim_restart.stopped.length &&
             Date.now() / 1000 - launchSt.sim_restart.t < 60 &&
             !launchSt.sim_restart.stopped.some(function (n) { return launchSt[n]; }))
      // virtuele robot: simulatorpagina herladen -> tijd terug naar 0 (launch_control_node)
      msg = "De simulator is herstart (pagina herladen of andere wereld): " +
        launchSt.sim_restart.stopped.map(function (n) { return n === "slam" ? "SLAM" : "Navigatie"; }).join(" en ") + " gestopt. Start opnieuw" +
        (launchSt.sim_restart.stopped.indexOf("slam") >= 0 ? " - een kaart die nog niet bewaard was, is verloren." : ".");
    alertEl.textContent = msg;
    alertEl.style.display = msg ? "" : "none";
  }

  new ROSLIB.Topic({ ros: ros, name: "/system_info", messageType: "std_msgs/msg/String" })
    .subscribe(function (msg) {
      var d;
      beat();
      try { d = JSON.parse(msg.data); } catch (e) { return; }
      d._recvMs = Date.now();
      fenixLaboCtx.sys = d;

      // Robot name + version
      var rn = document.getElementById("robotName");
      if (d.hostname) rn.textContent = d.hostname;
      var cfg = d.config || {};
      rn.setAttribute("data-tip", (d.hostname || "robot") + "\nTurtleBot3 " + (cfg.TURTLEBOT3_MODEL || "") + " · Raspberry Pi 5" +
        "\nIP " + (d.ip || window.location.hostname));
      var v = fenixVersion(d.image_version);
      // Na een update draait de robot een andere Fenix-versie dan toen deze
      // pagina laadde: herladen. Anders werkt de oude JavaScript na het
      // automatisch herverbinden door (23-10: Twist i.p.v. TwistStamped op
      // /cmd_vel_web, rosbridge weigerde daarna de nieuwe pagina's).
      if (d.image_version) {
        if (!loadedVersion) loadedVersion = d.image_version;
        else if (d.image_version !== loadedVersion && !reloading) {
          reloading = true;
          connEl.style.background = "var(--accent)"; connEl.style.color = "#0b1a2b";
          connEl.textContent = "Nieuwe Fenix-versie op de robot (" + v.short + ") - de pagina wordt herladen...";
          connEl.style.display = "";
          setTimeout(function () { location.reload(); }, 1500);
        }
      }
      var ve = document.getElementById("fxVer");
      ve.textContent = v.short;
      ve.setAttribute("data-tip", "Fenix " + v.text);
      if (d.hostname) document.title = "Fenix · " + d.hostname + " · " + fenixTitlePage;

      // Update hint: only that there is one; details are for the teacher
      var up = updateState(d);
      document.getElementById("fxUpd").style.display = up.level === "warn" ? "" : "none";

      // Wifi: 4 bars from the signal strength
      var dbm = d.wifi_signal_dbm, bars = 0, wl = "";
      if (dbm !== null && dbm !== undefined) {
        bars = dbm > -55 ? 4 : dbm > -65 ? 3 : dbm > -72 ? 2 : dbm > -80 ? 1 : 0;
        wl = dbm > -60 ? "ok" : dbm >= -70 ? "warn" : "bad";
      }
      var w = document.getElementById("hiWifi");
      w.style.color = hdrColor(wl);
      document.querySelectorAll(".hiWifiBar").forEach(function (b, i) { b.setAttribute("opacity", i < bars ? "1" : ".25"); });
      var ping = pingState(d);
      w.setAttribute("data-tip", dbm === null || dbm === undefined ? "Wifi: geen signaal" :
        "Wifi " + dbm + " dBm" + (d.wifi_quality_percent != null ? " (" + d.wifi_quality_percent + "%)" : "") +
        (wifiChannel(d) ? "\n" + wifiChannel(d) + " breed (" + d.wifi_freq_mhz + " MHz)" : "") +
        "\nIP " + (d.ip || "--") + "\nNaar de AP: " + ping.text);

      // Pi temperature (+ power and fan in the tooltip)
      var t = piTemp(d), p = piPower(d), f = fanState(d);
      var worst = [t.level, p.level].indexOf("bad") >= 0 ? "bad" : [t.level, p.level].indexOf("warn") >= 0 ? "warn" : t.level;
      var te = document.getElementById("hiTemp");
      te.style.color = hdrColor(worst);
      var c = d.cpu_temp_c;
      if (c != null) {
        var h = Math.max(1, Math.min(9, (c - 30) / 50 * 9));   // 30-80 °C -> fill height
        var fill = document.getElementById("hiTempFill");
        fill.setAttribute("y", (13 - h).toFixed(1)); fill.setAttribute("height", h.toFixed(1));
      }
      te.setAttribute("data-tip", "Raspberry Pi " + t.text + "\nVoeding: " + p.text + (d.ext5v_v ? " (" + d.ext5v_v.toFixed(2) + " V)" : "") +
        "\nVentilator: " + f.text);

      var i = internetState(d), ie = document.getElementById("hiInet");
      ie.style.color = hdrColor(i.level);
      document.getElementById("hiInetSlash").style.display = d.internet && !d.internet.ip ? "" : "none";
      ie.setAttribute("data-tip", internetDetail(d));
    });
}
