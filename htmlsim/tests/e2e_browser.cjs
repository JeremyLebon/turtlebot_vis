// End-to-end: dist/tb3-lite-sim-3d.html in headless Chromium + nep-rosbridge (poort 9090) met gesimuleerde studentnodes.
// Gebruik: node tests/e2e_browser.cjs [t1|t1x|t2|t3|t3x] [--bad]   (--bad = bewust foute studentoplossing)
// Omgeving: STU, NR (studentnaam/-nummer), MIDSHOT=<ms> (tussentijdse schermafbeelding), KEEP=1 (uitvoer bewaren)
const chromium = require('@sparticuz/chromium'), puppeteer = require('puppeteer-core'), fs = require('fs'), path = require('path'), { WebSocketServer } = require('ws');
const NM = path.join(__dirname, '..', 'node_modules'), OUT = path.join(__dirname, 'e2e_out');
if (!process.env.KEEP) fs.rmSync(OUT, { recursive: true, force: true }); fs.mkdirSync(OUT, { recursive: true });
const BAD = process.argv.includes('--bad');
const wss = new WebSocketServer({ port: 9090 });
let sock = null; const send = o => sock && sock.readyState === 1 && sock.send(JSON.stringify(o));
const log = (name, level, msg) => send({ op: 'publish', topic: '/rosout', msg: { level, name, msg } });
const st = { alarm: false, odom: null, task: null, t3: null };
const yaw = q => Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
let nCmd = 0, nOdom = 0;
// TWIST=1: het studentscript publiceert Twist (Humble, labo-robot) i.p.v. TwistStamped (Jazzy)
const CMD_TYPE = process.env.TWIST ? 'geometry_msgs/msg/Twist' : 'geometry_msgs/msg/TwistStamped'; let subCmdType = '';
const twist = (v, w) => ({ linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w } });
const cmd = (v, w) => { nCmd++; if (subCmdType && subCmdType !== CMD_TYPE) return;   // ander type: komt in ROS 2 niet aan
  send({ op: 'publish', topic: '/cmd_vel', msg: CMD_TYPE.endsWith('Stamped') ? { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'base_link' }, twist: twist(v, w) } : twist(v, w) }); };
wss.on('connection', ws => {
  sock = ws;
  ws.on('message', raw => {
    const m = JSON.parse(String(raw).replace(/\bInfinity\b/g, '1e999'));
    if (m.op === 'call_service') {                                    // rosapi-nabootsing: student publiceert op /cmd_vel
      const tp = m.args && m.args.topic, v = m.service === '/rosapi/topic_type' ? { type: tp === '/cmd_vel' ? CMD_TYPE : '' }
        : m.service === '/rosapi/subscribers' ? { subscribers: [] }
        : { publishers: tp === '/cmd_vel' ? ['/rosbridge_websocket', '/student_node'] : ['/rosbridge_websocket'] };
      return send({ op: 'service_response', id: m.id, service: m.service, result: true, values: v });
    }
    if (m.op === 'subscribe' && m.topic === '/cmd_vel') subCmdType = m.type;
    if (m.op !== 'publish') return;
    if (m.topic === '/scan' && (st.task === 't1' || st.task === 't1x')) {
      const r = m.msg.ranges, n = r.length, ok = x => Number.isFinite(x) && x > 0.12;
      const front = [...r.slice(0, 15), ...r.slice(-15)].filter(BAD ? (x => Number.isFinite(x)) : ok);
      const d = front.length ? Math.min(...front) : Infinity;
      if (st.task === 't1') {
        if (BAD) { if (d < 0.3) log('obstacle_alarm', 30, `obstacle at ${d.toFixed(2)} m!`); }            // geen flank, geen filter
        else if (d < 0.3 && !st.alarm) { st.alarm = true; log('obstacle_alarm', 30, `obstacle at ${d.toFixed(2)} m!`); }
        else if (d >= 0.3 && st.alarm) { st.alarm = false; log('obstacle_alarm', 20, 'free again'); }
      } else {
        const q = Math.floor(n / 4), k = Math.round((15 * Math.PI / 180) / m.msg.angle_increment);
        const sec = arr => { const v = arr.filter(ok); return v.length ? Math.min(...v).toFixed(2) : 'inf'; };
        const all = r.map((x, i) => [x, i]).filter(([x]) => ok(x)).sort((a, b) => a[0] - b[0])[0];
        const ang = all ? (all[1] * m.msg.angle_increment * 180 / Math.PI) : 0;
        log('obstacle_alarm', 20, `front ${sec([...r.slice(0, k), ...r.slice(-k)])} m | left ${sec(r.slice(q - k, q + k))} m | behind ${sec(r.slice(2 * q - k, 2 * q + k))} m | right ${sec(r.slice(3 * q - k, 3 * q + k))} m | closest at ${ang.toFixed(0)}°`);
      }
    }
    if (m.topic === '/sim/battery_state') {
      const v = m.msg.voltage, p = m.msg.percentage;
      log('battery_warning', 20, `voltage ${v.toFixed(2)} V, percentage ${(p * 100).toFixed(0)} %`);
      if (BAD ? p * 100 < 50 * 100 : (v < 11.2 || p < 0.5)) log('battery_warning', 30, 'battery low!');
    }
    if (m.topic === '/odom') nOdom++;
    if (m.topic === '/odom' && st.t3) {
      const o = m.msg.pose.pose, s = st.t3;
      if (!s.start) s.start = { x: o.position.x, y: o.position.y, th: yaw(o.orientation) };
      const th = yaw(o.orientation), dth = Math.atan2(Math.sin(th - s.th0), Math.cos(th - s.th0));
      if (s.mode === 'turn') {
        if (s.th0 === undefined) { s.th0 = th; return; }
        const rem = Math.PI / 2 - dth;
        if (rem > 0.002) cmd(0, Math.max(0.08, Math.min(0.6, 1.5 * rem))); else { cmd(0, 0); st.t3 = null; log('turn_90', 20, `turned ${(dth * 180 / Math.PI).toFixed(1)} deg`); }
      } else if (s.mode === 'square') {
        if (s.phase === undefined) Object.assign(s, { phase: 0, leg: 0, x0: o.position.x, y0: o.position.y, th0: th });
        if (s.phase === 0) {                                                   // rechtdoor 0,5 m
          const dist = Math.hypot(o.position.x - s.x0, o.position.y - s.y0), rem = 0.5 - dist;
          if (rem > 0.002) cmd(Math.max(0.03, Math.min(0.2, 1.5 * rem)), 0); else { cmd(0, 0); Object.assign(s, { phase: 1, th0: th }); }
        } else {                                                               // draai 90°
          const rem = Math.PI / 2 - dth;
          if (rem > 0.003) cmd(0, Math.max(0.08, Math.min(0.6, 1.5 * rem)));
          else { cmd(0, 0); s.leg++; if (s.leg >= 4) { st.t3 = null; return; } Object.assign(s, { phase: 0, x0: o.position.x, y0: o.position.y }); }
        }
      }
    }
  });
});
const wait = ms => new Promise(r => setTimeout(r, ms));
(async () => {
  const browser = await puppeteer.launch({ executablePath: await chromium.executablePath(), args: [...chromium.args, '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required'], headless: true, defaultViewport: { width: 1300, height: 820 } });
  const page = await browser.newPage(); const errs = [];
  page.on('console', m => { if (!/GPU stall|ERR_FAILED|Failed to load/.test(m.text())) errs.push(m.type() + ': ' + m.text()); }); page.on('pageerror', e => errs.push('PAGEERROR ' + e.message));
  const cdp = await page.createCDPSession(); await cdp.send('Browser.setDownloadBehavior', { behavior: 'allow', downloadPath: OUT });
  await page.setRequestInterception(true);
  page.on('request', r => { const u = r.url();
    if (u === 'http://sim.local/') return r.respond({ contentType: 'text/html', body: fs.readFileSync(path.join(__dirname, '..', 'dist', 'tb3-lite-sim-3d.html')) });
    const m = u.match(/cdn\.jsdelivr\.net\/npm\/(@?[^@]+)@[^/]+\/(.*)$/);
    if (m) { const f = path.join(NM, m[1], m[2]); if (fs.existsSync(f)) return r.respond({ contentType: 'application/javascript', body: fs.readFileSync(f) }); }
    r.abort(); });
  await page.goto('http://sim.local/', { waitUntil: 'domcontentloaded' }); await wait(3500);
  await page.type('#stuName', process.env.STU || 'Jan Peeters'); await page.type('#stuNr', process.env.NR || 'r0123456'); await page.click('#stuSet');
  await page.click('#connect'); await wait(1500);
  console.log('cmdInfo na verbinden:', await page.$eval('#cmdInfo', e => e.textContent), '| abonnement:', subCmdType);
  if (process.env.COMPAT) { await page.click('#compatBtn'); await wait(1500); console.log('compat:', await page.$$eval('#compatOut li', ls => ls.map(l => (l.dataset.s || '-') + ' ' + l.innerText.replace(/\n/g, ' — ')).join(' | '))); }
  const only = process.argv.find(a => /^t\d/.test(a));
  for (const task of only ? [only] : ['t1', 't1x', 't2', 't3', 't3x']) {
    st.task = task; st.alarm = false;
    await page.select('#taskSel', task); await wait(300);
    await page.click('#taskStart');
    if (task === 't3' || task === 't3x') { await wait(2500); st.t3 = { mode: task === 't3' ? 'turn' : 'square' }; }
    const t0 = Date.now();
    if (process.env.MIDSHOT) { await wait(+process.env.MIDSHOT); await page.screenshot({ path: path.join(OUT, 'shot_mid_' + task + '.png') }); }
    while (Date.now() - t0 < 240000) { await wait(1000); const ms = await page.$eval('#taskMsg', e => e.textContent); if (/GESLAAGD/.test(ms)) break;
      if ((Date.now()-t0) % 20000 < 1000) console.log('  …', Math.round((Date.now()-t0)/1000)+'s', 'cmd', nCmd, 'odom', nOdom, 'hud:', (await page.$eval('#hud', e => e.innerText)).split('\n').slice(1,4).join(' | '), 'st.t3', JSON.stringify(st.t3)); }
    const msg = await page.$eval('#taskMsg', e => e.textContent);
    const checks = await page.$$eval('#taskChecks li', ls => ls.map(l => `${l.dataset.s === 'ok' ? '✓' : l.dataset.s === 'fail' ? '✗' : l.dataset.s}  ${l.querySelector('span').textContent}${l.querySelector('small') ? '  — ' + l.querySelector('small').textContent : ''}`));
    console.log(`\n=== ${task}: ${msg}\n` + checks.join('\n'));
    if (await page.$eval('#taskOut', e => !e.hidden)) { await page.click('#dlReport'); await wait(800); }
    if (await page.$eval('#taskOut', e => !e.hidden) && await page.$eval('#dlVideo', e => !e.hidden)) { await page.click('#dlVideo'); await wait(1500); }
    st.t3 = null;
  }
  await page.screenshot({ path: path.join(OUT, 'shot_e2e.png') });
  console.log('\nCONSOLE:', errs.slice(0, 10).join('\n') || '(geen)');
  await browser.close(); wss.close(); process.exit(0);
})().catch(e => { console.log('FOUT', e.stack); process.exit(1); });
