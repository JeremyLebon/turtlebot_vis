// Buitenwerelden en extra sensoren: GPS, sonar, bumper, magnetometer op het bedrijfsterrein, boomgaard laadt.
// Gebruik: node tests/e2e_extra.cjs   (nep-rosbridge op poort 9092)
const chromium = require('@sparticuz/chromium'), puppeteer = require('puppeteer-core'), fs = require('fs'), path = require('path'), { WebSocketServer } = require('ws');
const NM = path.join(__dirname, '..', 'node_modules');
const EXPECT = { '/gps/fix': 'sensor_msgs/msg/NavSatFix', '/range/front': 'sensor_msgs/msg/Range', '/range/left': 'sensor_msgs/msg/Range',
  '/range/right': 'sensor_msgs/msg/Range', '/bumper': 'std_msgs/msg/Bool', '/imu/mag': 'sensor_msgs/msg/MagneticField',
  '/camera/depth/image_raw': 'sensor_msgs/msg/Image', '/camera/depth/camera_info': 'sensor_msgs/msg/CameraInfo',
  '/camera/depth/points': 'sensor_msgs/msg/PointCloud2', '/radar/points': 'sensor_msgs/msg/PointCloud2', '/points': 'sensor_msgs/msg/PointCloud2' };
const f32 = (b64, step, off) => { const b = Buffer.from(b64, 'base64'), out = []; for (let o = off; o + 4 <= b.length; o += step) out.push(b.readFloatLE(o)); return out; };
const adv = {}, last = {}, count = {}; let tfStatic = [], sensorState = null;
const wss = new WebSocketServer({ port: 9092 });
wss.on('connection', ws => ws.on('message', raw => {
  const m = JSON.parse(String(raw).replace(/\bInfinity\b/g, '1e999'));
  if (m.op === 'advertise') adv[m.topic] = m.type;
  if (m.op === 'call_service') return ws.send(JSON.stringify({ op: 'service_response', id: m.id, service: m.service, result: true, values: { publishers: [], subscribers: [], type: '' } }));
  if (m.op !== 'publish') return;
  if (EXPECT[m.topic]) { last[m.topic] = m.msg; count[m.topic] = (count[m.topic] || 0) + 1; }
  if (m.topic === '/tf_static') tfStatic = m.msg.transforms.map(t => t.child_frame_id);
  if (m.topic === '/sensor_state') sensorState = m.msg;
}));
const wait = ms => new Promise(r => setTimeout(r, ms));
let ok = true;
const check = (good, txt) => { ok &&= !!good; console.log(`${good ? '✓' : '✗'} ${txt}`); };
(async () => {
  const browser = await puppeteer.launch({ executablePath: await chromium.executablePath(), args: [...chromium.args, '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'], headless: true, defaultViewport: { width: 1300, height: 820 } });
  try {
    const page = await browser.newPage(); const errs = [];
    page.on('pageerror', e => errs.push('PAGEERROR ' + e.message));
    await page.setRequestInterception(true);
    page.on('request', r => { const u = r.url();
      if (u.startsWith('http://sim.local/')) return r.respond({ contentType: 'text/html', body: fs.readFileSync(path.join(__dirname, '..', 'dist', 'tb3-lite-sim-3d.html')) });
      const m = u.match(/cdn\.jsdelivr\.net\/npm\/(@?[^@]+)@[^/]+\/(.*)$/);
      if (m) { const f = path.join(NM, m[1], m[2]); if (fs.existsSync(f)) return r.respond({ contentType: 'application/javascript', body: fs.readFileSync(f) }); }
      r.abort(); });
    await page.goto('http://sim.local/?test', { waitUntil: 'domcontentloaded' }); await wait(3500);
    await page.evaluate(() => {
      document.querySelectorAll('.tabpane').forEach(p => { p.hidden = false; });
      const sel = document.getElementById('worldSel'); sel.value = 'yard'; sel.dispatchEvent(new Event('change'));
      for (const id of ['xGps', 'xSonar', 'xBump', 'xMag', 'xDepth', 'xRadar', 'l3On', 'l3Color']) document.getElementById(id).checked = true;
      document.getElementById('l3On').dispatchEvent(new Event('change'));
      // 1 m ten noorden van de oranje container (zuidkant op y = -4,28), naar het zuiden kijkend
      window.__tb3test.rebuild(-8.5, -3.3, -Math.PI / 2);
    });
    check(await page.$eval('#worldMsg', e => /Bedrijfsterrein/.test(e.textContent)), 'Bedrijfsterrein geladen');
    await page.$eval('#url', e => { e.value = 'ws://127.0.0.1:9092'; }); await page.click('#connect'); await wait(9000);
    for (const [t, ty] of Object.entries(EXPECT)) check(adv[t] === ty && count[t] > 0, `${t}  ${adv[t] || 'niet geadverteerd'}  ${count[t] || 0} berichten`);
    check(['gps_link', 'sonar_front_link', 'sonar_left_link', 'sonar_right_link'].every(f => tfStatic.includes(f)), `tf_static: ${tfStatic.join(', ')}`);
    const g = last['/gps/fix'], lat0 = 50.9528, lon0 = 3.1235;
    const n = (g.latitude - lat0) * Math.PI / 180 * 6378137, e = (g.longitude - lon0) * Math.PI / 180 * 6378137 * Math.cos(lat0 * Math.PI / 180);
    check(Math.hypot(e - -8.5, n - -3.25) < 1.2, `GPS terug naar wereld: O ${e.toFixed(2)} N ${n.toFixed(2)} (verwacht ≈ -8,50 / -3,25)`);
    const f = last['/range/front'].range;
    check(f > 0.75 && f < 1.0, `sonar voor ${f} m (container op ≈ 0,9 m)`);
    const mg = last['/imu/mag'].magnetic_field, hd = (Math.atan2(mg.y, mg.x) * 180 / Math.PI + 360) % 360;
    check(Math.abs(hd - 180) < 6, `kompas ${hd.toFixed(1)}° (zuiden = 180°)`);
    check(last['/bumper'].data === false, 'bumper vrij');
    // dieptecamera: midden van het beeld = container (camera 4 cm voor het midden van de robot)
    const di = last['/camera/depth/image_raw'], dz = f32(di.data, 4, 0), mid = dz[(di.height >> 1) * di.width + (di.width >> 1)];
    check(di.encoding === '32FC1' && di.width === 160 && mid > 0.8 && mid < 1.0, `diepte midden ${mid.toFixed(3)} m (≈ 0,9), ${dz.filter(z => !isNaN(z)).length}/${dz.length} geldig`);
    const dp = last['/camera/depth/points']; check(dp.width === 80 && dp.height === 60 && dp.is_dense === false, `dieptepunten ${dp.width}×${dp.height}`);
    // radar: detectie recht vooruit op ≈ 0,9 m, stilstaand
    const rp = last['/radar/points'], rx = f32(rp.data, 20, 0), ry = f32(rp.data, 20, 4), rv = f32(rp.data, 20, 12);
    const fi = rx.map((x, i) => [Math.hypot(x, ry[i]), Math.atan2(ry[i], x), rv[i]]).filter(([r, a]) => Math.abs(a) < 0.1);
    check(fi.some(([r]) => r > 0.75 && r < 1.0) && fi.every(([, , v]) => Math.abs(v) < 0.15), `radar: ${rp.width} detecties, vooruit ${fi.map(([r, , v]) => r.toFixed(2) + ' m ' + v.toFixed(2) + ' m/s').join(', ')}`);
    // gekleurde 3D-lidar: velden en kleur van de oranje container (#c8642c)
    const pc = last['/points'], names = pc.fields.map(f => f.name).join(','), buf = Buffer.from(pc.data, 'base64');
    const rgbOff = pc.fields.find(f => f.name === 'rgb')?.offset ?? -1;
    let orange = 0; for (let o = rgbOff; rgbOff >= 0 && o < buf.length; o += pc.point_step) if (buf.readUInt32LE(o) === 0xc8642c) orange++;
    check(names === 'x,y,z,intensity,ring,time,rgb' && orange > 20, `/points ${names}, ${orange} punten in de kleur van de container`);
    // naar de container rijden tot de bumper raakt
    await page.focus('body'); await page.keyboard.down('ArrowUp');
    let hit = false, vmin = 0;
    for (let i = 0; i < 60 && !hit; i++) {
      await wait(500); hit = last['/bumper'].data === true;
      const r = last['/radar/points'], x = f32(r.data, 20, 0), y = f32(r.data, 20, 4), v = f32(r.data, 20, 12);
      v.forEach((vv, k) => { if (Math.abs(Math.atan2(y[k], x[k])) < 0.1) vmin = Math.min(vmin, vv); });
    }
    check(vmin < -0.1, `radar Doppler tijdens het rijden: ${vmin.toFixed(2)} m/s (naderend = negatief)`);
    await page.keyboard.up('ArrowUp');
    check(hit, 'bumper raakt de container');
    await wait(600);
    check(sensorState && sensorState.bumper === 1, `/sensor_state bumper = ${sensorState && sensorState.bumper} (1 = voor)`);
    console.log('HUD:', (await page.$eval('#hud', e => e.textContent)).split('\n').filter(l => /gps|sonar|bumper|kompas/.test(l)).join(' | '));
    // boomgaard
    await page.evaluate(() => { const sel = document.getElementById('worldSel'); sel.value = 'orchard'; sel.dispatchEvent(new Event('change')); });
    await wait(1500);
    check(await page.$eval('#worldMsg', e => /Boomgaard/.test(e.textContent)), 'Boomgaard geladen: ' + await page.$eval('#worldMsg', e => e.textContent));
    // tussen twee rijen, naast een boom (plantafstand 1,8 m vanaf x = -8, rijen 3,5 m uit elkaar)
    await page.evaluate(() => window.__tb3test.rebuild(-4.4, -6.25, 0)); await wait(4000);
    const L = last['/range/left'].range, Rr = last['/range/right'].range, near = r => r > 1.3 && r < 2.0;
    check(near(L) || near(Rr), `sonar tussen de rijen: links ${L} m, rechts ${Rr} m (bomen op ≈ 1,7 m)`);
    // werelden met hoogteverschillen: robot op de grond, dieptecamera ziet de helling
    for (const [k, nm] of [['field', 'Akker'], ['hills', 'Heuvelland']]) {
      await page.evaluate(k => { const sel = document.getElementById('worldSel'); sel.value = k; sel.dispatchEvent(new Event('change')); }, k);
      await wait(3000);
      const r = await page.evaluate(() => { const t = window.__tb3test, p = t.sim.truthPose(); return { z: p.z, g: t.sim.groundZ(p.x, p.y), msg: document.getElementById('worldMsg').textContent }; });
      check(r.msg.includes(nm) && Math.abs(r.z - r.g) < 0.05, `${nm}: robot op z ${r.z.toFixed(3)}, grond ${r.g.toFixed(3)}`);
    }
    // robots per aandrijftype: laden en met de toetsen rijden (Bedrijfsterrein, vrije plek: bij (-6, 0) staat een doos)
    await page.evaluate(() => { const sel = document.getElementById('worldSel'); sel.value = 'yard'; sel.dispatchEvent(new Event('change')); });
    for (const k of ['skid', 'tracked', 'mecanum', 'ackermann']) {
      await page.evaluate(k => { const s = document.getElementById('presetSel'); s.value = k; s.dispatchEvent(new Event('change')); window.__tb3test.rebuild(-10, -1.5, 0); }, k);
      await wait(1500);
      const p0 = await page.evaluate(() => window.__tb3test.sim.truthPose());
      await page.focus('body');
      if (k === 'mecanum') { await page.keyboard.down('Shift'); await page.keyboard.down('ArrowLeft'); }
      else await page.keyboard.down('ArrowUp');
      await wait(5000);
      if (k === 'mecanum') { await page.keyboard.up('ArrowLeft'); await page.keyboard.up('Shift'); } else await page.keyboard.up('ArrowUp');
      const r = await page.evaluate(() => ({ p: window.__tb3test.sim.truthPose(), msg: document.getElementById('robotMsg').textContent }));
      const dx = r.p.x - p0.x, dy = r.p.y - p0.y;
      check(k === 'mecanum' ? dy > 0.1 && Math.abs(dx) < 0.05 : dx > 0.1, `${k}: ${r.msg.split('.')[0]}; gereden dx ${dx.toFixed(2)} dy ${dy.toFixed(2)} m`);
    }
    // rijknoppen in beeld: schuin (Ackermann, Burger), zijwaarts (mecanum); numpad 7 = vooruit + links
    const hold = async (sel, ms) => { const b = await page.$(sel), bb = await b.boundingBox(); await page.mouse.move(bb.x + bb.width / 2, bb.y + bb.height / 2); await page.mouse.down(); await wait(ms); await page.mouse.up(); };
    for (const [k, how] of [['ackermann', 'knop ◤'], ['', 'knop ◤'], ['mecanum', 'zijwaarts + ◀'], ['ackermann', 'numpad 7']]) {
      await page.evaluate(k => { if (k) { const s = document.getElementById('presetSel'); s.value = k; s.dispatchEvent(new Event('change')); } else document.getElementById('builtin').click();
        window.__tb3test.rebuild(-10, -1.5, 0); }, k);
      await wait(1500);
      const p0 = await page.evaluate(() => window.__tb3test.sim.truthPose());
      if (how === 'knop ◤') await hold('.pad .fl', 5000);
      else if (how === 'numpad 7') { await page.focus('body'); await page.keyboard.down('Numpad7'); await wait(5000); await page.keyboard.up('Numpad7'); }
      else { await page.click('#padStrafe'); await hold('.pad .l', 5000); await page.click('#padStrafe'); }
      const p = await page.evaluate(() => window.__tb3test.sim.truthPose()), dth = (p.th - p0.th) * 57.3, dx = p.x - p0.x, dy = p.y - p0.y;
      const good = how === 'zijwaarts + ◀' ? dy > 0.1 && Math.abs(dth) < 3 : dx > 0.1 && dy > 0.02 && dth > 10;
      check(good, `${k || 'burger'} met ${how}: dx ${dx.toFixed(2)} dy ${dy.toFixed(2)} m, Δθ ${dth.toFixed(0)}°`);
    }
    // slepen op de knoppen (joystick): ▲ indrukken, naar ◤ schuiven; daarna: robot kiezen in de lijst en meteen twee pijltjes
    { await page.evaluate(() => { const s = document.getElementById('presetSel'); s.value = 'ackermann'; s.dispatchEvent(new Event('change')); window.__tb3test.rebuild(-10, -1.5, 0); });
      await wait(1500); const p0 = await page.evaluate(() => window.__tb3test.sim.truthPose());
      const f = await (await page.$('.pad .f')).boundingBox(), fl = await (await page.$('.pad .fl')).boundingBox();
      await page.mouse.move(f.x + f.width / 2, f.y + f.height / 2); await page.mouse.down(); await wait(1500);
      await page.mouse.move(fl.x + fl.width / 2, fl.y + fl.height / 2, { steps: 5 }); await wait(3500); await page.mouse.up();
      const p = await page.evaluate(() => window.__tb3test.sim.truthPose());
      check(p.x - p0.x > 0.3 && (p.th - p0.th) > 0.3, `ackermann slepen ▲ → ◤: dx ${(p.x - p0.x).toFixed(2)} m, Δθ ${((p.th - p0.th) * 57.3).toFixed(0)}°`); }
    { await page.focus('#presetSel'); await page.select('#presetSel', 'mecanum');
      await page.evaluate(() => window.__tb3test.rebuild(-10, -1.5, 0)); await wait(1500);
      const p0 = await page.evaluate(() => window.__tb3test.sim.truthPose());
      await page.keyboard.down('ArrowUp'); await page.keyboard.down('ArrowLeft'); await wait(3000); await page.keyboard.up('ArrowLeft'); await page.keyboard.up('ArrowUp');
      const r = await page.evaluate(() => ({ p: window.__tb3test.sim.truthPose(), sel: document.getElementById('presetSel').value }));
      check(r.sel === 'mecanum' && r.p.x - p0.x > 0.2, `na de keuzelijst sturen de pijltjes de robot (lijst: ${r.sel}, dx ${(r.p.x - p0.x).toFixed(2)} m)`); }
    // joystick (optie): schuin omhoog-links slepen = vooruit + links; mecanum met zijwaarts: links = opzij
    await page.select('#padMode', 'joy'); await wait(300);
    for (const [k, strafe] of [['ackermann', false], ['mecanum', true]]) {
      await page.evaluate(k => { const s = document.getElementById('presetSel'); s.value = k; s.dispatchEvent(new Event('change')); window.__tb3test.rebuild(-10, -1.5, 0); }, k);
      await wait(1500); if (strafe) await page.click('#padStrafe');
      const p0 = await page.evaluate(() => window.__tb3test.sim.truthPose()), j = await (await page.$('#joy')).boundingBox();
      const cx = j.x + j.width / 2, cy = j.y + j.height / 2;
      await page.mouse.move(cx, cy); await page.mouse.down();
      await page.mouse.move(strafe ? cx - 60 : cx - 30, strafe ? cy : cy - 50, { steps: 5 }); await wait(4000); await page.mouse.up();
      if (strafe) await page.click('#padStrafe');
      const p = await page.evaluate(() => window.__tb3test.sim.truthPose()), dx = p.x - p0.x, dy = p.y - p0.y, dth = (p.th - p0.th) * 57.3;
      check(strafe ? dy > 0.3 && Math.abs(dth) < 3 : dx > 0.3 && dth > 10, `joystick ${k}${strafe ? ' zijwaarts' : ''}: dx ${dx.toFixed(2)} dy ${dy.toFixed(2)} m, Δθ ${dth.toFixed(0)}°`);
    }
    check(await page.$eval('#joyKnob', e => e.style.transform === ''), 'joystick veert terug naar het midden');
    await page.select('#padMode', 'buttons');
    await page.screenshot({ path: process.env.SHOT || '/dev/null' }).catch(() => {});
    check(!errs.length, errs.join('\n') || 'geen paginafouten');
  } finally { await browser.close(); wss.close(); }
  process.exit(ok ? 0 : 1);
})().catch(e => { console.log('FOUT', e.stack); process.exit(1); });
