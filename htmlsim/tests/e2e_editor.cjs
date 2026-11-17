// Robot-editor: sensorframes live verplaatsen, sensor toevoegen, /tf_static, URDF-export.
// Gebruik: node tests/e2e_editor.cjs [map voor schermafbeeldingen]   (nep-rosbridge op poort 9092)
const chromium = require('@sparticuz/chromium'), puppeteer = require('puppeteer-core'), fs = require('fs'), path = require('path'), { WebSocketServer } = require('ws');
const { DOMParser } = require('@xmldom/xmldom');
const NM = path.join(__dirname, '..', 'node_modules'), OUT = process.argv[2];
const tfStatic = [];
const wss = new WebSocketServer({ port: 9092 });
wss.on('connection', ws => ws.on('message', raw => {
  const m = JSON.parse(String(raw).replace(/\bInfinity\b/g, '1e999'));
  if (m.op === 'publish' && m.topic === '/tf_static') tfStatic.push(m.msg.transforms);
  if (m.op === 'call_service') return ws.send(JSON.stringify({ op: 'service_response', id: m.id, service: m.service, result: true, values: { publishers: [], subscribers: [], type: '' } }));
}));
const wait = ms => new Promise(r => setTimeout(r, ms));
let fails = 0; const ok = (name, c, info = '') => { console.log(`${c ? '✓' : '✗'} ${name}${info ? '  — ' + info : ''}`); if (!c) fails++; };
(async () => {
  const browser = await puppeteer.launch({ executablePath: await chromium.executablePath(), args: [...chromium.args, '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'], headless: true, defaultViewport: { width: 1400, height: 860 } });
  try {
    const page = await browser.newPage(); const errs = [];
    page.on('pageerror', e => errs.push('PAGEERROR ' + e.message));
    page.on('console', m => { if (m.type() === 'error' && !/ERR_FAILED/.test(m.text())) errs.push('CONSOLE ' + m.text()); });   // afgebroken CDN-/fontverzoeken
    await page.setRequestInterception(true);
    page.on('request', r => { const u = r.url();
      if (u.startsWith('http://sim.local/')) return r.respond({ contentType: 'text/html', body: fs.readFileSync(path.join(__dirname, '..', 'dist', 'tb3-lite-sim-3d.html')) });
      const m = u.match(/cdn\.jsdelivr\.net\/npm\/(@?[^@]+)@[^/]+\/(.*)$/);
      if (m) { const f = path.join(NM, m[1], m[2]); if (fs.existsSync(f)) return r.respond({ contentType: 'application/javascript', body: fs.readFileSync(f) }); }
      r.abort(); });
    await page.goto('http://sim.local/?test', { waitUntil: 'domcontentloaded' }); await wait(3500);
    await page.evaluate(() => { try { localStorage.clear(); } catch {} });
    // verbinden met de nep-rosbridge
    await page.evaluate(() => { document.getElementById('tabBtn-cfg').click(); document.getElementById('url').value = 'ws://127.0.0.1:9092'; document.getElementById('connect').click(); });
    await wait(1200);
    await page.evaluate(() => { document.getElementById('edSec').open = true; });
    await wait(300);
    const opts = await page.$$eval('#edJoint option', o => o.map(x => x.value));
    ok('lijst met verplaatsbare frames', opts.includes('scan_joint') && opts.includes('camera_joint') && opts.includes('imu_joint') && !opts.some(o => /optical|base_joint|wheel/.test(o)), opts.join(', '));
    ok('gizmo hangt aan het eerste sensorframe', await page.evaluate(() => { const t = window.__tb3test; return !!t.ed().sel; }));
    // lidar via de invoervelden 10 cm hoger
    await page.select('#edJoint', 'scan_joint'); await wait(200);
    const z0 = await page.evaluate(() => window.__tb3test.sim.linkWorld('base_scan').p.z);
    const pose0 = await page.evaluate(() => window.__tb3test.sim.truthPose());
    const nStatic = tfStatic.length;
    await page.evaluate(() => { const e = document.getElementById('edZ'); e.value = 272; e.dispatchEvent(new Event('change')); });
    await wait(400);
    const z1 = await page.evaluate(() => window.__tb3test.sim.linkWorld('base_scan').p.z);
    ok('lidar 100 mm hoger via invoerveld', Math.abs(z1 - z0 - 0.1) < 0.002, `Δz ${(z1 - z0).toFixed(4)} m`);
    const pose1 = await page.evaluate(() => window.__tb3test.sim.truthPose());
    ok('geen herbouw: robot op dezelfde plaats', Math.hypot(pose1.x - pose0.x, pose1.y - pose0.y) < 1e-3);
    const last = tfStatic.slice(nStatic).pop() || [], scanTf = last.find(t => t.child_frame_id === 'base_scan');
    ok('/tf_static opnieuw gepubliceerd met de nieuwe hoogte', scanTf && Math.abs(scanTf.transform.translation.z - 0.272) < 1e-6, scanTf ? `z ${scanTf.transform.translation.z}` : 'geen bericht');
    ok('optisch cameraframe blijft in /tf_static', last.some(t => t.child_frame_id === 'camera_rgb_optical_frame'));
    const info = await page.$eval('#edInfo', e => e.textContent);
    ok('info toont hoogte boven de vloer', /hoogte boven de vloer \d+ mm/.test(info), info.split('\n')[1]);
    await page.evaluate(() => { const e = document.getElementById('edZ'); e.value = 400; e.dispatchEvent(new Event('change')); });
    await wait(200);
    ok('waarschuwing bij scanvlak boven 35 cm', /boven 35 cm/.test(await page.$eval('#edInfo', e => e.textContent)));
    await page.evaluate(() => { const e = document.getElementById('edZ'); e.value = 200; e.dispatchEvent(new Event('change')); const p = document.getElementById('edP'); p.value = 5; p.dispatchEvent(new Event('change')); });
    await wait(200);
    const tiltInfo = await page.$eval('#edInfo', e => e.textContent);
    ok('waarschuwing bij hellend scanvlak', /helt [45]\.\d°: raakt de vloer op/.test(tiltInfo), tiltInfo.replace(/\n/g, ' | '));
    // camera naar beneden kantelen → het camerakader volgt (robotCam hangt aan de link)
    await page.select('#edJoint', 'camera_joint'); await wait(200);
    await page.evaluate(() => { const e = document.getElementById('edP'); e.value = 30; e.dispatchEvent(new Event('change')); });
    await wait(300);
    ok('camera kijkt schuin naar beneden', /schuin naar beneden \(-30°\)/.test(await page.$eval('#edInfo', e => e.textContent)));
    // gizmo: link verschuiven zoals een sleep met de muis, dan loslaten
    const giz = await page.evaluate(() => {
      const t = window.__tb3test, ed = t.ed(); ed.sel = 'imu_joint'; document.getElementById('edJoint').value = 'imu_joint'; document.getElementById('edJoint').dispatchEvent(new Event('change'));
      return t.sim.linkWorld('imu_link').p.y;
    });
    await page.evaluate(() => { const t = window.__tb3test; t.edApply('imu_joint', { p: { x: -0.032, y: 0.05, z: 0.068 }, q: { x: 0, y: 0, z: 0, w: 1 } }, true); });
    const giz1 = await page.evaluate(() => window.__tb3test.sim.linkWorld('imu_link').p.y);
    ok('edApply verplaatst ook de IMU', Math.abs(Math.abs(giz1 - giz) - 0.05) < 0.002);
    // sensor toevoegen: 2D-lidar op base_link
    await page.select('#edAddKind', 'lidar'); await page.select('#edAddParent', 'base_link');
    await page.click('#edAdd'); await wait(800);
    const added = await page.evaluate(() => { const t = window.__tb3test; return { sel: t.ed().sel, n: t.ed().added.length, lidar: document.getElementById('cfgLidar').value }; });
    ok('2D-lidar toegevoegd en als lidar van de sim gekozen', added.n === 1 && added.sel === 'laser_joint' && added.lidar === 'laser_link', JSON.stringify(added));
    const scanN = await page.evaluate(() => window.__tb3test.sim.scan('laser_link').pts.length);
    ok('nieuwe lidar scant', scanN > 50, `${scanN} punten`);
    if (OUT) { fs.mkdirSync(OUT, { recursive: true });
      await page.evaluate(() => { document.getElementById('edColl').checked = true; document.getElementById('edColl').dispatchEvent(new Event('change')); document.getElementById('edZoom').click(); });
      await wait(900); await page.screenshot({ path: path.join(OUT, 'editor_1.png') });
      await page.select('#edJoint', 'camera_joint'); await page.evaluate(() => document.querySelector('#edSec .seg button[data-g="rotate"]').click());
      await wait(600); await page.screenshot({ path: path.join(OUT, 'editor_2.png') });
      const pane = await page.$('.panel'); if (pane) await pane.screenshot({ path: path.join(OUT, 'editor_panel.png') });
    }
    // export
    const out = await page.$eval('#edOut', e => e.value);
    ok('wijzigingen met origin en static_transform_publisher', /scan_joint: base_link → base_scan/.test(out) && /laser_joint \(nieuw\)/.test(out) && /static_transform_publisher --x/.test(out), out.split('\n')[1]);
    const urdf = await page.evaluate(() => { let got = null; const orig = URL.createObjectURL; return new Promise(res => {
      HTMLAnchorElement.prototype.click = function () { fetch(this.href).then(r => r.text()).then(res); };
      document.getElementById('edDl').click(); setTimeout(() => res(null), 2000); }); });
    ok('URDF downloaden', !!urdf && /<robot/.test(urdf));
    if (urdf) {
      const doc = new DOMParser().parseFromString(urdf, 'text/xml'), J = n => Array.from(doc.getElementsByTagName('joint')).find(j => j.getAttribute('name') === n);
      const o = n => J(n) && J(n).getElementsByTagName('origin')[0];
      ok('export: lidar op 200 mm met 5° pitch', o('scan_joint') && /0\.2$/.test(o('scan_joint').getAttribute('xyz')) && Math.abs(+o('scan_joint').getAttribute('rpy').split(' ')[1] - 5 * Math.PI / 180) < 1e-4, o('scan_joint') && o('scan_joint').getAttribute('xyz') + ' / ' + o('scan_joint').getAttribute('rpy'));
      ok('export: nieuwe laser_link + joint', !!J('laser_joint') && Array.from(doc.getElementsByTagName('link')).some(l => l.getAttribute('name') === 'laser_link'));
      ok('export: niet-gewijzigde joints ongemoeid', /<joint name="wheel_left_joint" type="continuous">/.test(urdf));
      if (OUT) fs.writeFileSync(path.join(OUT, 'export.urdf'), urdf);
    }
    // terugzetten
    await page.select('#edJoint', 'scan_joint'); await page.click('#edResetAll'); await wait(300);
    ok('alles terugzetten', Math.abs(await page.evaluate(() => window.__tb3test.sim.linkWorld('base_scan').p.z) - z0) < 0.002);
    await page.click('#edDel').catch(() => {});
    await page.select('#edJoint', 'laser_joint').catch(() => {}); await page.click('#edDel'); await wait(600);
    ok('toegevoegde sensor verwijderen', await page.evaluate(() => window.__tb3test.ed().added.length === 0 && document.getElementById('cfgLidar').value === 'base_scan'));
    // editor dicht → gizmo weg
    await page.evaluate(() => { document.getElementById('edSec').open = false; }); await wait(300);
    ok('editor dicht: geen fouten in de pagina', errs.length === 0, errs.slice(0, 3).join(' | '));
  } finally { await Promise.race([browser.close(), wait(5000)]); for (const c of wss.clients) c.terminate(); wss.close(); }
  console.log(fails ? `\n${fails} test(s) mislukt` : '\nAlle tests geslaagd'); process.exit(fails ? 1 : 0);
})();
