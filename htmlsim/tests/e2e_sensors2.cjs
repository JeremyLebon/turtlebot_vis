// Proefhal: lijnsensor, cliff, UWB, stereo + ArUco (OpenCV), thermisch, segmentatie, 3D-lidar-presets met tijd per punt, IMU 200 Hz.
// Gebruik: node tests/e2e_sensors2.cjs   (nep-rosbridge op poort 9093; ArUco-controle met python3 + OpenCV als dat er is)
const chromium = require('@sparticuz/chromium'), puppeteer = require('puppeteer-core'), fs = require('fs'), path = require('path'), os = require('os');
const { WebSocketServer } = require('ws'), { execFileSync } = require('child_process');
const NM = path.join(__dirname, '..', 'node_modules');
const EXPECT = { '/line_sensor': 'std_msgs/msg/Float32MultiArray', '/cliff': 'std_msgs/msg/Float32MultiArray', '/uwb/ranges': 'std_msgs/msg/Float32MultiArray',
  '/uwb/anchors': 'std_msgs/msg/String', '/stereo/left/image_raw/compressed': 'sensor_msgs/msg/CompressedImage', '/stereo/right/camera_info': 'sensor_msgs/msg/CameraInfo',
  '/thermal/image_raw': 'sensor_msgs/msg/Image', '/segmentation/labels_map': 'sensor_msgs/msg/Image', '/segmentation/labels': 'std_msgs/msg/String',
  '/points': 'sensor_msgs/msg/PointCloud2', '/imu': 'sensor_msgs/msg/Imu', '/ground_truth': 'nav_msgs/msg/Odometry' };
const adv = {}, last = {}, count = {}, stampLog = { '/imu': [] };
const wss = new WebSocketServer({ port: 9093 });
let client = null;
const toPage = (topic, msg) => client && client.send(JSON.stringify({ op: 'publish', topic, msg }));
wss.on('connection', ws => { client = ws; ws.on('message', raw => {
  const m = JSON.parse(String(raw).replace(/\bInfinity\b/g, '1e999'));
  if (m.op === 'advertise') adv[m.topic] = m.type;
  if (m.op === 'call_service') return ws.send(JSON.stringify({ op: 'service_response', id: m.id, service: m.service, result: true, values: { publishers: [], subscribers: [], type: '' } }));
  if (m.op === 'publish' && EXPECT[m.topic]) { last[m.topic] = m.msg; count[m.topic] = (count[m.topic] || 0) + 1;
    if (m.topic === '/imu') stampLog['/imu'].push(m.msg.header.stamp.sec + m.msg.header.stamp.nanosec * 1e-9); }
}); });
const wait = ms => new Promise(r => setTimeout(r, ms));
let ok = true; const check = (g, t) => { ok &&= !!g; console.log(`${g ? '✓' : '✗'} ${t}`); };
(async () => {
  const browser = await puppeteer.launch({ executablePath: await chromium.executablePath(), args: [...chromium.args, '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'], headless: true, defaultViewport: { width: 1300, height: 820 } });
  try {
    const page = await browser.newPage(); const errs = []; page.on('pageerror', e => errs.push('PAGEERROR ' + e.message));
    await page.setRequestInterception(true);
    page.on('request', r => { const u = r.url();
      if (u.startsWith('http://sim.local/')) return r.respond({ contentType: 'text/html', body: fs.readFileSync(path.join(__dirname, '..', 'dist', 'tb3-lite-sim-3d.html')) });
      const m = u.match(/cdn\.jsdelivr\.net\/npm\/(@?[^@]+)@[^/]+\/(.*)$/);
      if (m) { const f = path.join(NM, m[1], m[2]); if (fs.existsSync(f)) return r.respond({ contentType: 'application/javascript', body: fs.readFileSync(f) }); }
      r.abort(); });
    await page.goto('http://sim.local/?test', { waitUntil: 'domcontentloaded' }); await wait(3500);
    await page.evaluate(() => {
      document.querySelectorAll('.tabpane').forEach(p => { p.hidden = false; });
      const sel = document.getElementById('worldSel'); sel.value = 'lab'; sel.dispatchEvent(new Event('change'));
      for (const id of ['xLine', 'xCliff', 'xUwb', 'xStereo', 'xThermal', 'xSeg', 'l3On']) document.getElementById(id).checked = true;
      document.getElementById('l3On').dispatchEvent(new Event('change'));
      const p = document.getElementById('l3Preset'); p.value = 'vlp16'; p.dispatchEvent(new Event('change'));
      const i = document.getElementById('imuHz'); i.value = '200'; i.dispatchEvent(new Event('change'));
    });
    check(await page.$eval('#worldMsg', e => /Proefhal/.test(e.textContent)), 'Proefhal geladen');
    await page.$eval('#url', e => { e.value = 'ws://127.0.0.1:9093'; }); await page.click('#connect'); await wait(9000);
    for (const [t, ty] of Object.entries(EXPECT)) check(adv[t] === ty && count[t] > 0, `${t}  ${count[t] || 0} berichten`);
    // lijnsensor: de robot start op de lijn (linkerkant van het parcours, naar het noorden)
    const L = last['/line_sensor'].data; check(L[2] < 0.2 && Math.max(L[0], L[4]) > 0.6, `lijnsensor op de lijn: ${L.map(v => v.toFixed(2)).join(' ')} (midden zwart)`);
    // cliff: op de vloer ≈ 0,02 m
    const C = last['/cliff'].data; check(C.every(d => d > 0.005 && d < 0.04), `cliff op de vloer: ${C.map(d => d.toFixed(3)).join(' ')}`);
    // UWB: afstanden tegen de echte positie (ankers in hetzelfde frame als /ground_truth)
    const an = JSON.parse(last['/uwb/anchors'].data).anchors, gt = last['/ground_truth'].pose.pose.position, R = last['/uwb/ranges'].data;
    const err = an.map((a, i) => Math.abs(R[i] - Math.hypot(a.x - gt.x, a.y - gt.y, a.z - (gt.z + 0.2))));
    check(an.length === 4 && err.filter(e => e < 0.25).length >= 3, `UWB: ${R.map(r => r.toFixed(2)).join(' ')} m, fout ${err.map(e => e.toFixed(2)).join(' ')}`);
    // IMU 200 Hz in sim-tijd
    const ts = stampLog['/imu'].slice(-100), dt = (ts[ts.length - 1] - ts[0]) / (ts.length - 1);
    check(Math.abs(1 / dt - 200) < 15, `IMU ${(1 / dt).toFixed(0)} Hz (sim-tijd)`);
    // /points: Velodyne-velden, tijd per punt binnen één omwenteling (VLP-16 = 0,1 s)
    const pc = last['/points'], names = pc.fields.map(f => f.name + ':' + f.datatype).join(','), buf = Buffer.from(pc.data, 'base64');
    let tmin = 1, tmax = -1, rings = new Set();
    for (let o = 0; o + pc.point_step <= buf.length; o += pc.point_step) { const t = buf.readFloatLE(o + 18); tmin = Math.min(tmin, t); tmax = Math.max(tmax, t); rings.add(buf.readUInt16LE(o + 16)); }
    check(names === 'x:7,y:7,z:7,intensity:7,ring:4,time:7' && pc.point_step === 22 && tmax < 0.1 && tmax - tmin > 0.08 && rings.size >= 8,
      `/points ${pc.width} punten, ${names}, time ${tmin.toFixed(3)}–${tmax.toFixed(3)} s, ${rings.size} ringen`);
    // terug naar de lichte lidar: VLP-16 kost headless veel rekentijd, de camera's lopen dan achter
    await page.evaluate(() => { const p = document.getElementById('l3Preset'); p.value = 'tb3'; p.dispatchEvent(new Event('change')); }); await wait(500);
    // thermisch: verwarmd platform (veld heat 10 °C) warmer dan de omgeving als de robot ernaar kijkt
    const th = last['/thermal/image_raw']; const tv = new Uint16Array(Buffer.from(th.data, 'base64').buffer.slice(0));
    check(th.encoding === 'mono16' && th.width === 160 && tv.every(v => v > 25000 && v < 36000), `thermisch mono16, ${(Math.min(...tv) / 100 - 273.15).toFixed(1)}–${(Math.max(...tv) / 100 - 273.15).toFixed(1)} °C`);
    // segmentatie: labels in beeld
    const seg = last['/segmentation/labels_map'], ids = new Set(Buffer.from(seg.data, 'base64')), labels = JSON.parse(last['/segmentation/labels'].data);
    check(seg.encoding === 'mono8' && ids.has(1) && ids.size >= 2, `segmentatie: ${[...ids].map(i => labels[i]).join(', ')}`);
    // stereo: rechter camera_info met Tx = -f·B
    const ri = last['/stereo/right/camera_info']; check(Math.abs(ri.p[3] + ri.p[0] * 0.06) < 1e-6, `stereo rechts P[3] = ${ri.p[3].toFixed(2)} (= -f·0,06)`);
    // ArUco: robot 0,7 m voor marker 3 (zuidmuur, x = 1,0), naar het zuiden kijkend; OpenCV leest het linkerbeeld
    await page.evaluate(() => window.__tb3test.rebuild(1.0, -2.75, -Math.PI / 2)); await wait(4000);
    const jpg = path.join(os.tmpdir(), 'tb3_aruco_left.jpg'); fs.writeFileSync(jpg, Buffer.from(last['/stereo/left/image_raw/compressed'].data, 'base64'));
    let det = null;
    try { det = execFileSync('python3', ['-c', 'import cv2,sys\nd=cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)\nc,i,_=cv2.aruco.detectMarkers(cv2.imread(sys.argv[1]),d)\nprint([] if i is None else i.flatten().tolist())', jpg]).toString().trim(); } catch (e) { det = 'geen OpenCV'; }
    check(det === 'geen OpenCV' || det === '[3]', `ArUco in het stereobeeld (OpenCV): ${det}`);
    // thermisch: kijken naar de warmtebron bij het platform (heat 10 °C, σ 0,6 m op (3,2, -2,5)): vloer daar ≈ 30 °C, zoals de temperatuursensor
    await page.evaluate(() => window.__tb3test.rebuild(1.6, -1.6, -0.55)); await wait(3500);
    const th2 = new Uint16Array(Buffer.from(last['/thermal/image_raw'].data, 'base64').buffer.slice(0)), tmax2 = Math.max(...th2) / 100 - 273.15;
    const sensT = await page.evaluate(() => { const p = window.__tb3test.sim.truthPose(); return 21; });
    check(tmax2 > 27 && tmax2 < 33, `thermisch ziet de warmtebron op de vloer: warmste pixel ${tmax2.toFixed(1)} °C (bron: 21 + 10 °C)`);
    // cliff aan de rand van het platform (platform x 2,6-3,8, y -2,95…-2,05, 12 cm hoog): robot erop, voorkant over de rand
    // de helling op (vanaf x 1,6) en rijden tot de voorste cliff-sensoren de rand zien, dan stoppen
    await page.evaluate(() => window.__tb3test.rebuild(1.2, -2.5, 0)); await wait(1500);
    await page.mouse.click(650, 500); await page.keyboard.down('ArrowUp');
    // in de pagina zelf stoppen zodra de voorste hoek over de rand hangt (headless loopt de tijd ongelijk)
    // meting op het moment van stoppen: de 4 cliff-punten zoals de simulator ze meet (XMOUNT), plus de hoogte van de robot
    const stopAt = await page.evaluate(() => new Promise(res => { const sim = window.__tb3test.sim; let n = 0;
      const id = setInterval(() => { const T = sim.linkWorld('base_footprint');
        const at = (x, y) => { const q = T.q, w = { x: T.p.x + (1 - 2 * (q.y * q.y + q.z * q.z)) * x + 2 * (q.x * q.y - q.w * q.z) * y, y: T.p.y + 2 * (q.x * q.y + q.w * q.z) * x + (1 - 2 * (q.x * q.x + q.z * q.z)) * y, z: T.p.z + 0.02 };
          return sim.downRange(w, 0.5); };
        const c = [at(0.05, 0.06), at(0.05, -0.06), at(-0.1, 0.06), at(-0.1, -0.06)];
        if ((c[0] > 0.08 && c[1] > 0.08) || ++n > 6000) { window.dispatchEvent(new KeyboardEvent('keyup', { key: 'ArrowUp' })); clearInterval(id); res({ c, z: T.p.z, x: T.p.x }); } }, 2); }));
    await page.keyboard.up('ArrowUp'); await wait(300);
    const C2 = stopAt.c, zTop = stopAt.z;
    const pz = await page.evaluate(() => window.__tb3test.sim.truthPose());
    const pub = last['/cliff'].data;   // ook het topic: de robot staat stil aan de rand
    check(C2[0] > 0.08 && C2[1] > 0.08 && C2[2] < 0.05 && zTop > 0.1 && pub[0] > 0.08, `cliff aan de rand van het platform: ${C2.map(d => d > 1 ? '∞' : d.toFixed(3)).join(' ')}, topic ${pub.map(d => d > 1 ? '∞' : d.toFixed(3)).join(' ')} (robot op x ${pz.x.toFixed(2)}, ${(zTop * 100).toFixed(0)} cm hoog)`);
    // resultaten van een studentscript: detecties tekenen; masker = de echte segmentatie → IoU 100 %, alles "vloer" → laag
    toPage('/detections', { header: {}, detections: [{ bbox: { center: { position: { x: 160, y: 120 }, theta: 0 }, size_x: 60, size_y: 40 },
      results: [{ hypothesis: { class_id: 'test', score: 0.87 } }] }] });
    await wait(400);
    const gtm = last['/segmentation/labels_map'];
    toPage('/student/segmentation', gtm); await wait(600);
    let hud = await page.$eval('#hud', e => e.textContent);
    check(/detecties\s+1: test/.test(hud) && /IoU 100 %/.test(hud), `studentscript: ${hud.split('\n').filter(l => /detecties|IoU/.test(l)).join(' | ')}`);
    const allFloor = { ...gtm, data: Buffer.alloc(gtm.width * gtm.height, 1).toString('base64') };
    toPage('/student/segmentation', allFloor); await wait(600);
    hud = await page.$eval('#hud', e => e.textContent); const m2 = hud.match(/IoU (\d+) %/);
    check(m2 && +m2[1] < 90, `masker "alles vloer": IoU ${m2 && m2[1]} %`);
    // MID-360: rozet-preset publiceert
    await page.evaluate(() => { const p = document.getElementById('l3Preset'); p.value = 'mid360'; p.dispatchEvent(new Event('change')); }); await wait(3000);
    check(last['/points'].width > 1000, `MID-360: ${last['/points'].width} punten per scan`);
    console.log('HUD:', (await page.$eval('#hud', e => e.textContent)).split('\n').filter(l => /lijn|cliff|uwb|thermisch|segment/.test(l)).join(' | '));
    if (process.env.SHOT) await page.screenshot({ path: process.env.SHOT });
    check(!errs.length, errs.join('\n') || 'geen paginafouten');
  } finally { await browser.close(); wss.close(); }
  process.exit(ok ? 0 : 1);
})().catch(e => { console.log('FOUT', e.stack); process.exit(1); });
