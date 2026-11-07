// Omgevingssensoren: publiceert de simulator /sensors/... met de types van de robot (sim_sensors_node.py)?
// Gebruik: node tests/e2e_sensors.cjs   (poort 9090 moet vrij zijn)
const chromium = require('@sparticuz/chromium'), puppeteer = require('puppeteer-core'), fs = require('fs'), path = require('path'), { WebSocketServer } = require('ws');
const NM = path.join(__dirname, '..', 'node_modules');
const EXPECT = { '/sensors/temperature': 'sensor_msgs/msg/Temperature', '/sensors/humidity': 'sensor_msgs/msg/RelativeHumidity',
  '/sensors/pressure': 'sensor_msgs/msg/FluidPressure', '/sensors/illuminance': 'sensor_msgs/msg/Illuminance',
  '/sensors/gas/co2': 'std_msgs/msg/Float32', '/sensors/gas/tvoc': 'std_msgs/msg/Float32', '/sensors/gas/co': 'std_msgs/msg/Float32',
  '/sensors/sound': 'std_msgs/msg/Float32', '/sensors/summary': 'std_msgs/msg/String', '/sim/temperature': 'sensor_msgs/msg/Temperature' };
const adv = {}, last = {}, count = {};
const wss = new WebSocketServer({ port: 9090 });
wss.on('connection', ws => ws.on('message', raw => {
  const m = JSON.parse(String(raw).replace(/\bInfinity\b/g, '1e999'));
  if (m.op === 'advertise') adv[m.topic] = m.type;
  if (m.op === 'call_service') return ws.send(JSON.stringify({ op: 'service_response', id: m.id, service: m.service, result: true, values: { publishers: [], subscribers: [], type: '' } }));
  if (m.op === 'publish' && /^\/(sensors|sim)\//.test(m.topic)) { last[m.topic] = m.msg; count[m.topic] = (count[m.topic] || 0) + 1; }
}));
const wait = ms => new Promise(r => setTimeout(r, ms));
(async () => {
  const browser = await puppeteer.launch({ executablePath: await chromium.executablePath(), args: [...chromium.args, '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'], headless: true, defaultViewport: { width: 1300, height: 820 } });
  const page = await browser.newPage(); const errs = [];
  page.on('pageerror', e => errs.push('PAGEERROR ' + e.message));
  await page.setRequestInterception(true);
  page.on('request', r => { const u = r.url();
    if (u === 'http://sim.local/') return r.respond({ contentType: 'text/html', body: fs.readFileSync(path.join(__dirname, '..', 'dist', 'tb3-lite-sim-3d.html')) });
    const m = u.match(/cdn\.jsdelivr\.net\/npm\/(@?[^@]+)@[^/]+\/(.*)$/);
    if (m) { const f = path.join(NM, m[1], m[2]); if (fs.existsSync(f)) return r.respond({ contentType: 'application/javascript', body: fs.readFileSync(f) }); }
    r.abort(); });
  await page.goto('http://sim.local/', { waitUntil: 'domcontentloaded' }); await wait(3500);
  await page.evaluate(() => document.querySelectorAll('.tabpane').forEach(p => { p.hidden = false; }));
  await page.click('#connect'); await wait(12000);
  let ok = true;
  for (const [t, ty] of Object.entries(EXPECT)) {
    const good = adv[t] === ty && count[t] > 0; ok &&= good;
    console.log(`${good ? '✓' : '✗'} ${t}  ${adv[t] || 'niet geadverteerd'}  ${count[t] || 0} berichten  ${JSON.stringify(last[t]).slice(0, 90)}`);
  }
  console.log('HUD:', (await page.$eval('#hud', e => e.innerText)).split('\n').filter(l => /gas|temp|vocht|licht|geluid/.test(l)).join(' | '));
  console.log(errs.join('\n') || '(geen paginafouten)');
  await browser.close(); wss.close(); process.exit(ok && !errs.length ? 0 : 1);
})().catch(e => { console.log('FOUT', e.stack); process.exit(1); });
