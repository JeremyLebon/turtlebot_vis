// Sessie 4: meetpunten en bronnen per studentnummer (Klaslokaal) en de taak "Project: meetronde".
// Gebruik: node tests/e2e_project.cjs [map voor schermafbeeldingen]   (nep-rosbridge op poort 9091)
const chromium = require('@sparticuz/chromium'), puppeteer = require('puppeteer-core'), fs = require('fs'), path = require('path'), { WebSocketServer } = require('ws');
const NM = path.join(__dirname, '..', 'node_modules'), OUT = process.argv[2];
const wss = new WebSocketServer({ port: 9091 });
wss.on('connection', ws => ws.on('message', raw => {
  const m = JSON.parse(String(raw).replace(/\bInfinity\b/g, '1e999'));
  if (m.op === 'call_service') return ws.send(JSON.stringify({ op: 'service_response', id: m.id, service: m.service, result: true, values: { publishers: [], subscribers: [], type: '' } }));
}));
const wait = ms => new Promise(r => setTimeout(r, ms));
let fails = 0; const ok = (name, c, info = '') => { console.log(`${c ? '✓' : '✗'} ${name}${info ? '  — ' + info : ''}`); if (!c) fails++; };
(async () => {
  const browser = await puppeteer.launch({ executablePath: await chromium.executablePath(), args: [...chromium.args, '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'], headless: true, defaultViewport: { width: 1400, height: 860 } });
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
    await page.evaluate(() => { try { localStorage.clear(); } catch {} });
    await page.select('#worldSel', 'classroom'); await wait(800);
    const setStu = async (name, nr) => page.evaluate(async (name, nr) => {
      document.querySelector('.student').classList.remove('set');
      document.getElementById('stuName').value = name; document.getElementById('stuNr').value = nr; document.getElementById('stuSet').click();
    }, name, nr);
    const planOf = async () => { await wait(400); return page.evaluate(() => ({ pts: window.__tb3test.planPtsMap(), fields: window.__tb3test.plan.fields })); };
    await setStu('Jan Peeters', 'r0123456'); const a = await planOf();
    await setStu('Jan Peeters', 'r0123456'); const a2 = await planOf();
    await setStu('An Smet', 'r0999999'); const b = await planOf();
    ok('3 meetpunten voor r0123456', a.pts.length === 3, JSON.stringify(a.pts));
    ok('zelfde nummer → zelfde punten en bronnen', JSON.stringify(a) === JSON.stringify(a2));
    ok('ander nummer → andere punten', JSON.stringify(a.pts) !== JSON.stringify(b.pts), JSON.stringify(b.pts));
    ok('ander nummer → andere bronnen', JSON.stringify(a.fields) !== JSON.stringify(b.fields));
    const dmin = p => Math.min(...p.flatMap((q, i) => p.slice(i + 1).map(r => Math.hypot(q.x - r.x, q.y - r.y))));
    ok('punten ≥ 1,5 m uit elkaar', dmin(a.pts) >= 1.5 && dmin(b.pts) >= 1.5, `${dmin(a.pts).toFixed(2)} / ${dmin(b.pts).toFixed(2)} m`);
    ok('5 bronnen, één per soort', a.fields.length === 5 && new Set(a.fields.map(f => f.kind)).size === 5);
    // spreiding over veel nummers
    const seen = new Set(); for (let k = 0; k < 40; k++) { await setStu('X', 'r0' + String(100000 + k * 7919).padStart(6, '0')); seen.add(JSON.stringify((await planOf()).pts)); }
    ok('40 nummers → verschillende puntensets', seen.size >= 38, `${seen.size} verschillend`);
    await setStu('Jan Peeters', 'r0123456'); await wait(400);
    console.log('Paneel:', await page.$eval('#stuPlan', e => e.innerText.replace(/\n/g, ' | ')));
    // frames: P1-P3 in het echte map-frame (odomOrigin + map → odom), niet vast t.o.v. de startpose
    const fr = await page.evaluate(() => {
      const t = window.__tb3test, s = t.sim, P = t.plan.pts[0], r = v => +v.toFixed(3);
      const startOk = JSON.stringify(t.planPtsMap()[0]) === JSON.stringify((m => ({ punt: 'P1', x: +m.x.toFixed(2), y: +m.y.toFixed(2), yaw_deg: Math.round(m.yaw * 180 / Math.PI) }))(t.toMap(P.x, P.y, P.yaw)));
      t.rebuild(P.x, P.y, P.yaw);                      // robot op P1 vóór het mappen: map-oorsprong = P1
      const moved = t.worldToMap(P.x, P.y, P.yaw);
      const s2 = Math.SQRT1_2; t.setMapOdom({ p: { x: 1, y: 0.5, z: 0 }, q: { x: 0, y: 0, z: s2, w: s2 } });   // AMCL: odom 90° gedraaid
      const tf = t.worldToMap(P.x, P.y, P.yaw);
      t.setMapOdom(null); t.rebuild();
      // kaartanker: Navigatie met een opgeslagen kaart → anker gaat voor op /tf (AMCL)
      localStorage.setItem('tb3sim.mapAnchors', JSON.stringify({ ['Klaslokaal|ankertest']: { x: P.x, y: P.y, yaw: P.yaw } }));
      Object.assign(t.navState, { navigation: true, map: 'ankertest' }); t.setMapOdom({ p: { x: 5, y: 5, z: 0 }, q: { x: 0, y: 0, z: 0, w: 1 } });
      const an = t.worldToMap(P.x, P.y, P.yaw);
      Object.assign(t.navState, { navigation: false, map: null }); t.setMapOdom(null); localStorage.removeItem('tb3sim.mapAnchors');
      return { startOk, moved: [r(moved.x), r(moved.y), r(moved.yaw)], tf: [r(tf.x), r(tf.y), r(tf.yaw)], live: tf.live, an: [r(an.x), r(an.y), r(an.yaw)], anSrc: an.src };
    });
    ok('zonder kaart en robot op de start: P1 t.o.v. de startpose', fr.startOk);
    ok('robot verplaatst vóór het mappen: P1 volgt de nieuwe oorsprong', Math.hypot(fr.moved[0], fr.moved[1]) < 0.01 && Math.abs(fr.moved[2]) < 0.01, JSON.stringify(fr.moved));
    ok('map → odom uit /tf wordt toegepast', Math.abs(fr.tf[0] - 1) < 0.01 && Math.abs(fr.tf[1] - 0.5) < 0.01 && Math.abs(fr.tf[2] - Math.PI / 2) < 0.01 && fr.live, JSON.stringify(fr.tf));
    ok('kaartanker gaat voor op /tf bij Navigatie', fr.anSrc === 'kaart' && Math.hypot(fr.an[0], fr.an[1]) < 0.01 && Math.abs(fr.an[2]) < 0.01, JSON.stringify(fr.an) + ' ' + fr.anSrc);
    if (OUT) {
      await page.evaluate(() => { document.querySelectorAll('.tabpane').forEach(p => { p.hidden = p.id !== 'tab-task'; }); document.querySelectorAll('[role=tab]').forEach(t => t.setAttribute('aria-selected', t.id === 'tabBtn-task')); });
      await wait(500); await page.screenshot({ path: path.join(OUT, 'proj_paneel.png') });
    }
    // projecttaak: verbinden, starten zonder opname, robot naar P1 en P2 zetten, stoppen
    await page.evaluate(() => { document.querySelectorAll('.tabpane').forEach(p => { p.hidden = false; }); });
    await page.$eval('#url', e => { e.value = 'ws://127.0.0.1:9091'; }); await page.$eval('#connect', e => e.click()); await wait(4000);
    await page.select('#taskSel', 'proj'); await page.select('#recSrc', 'none');
    await page.$eval('#taskStart', e => e.click()); await wait(3000);
    for (const i of [0, 1]) {
      await page.evaluate(i => { const t = window.__tb3test, p = t.plan.pts[i]; t.rebuild(p.x + 0.1, p.y, p.yaw); }, i);
      await wait(4000);
    }
    if (OUT) await page.screenshot({ path: path.join(OUT, 'proj_rit.png') });
    const checks = await page.$$eval('#taskChecks li', ls => ls.map(l => l.dataset.s + ' ' + l.innerText.replace(/\n/g, ' ')));
    console.log(checks.join('\n'));
    await page.$eval('#taskStop', e => e.click()); await wait(2500);
    const rep = await page.evaluate(() => { const b = [...document.querySelectorAll('#taskMsg')][0].textContent; return b; });
    console.log('Melding:', rep);
    const ch = await page.$$eval('#taskChecks li', ls => ls.map(l => l.dataset.s));
    ok('P1 en P2 bereikt, P3 niet', ch[0] === 'ok' && ch[1] === 'ok' && ch[2] === 'fail', ch.join(','));
    ok('resultaat NIET GESLAAGD (P3 ontbreekt)', /NIET GESLAAGD/.test(rep));
    console.log(errs.join('\n') || '(geen paginafouten)'); ok('geen paginafouten', !errs.length);
  } finally { await browser.close(); wss.close(); }
  process.exit(fails ? 1 : 0);
})().catch(e => { console.log('FOUT', e.stack); process.exit(1); });
