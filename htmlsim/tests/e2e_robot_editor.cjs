// Aparte robot-editor (dist/robot-editor.html): robot bouwen, aanklikken, verslepen, export, doorsturen naar de simulator.
// Gebruik: node tests/e2e_robot_editor.cjs [map voor schermafbeeldingen]
const chromium = require('@sparticuz/chromium'), puppeteer = require('puppeteer-core'), fs = require('fs'), path = require('path');
const R = require('@dimforge/rapier3d-compat'), { DOMParser } = require('@xmldom/xmldom');
const NM = path.join(__dirname, '..', 'node_modules'), OUT = process.argv[2], DIST = path.join(__dirname, '..', 'dist');
const core = fs.readFileSync(path.join(__dirname, '../src/core.js'), 'utf8');
const C = new Function('R', core + ';return {createSim,parseURDF,analyzeModel,WORLDS};')();
const wait = ms => new Promise(r => setTimeout(r, ms));
let fails = 0; const ok = (name, c, info = '') => { console.log(`${c ? '✓' : '✗'} ${name}${info ? '  — ' + info : ''}`); if (!c) fails++; };
(async () => {
  await R.init();
  const browser = await puppeteer.launch({ executablePath: await chromium.executablePath(), args: [...chromium.args, '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'], headless: true, defaultViewport: { width: 1440, height: 880 } });
  const errs = [];
  const open = async (file, hash = '') => {
    const page = await browser.newPage();
    page.on('pageerror', e => errs.push(file + ' PAGEERROR ' + e.message));
    page.on('console', m => { if (m.type() === 'error' && !/ERR_FAILED/.test(m.text())) errs.push(file + ' CONSOLE ' + m.text()); });
    await page.setRequestInterception(true);
    page.on('request', r => { const u = r.url();
      const loc = u.match(/^http:\/\/sim\.local\/([^?#]*)/); if (loc) { const f = path.join(DIST, loc[1] || 'tb3-lite-sim-3d.html'); return fs.existsSync(f) ? r.respond({ contentType: 'text/html', body: fs.readFileSync(f) }) : r.abort(); }
      const m = u.match(/cdn\.jsdelivr\.net\/npm\/(@?[^@]+)@[^/]+\/(.*)$/);
      if (m) { const f = path.join(NM, m[1], m[2]); if (fs.existsSync(f)) return r.respond({ contentType: 'application/javascript', body: fs.readFileSync(f) }); }
      r.abort(); });
    await page.goto(`http://sim.local/${file}?test${hash}`, { waitUntil: 'domcontentloaded' });
    return page;
  };
  try {
    let ed = await open('robot-editor.html'); await wait(2500);
    await ed.evaluate(() => { try { localStorage.clear(); } catch {} }); await ed.reload({ waitUntil: 'domcontentloaded' }); await wait(2500);
    const tree0 = await ed.$$eval('#tree li', l => l.map(x => x.dataset.l));
    ok('start met de Burger', tree0.includes('wheel_left_link') && tree0.includes('base_scan'), `${tree0.length} links`);
    ok('controle: aandrijving herkend', /Aandrijving herkend/.test(await ed.$eval('#checks', e => e.textContent)));
    // lege robot en opbouwen
    await ed.click('#newBtn'); await ed.click('[data-new="empty"]'); await wait(400);
    ok('lege robot: geen aandrijving', /Geen aandrijfwielen/.test(await ed.$eval('#checks', e => e.textContent)));
    await ed.evaluate(() => window.__edtest.select('base_link'));
    await ed.click('[data-add="wheels"]'); await wait(300);
    let chk = await ed.$eval('#checks', e => e.textContent);
    ok('wielen (paar): aandrijving herkend, kantelwaarschuwing', /Aandrijving herkend: wheel_left_joint \+ wheel_right_joint/.test(chk) && /kantelt/.test(chk), chk.split('.')[0]);
    await ed.evaluate(() => window.__edtest.select('base_link')); await ed.click('[data-add="caster"]'); await wait(300);
    await ed.evaluate(() => window.__edtest.select('base_link')); await ed.click('[data-add="lidar"]'); await wait(300);
    await ed.evaluate(() => window.__edtest.select('base_link')); await ed.click('[data-add="camera"]'); await wait(300);
    chk = await ed.$eval('#checks', e => e.textContent);
    ok('zwenkwiel en lidar in orde', /Zwenkwiel raakt de vloer/.test(chk) && /2D-lidar base_scan op/.test(chk) && !/kantelt/.test(chk), chk.replace(/\s+/g, ' ').slice(0, 160));
    // aanklikken in 3D: lidar
    const scr = await ed.evaluate(() => {
      const t = window.__edtest, o = t.nodes().base_scan.vis[0], v = o.getWorldPosition(new o.position.constructor());
      const cam = t.tc.camera; v.project(cam); const r = document.querySelector('#stage canvas').getBoundingClientRect();
      return { x: r.left + (v.x + 1) / 2 * r.width, y: r.top + (1 - v.y) / 2 * r.height };
    });
    await ed.mouse.click(scr.x, scr.y); await wait(300);
    ok('lidar aanklikken in 3D selecteert hem', await ed.evaluate(() => document.getElementById('propTitle').textContent) === 'base_scan');
    // echte sleep met de muis aan de z-pijl van de gizmo
    const z0 = await ed.evaluate(() => window.__edtest.M.joints.find(j => j.child === 'base_scan').origin.p.z);
    const handle = await ed.evaluate(() => {
      // middelpunt van de pick-zone "Z" van de translate-gizmo (onzichtbare picker in TransformControls)
      const t = window.__edtest, r = document.querySelector('#stage canvas').getBoundingClientRect(); let z = null;
      t.tc.updateMatrixWorld(true);
      const picker = t.tc._gizmo.picker.translate; picker.updateMatrixWorld(true);
      picker.traverse(o => { if (!z && o.name === 'Z' && o.isMesh) z = o; });
      const c = z.geometry.boundingBox || (z.geometry.computeBoundingBox(), z.geometry.boundingBox);
      const w = c.getCenter(t.tc.camera.position.clone()).applyMatrix4(z.matrixWorld).project(t.tc.camera);
      return { a: { x: r.left + (w.x + 1) / 2 * r.width, y: r.top + (1 - w.y) / 2 * r.height } };
    });
    await ed.mouse.move(handle.a.x, handle.a.y); await wait(100); await ed.mouse.down(); await wait(50);
    for (let i = 1; i <= 8; i++) { await ed.mouse.move(handle.a.x, handle.a.y - i * 8); await wait(30); }
    await ed.mouse.up(); await wait(300);
    const z1 = await ed.evaluate(() => window.__edtest.M.joints.find(j => j.child === 'base_scan').origin.p.z);
    ok('lidar met de muis omhoog gesleept (z-pijl)', z1 > z0 + 0.005 && Math.abs(z1 * 1000 - Math.round(z1 * 200) * 5) < 1e-6, `${(z0 * 1000).toFixed(0)} → ${(z1 * 1000).toFixed(1)} mm (snap 5 mm)`);
    ok('na het slepen: geschiedenis + invoerveld bijgewerkt', +(await ed.$eval('#pZ', e => e.value)) === +(z1 * 1000).toFixed(1));
    // spiegelen + ongedaan maken
    await ed.evaluate(() => window.__edtest.select('camera_link'));
    await ed.evaluate(() => { const j = window.__edtest.M.joints.find(j => j.child === 'camera_link'); j.origin.p.y = 0.04; window.__edtest.commit(); });
    await ed.click('#aMir'); await wait(300);
    const mir = await ed.evaluate(() => { const M = window.__edtest.M, j = M.joints.find(j => j.child === 'camera_spiegel_link'); return j && { y: j.origin.p.y, opt: M.joints.some(x => x.parent === 'camera_spiegel_link') }; });
    ok('spiegelen: kopie op −y, met optisch frame eronder', mir && Math.abs(mir.y + 0.04) < 1e-9 && mir.opt, JSON.stringify(mir));
    await ed.keyboard.down('Control'); await ed.keyboard.press('z'); await ed.keyboard.up('Control'); await wait(300);
    ok('Ctrl+Z maakt het spiegelen ongedaan', await ed.evaluate(() => !window.__edtest.M.links.camera_spiegel_link));
    // armsegment met preview
    await ed.evaluate(() => window.__edtest.select('base_link')); await ed.click('[data-add="arm"]'); await wait(300);
    ok('armsegment: revolute met limieten', await ed.evaluate(() => { const j = window.__edtest.M.joints.find(j => j.child === 'arm_link'); return j.type === 'revolute' && j.limit.upper > 1.5; }));
    await ed.evaluate(() => { const r = document.getElementById('pPrev'); r.value = 0.8; r.dispatchEvent(new Event('input')); }); await wait(200);
    if (OUT) { fs.mkdirSync(OUT, { recursive: true }); await ed.screenshot({ path: path.join(OUT, 'robot_editor_1.png') }); }
    await ed.click('#aDel'); await wait(200);
    // vorm: maat via eigenschappen
    await ed.evaluate(() => window.__edtest.select('base_link'));
    await ed.evaluate(() => { const e = document.getElementById('gX'); e.value = 260; e.dispatchEvent(new Event('change')); });
    ok('maat van de plaat aangepast, collision volgt', await ed.evaluate(() => { const L = window.__edtest.M.links.base_link; return Math.abs(L.visuals[0].geom.size[0] - 0.26) < 1e-9 && Math.abs(L.collisions[0].geom.size[0] - 0.26) < 1e-9; }));
    // massa, inertie, wrijving, jointparameters
    ok('controle toont totale massa en zwaartepunt', /Totale massa [\d.]+ kg · zwaartepunt/.test(await ed.$eval('#checks', e => e.textContent)));
    ok('inertie automatisch: tekst met zwaartepunt en ixx', /zwaartepunt .* mm · ixx/.test(await ed.$eval('#props', e => e.textContent)));
    await ed.evaluate(() => { const e = document.getElementById('mMode'); e.value = 'man'; e.dispatchEvent(new Event('change')); }); await wait(200);
    const izz0 = await ed.evaluate(() => window.__edtest.M.links.base_link.inertial.I.zz);
    await ed.evaluate(() => { const e = document.getElementById('mKg'); e.value = 1.6; e.dispatchEvent(new Event('change')); }); await wait(200);
    const ie1 = await ed.evaluate(() => window.__edtest.M.links.base_link.inertial);
    ok('handmatige inertie schaalt mee met de massa', !ie1.auto && Math.abs(ie1.I.zz / izz0 - 2) < 1e-6, `izz ${izz0.toPrecision(3)} → ${ie1.I.zz.toPrecision(3)}`);
    if (OUT) { const pane = await ed.$('.edright'); await pane.screenshot({ path: path.join(OUT, 'robot_editor_props.png') }); }
    await ed.evaluate(() => { const e = document.getElementById('iZZ'); e.value = 0.05; e.dispatchEvent(new Event('change')); }); await wait(200);
    await ed.evaluate(() => window.__edtest.select('caster_back_link'));
    await ed.evaluate(() => { const e = document.getElementById('mMu'); e.value = 0.05; e.dispatchEvent(new Event('change')); }); await wait(200);
    await ed.evaluate(() => window.__edtest.select('base_link')); await ed.click('[data-add="arm"]'); await wait(300);
    await ed.evaluate(() => { const e = document.getElementById('pEff'); e.value = 3; const v = document.getElementById('pVel'); v.value = 0.8; e.dispatchEvent(new Event('change')); }); await wait(200);
    await ed.evaluate(() => { const v = document.getElementById('pVel'); v.value = 0.8; v.dispatchEvent(new Event('change')); }); await wait(200);
    await ed.evaluate(() => window.__edtest.select('wheel_left_link'));
    await ed.evaluate(() => { const e = document.getElementById('pDmp'); e.value = 0.01; e.dispatchEvent(new Event('change')); }); await wait(200);
    const dynU = await ed.evaluate(() => window.__edtest.exportText());
    ok('export: handmatige izz, µ zwenkwiel, effort/velocity arm, demping wiel',
      /izz="0\.05"/.test(dynU) && /<gazebo reference="caster_back_link"><mu1>0\.05<\/mu1><mu2>0\.05<\/mu2><\/gazebo>/.test(dynU) && /effort="3" velocity="0\.8"/.test(dynU) && /<dynamics damping="0\.01"\/>/.test(dynU),
      (dynU.match(/<limit[^>]*effort="3"[^>]*>/) || ['geen arm-limit'])[0]);
    await ed.evaluate(() => window.__edtest.select('arm_link')); await ed.click('#aDel'); await wait(200);
    await ed.evaluate(() => { const e = document.getElementById('robotName'); e.value = 'testbot'; e.dispatchEvent(new Event('change')); });
    if (OUT) { await ed.evaluate(() => window.__edtest.select('base_scan')); await wait(300); await ed.screenshot({ path: path.join(OUT, 'robot_editor_2.png') }); }
    // export → rijdt in de fysica
    const urdf = await ed.evaluate(() => window.__edtest.exportText());
    if (OUT) fs.writeFileSync(path.join(OUT, 'testbot.urdf'), urdf);
    const m = C.parseURDF(urdf, DOMParser), cfg = C.analyzeModel(m).cfg, s = C.createSim(R); s.loadWorld(C.WORLDS.empty); s.buildRobot(m, cfg, 0, 0, 0);
    for (let i = 0; i < 2 / s.DT; i++) s.step({ v: 0.2, w: 0 });
    const p = s.truthPose(), q = s.robot.rootBody.rb.rotation(), tilt = 2 * Math.asin(Math.min(1, Math.hypot(q.x, q.y)));
    ok('zelfgebouwde robot rijdt rechtdoor in de simulator', p.x > 0.3 && Math.abs(p.y) < 0.03 && tilt < 0.05, `x ${p.x.toFixed(3)} y ${p.y.toFixed(3)} kanteling ${(tilt * 57.3).toFixed(1)}°`);
    const sc = s.scan(cfg.lidar);
    ok('lidar van de zelfgebouwde robot scant', sc.pts.length > 50, `${sc.pts.length} punten`);
    // naar de simulator (open tabblad, BroadcastChannel)
    const sim = await open('tb3-lite-sim-3d.html'); await wait(3500);
    await ed.bringToFront(); await ed.click('#toSimBtn'); await wait(1500);
    const simMsg = await sim.$eval('#robotMsg', e => e.textContent);
    ok('"Testen in de simulator" laadt de robot in het open simulatortabblad', /robot-editor: testbot/.test(simMsg), simMsg.split('\n')[0]);
    ok('editor meldt dat de simulator hem kreeg', /Robot staat in de simulator/.test(await ed.$eval('#msg', e => e.textContent)));
    // simulator → editor
    await sim.evaluate(() => { document.getElementById('tabBtn-cfg').click(); });
    const pages0 = (await browser.pages()).length;
    await sim.evaluate(() => { HTMLAnchorElement.prototype._c = HTMLAnchorElement.prototype.click; HTMLAnchorElement.prototype.click = function () { window.__opened = this.href; }; document.getElementById('openEditor').click(); });
    const opened = await sim.evaluate(() => window.__opened);
    ok('simulator opent de editor met #from-sim', /robot-editor\.html#from-sim$/.test(opened || ''), opened);
    const ed2 = await open('robot-editor.html', '#from-sim'); await wait(2500);
    ok('editor laadt de robot uit de simulator', /Uit de simulator/.test(await ed2.$eval('#msg', e => e.textContent)) && await ed2.evaluate(() => window.__edtest.M.name) === 'testbot');
    // zoals in een artifact: beide pagina's in een iframe, navigatie in hetzelfde kader
    const fr = await browser.newPage(); await fr.setRequestInterception(true);
    fr.on('request', r => { const u = r.url();
      if (u === 'http://sim.local/frame.html') return r.respond({ contentType: 'text/html', body: '<!doctype html><body style="margin:0"><iframe id="f" src="tb3-lite-sim-3d.html" style="width:1400px;height:860px;border:0"></iframe>' });
      const loc = u.match(/^http:\/\/sim\.local\/([^?#]*)/); if (loc) { const f = path.join(DIST, loc[1]); return fs.existsSync(f) ? r.respond({ contentType: 'text/html', body: fs.readFileSync(f) }) : r.abort(); }
      const m = u.match(/cdn\.jsdelivr\.net\/npm\/(@?[^@]+)@[^/]+\/(.*)$/);
      if (m) { const f = path.join(NM, m[1], m[2]); if (fs.existsSync(f)) return r.respond({ contentType: 'application/javascript', body: fs.readFileSync(f) }); }
      r.abort(); });
    fr.on('pageerror', e => errs.push('frame PAGEERROR ' + e.message));
    await fr.goto('http://sim.local/frame.html'); await wait(3500);
    const frame = () => fr.frames().find(f => f !== fr.mainFrame());
    await frame().evaluate(() => document.getElementById('editorLink').click()); await wait(3000);
    ok('in een iframe: Robot-editor opent in hetzelfde kader', /robot-editor\.html#from-sim$/.test(frame().url()), frame().url());
    for (let i = 0; i < 30; i++) { if (await frame().evaluate(() => !!(document.getElementById('toSimBtn') && document.getElementById('toSimBtn').onclick && document.getElementById('msg').textContent)).catch(() => false)) break; await wait(300); }
    await frame().evaluate(() => { const e = document.getElementById('robotName'); e.value = 'kaderbot'; e.dispatchEvent(new Event('change')); });
    await frame().evaluate(() => document.getElementById('toSimBtn').click());
    for (let i = 0; i < 40; i++) { await wait(500); const t = await frame().evaluate(() => document.getElementById('robotMsg') && document.getElementById('robotMsg').textContent).catch(() => ''); if (/kaderbot/.test(t || '')) break; }
    const fm = await frame().evaluate(() => document.getElementById('robotMsg') && document.getElementById('robotMsg').textContent);
    ok('in een iframe: Testen in de simulator gaat terug met de robot', /tb3-lite-sim-3d\.html#editor-robot$/.test(frame().url()) && /robot-editor: kaderbot/.test(fm || ''), (fm || frame().url()).split('\n')[0]);
    ok('geen fouten in de pagina\'s', errs.length === 0, errs.slice(0, 3).join(' | '));
  } finally { await Promise.race([browser.close(), wait(5000)]); }
  console.log(fails ? `\n${fails} test(s) mislukt` : '\nAlle tests geslaagd'); process.exit(fails ? 1 : 0);
})();
