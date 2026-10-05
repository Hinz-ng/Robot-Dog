#pragma once
#include <Arduino.h>
// ============================================================================
// web_page.h -- the phone terminal page, served from flash by web_term.h.
// ============================================================================
// Self-contained: no external scripts, fonts or CDNs (the phone is on the
// ESP32's own network with no internet). Phone-first layout:
//   header: link dot, bus/log pills, big STOP
//   joint cards (tap one to make it the target of the quick buttons)
//   quick buttons -> the terminal -> command input
// Every quick button just sends the same text you could type, and the
// terminal echoes it, so nothing here is hidden behaviour.
// ============================================================================

static const char WEB_PAGE[] PROGMEM = R"PAGE(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0f1115">
<title>Leg master</title>
<style>
:root{--bg:#0f1115;--panel:#171a21;--line:#262b36;--text:#d7dce5;--dim:#7d8596;
--acc:#4d9fff;--ok:#3ecf72;--warn:#f0b43c;--bad:#ff5a5a;--echo:#6fd3e8;}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;height:100%;background:var(--bg);color:var(--text);
font:14px/1.35 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
body{display:flex;flex-direction:column;height:100dvh}
header{display:flex;align-items:center;gap:8px;padding:8px 10px;background:var(--panel);
border-bottom:1px solid var(--line)}
.title{font-weight:700;letter-spacing:.3px}
.dot{width:10px;height:10px;border-radius:50%;background:#555;flex:none}
.dot.on{background:var(--ok);box-shadow:0 0 6px var(--ok)}
.pill{font-size:11px;padding:2px 7px;border-radius:10px;background:#222833;color:var(--dim);white-space:nowrap}
.pill.bad{background:#3a1717;color:var(--bad)} .pill.rec{background:#3a2c10;color:var(--warn)}
.spacer{flex:1}
#stop{background:var(--bad);color:#fff;border:0;border-radius:10px;font-weight:800;
font-size:16px;padding:11px 18px;letter-spacing:.5px;box-shadow:0 0 0 2px #ff9a9a33}
#stop:active{transform:scale(.97)}
#estopBar{display:none;background:#5a1414;color:#fff;padding:8px 10px;align-items:center;gap:8px;font-weight:600}
#estopBar.show{display:flex}
#estopBar button{margin-left:auto}
.top{padding:8px 10px 4px;display:flex;flex-direction:column;gap:8px}
.cards{display:flex;gap:8px;overflow-x:auto}
.card{flex:1 0 150px;background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:8px 10px}
.card.sel{border-color:var(--acc);box-shadow:0 0 0 1px var(--acc) inset}
.card .h{display:flex;align-items:center;gap:6px;font-weight:700}
.chip{font-size:10px;font-weight:700;padding:1px 6px;border-radius:8px;background:#2a2f3a;color:var(--dim)}
.chip.arm{background:#13351f;color:var(--ok)} .chip.flt{background:#3a1717;color:var(--bad)}
.chip.ph{background:#3a2c10;color:var(--warn)} .chip.gone{background:#222;color:#666}
.card .r{display:flex;justify-content:space-between;font:12px/1.5 ui-monospace,Menlo,Consolas,monospace;color:var(--dim)}
.card .r b{color:var(--text);font-weight:500}
.tgt{display:flex;gap:6px;align-items:center;font-size:12px;color:var(--dim)}
.seg{display:flex;border:1px solid var(--line);border-radius:8px;overflow:hidden}
.seg button{background:transparent;border:0;color:var(--dim);padding:6px 12px;font-weight:600}
.seg button.on{background:var(--acc);color:#fff}
.btns{display:grid;grid-template-columns:repeat(5,1fr);gap:6px}
.btns button,.btn{background:#1f2430;color:var(--text);border:1px solid var(--line);border-radius:8px;
padding:9px 4px;font-size:13px;font-weight:600;min-height:40px}
.btns button:active,.btn:active{background:#2a3140}
.btns .g{color:var(--ok)} .btns .y{color:var(--warn)} .btns .r{color:var(--bad)}
details summary{color:var(--dim);font-size:12px;cursor:pointer;padding:2px 0}
.tbar{display:flex;align-items:center;gap:6px;margin:4px 10px 0;font-size:12px;color:var(--dim)}
.tbar .btn{min-height:32px;padding:4px 12px;font-size:12px}
#copyBox{display:none;position:fixed;inset:0;background:#000c;z-index:10;flex-direction:column;padding:12px;gap:8px}
#copyBox.show{display:flex}
#copyBox textarea{flex:1;background:#0a0c10;color:var(--text);border:1px solid var(--acc);border-radius:8px;
font:12px/1.4 ui-monospace,Menlo,Consolas,monospace;padding:8px}
#copyBox .hint{color:#fff;font-weight:600}
#term{flex:1;overflow-y:auto;margin:4px 10px 0;padding:8px;background:#0a0c10;border:1px solid var(--line);
border-radius:10px;font:12px/1.4 ui-monospace,Menlo,Consolas,monospace;white-space:pre-wrap;word-break:break-all}
#term .ev{color:var(--warn)} #term .err{color:var(--bad)} #term .echo{color:var(--echo)}
#term .sys{color:var(--dim);font-style:italic}
#newBtn{display:none;position:fixed;right:18px;bottom:76px;background:var(--acc);color:#fff;border:0;
border-radius:16px;padding:6px 12px;font-weight:700}
form{display:flex;gap:6px;padding:8px 10px calc(8px + env(safe-area-inset-bottom))}
#in{flex:1;min-width:0;background:#0a0c10;color:var(--text);border:1px solid var(--line);border-radius:8px;
padding:10px;font:15px ui-monospace,Menlo,Consolas,monospace}
#in:focus{outline:none;border-color:var(--acc)}
form .btn{padding:0 14px}
#send{background:var(--acc);border-color:var(--acc);color:#fff}
</style></head><body>

<header>
  <span class="dot" id="dot"></span><span class="title">Leg master</span>
  <span class="pill" id="busPill">bus ?</span>
  <span class="pill" id="logPill" style="display:none">REC</span>
  <span class="spacer"></span>
  <button id="stop" onclick="doStop()">STOP</button>
</header>
<div id="estopBar">ESTOP latched <button class="btn" onclick="send('reset')">reset</button></div>

<div class="top">
  <div class="cards" id="cards"></div>
  <div class="tgt">target
    <div class="seg" id="seg"></div>
  </div>
  <div class="btns">
    <button class="g" onclick="tsend('arm')">arm</button>
    <button class="r" onclick="tsend('disarm')">disarm</button>
    <button onclick="tsend('hold')">hold</button>
    <button class="y" onclick="tsend('clear')">clear</button>
    <button onclick="tsend('zero')">zero</button>
    <button onclick="send('ident')">ident</button>
    <button onclick="send('status')">status</button>
    <button onclick="send('mon')">mon</button>
    <button onclick="send('log')">log</button>
    <button onclick="send('log stop')">log stop</button>
  </div>
  <details><summary>more</summary>
    <div class="btns" style="margin-top:6px">
      <button onclick="location.href='/log.csv'">log.csv</button>
      <button onclick="send('help')">help</button>
      <button onclick="send('t')">self-test</button>
      <button class="r" onclick="send('disarm all')">disarm all</button>
    </div>
  </details>
</div>

<div class="tbar"><span id="lineCount">0 lines</span><span class="spacer"></span>
  <button class="btn" id="copyBtn" onclick="copyAll()">copy all</button>
  <button class="btn" onclick="saveTxt()">save .txt</button>
  <button class="btn" onclick="clearTerm()">clear</button>
</div>
<div id="term"></div>
<div id="copyBox"><div class="hint">Copy was blocked by the browser. Long-press the text, Select all, Copy.</div>
  <textarea id="copyText" readonly></textarea>
  <button class="btn" onclick="document.getElementById('copyBox').className=''">close</button>
</div>
<button id="newBtn" onclick="toBottom()">new lines</button>

<form onsubmit="submitLine(event)" autocomplete="off">
  <button type="button" class="btn" onclick="histUp()">&#8593;</button>
  <input id="in" placeholder="command, e.g. set 1 kp 10 kd 0.1"
         autocapitalize="off" autocorrect="off" spellcheck="false" enterkeyhint="send">
  <button class="btn" id="send">Send</button>
</form>

<script>
const term = document.getElementById('term');
const MAX_LINES = 2000;
let next = 0, target = '1', hist = [], hi = 0, nodes = [];

function cls(l) {
  if (l.startsWith('> ')) return 'echo';
  if (l.startsWith('**')) return 'ev';
  // Case-sensitive on purpose: "arblost=0" in the bus line must not turn it red.
  if (l.startsWith('!!') || /refused|REFUSED|FAILED| LOST |ESTOP LATCHED|^ESTOP/.test(l)) return 'err';
  return '';
}
function nearBottom() { return term.scrollHeight - term.scrollTop - term.clientHeight < 40; }
function toBottom() { term.scrollTop = term.scrollHeight; document.getElementById('newBtn').style.display = 'none'; }
function addLines(arr, forceCls) {
  if (!arr.length) return;
  const stick = nearBottom();
  const frag = document.createDocumentFragment();
  for (const l of arr) {
    const d = document.createElement('div');
    d.textContent = l.length ? l : ' ';
    const c = forceCls || cls(l);
    if (c) d.className = c;
    frag.appendChild(d);
  }
  term.appendChild(frag);
  while (term.childElementCount > MAX_LINES) term.removeChild(term.firstChild);
  updCount();
  if (stick) toBottom(); else document.getElementById('newBtn').style.display = 'block';
}
function sys(l) { addLines([l], 'sys'); }
function clearTerm() { term.innerHTML = ''; updCount(); }
function updCount() { document.getElementById('lineCount').textContent = term.childElementCount + ' lines'; }
// Everything currently in the terminal (since page load, last MAX_LINES), as text.
function termText() { return Array.from(term.children, d => d.textContent).join(String.fromCharCode(10)); }
function flash(id, txt) {
  const b = document.getElementById(id), old = b.textContent;
  b.textContent = txt; setTimeout(() => { b.textContent = old; }, 1500);
}
// Plain http://192.168.4.1 is not a "secure context", so phone browsers hide
// navigator.clipboard. Try it anyway (desktop/localhost), then the legacy
// execCommand path (allowed on http inside a tap), then show the text for a
// manual long-press copy. Never fails silently.
function legacyCopy(text) {
  const ta = document.createElement('textarea');
  ta.value = text; ta.setAttribute('readonly', '');
  ta.style.cssText = 'position:fixed;top:0;left:0;opacity:0;font-size:16px';
  document.body.appendChild(ta);
  ta.focus(); ta.select(); ta.setSelectionRange(0, text.length);
  let ok = false;
  try { ok = document.execCommand('copy'); } catch (e) { ok = false; }
  document.body.removeChild(ta);
  return ok;
}
function manualCopy(text) {
  const box = document.getElementById('copyBox'), ta = document.getElementById('copyText');
  ta.value = text; box.className = 'show';
  ta.focus(); ta.select(); ta.setSelectionRange(0, text.length);
}
async function copyAll() {
  const text = termText(), n = term.childElementCount;
  if (navigator.clipboard && window.isSecureContext) {
    try { await navigator.clipboard.writeText(text); flash('copyBtn', 'copied ' + n); return; } catch (e) {}
  }
  if (legacyCopy(text)) { flash('copyBtn', 'copied ' + n); return; }
  manualCopy(text);
}
function saveTxt() {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([termText()], { type: 'text/plain' }));
  const t = new Date(), p2 = v => String(v).padStart(2, '0');
  a.download = 'leg_terminal_' + t.getFullYear() + p2(t.getMonth() + 1) + p2(t.getDate()) + '_' +
               p2(t.getHours()) + p2(t.getMinutes()) + p2(t.getSeconds()) + '.txt';
  document.body.appendChild(a); a.click(); document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}
term.addEventListener('scroll', () => { if (nearBottom()) document.getElementById('newBtn').style.display = 'none'; });

async function get(url, ms) {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), ms);
  try { return await fetch(url, { cache: 'no-store', signal: ac.signal }); }
  finally { clearTimeout(t); }
}

async function send(line) {
  line = line.trim();
  if (!line) return;
  if (hist[hist.length - 1] !== line) hist.push(line);
  hi = hist.length;
  try {
    const r = await get('/cmd?c=' + encodeURIComponent(line), 2000);
    if (!r.ok) sys('not sent (' + r.status + '): ' + line);
  } catch (e) { sys('not sent (no link): ' + line); }
}
function tsend(verb) { send(verb + ' ' + target); }
function submitLine(e) {
  e.preventDefault();
  const i = document.getElementById('in');
  send(i.value); i.value = '';
}
function histUp() {
  if (!hist.length) return;
  hi = (hi - 1 + hist.length) % hist.length;
  const i = document.getElementById('in'); i.value = hist[hi]; i.focus();
}
document.getElementById('in').addEventListener('keydown', e => {
  if (e.key === 'ArrowUp') { e.preventDefault(); histUp(); }
});

async function doStop() {
  const b = document.getElementById('stop');
  b.textContent = '...';
  try { await get('/stop', 1500); b.textContent = 'STOP'; }
  catch (e) { b.textContent = 'STOP'; sys('STOP NOT DELIVERED -- use the button / pull the pack'); }
}

function renderSeg() {
  const seg = document.getElementById('seg');
  const opts = nodes.map(n => String(n.id)).concat(['all']);
  seg.innerHTML = '';
  for (const o of opts) {
    const b = document.createElement('button');
    b.textContent = o === 'all' ? 'all' : 'J' + String(o).padStart(2, '0');
    if (o === target) b.className = 'on';
    b.onclick = () => { target = o; renderSeg(); renderCards(); };
    seg.appendChild(b);
  }
}
function renderCards() {
  const c = document.getElementById('cards');
  c.innerHTML = '';
  for (const n of nodes) {
    const el = document.createElement('div');
    el.className = 'card' + (String(n.id) === target ? ' sel' : '');
    el.onclick = () => { target = String(n.id); renderSeg(); renderCards(); };
    const gone = n.age > 500;
    let chips = '';
    if (gone) chips += '<span class="chip gone">no reply</span>';
    else if (n.armed) chips += '<span class="chip arm">' + (n.cmd ? 'ARMED' : 'ARMED*') + '</span>';
    else chips += '<span class="chip">off</span>';
    if (n.armed && n.ph !== 'LIVE') chips += '<span class="chip ph">' + n.ph + '</span>';
    if (n.fl !== 'none') chips += '<span class="chip flt">' + n.fl + '</span>';
    const deg = (n.p * 57.29578).toFixed(1);
    el.innerHTML =
      '<div class="h">J' + String(n.id).padStart(2, '0') + ' ' + chips + '</div>' +
      '<div class="r">p <b>' + n.p.toFixed(4) + ' (' + deg + '&deg;)</b></div>' +
      '<div class="r">v <b>' + n.v.toFixed(3) + '</b> tau <b>' + n.tau.toFixed(3) + '</b></div>' +
      '<div class="r">miss <b>' + n.miss + '</b> rep <b>' + n.rep + '</b></div>';
    c.appendChild(el);
  }
}

let lastIds = '';
async function poll() {
  let delay = 250;
  try {
    const r = await get('/poll?since=' + next, 2000);
    const j = await r.json();
    document.getElementById('dot').className = 'dot on';
    if (j.next < next) sys('--- master restarted ---');
    if (j.dropped) sys('[' + j.dropped + ' lines missed]');
    addLines(j.lines);
    next = j.next;
    if (j.more) delay = 0;
    const bp = document.getElementById('busPill');
    bp.textContent = j.bus + (j.tec ? ' TEC ' + j.tec : '');
    bp.className = 'pill' + (j.bus === 'RUNNING' && !j.tec ? '' : ' bad');
    const lp = document.getElementById('logPill');
    lp.style.display = j.log ? '' : 'none';
    lp.className = 'pill rec'; lp.textContent = 'REC ' + j.logn;
    document.getElementById('estopBar').className = j.estop ? 'show' : '';
    nodes = j.nodes;
    const ids = nodes.map(n => n.id).join(',');
    if (ids !== lastIds) { lastIds = ids; renderSeg(); }
    renderCards();
  } catch (e) {
    document.getElementById('dot').className = 'dot';
    delay = 1000;
  }
  setTimeout(poll, delay);
}
sys('connecting to the leg master...');
poll();
</script>
</body></html>
)PAGE";
