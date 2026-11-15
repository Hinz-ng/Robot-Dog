// Run against BOTH compiled WEB_PAGE variants; no ESP32 or network connection.
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const html = fs.readFileSync(process.argv[2], 'utf8');
const leg = process.argv[3] === 'leg';
assert(html.includes('id="pairControls"'), 'paired controls must be a named section');
assert(html.includes('<details id="singlePanel">'), 'individual controls must start collapsed');
assert(html.includes('id="singleControls" disabled'), 'individual controls disabled before profile initialization');
class Element {
  constructor() { this.style = {}; this.children = []; this.textContent = ''; this.className = ''; }
  set innerHTML(v) { this.html = v; this.children = []; }
  get innerHTML() { return this.html || ''; }
  get childElementCount() { return this.children.length; }
  get firstChild() { return this.children[0]; }
  appendChild(x) { this.children.push(x); return x; }
  removeChild(x) { this.children.splice(this.children.indexOf(x), 1); }
  addEventListener() {} setAttribute() {} focus() {} click() {} remove() {}
}
const elements = {}, ready = [];
const context = vm.createContext({console, URL, Blob, AbortController,
  setTimeout: () => 0, clearTimeout: () => {}, navigator: {}, confirm: () => true,
  window: {addEventListener: (event, callback) => { if (event === 'DOMContentLoaded') ready.push(callback); }},
  document: {getElementById: id => elements[id] ||= new Element(),
    createElement: () => new Element(), createDocumentFragment: () => new Element(), body: new Element()},
  fetch: async () => ({ok: true, json: async () => ({next: 0, lines: [], bus: 'RUNNING', tec: 0,
    log: 0, logn: 0, logcap: 4000, nodes: []})})
});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
for (const callback of ready) callback();
assert.strictEqual(vm.runInContext('LEG_PROFILE', context), leg);
assert.strictEqual(elements.singleControls.disabled, leg);
assert.strictEqual(elements.pairControls.style.display, leg ? '' : 'none');
vm.runInContext('var sent = []; send = line => sent.push(line);', context);
function click(id) {
  const tag = html.match(new RegExp('<button[^>]*id="' + id + '"[^>]*>'));
  assert(tag, 'missing button ' + id);
  const action = tag[0].match(/onclick="([^"]+)"/)[1];
  vm.runInContext(action, context);
}
if (leg) {
  for (const id of ['pairArm','pairDisarm','pairZero','pairAngles','pairFoot','pairGains',
    'moveJ01','moveJ03','moveZero']) click(id);
  assert.deepStrictEqual(Array.from(context.sent), ['leg arm','leg disarm','leg zero','leg status',
    'leg foot','leg gains 2 0.02','leg move 0.010 0 0.6','leg move 0 0.010 0.6','leg move 0 0 0.6']);
  context.sent.length = 0;
  for (const verb of ['arm','zero','hold','clear','disarm']) vm.runInContext(`singleSend('${verb}')`, context);
  assert.strictEqual(context.sent.length, 0, 'individual control must never redirect to pair');
  vm.runInContext("selectTarget('3')", context);
  assert.strictEqual(vm.runInContext('target', context), '1', 'pair cards must not change target');
} else {
  vm.runInContext("selectTarget('3'); singleSend('arm'); singleSend('hold'); singleSend('zero');", context);
  assert.deepStrictEqual(Array.from(context.sent), ['arm 3','hold 3','zero 3']);
}
context.sent.length = 0;
for (const id of ['allIdent','allStatus','recordLog','pauseLog','clearLog']) click(id);
assert.deepStrictEqual(Array.from(context.sent), ['ident','status','log','log stop','log clear']);
assert(html.includes('onclick="doStop()"'), 'STOP retains dedicated endpoint');
console.log(`PASS ${leg ? 'leg' : 'normal'}: profile isolation, button commands, target scope and logging`);
