// Web login (WEB-AUTH-01): offline jsdom test of src/web_functions/web_login_page.cpp.
//
// The login page markup (between the C++ comments LOGIN-HTML-BEGIN / LOGIN-HTML-END) and the scaffold JS
// (LOGIN-JS-BEGIN / LOGIN-JS-END) are extracted by concatenating the string literals of their web_client.println
// calls and run in jsdom against a fake fetch that speaks /?loginnonce, /?login=<nonce>.<mac> and /?logout.
// Checks: SHA-256 and HMAC-SHA256 against Node crypto (NIST / RFC 4231 vectors, UTF-8 password), the form (Enter
// submits), the nonce/HMAC exchange (password never in a URL), error messages, logout, focus, and that no println
// literal exceeds 512 bytes (RAK4631 write cap).
//
// Prerequisites (not part of the repo):  npm i jsdom  in ${MESHCOM_JSDOM_DIR:-$HOME/.cache/meshcom-jsdom}
// Run:   NODE_PATH=$HOME/.cache/meshcom-jsdom/node_modules node tools/webgui_login_test.js
// Env:   LOGIN_SRC=<file>  take the HTML block from another file (regression run against the old markup; the JS
//                          block always comes from web_login_page.cpp)
// Mutation check (the test must then FAIL):  LOGIN_MUTATE=noform|plainurl|nofocus node tools/webgui_login_test.js
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const { JSDOM, VirtualConsole } = require('jsdom');

const PAGE = path.join(__dirname, '..', 'src', 'web_functions', 'web_login_page.cpp');
const SRC = process.env.LOGIN_SRC || PAGE;
const MUT = process.env.LOGIN_MUTATE || '';

let failures = 0, passes = 0;
function check(name, cond, detail) {
  console.log((cond ? 'PASS ' : 'FAIL ') + name + (detail !== undefined && !cond ? '  [' + detail + ']' : ''));
  if (cond) passes++; else failures++;
}

function decodeLiteral(s) {
  let o = '';
  for (let i = 0; i < s.length; i++) {
    if (s[i] !== '\\') { o += s[i]; continue; }
    const c = s[++i];
    if (c === 'n') o += '\n';
    else if (c === 't') o += '\t';
    else if (c === '\\' || c === '"' || c === "'") o += c;
    else throw new Error('unsupported escape \\' + c);
  }
  return o;
}

// text + the byte size of each println call (adjacent literals of one call count together)
function block(file, tag) {
  const src = fs.readFileSync(file, 'utf8');
  const a = src.indexOf('// ' + tag + '-BEGIN'), b = src.indexOf('// ' + tag + '-END');
  if (a < 0 || b < a) throw new Error('marker ' + tag + ' not found in ' + file);
  const body = src.slice(src.indexOf('\n', a), b).split('\n').filter((l) => !/^\s*\/\//.test(l)).join('\n');
  let out = '';
  const sizes = [];
  for (const c of body.matchAll(/web_client\.println\s*\(((?:\s*"(?:[^"\\]|\\.)*")*)\s*\)/g)) {
    let call = '';
    for (const m of c[1].matchAll(/"((?:[^"\\]|\\.)*)"/g)) call += decodeLiteral(m[1]);
    sizes.push(Buffer.byteLength(call));
    out += call;
  }
  return { text: out, sizes };
}
function mutate(t, from, to) { if (!t.includes(from)) throw new Error('mutation anchor not found: ' + from); return t.replace(from, to); }

const hb = block(SRC, 'LOGIN-HTML'), jb = block(PAGE, 'LOGIN-JS');
let html = hb.text, js = jb.text;
if (MUT === 'noform') html = mutate(mutate(html, '<form', '<div'), '</form>', '</div>');
if (MUT === 'plainurl') js = mutate(js, "'/?login='+n+'.'+mcHmacHex(pwd,n)", "'/?nodepassword='+pwd");
if (MUT === 'nofocus') js = mutate(js, "function mcLoginFocus(){var p=document.getElementById('nodepassword');if(p)p.focus();}", 'function mcLoginFocus(){}');
if (MUT) console.log('NOTE mutation active: ' + MUT);

let reloads = 0;
const vc = new VirtualConsole();
vc.on('jsdomError', (e) => {
  if (/navigation/i.test(e.message)) reloads++; // location.reload() is not implemented in jsdom: count it
  else { console.log('JSDOM ERROR ' + e.message); failures++; }
});
const dom = new JSDOM('<!doctype html><body><div id="content_inner">' + html + '</div></body>',
  { runScripts: 'dangerously', url: 'http://node.local/', virtualConsole: vc });
const w = dom.window;

const calls = [];
let handler = () => ({ status: 404, body: '' });
w.fetch = (url) => {
  calls.push(url);
  const r = handler(url);
  return Promise.resolve({ status: r.status, text: () => Promise.resolve(r.body) });
};
w.eval(js);
const $ = (i) => w.document.getElementById(i);
const flush = () => new Promise((r) => setImmediate(r));
async function settle() { for (let i = 0; i < 8; i++) await flush(); }
const hex = (a) => Buffer.from(Array.from(a)).toString('hex');
const sha = (buf) => crypto.createHash('sha256').update(buf).digest('hex');
const hmac = (k, m) => crypto.createHmac('sha256', Buffer.from(k)).update(Buffer.from(m)).digest('hex');
const errText = () => ($('loginerr') || {}).textContent;
function reset() { calls.length = 0; reloads = 0; if ($('loginerr')) $('loginerr').textContent = ''; }

const NONCE = '00112233445566778899aabbccddeeff';
const PW = 'pä§w';
function okHandler(pw) {
  return (url) => {
    if (url === '/?loginnonce') return { status: 200, body: NONCE };
    if (url.startsWith('/?login=')) {
      const [n, mac] = url.slice(8).split('.');
      return mac === hmac(pw, n) ? { status: 200, body: '{"ok":true}' } : { status: 403, body: '{"ok":false,"err":"badpw"}' };
    }
    if (url === '/?logout') return { status: 200, body: '' };
    return { status: 404, body: '' };
  };
}

(async () => {
  // SHA-256
  const sha256 = (u8) => hex(w.mcSha256(Array.from(u8)));
  check('sha256("")', sha256(Buffer.alloc(0)) === sha(Buffer.alloc(0)));
  check('sha256("abc")', sha256(Buffer.from('abc')) === 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad');
  const nist448 = Buffer.from('abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq');
  check('sha256 NIST 448-bit message', nist448.length === 56 && sha256(nist448) === '248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1');
  const k1000 = Buffer.alloc(1000); for (let i = 0; i < 1000; i++) k1000[i] = (i * 7 + 3) & 255;
  check('sha256 1000-byte message vs node', sha256(k1000) === sha(k1000));
  check('sha256 55/56/64-byte padding edges vs node', [55, 56, 63, 64, 65].every((n) => sha256(k1000.subarray(0, n)) === sha(k1000.subarray(0, n))));
  // HMAC
  const h = (k, m) => w.mcHmacHex(k, m);
  check('hmac RFC 4231 case 1', h('\x0b'.repeat(20), 'Hi There') === hmac('\x0b'.repeat(20), 'Hi There') && h('\x0b'.repeat(20), 'Hi There') === 'b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7');
  check('hmac RFC 4231 case 2', h('Jefe', 'what do ya want for nothing?') === '5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843');
  // RFC 4231 case 6 uses 0xaa bytes, which are not a UTF-8 string: same shape (131-byte key, hashed first) with ASCII
  const k131 = 'aZ'.repeat(66).slice(0, 131), m6 = 'Test Using Larger Than Block-Size Key - Hash Key First';
  check('hmac 131-byte key (case 6 shape) vs node', k131.length === 131 && h(k131, m6) === hmac(k131, m6));
  check('hmac UTF-8 password vs node', h(PW, NONCE) === hmac(Buffer.from(PW, 'utf8'), Buffer.from(NONCE, 'utf8')));

  // page
  const pw = $('nodepassword'), form = pw && pw.closest('form');
  check('password input is type=password inside a <form>', !!pw && pw.type === 'password' && !!form);
  check('form has a submit button', !!form && !!form.querySelector('button[type=submit]'));
  check('input has autocomplete=current-password, autofocus, maxlength 20', !!pw && pw.getAttribute('autocomplete') === 'current-password' && pw.hasAttribute('autofocus') && pw.getAttribute('maxlength') === '20');
  check('#loginerr present', !!$('loginerr'));
  check('subheader/structure kept (cardlayout, cardlabel)', /class="cardlayout"/.test(html) && /class="cardlabel">Enter Password</.test(html));

  // Enter / submit -> nonce -> login
  handler = okHandler(PW);
  reset();
  let submitted = false;
  try { pw.value = PW; form.requestSubmit(); submitted = true; } catch (e) { submitted = false; }
  await settle();
  const want = '/?login=' + NONCE + '.' + hmac(Buffer.from(PW, 'utf8'), Buffer.from(NONCE, 'utf8'));
  check('requestSubmit calls /?loginnonce then /?login=<nonce>.<mac>', submitted && calls.length === 2 && calls[0] === '/?loginnonce' && calls[1] === want, JSON.stringify(calls));
  check('reload on 200', reloads === 1, String(reloads));
  check('password never in a fetched URL', calls.length > 0 && calls.every((u) => !u.includes(PW) && !u.includes(encodeURIComponent(PW)) && !/nodepassword/.test(u)), JSON.stringify(calls));

  // wrong password
  handler = okHandler('other');
  reset(); pw.value = PW; w.login(PW); await settle();
  check('403 badpw shows "Wrong password", no reload', errText() === 'Wrong password' && reloads === 0, errText() + ' reloads=' + reloads);
  check('403 clears and refocuses the field', pw.value === '' && w.document.activeElement === pw);

  // login throttled / other errors
  handler = (u) => (u === '/?loginnonce' ? { status: 403, body: '{"ok":false,"err":"throttled"}' } : { status: 500, body: '' });
  reset(); w.login(PW); await settle();
  check('throttled nonce shows throttle message, no /?login', errText() === 'Too many attempts, wait a minute' && calls.length === 1 && reloads === 0, errText() + ' ' + JSON.stringify(calls));
  handler = (u) => (u === '/?loginnonce' ? { status: 200, body: NONCE } : { status: 403, body: '{"ok":false,"err":"nonce"}' });
  reset(); w.login(PW); await settle();
  check('other error code shows generic message', errText() === 'Login failed, try again' && reloads === 0, errText());
  handler = () => ({ status: 500, body: '' });
  reset(); w.login(PW); await settle();
  check('nonce 500 shows generic message', errText() === 'Login failed, try again' && reloads === 0);
  reset(); w.login(''); await settle();
  check('empty password: message, no request', calls.length === 0 && errText() !== '');

  // logout
  handler = okHandler(PW);
  reset(); w.logout(); await settle();
  check('logout() hits /?logout and reloads', calls.length === 1 && calls[0] === '/?logout' && reloads === 1, JSON.stringify(calls) + ' ' + reloads);

  // focus
  pw.blur();
  w.mcLoginFocus();
  check('mcLoginFocus() focuses the field', w.document.activeElement === pw);
  const w2 = new JSDOM('<!doctype html><body></body>', { runScripts: 'outside-only' }).window;
  w2.eval(js);
  let noThrow = true; try { w2.mcLoginFocus(); } catch (e) { noThrow = false; }
  check('mcLoginFocus() without the field is a no-op', noThrow);

  // sizes
  const all = hb.sizes.concat(jb.sizes);
  const big = Math.max.apply(null, all);
  check('every println literal <= 512 bytes (largest ' + big + ')', big <= 512 && hb.sizes.length > 0 && jb.sizes.length > 0);
  console.log('INFO js bytes ' + Buffer.byteLength(js) + ', println calls ' + jb.sizes.length + ', largest js call ' + Math.max.apply(null, jb.sizes));

  console.log((failures ? 'FAIL' : 'PASS') + ' summary: ' + passes + ' passed, ' + failures + ' failed');
  process.exit(failures ? 1 : 0);
})();
