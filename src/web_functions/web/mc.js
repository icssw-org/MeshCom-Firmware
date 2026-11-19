// RM GUI: every request of the page carries `X-MC: 1`. The node refuses state-changing requests
// (setparam, callfunction, sendmessage, every POST) without it or without a same-host Origin: a foreign
// web page cannot set a custom header without a CORS preflight, which the node does not answer.
(function(){var f=window.fetch;if(f){window.fetch=function(u,o){o=o||{};var h=new Headers(o.headers||{});h.set('X-MC','1');o.headers=h;return f.call(window,u,o);};}var s=window.XMLHttpRequest&&XMLHttpRequest.prototype.send;if(s){XMLHttpRequest.prototype.send=function(b){try{this.setRequestHeader('X-MC','1');}catch(e){}return s.call(this,b);};}})();
// these variables will hold the last loaded page name and sender in order to force a refresh
cpage="info";csender=undefined;
setInterval(autorefresh,30000);
// This function will be called in intervalls - can be used to auto-refresh content depending on what page is loaded
function autorefresh() {if(cpage=='messages')updateMessages();if(cpage=='wx')loadPage('wx',csender,false);if(cpage=='position')loadPage('position',csender,false);if(cpage=='mheard')loadPage('mheard',csender,false);if(cpage=='path')loadPage('path',csender,false);if(cpage=='rxlog')loadPage('rxlog',csender,false);};
// this function is used to send a message from the browser via node to the mesh
//
// BP-09: the input fields used to be cleared unconditionally, right after
// xhttp.send() and without ever looking at the answer -- so a message the
// node refused (QRT) or dropped (QTA) took the operator's typed text with
// it, exactly the loss BP-09 fixes on the T-Deck and T-Deck Pro. The clear
// now waits for the response and only fires on "sendmessage ok"; every
// other outcome (refused / dropped / invalid / failed) leaves the text in
// place so it can be resent after the QRV. updateCharsLeft() is called
// from the handler because the onclick chain has long since run its own
// call by the time the answer arrives.
//
// The destination survives the send when it is a group number: the tab
// bar puts the selected group into #sendcall, and clearing it after every
// send silently turned the next quick message into a broadcast to '*'
// (DJ8MEH, 2026-09-11). A DM call sign is still cleared as before.
function sendMessage() {var xhttp=new XMLHttpRequest();xhttp.onreadystatechange=function(){if(this.readyState==4 && this.status==200 && this.responseText.indexOf("sendmessage ok")>=0){var sc=document.getElementById("sendcall");if(!/^[0-9]+$/.test(sc.value))sc.value=""; document.getElementById("messagetext").value=""; updateCharsLeft();}};xhttp.open("GET","/?sendmessage&tocall="+encodeURIComponent(document.getElementById("sendcall").value)+"&message="+encodeURIComponent(document.getElementById("messagetext").value),true);xhttp.send();}
// counts what is left of the node's 150-byte message limit (send_message() adds ":" or ":{call}").
// #1173: count UTF-8 bytes, not characters -- "ą" costs 2, an emoji 4 -- and cut an over-long
// text between characters (for..of walks code points, so no surrogate pair is split).
var mcEnc=new TextEncoder();function updateCharsLeft() {let maxlength=149;let call=document.getElementById("sendcall").value;if(call.length>0) {maxlength-=mcEnc.encode(call).length+2;}let t=document.getElementById("messagetext");let msglength=mcEnc.encode(t.value).length;if(msglength>maxlength){let out="";msglength=0;for(const ch of t.value){let b=mcEnc.encode(ch).length;if(msglength+b>maxlength)break;out+=ch;msglength+=b;}t.value=out;}document.getElementById("indicator_charsleft").innerHTML=maxlength-msglength;}
// MC-msg-history: BLEtoPhoneBuff/MAX_RING is only 20 slots and is shared
// with positions and acks, so a handful of new messages can push an old
// message out of the node's own ring within minutes. The browser tab
// keeps every message it has seen for the life of the page in
// mcHistory/mcSeen (capped at MC_HIST_MAX, oldest dropped first) so
// switching Info -> Messages -> Info -> Messages does not lose messages
// that scrolled out of the node's ring. This is browser memory only --
// node RAM is unchanged, and persisting the history to localStorage is
// deferred (not implemented in this round).
var mcSeen={};
var mcHistory=[];
var MC_HIST_MAX=200;
// MC-msg-tabs: this and the rest of the mcTab* functions live in the
// scaffold rather than sub_page_messages() because sub-pages are
// injected via innerHTML and their own <script> tags never run.
var mcTabSel='all';
try{var mcTabStored=localStorage.getItem('mcTab');if(mcTabStored!=null)mcTabSel=mcTabStored;}catch(e){}
// MC-msg-badges: unread-state model. mcReadIds is a session-only set of
// message ids the operator has definitely seen -- it survives while
// mcHistory holds the entry, but not a browser restart. mcWm holds, per
// tab key, a unix-time watermark below which everything on that tab
// counts as read; it is the only read-state that is persisted
// (localStorage 'mcWm'), because persisting the exact id set across days
// of ring turnover would grow without bound. A message is unread for a
// tab key when it is inbound (an own message-send is never unread), it
// matches that key's dst filter, its id has not been marked read this
// session, and its ts is strictly greater than that key's watermark --
// equal timestamps fall back to the id set, since an unsynced node clock
// can hand two independent messages the same second. Viewing "All"
// raises every tab's watermark to the newest message seen, because
// looking at the merged stream is defined as having read everything in
// it, not only the "all" tab itself.
var mcReadIds={};
var mcWm={};
try{var mcWmStored=localStorage.getItem('mcWm');if(mcWmStored!=null)mcWm=JSON.parse(mcWmStored);}catch(e){}if(typeof mcWm!=='object' || mcWm===null)mcWm={};
function mcSaveWm(){try{localStorage.setItem('mcWm',JSON.stringify(mcWm));}catch(e){}}
function mcTabMatchKey(key,dst){if(key=='all')return true;if(key=='*')return dst=='*';if(key=='dm')return dst!='*' && !/^[0-9]+$/.test(dst);return dst==key;}
function mcTabMatch(dst){return mcTabMatchKey(mcTabSel,dst);}
function mcUnreadCount(key){var wm=mcWm[key]||0;var n=0;for(var i=0;i<mcHistory.length;i++){var m=mcHistory[i];if(m.rx && mcTabMatchKey(key,m.dst) && !mcReadIds[m.id] && m.ts>wm)n++;}return n;}
// seeds the watermark of any tab key that has never had one (first visit
// to this browser, or a group number just added in setup) to the newest
// ts already known, so first load never shows a wall of unread badges
function mcSeedWm(){var btns=document.querySelectorAll('#mctabs .mctab');if(btns.length==0)return;var maxTs=0;for(var i=0;i<mcHistory.length;i++){if(mcHistory[i].ts>maxTs)maxTs=mcHistory[i].ts;}var changed=false;for(var j=0;j<btns.length;j++){var key=btns[j].getAttribute('data-tab');if(!(key in mcWm)){mcWm[key]=maxTs;changed=true;}}if(changed)mcSaveWm();}
// marks every history entry on the currently selected tab as read; viewing
// 'all' counts as having read every tab, so its watermark propagates to
// every configured tab key, not only 'all' itself
function mcMarkRead(){if(document.visibilityState==='hidden')return;var maxTs=0;var changed=false;for(var i=0;i<mcHistory.length;i++){var m=mcHistory[i];if(mcTabMatchKey(mcTabSel,m.dst)){if(!mcReadIds[m.id]){mcReadIds[m.id]=true;changed=true;}if(m.ts>maxTs)maxTs=m.ts;}}if(maxTs>(mcWm[mcTabSel]||0)){mcWm[mcTabSel]=maxTs;changed=true;}if(mcTabSel=='all'){var btns=document.querySelectorAll('#mctabs .mctab');for(var j=0;j<btns.length;j++){var key=btns[j].getAttribute('data-tab');if(maxTs>(mcWm[key]||0)){mcWm[key]=maxTs;changed=true;}}}if(changed)mcSaveWm();}
// the tab bar is re-rendered by the server on every loadPage('messages'),
// so badges are painted here on every call, never once at page load
function mcApplyTab(){var panel=document.getElementById('messages_panel');if(!panel)return;var els=panel.querySelectorAll('.message[data-dst]');for(var i=0;i<els.length;i++){els[i].hidden=!mcTabMatch(els[i].getAttribute('data-dst'));}mcSeedWm();mcMarkRead();var btns=document.querySelectorAll('#mctabs .mctab');for(var j=0;j<btns.length;j++){var key=btns[j].getAttribute('data-tab');btns[j].classList.toggle('mctab-on',key==mcTabSel);var label=key=='all'?'All':(key=='dm'?'DM':key);var cnt=mcUnreadCount(key);btns[j].innerHTML=label+(cnt>0?' <span class="mcbadge">'+cnt+'</span>':'');btns[j].classList.toggle('mctab-new',cnt>0);}}
function mcTab(btn){mcTabSel=btn.getAttribute('data-tab');try{localStorage.setItem('mcTab',mcTabSel);}catch(e){}var sc=document.getElementById('sendcall');if(sc){if(/^[0-9]+$/.test(mcTabSel))sc.value=mcTabSel;else if(mcTabSel=='*')sc.value='';}if(typeof updateCharsLeft==='function' && sc)updateCharsLeft();mcApplyTab();}
// messages that arrive while the tab is hidden must not count as read
// until the operator actually comes back to look at them
document.addEventListener('visibilitychange',function(){if(cpage=='messages')mcApplyTab();});
function mcHistIndex(id){for(var i=0;i<mcHistory.length;i++){if(mcHistory[i].id==id)return i;}return -1;}
// a tick is rank-monotone: the server's status ring is shared and churned and drops the tick of an old message, so a ranked-higher tick of the remembered entry survives (rank = index 0,1,4,3,2: heard < held < failed < ACK)
function mcTickKeep(h,el){if(!el.classList.contains('message-send'))return;var t=document.createElement('template');t.innerHTML=h;var o=t.content.querySelector('.mctick');if(!o)return;var n=el.querySelector('.mctick');var R=[0,1,4,3,2];if(n&&R[n.getAttribute('data-st')]>=R[o.getAttribute('data-st')])return;if(n)n.replaceWith(o);else{var p=el.querySelector('p');if(p)p.insertBefore(o,p.firstChild);}}
function mcMergeEntry(el){var id=el.getAttribute('data-id');var idx=mcHistIndex(id);if(idx>=0)mcTickKeep(mcHistory[idx].html,el);var html=el.outerHTML;var dst=el.getAttribute('data-dst')||'';var ts=parseInt(el.getAttribute('data-ts'))||0;var rx=el.classList.contains('message-received');if(idx<0){mcHistory.push({id:id,html:html,dst:dst,ts:ts,rx:rx});mcSeen[id]=true;if(mcHistory.length>MC_HIST_MAX){var dropped=mcHistory.shift();delete mcSeen[dropped.id];}}else{mcHistory[idx].html=html;mcHistory[idx].dst=dst;mcHistory[idx].ts=ts;mcHistory[idx].rx=rx;}}
function mcRemovePlaceholder(panel){var kids=panel.children;for(var i=kids.length-1;i>=0;i--){if(kids[i].tagName=='P')panel.removeChild(kids[i]);}}
// this function is an ayncronous loader that is used to update the received messages without re-loading the whole page, it will re-call itself after a timeout as long as the message-page is displayed
// it merges the response into mcHistory/mcSeen instead of overwriting the panel outright, so a message already on screen keeps its DOM position when only its ack mark changed
function mcProcessMessages(text,panel){var tmpl=document.createElement('template');tmpl.innerHTML=text;var els=tmpl.content.querySelectorAll('.message[data-id]');for(var i=0;i<els.length;i++){var el=els[i];var id=el.getAttribute('data-id');var existed=mcSeen.hasOwnProperty(id);mcMergeEntry(el);if(existed){var old=panel.querySelector('.message[data-id="'+id+'"]');if(old)old.replaceWith(el);}else{panel.appendChild(el);}}mcRemovePlaceholder(panel);if(mcHistory.length==0)panel.innerHTML='<p>No messages available.</p>';if(typeof mcApplyTab==='function')mcApplyTab();}
function updateMessages() {var xhttp=new XMLHttpRequest();xhttp.onreadystatechange=function(){if(this.readyState==4 && this.status==200){var panel=document.getElementById('messages_panel');if(panel!=null)mcProcessMessages(this.responseText,panel);}};setTimeout(function(){xhttp.open('GET','/?getmessages',true);xhttp.send();},1000);}
// rebuilds #messages_panel from mcHistory when the messages page is (re-)injected by loadPage(); first merges the server-rendered entries already sitting in the panel into mcHistory (same dedupe as mcProcessMessages) so nothing the server just sent is lost, then renders the full remembered history in order
function mcRenderHistory(){var panel=document.getElementById('messages_panel');if(!panel)return;var els=panel.querySelectorAll('.message[data-id]');for(var i=0;i<els.length;i++){mcMergeEntry(els[i]);}var html='';for(var j=0;j<mcHistory.length;j++){html+=mcHistory[j].html;}panel.innerHTML=html.length>0?html:'<p>No messages available.</p>';if(typeof mcApplyTab==='function')mcApplyTab();}
// this function sends a parameter:value request to the backend
// TRK-01: bei Erfolg (returncode==0, die Seite wird hier NICHT neu geladen) den Warnhinweis
// "<id>_warn" live ein-/ausblenden -- generisch ueber param, damit kuenftige Switches denselben Mechanismus erben
function setvalue(param,value,refresh) {fetch("/setparam/?"+param+"="+encodeURIComponent(value)).then(function(response){return response.json();}).then(function(jsonResponse){if(jsonResponse['returncode']==1)alert("Value could not be set.");if(jsonResponse['returncode']==2)alert("Parameter unknown to node.");if(jsonResponse['returncode']==0){var w=document.getElementById(param+"_warn");if(w)w.style.display=(value=="on")?"":"none";}if(jsonResponse['returncode']>0){loadPage(cpage,csender,false)}if(refresh)loadPage(cpage,csender,false);});}
// this function invokes a function call to the backend passing the function name and an optional parameter (e.g. sendpos)
function callfunction(functionname,functionparameter){fetch("/callfunction/?"+functionname+"="+functionparameter).then(function(response){return response.json();}).then(function (jsonResponse) {/*Nothing todo yet.*/})}
// CS-03: config restore. Lives here and not in the setup page, because the
// sub-pages are injected with innerHTML -- a <script> inside them never runs.
function uploadconfig(){var e=document.getElementById("cfgfile");if(!e||e.files.length==0){alert("choose a config file first");return;}if(!confirm("Restore this configuration and reboot the node?"))return;var r=new FileReader();r.onload=function(){fetch("/config",{method:"POST",headers:{"Content-Type":"application/json"},body:r.result}).then(function(x){return x.text().then(function(t){var m=t.replace(/<[^>]*>/g," ").trim();if(x.ok){alert("config imported: "+m);}else{alert("import failed: "+m);}});}).catch(function(err){alert("upload failed: "+err);});};r.readAsText(e.files[0]);}
// This function is used to toggle a css class so setup cars can collapse / expand
function togglecard(element){element.parentElement.classList.toggle("cardopen");}
// Remote Admin card of the Settings page (sub_page_setup()): scripts of sub-pages never run, so the logic lives here;
// loadPage() calls raSt() after the setup fragment is in. Password only in the POST body, never in a URL.
// RA-JS-BEGIN
var raA=0,raT=0;
function raM(t,b){var e=rmEl('ra_msg');if(e){e.textContent=t;e.style.color=b?'var(--mcred)':'';}}
function raX(e,t){raM(e===0?'Log in again to change these settings.':t,1);}
function raDis(m){clearTimeout(raT);raA=0;var b=rmEl('ra_clr');if(b)b.textContent='Clear';if(m)raM('');}
function raSt(){fetch('/rmstatus').then(rmJson).then(function(j){var o=rmEl('ra_on');if(!o)return;o.checked=!!j.on;o.disabled=!j.pw&&!j.on;
rmEl('ra_hint').textContent=j.pw?'':'Set a password first, then switch remote management on.';
rmEl('ra_st').textContent='password: '+(j.pw?'set':'not set')+(j.lock?', a sender is blocked for '+(j.lockS||1)+' s after wrong attempts':'');})
.catch(function(e){raX(e,'Could not read the state of this node. Reload the page.');});}
function raP(b,t){rmPost('/rmpasswd',b).then(rmJson).then(function(j){if(j&&j.ok)raM(t);else raM(rmErrText(j),1);raSt();})
.catch(function(e){raX(e,'The node did not answer. Try again.');});}
function raSet(){var i=rmEl('ra_pw'),p=i.value,e=rmPwProblem(p);raDis();
if(e){raM(e,1);return;}i.value='';raP('act=set&pw='+rmEnc(p),'Password set.');}
function raClr(){if(!raA){raA=1;rmEl('ra_clr').textContent='Really? tap again';raM('Clear the password? Remote management will be switched off.');raT=setTimeout(function(){raDis(1);},4000);return;}
raDis();raP('act=clear','Password cleared. Remote management is off.');}
function raOn(){var o=rmEl('ra_on'),v=o.checked?'on':'off';o.disabled=true;fetch('/setparam/?rm='+v).then(rmJson).then(function(j){var k=j&&j.returncode==0;
raM(k?'Remote management is '+v+'.':'Could not change it. Is a password set?',!k);raSt();})
.catch(function(e){raX(e,'The node did not answer. Try again.');raSt();});}
// RA-JS-END
// T-Deck display theme buttons of the Settings page (TD-THEME-01 W3): one /setparam/?tdecktheme=auto|dark|light, no reload.
// TD-JS-BEGIN
function tdMark(v){['auto','dark','light'].forEach(function(n){var b=document.getElementById('tdtheme_'+n);if(b){var on=n==v;b.setAttribute('aria-pressed',on?'true':'false');b.classList.toggle('mctab-on',on);}});}
function tdSet(v){fetch('/setparam/?tdecktheme='+encodeURIComponent(v)).then(function(r){return r.json();}).then(function(j){if(j&&j.returncode==0)tdMark(v);else alert('The T-Deck theme could not be set.');}).catch(function(){alert('The node did not answer. Try again.');});}
// TD-JS-END
// WEB-TD-01: the page time (#mc_clock, data-epoch = node local date/time as UTC seconds) advances once a second from performance.now(),
// not from the PC clock. A new render (new data-epoch) re-syncs; no #mc_clock (node without valid time) = nothing ticks.
// mcClockTick() also runs right after each page render (loadPage) and at load, so t0 is the render, not the first interval tick.
// Sleep guard: performance.now() may pause while the PC sleeps. Date.now() is used ONLY to notice that (its delta since the sync vs the
// performance.now() delta, > 2 s apart): the clock then stops (the shown text is never advanced from a doubtful base), is dimmed, and is
// re-synced from a fresh render of the info page (GET ?page=info, never the current page: ?page=spectrum would start a radio scan;
// only the data-epoch is taken from it, the page itself is not replaced,
// so form input stays). If the node does not answer it stays stopped and dimmed and retries every 30 s. The PC clock never sets the shown time.
// Known limit (DST): the zone label (CET/CEST) and the offset are those of the last render. A page left open across a DST switch is 1 h
// off until the next render (reload or page change); the node's own offset is not known to JS.
// CLOCK-JS-BEGIN
var mcClk={e:null,b:0,t0:0,w0:0,stale:null};
function mcClockText(s){var d=new Date(s*1000),z=function(n){return(n<10?'0':'')+n;};
return d.getUTCFullYear()+'-'+z(d.getUTCMonth()+1)+'-'+z(d.getUTCDate())+'\u00a0'+z(d.getUTCHours())+':'+z(d.getUTCMinutes())+':'+z(d.getUTCSeconds());}
function mcClockStale(el,e){mcClk.stale=e;el.style.opacity='.5';
var back=function(){setTimeout(function(){if(mcClk.stale===e){mcClk.stale=null;mcClockTick();}},30000);};
fetch('?page=info').then(function(r){return r.text();}).then(function(t){var m=/id="mc_clock" data-epoch="(\d+)"/.exec(t),c=document.getElementById('mc_clock');
if(m&&c&&c.getAttribute('data-epoch')===e){c.setAttribute('data-epoch',m[1]);c.style.opacity='';mcClk.stale=null;mcClockTick();}else back();}).catch(back);}
function mcClockTick(){var el=document.getElementById('mc_clock');if(!el)return;var e=el.getAttribute('data-epoch'),n=window.performance.now();
if(e!==mcClk.e){mcClk.e=e;mcClk.b=parseInt(e,10);mcClk.t0=n;mcClk.w0=Date.now();mcClk.stale=null;el.style.opacity='';}
if(isNaN(mcClk.b)||e===mcClk.stale)return;
if(Math.abs((Date.now()-mcClk.w0)-(n-mcClk.t0))>2000){mcClockStale(el,e);return;}
var t=mcClockText(mcClk.b+Math.floor((n-mcClk.t0)/1000));if(el.textContent!==t)el.textContent=t;}
setInterval(mcClockTick,1000);
document.addEventListener('DOMContentLoaded',mcClockTick);
document.addEventListener('visibilitychange',function(){if(document.visibilityState!=='hidden')mcClockTick();});
// CLOCK-JS-END
// WEB-LOOK-01: look of this web page only (theme Auto|Light|Dark, font size S|M|L), kept per browser in localStorage.
// Auto removes the attribute (follows the OS); mc.css reads data-theme / data-font on <html>. Runs at load, in the head, before first paint.
// WEBLOOK-JS-BEGIN
function mcLookGet(k){try{return localStorage.getItem(k);}catch(e){return null;}}
function mcLookApply(){var h=document.documentElement,t=mcLookGet('mcTheme'),f=mcLookGet('mcFont');
if(t==='light'||t==='dark')h.setAttribute('data-theme',t);else h.removeAttribute('data-theme');
if(f==='s'||f==='l')h.setAttribute('data-font',f);else h.removeAttribute('data-font');}
function mcLookMark(){var g={mcTheme:['theme',['auto','light','dark'],'auto'],mcFont:['font',['s','m','l'],'m']};
Object.keys(g).forEach(function(k){var cur=mcLookGet(k)||g[k][2];if(g[k][1].indexOf(cur)<0)cur=g[k][2];
g[k][1].forEach(function(n){var b=document.getElementById('weblook_'+g[k][0]+'_'+n);if(b){var on=n===cur;b.setAttribute('aria-pressed',on?'true':'false');b.classList.toggle('mctab-on',on);}});});}
function mcLookSet(k,v){try{if(v==='auto'||v==='m')localStorage.removeItem(k);else localStorage.setItem(k,v);}catch(e){}mcLookApply();mcLookMark();}
mcLookApply();
// WEBLOOK-JS-END
// T-Deck font size / display mode / brightness of the Settings page (WEB-TD-01): /setparam/?tdchatfont=s|m|l, tdbldisp=auto|light|dark, tdblbright=1..16.
// TD2-JS-BEGIN
function tdMark2(g,names,v){names.forEach(function(n){var b=document.getElementById(g+'_'+n);if(b){var on=n==v;b.setAttribute('aria-pressed',on?'true':'false');b.classList.toggle('mctab-on',on);}});}
function tdPut(k,v,ok){fetch('/setparam/?'+k+'='+encodeURIComponent(v)).then(function(r){return r.json();}).then(function(j){if(j&&j.returncode==0)ok(j);else alert('The T-Deck setting could not be set.');}).catch(function(){alert('The node did not answer. Try again.');});}
function tdFont(v){tdPut('tdchatfont',v,function(){tdMark2('tdfont',['s','m','l'],v);});}
function tdDisp(v){tdPut('tdbldisp',v,function(j){tdMark2('tddisp',['auto','light','dark'],v);var s=document.getElementById('tdbright'),o=document.getElementById('tdbright_val');if(s&&j&&j.tdbldisp!==undefined){var l=parseInt(String(j.tdbldisp).split(':')[1],10);if(l>0){s.value=l;if(o)o.textContent=l;}}});}
function tdBright(v){var o=document.getElementById('tdbright_val');if(o)o.textContent=v;tdPut('tdblbright',v,function(){});}
// TD2-JS-END
// Web login password card (WEB-PWD-01): the login of this web GUI, NOT the RM/KISS password. Body only, never a URL.
// WP-JS-BEGIN
var wpA=0,wpT=0;
function wpM(t,b){var e=rmEl('wp_msg');if(e){e.textContent=t;e.style.color=b?'var(--mcred)':'';}}
function wpDis(m){clearTimeout(wpT);wpA=0;var b=rmEl('wp_clr');if(b)b.textContent='Clear';if(m)wpM('');}
function wpProblem(p){return !p.length?'Enter a password.':p.length>19?'The web password can be at most 19 characters.':/[^\x20-\x7e]/.test(p)?'Use only plain letters, digits and symbols (no umlauts or special characters).':(p.charAt(0)==' '||p.charAt(p.length-1)==' ')?'The password must not start or end with a space.':p=='none'?'The word none is reserved. Choose another one.':'';}
function wpP(b,t,ok){rmPost('/rmwebpwd',b).then(rmJson).then(function(j){if(j&&j.ok){wpM(t);rmEl('wp_st').textContent='web login password: '+ok;if(ok=='set')setTimeout(function(){location.reload();},1500);}else wpM(rmErrText(j),1);})
.catch(function(e){wpM(e===0?'Log in again to change these settings.':'The node did not answer. Try again.',1);});}
function wpSet(){var i=rmEl('wp_pw'),k=rmEl('wp_pw2'),p=i.value,e=wpProblem(p);wpDis();
if(!e&&p!=k.value)e='The two entries do not match.';
if(e){wpM(e,1);return;}i.value='';k.value='';wpP('act=set&pw='+rmEnc(p),'Web password set. You are logged out, log in again.','set');}
function wpClr(){if(!wpA){wpA=1;rmEl('wp_clr').textContent='Really? tap again';wpM('Clear the web password? Everyone on the network can then open this page.');wpT=setTimeout(function(){wpDis(1);},4000);return;}
wpDis();wpP('act=clear','Web password cleared. This page is now open without a login.','not set');}
// WP-JS-END
// WQ-01: LoRa Queue panel on the rxlog page. rxlog is fetched by loadPage()
// and injected with innerHTML, which never runs a <script> tag it carries,
// so the rendering logic has to live here in the scaffold instead and be
// invoked from loadPage() after each fragment swap (that covers both the
// initial page load and the 30s autorefresh). The fragment itself only
// emits an empty #mcq div carrying data-* attributes; all markup below is
// built from those. JS strings use single quotes and HTML attribute
// values are written unquoted (none of them ever contain a space) so
// nothing here needs quote-escaping in the C string literals.
var mcQueueOpen=true;
function mcQueueToggle(btn){mcQueueOpen=!mcQueueOpen;var b=document.getElementById('mcq');if(b)b.hidden=!mcQueueOpen;if(btn)btn.textContent=mcQueueOpen?'hide':'show';}
function mcRenderQueue(){
var d=document.getElementById('mcq');
if(!d)return;
var p=[0,0,0,0,0,0];
var w=[0,0,0,0,0,0];
for(var i=1;i<=5;i++){p[i]=parseInt(d.getAttribute('data-p'+i))||0;w[i]=parseInt(d.getAttribute('data-w'+i))||0;}
var w0=parseInt(d.getAttribute('data-w0'))||0;
var ring=parseInt(d.getAttribute('data-ring'))||0;
var used=parseInt(d.getAttribute('data-p0'))||0;
var bp=parseInt(d.getAttribute('data-bp'))||0;
var bpname=d.getAttribute('data-bpname')||'';
var qrs=parseInt(d.getAttribute('data-qrs'))||0;
var qrsf=parseInt(d.getAttribute('data-qrsf'))||qrs;
var qrt=parseInt(d.getAttribute('data-qrt'))||0;
var win=parseInt(d.getAttribute('data-win'))||0;
var rx=parseInt(d.getAttribute('data-rx'))||0;
var tx=parseInt(d.getAttribute('data-tx'))||0;
var intv=parseInt(d.getAttribute('data-int'))||300;
var newid=parseInt(d.getAttribute('data-newid'))||0;
var dup=parseInt(d.getAttribute('data-dup'))||0;
var dwin=parseInt(d.getAttribute('data-dwin'))||0;
var age=parseInt(d.getAttribute('data-age'))||0;
var empty=ring-used;
if(empty<0)empty=0;
var qrsPct=ring>0?(qrs/ring*100):0;
var qrsfPct=ring>0?(qrsf/ring*100):0;
var qrtPct=ring>0?(qrt/ring*100):0;
var colors=['var(--mcred)','var(--mcp2)','var(--mcp3)','var(--mcp4)','var(--mcp5)'];
var names=['crit','high','normal','low','bg'];
var html='';
html+='<div class=font-bold>TX queue</div>';
html+='<div class=mcq-barwrap><div class=mcq-bar>';
for(var pr=1;pr<=5;pr++){var wn=Math.min(w[pr],p[pr]);for(var c=0;c<p[pr]-wn;c++){html+='<div class=mcq-cell style=background:'+colors[pr-1]+'></div>';}for(var cw=0;cw<wn;cw++){html+='<div class=mcq-cell-wait style=border-color:'+colors[pr-1]+'></div>';}}
for(var c2=0;c2<empty;c2++){html+='<div class=mcq-cell-empty></div>';}
html+='</div>';
html+='<div class=mcq-tick-faint style=left:'+qrsPct+'%></div>';
html+='<div class=mcq-tick style=left:'+qrsfPct+'%></div>';
html+='<div class=mcq-tick style=left:'+qrtPct+'%></div>';
html+='</div>';
html+='<div class=mcq-ticklabels>';
html+='<span class=font-small style=position:absolute;left:'+qrsfPct+'%;transform:translateX(-50%)>QRS</span>';
html+='<span class=font-small style=position:absolute;left:'+qrtPct+'%;transform:translateX(-50%)>QRT</span>';
html+='</div>';
html+='<div class=mcq-legend>'+used+'/'+ring+' queued&nbsp;|&nbsp;';
for(var pr2=1;pr2<=5;pr2++){html+='<span class=mcq-swatch style=background:'+colors[pr2-1]+'></span>'+names[pr2-1]+' '+p[pr2]+' ';}
if(w0>0){html=html.replace(/ +$/,'')+', '+w0+' waiting for ACK';}
html+='</div>';
var bpcolor=(bp==0)?'var(--mcok)':((bp==1)?'var(--mcsw2)':'var(--mcred)');
var bptxt=(bp==0)?'accepted':((bp==1)?'slow down':'refused');
html+='<div class=font-bold>Your messages: <span style=color:'+bpcolor+'>'+bptxt+'</span></div>';
var drop=parseInt(d.getAttribute('data-drop'))||0;
var iv=intv*1000;
html+='<div class=font-bold>Queue: '+used+'/'+ring+' queued'+(win?', '+drop+' dropped in the last '+Math.round(intv/60)+' min, TX '+(tx/iv*100).toFixed(1)+' % / RX '+(rx/iv*100).toFixed(1)+' %':'')+'</div>';
html+='<div class=font-small>(QRS forecast at&nbsp;'+qrsf+' for your next msgs, line&nbsp;&ge;'+qrs+', QRT at&nbsp;&ge;'+qrt+' of '+ring+')</div>';
if(win==0){
html+='<div class=font-bold>Dedup window: n/a (no completed 5-min window yet)</div>';
}else if(dwin==0){
html+='<div class=font-bold>Dedup window: n/a (no new ids in the last window)</div>';
}else{
var dwc=(dwin<40||dwin>48)?'var(--mcsw2)':'inherit';
html+='<div class=font-bold>Dedup window:&nbsp;&asymp;<span style=color:'+dwc+'>'+dwin+'</span> min</div>';
html+='<div class=font-small>('+newid+' new ids, '+dup+' dups in last '+Math.round(intv/60)+' min; safe corridor 40-48 min)</div>';
}
html+='<div class=font-bold>Channel utilisation (last 5 min)</div>';
if(win==0){
html+='<div class=font-small>n/a</div>';
}else{
var rxPct=rx/(intv*1000)*100;if(rxPct>100)rxPct=100;
var txPct=tx/(intv*1000)*100;if(txPct>100)txPct=100;
var totPct=(rx+tx)/(intv*1000)*100;if(totPct>100)totPct=100;
html+='<div class=mcq-util-row><span class=mcq-util-label>rx '+rxPct.toFixed(1)+'%</span><div class=mcq-util-track><div style=height:100%;width:'+rxPct+'%;background:#3B7DD8></div></div></div>';
html+='<div class=mcq-util-row><span class=mcq-util-label>tx '+txPct.toFixed(1)+'%</span><div class=mcq-util-track><div style=height:100%;width:'+txPct+'%;background:var(--mcred)></div></div></div>';
html+='<div class=font-small>total '+totPct.toFixed(1)+'% ('+age+' s ago)</div>';
}
d.innerHTML=html;
var btn=document.getElementById('mcqtogglebtn');
d.hidden=!mcQueueOpen;
if(btn)btn.textContent=mcQueueOpen?'hide':'show';
}
// Pins page (sub_page_pins()): live view and switches of the board LED, the charge LED and the MCP23017 pins.
// Sub-pages are injected with innerHTML, so the logic lives here; loadPage() calls pinsInit() when the page is in and
// pinsLeave() before the next page. GET /pinstatus every 5 s only while the page is shown; the answer is the node's
// cached state (see src/pin_status_json.h), an input level can be ~5 s old. A failed poll keeps the last table; HTTP 401 (session over) stops the polling and says so.
// Switches use /setparam/ (boardled, chargeled, mcpout<P>, mcpio<P>, mcpname<P>): fetch() carries X-MC through the wrapper above.
// Names are user text: only ever put into value/textContent, never into HTML.
// PINS-JS-BEGIN
var pnT=0,pnG=0,pnOn=false,pnBusy=false,pnRows={},pnAsg={},pnAsgOk={},pnAsgAny=false,pnAsgKnown=false,pnHw=false,pnGp={},pnGpAny=false,pnGpKnown=false;
var pnLoginNote='Session expired: reload this page and log in again.';
function pnBytes(s){var n=0;for(var i=0;i<s.length;i++){var c=s.charCodeAt(i);if(c<128)n++;else if(c<2048)n+=2;else if(c>=55296&&c<56320){n+=4;i++;}else n+=3;}return n;}
function pnCut(s){while(s.length&&pnBytes(s)>15){var k=s.length-1;if(k>0&&s.charCodeAt(k)>=56320&&s.charCodeAt(k)<57344&&s.charCodeAt(k-1)>=55296&&s.charCodeAt(k-1)<56320)k--;s=s.slice(0,k);}return s;}
function pnEl(i){return document.getElementById(i);}
function pnTxt(i,t){var e=pnEl(i);if(e)e.textContent=t;}
function pnArm(){clearTimeout(pnT);pnT=0;if(pnOn)pnT=setTimeout(pnPoll,5000);}
function pnPoll(){
if(!pnOn)return;
pnT=0;
if(pnBusy||document.visibilityState==='hidden'){pnArm();return;}
var g=pnG;pnBusy=true;
fetch('/pinstatus').then(function(r){if(r.status==401)return null;if(!r.ok)throw 0;return r.json();}).then(function(j){
if(g!=pnG)return;
if(!j){pnBusy=false;pnTxt('pn_note',pnLoginNote);return;}
pnRender(j);pnTxt('pn_note','');pnAssign(g);
}).catch(function(){
if(g!=pnG)return;pnBusy=false;pnTxt('pn_note','Stale: the node did not answer, showing the last values.');pnArm();
});
}
function pinsInit(){pinsLeave();pnOn=true;pnG++;pnPoll();}
function pinsLeave(){pnOn=false;pnG++;clearTimeout(pnT);pnT=0;pnBusy=false;pnRows={};pnAsg={};pnAsgOk={};pnAsgAny=false;pnAsgKnown=false;pnHw=false;pnGp={};pnGpAny=false;pnGpKnown=false;}
function pnSet(param,val,refuse){
fetch('/setparam/?'+param+'='+encodeURIComponent(val)).then(function(r){if(r.status==401)return null;return r.json();}).then(function(j){
if(!j){pnTxt('pn_msg',pnLoginNote);return;}
pnTxt('pn_msg',(j.returncode===0||refuse)?'':(j.returncode==2?'The node does not know this setting.':'The node refused the change.'));
if(refuse)refuse(j.returncode===0?'Saved.':(j.returncode==2?'The node does not know this setting.':'Refused by pin policy.'));
if(pnOn){clearTimeout(pnT);pnT=0;pnPoll();}
}).catch(function(){if(!refuse)pnTxt('pn_msg','The request failed.');if(refuse)refuse('The request failed.');});
}
// "No switchable outputs" only when there is neither an LED / MCP pin nor an assignment row; undecided until /pinassign answered once.
function pnNone(){var e=pnEl('pn_none');if(e)e.hidden=!pnAsgKnown||!pnGpKnown||pnHw||pnAsgAny||pnGpAny;}
// Pin assignments (GET /pinassign, src/pin_assign_json.h): 1-Wire, button, analog, anemometer. Writes use the same setparam names as the Setup page.
var pnAsgName={ow:'1-Wire',btn:'Button',an:'Analog',wind:'Anemometer'};
// the "ok" mask is 0x + 16 hex digits (64 bits do not fit a JS number): bit n is digit 17-floor(n/4), bit n%4
// the button also takes 99 = "no button" (not a bit of the mask): the setter accepts it always
function pnPinOk(ok,n,id){if(id=='btn'&&n==99)return true;if(typeof ok!='string'||ok.length!=18)return true;if(n<0||n>63)return false;var d=parseInt(ok.charAt(17-(n>>2)),16);return !isNaN(d)&&((d>>(n&3))&1)==1;}
function pnAsgRow(f){
var r=pnAsg[f.id];if(r)return r;
var tr=document.createElement('tr'),th=document.createElement('th'),tp=document.createElement('td'),td=document.createElement('td'),te=document.createElement('td'),tn=document.createElement('td');
th.scope='row';th.textContent=pnAsgName[f.id]||f.id;
var inp=document.createElement('input');inp.type='text';inp.inputMode='numeric';inp.maxLength=3;inp.size=4;inp.setAttribute('aria-label','Pin of '+(pnAsgName[f.id]||f.id));
inp.oninput=function(){inp.setAttribute('data-dirty','1');};
var note=document.createElement('span'),hint=document.createElement('span'),msg=document.createElement('span');
hint.id='pn_asg_h_'+f.id;hint.className='font-small';inp.setAttribute('aria-describedby',hint.id);
msg.setAttribute('role','status');
var sb=pnBtn('set',function(){
inp.removeAttribute('data-dirty');msg.textContent='';
var t=inp.value.trim();
if(!/^[0-9]{1,3}$/.test(t)){msg.textContent='Enter a pin number.';return;}
var v=parseInt(t,10);
if(!pnPinOk(pnAsgOk[f.id],v,f.id)){msg.textContent='Refused by pin policy.';return;}
pnSet(f.param,v,function(m){msg.textContent=m;});
});
sb.setAttribute('aria-label','Set the pin of '+(pnAsgName[f.id]||f.id));
tp.appendChild(inp);tp.appendChild(sb);tn.appendChild(note);tn.appendChild(document.createTextNode(' '));tn.appendChild(hint);tn.appendChild(document.createTextNode(' '));tn.appendChild(msg);
tr.appendChild(th);tr.appendChild(tp);tr.appendChild(td);tr.appendChild(te);tr.appendChild(tn);
pnEl('pn_asg_body').appendChild(tr);
r=pnAsg[f.id]={inp:inp,sb:sb,def:td,eff:te,note:note,hint:hint};return r;
}
function pnAsgRender(j){
var fs=j.fn||[];
pnAsgAny=false;for(var q=0;q<fs.length;q++)if(fs[q].avail)pnAsgAny=true;
pnAsgKnown=true;pnNone();
var e=pnEl('pn_asg');if(e)e.hidden=!fs.length;
e=pnEl('pn_asg_hint');if(e)e.hidden=!fs.length;
for(var i=0;i<fs.length;i++){
var f=fs[i],r=pnAsgRow(f);pnAsgOk[f.id]=f.ok;
if(!f.avail){r.inp.value='';r.inp.disabled=true;r.sb.hidden=true;r.def.textContent='';r.eff.textContent='';r.note.textContent='not on this board';r.hint.textContent='';continue;}
r.inp.disabled=false;r.sb.hidden=false;
var un=f.id=='an'&&(f.pin==0||f.pin>=99);r.inp.placeholder=un?'unset':'';
if(!r.inp.getAttribute('data-dirty')&&document.activeElement!==r.inp)r.inp.value=un?'':f.pin;
r.def.textContent=f.def>=0?f.def:'none';
r.eff.textContent=f.eff>=0?f.eff:'none';
r.note.textContent=(f.on?'on':'off')+', '+(f.restart?'a change applies after a restart':'a change applies at once');
// 0 only where the setter takes it; 99 only for the button
var h=[];if(pnPinOk(f.ok,0,f.id))h.push(f.id=='wind'?'0 = off':'0 = board default');if(f.id=='btn')h.push('99 = no button');
r.hint.textContent=h.length?'('+h.join(', ')+')':'';
}
}
// Chained after the /pinstatus answer: one connection at a time (W5100S sockets). pnBusy stays set until it is done.
function pnAssign(g){
fetch('/pinassign').then(function(r){if(!r.ok)throw 0;return r.json();}).then(function(j){
if(g!=pnG)return;pnAsgRender(j);pnGpio(g);
}).catch(function(){
if(g!=pnG)return;pnAsgKnown=true;pnNone();pnGpio(g);
});
}
// Free GPIO (GET /gpiostatus, src/gpio_status_json.h): chained after /pinassign, the last link of the chain (pnBusy ends here).
// A failed answer keeps the rows. The master is switched by /setparam/?gpioctl=on|off (X-MC), the pins by /setparam/?gpio<N>=on|off|in|release.
var pnGpOwn={ow:'1-Wire',btn:'Button',an:'Analog',wind:'Anemometer',led:'Board LED',fn:'a function'};
var pnGpOps=['on','off','in','release'];
function pnGpRow(pin){
var r=pnGp[pin];if(r)return r;
var tr=document.createElement('tr'),th=document.createElement('th'),to=document.createElement('td'),td=document.createElement('td'),tl=document.createElement('td'),ts=document.createElement('td');
th.scope='row';th.textContent=pin;
var msg=document.createElement('span'),bs={};
msg.setAttribute('role','status');
pnGpOps.forEach(function(op){
var b=pnBtn(op,function(){msg.textContent='';pnSet('gpio'+pin,op,function(m){msg.textContent=m=='Saved.'?'':(m=='Refused by pin policy.'?'Refused: GPIO control is off or the pin is in use.':m);});},'mctab mcledbtn');
b.setAttribute('aria-label','GPIO '+pin+' '+op);b.setAttribute('aria-pressed','false');bs[op]=b;ts.appendChild(b);
});
ts.appendChild(document.createTextNode(' '));ts.appendChild(msg);
tr.appendChild(th);tr.appendChild(to);tr.appendChild(td);tr.appendChild(tl);tr.appendChild(ts);
pnEl('pn_gpio_body').appendChild(tr);
r=pnGp[pin]={tr:tr,own:to,dir:td,lvl:tl,bs:bs,msg:msg};return r;
}
function pnGpioRender(j){
var ps=j.pins||[],seen={};
pnGpAny=!!j.avail&&ps.length>0;pnGpKnown=true;pnNone();
var e=pnEl('pn_gpio');if(e)e.hidden=!pnGpAny;
e=pnEl('pn_gpio_off');if(e)e.hidden=!pnGpAny||!!j.master;
e=pnEl('pn_gpioctl');if(e)e.hidden=!j.avail;
for(var m=0;m<2;m++){var mo=m?'on':'off',mb=pnEl('pn_gpioctl_'+mo),mon=(mo=='on')==!!j.master;if(mb){mb.setAttribute('aria-pressed',mon?'true':'false');mb.classList.toggle('mctab-on',mon);}}
for(var i=0;i<ps.length&&pnGpAny;i++){
var p=ps[i],r=pnGpRow(p.pin);seen[p.pin]=1;
r.own.textContent=p.own?(pnGpOwn[p.own]||p.own):'free';
r.dir.textContent=p.dir;
r.lvl.textContent=p.lvl==1?'HIGH':p.lvl==0?'LOW':'-';
var sw=!!j.master&&!p.own;
for(var k=0;k<pnGpOps.length;k++){
var op=pnGpOps[k],b=r.bs[op],on=(op=='on'&&p.dir=='out'&&p.lvl==1)||(op=='off'&&p.dir=='out'&&p.lvl==0)||(op=='in'&&p.dir=='in');
b.hidden=!sw;b.setAttribute('aria-pressed',on?'true':'false');b.classList.toggle('mctab-on',on);
}
}
for(var q in pnGp)if(!seen[q]){var o=pnGp[q];if(o.tr.parentNode)o.tr.parentNode.removeChild(o.tr);delete pnGp[q];}
}
function pnGpio(g){
fetch('/gpiostatus').then(function(r){if(!r.ok)throw 0;return r.json();}).then(function(j){
if(g!=pnG)return;pnBusy=false;pnGpioRender(j);pnArm();
}).catch(function(){
if(g!=pnG)return;pnBusy=false;pnGpKnown=true;pnNone();pnArm();
});
}
function pinsLed(m){pnSet('boardled',m);}
function pinsGpioCtl(m){pnSet('gpioctl',m);}
function pinsChg(m){pnSet('chargeled',m=='auto'?'on':'off');}
function pnBtn(txt,fn,cls){var b=document.createElement('button');b.type='button';b.className=cls||'mctab';b.textContent=txt;b.onclick=fn;return b;}
// maxLength counts characters; the store takes 15 bytes, so pnCut() caps by UTF-8 bytes
function pnRow(id){
var r=pnRows[id];if(r)return r;
var tr=document.createElement('tr'),th=document.createElement('th'),tn=document.createElement('td'),td=document.createElement('td'),tl=document.createElement('td');
th.scope='row';th.textContent=id;
var inp=document.createElement('input');inp.type='text';inp.maxLength=15;inp.setAttribute('aria-label','Name of pin '+id);
inp.oninput=function(){var v=pnCut(inp.value);if(v!=inp.value)inp.value=v;inp.setAttribute('data-dirty','1');};
var sb=pnBtn('set',function(){inp.removeAttribute('data-dirty');pnSet('mcpname'+id,pnCut(inp.value));});
sb.setAttribute('aria-label','Set the name of pin '+id);
tn.appendChild(inp);tn.appendChild(sb);
var db=pnBtn('',function(){pnSet('mcpio'+id,db.getAttribute('data-d')=='out'?'in':'out');},'mctab mcledbtn');
td.appendChild(db);
var lb=pnBtn('',function(){pnSet('mcpout'+id,lb.getAttribute('aria-pressed')=='true'?'off':'on');},'mctab mcledbtn');
var lt=document.createElement('span');
tl.appendChild(lb);tl.appendChild(lt);
tr.appendChild(th);tr.appendChild(tn);tr.appendChild(td);tr.appendChild(tl);
pnEl('pn_body').appendChild(tr);
r=pnRows[id]={inp:inp,db:db,lb:lb,lt:lt};return r;
}
function pnRender(j){
var l=j.led||{},c=j.chg||{},m=j.mcp||{},ms=['off','flash','steady'];
var e=pnEl('pn_led');if(e)e.hidden=!l.fitted;
if(l.fitted){
for(var i=0;i<3;i++){var b=pnEl('pn_led_'+ms[i]);if(b){var on=l.mode==ms[i];b.setAttribute('aria-pressed',on?'true':'false');b.classList.toggle('mctab-on',on);b.disabled=!!l.busy;}}
pnTxt('pn_led_st','Now: '+l.state+(l.busy?' (pin busy: OneWire owns the pad)':'')+(l.rm?' (held by a remote command: '+l.rm+')':''));
}
e=pnEl('pn_chg');if(e)e.hidden=!c.fitted;
if(c.fitted){
var cm=['off','auto'];
for(var q=0;q<2;q++){var cb=pnEl('pn_chg_'+cm[q]);if(cb){var con=c.mode==cm[q];cb.setAttribute('aria-pressed',con?'true':'false');cb.classList.toggle('mctab-on',con);}}
}
if(c.fitted)pnTxt('pn_chg_st',c.eff+(c.rm?' (remote override, console setting: '+c.mode+')':' (console setting)'));
e=pnEl('pn_mcp');if(e)e.hidden=!m.present;
pnHw=!!(l.fitted||c.fitted||m.present);pnNone();
if(!m.present)return;
var ps=m.pins||[];
for(var k=0;k<ps.length;k++){
var p=ps[k],r=pnRow(p.id),o=p.d=='out';
if(!r.inp.getAttribute('data-dirty')&&document.activeElement!==r.inp)r.inp.value=p.n;
r.db.textContent=p.d;r.db.setAttribute('data-d',p.d);r.db.setAttribute('aria-label','Pin '+p.id+' is an '+(o?'output':'input')+', switch direction');
r.lb.hidden=!o;r.lt.hidden=o;
if(o){var hi=p.l==1;r.lb.textContent=hi?'HIGH':'LOW';r.lb.setAttribute('aria-pressed',hi?'true':'false');r.lb.classList.toggle('mctab-on',hi);r.lb.setAttribute('aria-label','Pin '+p.id+' output is '+(hi?'high':'low')+', switch');}
else r.lt.textContent=p.l==1?'HIGH':'LOW';
}
}
// PINS-JS-END
