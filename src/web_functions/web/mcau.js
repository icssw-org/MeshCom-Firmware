// Info page "Auto update" row (campaign-au-webgui-ux): one renderer driven by GET /austatus. ESP32 only (/mcau.js).
// Lives in the scaffold because the sub-pages are injected with innerHTML (a <script> inside never runs);
// loadPage() calls auPageLeave() / auPageInit(). The state selector and the error table mirror
// au_ui_state.h (auUiState, auErrText); tools/webgui_au_test.js runs this code against its golden strings.
var auS=null,auF=null,auBusy=false,auLoc='',auShown=false,auInfl=false,auFInfl=false,auFPct=0,auFMsg='',auFDead=false,auUpAt=0,auT={poll:0,tick:0};
function auEl(i){return document.getElementById(i);}
function auKey(){return 'auF:'+location.host;}
function auFGet(){try{var v=sessionStorage.getItem(auKey());return v?JSON.parse(v):null;}catch(e){return null;}}
function auFSet(o){try{if(o)sessionStorage.setItem(auKey(),JSON.stringify(o));else sessionStorage.removeItem(auKey());}catch(e){}}
function auReload(){location.reload();}
function auWord(e){return String(e||'').split(' ')[0];}
function auIn(w,l){return l.indexOf(w)>=0;}
function auErrText(job,err){var w=auWord(err),c=job=='check',d=job=='download'||job=='stagelan';
if(auIn(w,['tls','conn','http','begin','timeout']))return 'No connection to GitHub';
if(c&&auIn(w,['short','stall','toobig','chunk','feed','enc','json','noenv','badver']))return 'The release list could not be read';
if(d&&auIn(w,['short','stall','digest','readback','crc','write','enc','feed','chunk','inflate_chk','image']))return 'Download damaged, it was discarded -- try again';
if(d&&auIn(w,['noasset','nodigest','env']))return 'This release has no image for this board';
if(d&&auIn(w,['notota0','imglen','size','overlap','small','inflate','layout']))return 'This node'+String.fromCharCode(39)+'s partition layout cannot take the image -- use the Webflasher';
if(d&&auIn(w,['erase','noflash','nvs']))return 'Could not write the image to flash';
if(w=='nomem')return 'Not enough free memory for the update';
return 'Update failed';}
function auLowHeap(e){var w=auWord(e);return w=='lowheap'||w=='lowheap_phone'||w=='lowheap_ble_off'||w=='lowheap_tls';}
function auState(s,f){if(!s.sbok)return 'S';if(f)return 'F';
if(s.st=='busy'){if(s.job=='check')return 'B';return s.pct>=100?'d':'D';}
if(s.staged)return 'E';var w=auWord(s.err);
if(w=='lowheap_phone')return s.phone?'G':'g';
if(w=='lowheap_ble_off'||w=='lowheap'||w=='lowheap_tls')return 'H';
if(s.newer&&s.inst)return 'C';return 'A';}
function auDraw(g,m,e,h,lab,act,dis,red){var t=auEl('au_tr'),b=auEl('au_btn');if(!t||!b)return;
t.className=g?'au-green':'';auEl('au_msg').textContent=m;auEl('au_err').textContent=e;auEl('au_hint').textContent=h;
b.textContent=lab;b.className=red?'au-red':'';b.disabled=!!(dis||auBusy);b.setAttribute('data-act',act);}
function auRender(){var s=auS;if(!s)return;var L=auState(s,0),g=0,m='',lab='Check now',act='check',dis=0,red=0,e='',h='';
if(L=='S'){m=s.sbwhy=='dio'?'! Bootloader (DIO) cannot start the Safeboot: renew bootloader and Safeboot with the Webflasher':'! Use Webflasher to renew Auto Update aware Safeboot';dis=1;}
else if(L=='B'){lab='Checking now ...';dis=1;}
else if(L=='D'){g=1;m='Firmware '+s.avail+' available';lab='Downloading now ... '+s.pct+' %';dis=1;}
else if(L=='d'){g=1;lab='Verifying download ...';dis=1;}
else if(L=='E'){g=1;m='Firmware '+s.staged+' downloaded and verified';lab='Install now';act='apply';red=1;}
else if(L=='G'){m='! A connected phone uses the memory the update needs';lab='Pause Bluetooth and check';act='ble';}
else if(L=='g'){m='The phone has disconnected -- check again';}
else if(L=='H'){m='! Not enough free memory for the update. Reboot the node and try again.';}
else if(L=='C'){g=1;m='Firmware '+s.avail+' available';lab='Download';act='install';}
else{m=s.newer&&!s.inst?'Firmware '+s.avail+' exists, but has no image for this board':(s.avail?'up to date':'available none');}
if((L=='A'||L=='C'||L=='E')&&(auLoc||(s.err&&!auLowHeap(s.err))))e=auLoc||auErrText(s.job,s.err)+' ['+s.err+']';
var r=auEl('au_row');if(s.phone&&(L=='A'||L=='C')&&!e&&r&&r.getAttribute('data-blepause')=='1')h='(i) A connected phone may block the update on this board';
// BLE guard (section 7): the phone link is dropped for the job; the row says so until the guard is released
if(s.guard&&L!='S'&&L!='E'){m='Bluetooth paused for the update';e='';h='';dis=1;act='none';if(L!='B'&&L!='D'&&L!='d')lab='Checking now ...';}
auDraw(g,m,e,h,lab,act,dis,red);
// install schedule: the countdown notice of the node (text built in fw_schedule.h); never while a job runs, the update cannot start or the release has no image
var sc=auEl('au_sched');if(sc)sc.textContent=(s.sched&&!s.guard&&(L=='A'||L=='C'||L=='E')&&!(s.newer&&!s.inst))?s.sched:'';}
function auSched(){clearTimeout(auT.poll);auT.poll=0;if(!auShown||!auS||auF)return;var L=auState(auS,0),ms=0;
if(auS.st=='busy'||auS.guard)ms=1000;else if(L=='G')ms=10000;
if(ms)auT.poll=setTimeout(auFetch,ms);}
function auFetch(){if(!auShown)return;if(cpage!='info'||!auEl('au_row')){auPageLeave();return;}if(auInfl)return;auInfl=true;
fetch('/austatus').then(function(r){if(r.status==404){var t=auEl('au_tr');if(t)t.style.display='none';return null;}if(!r.ok)throw 0;return r.json();}).then(function(j){auInfl=false;
if(!auShown||!j)return;if(cpage!='info'||!auEl('au_row')){auPageLeave();return;}auApply(j);}).catch(function(){auInfl=false;auBusy=false;auRender();auSched();if(auShown&&!auF&&!auT.poll)auT.poll=setTimeout(auFetch,5000);});}
function auApply(s){auS=s;auUpAt=Date.now();auBusy=false;auRender();auSched();}
function auClick(){if(auBusy||auF||!auS)return;var b=auEl('au_btn'),a=b?b.getAttribute('data-act'):'';if(!a||b.disabled)return;
auBusy=true;b.disabled=true;auLoc='';
if(a!='apply'){auGo(a,auS.staged);return;}
clearTimeout(auT.poll);auT.poll=0;
fetch('/austatus').then(function(r){if(!r.ok)throw 0;return r.json();}).then(function(j){
if(!j||typeof j.staged!='string'||!j.staged){auApply(j&&typeof j.st=='string'?j:auS);return;}
auS=j;auUpAt=Date.now();auGo(a,j.staged);}).catch(function(){auBusy=false;auLoc='No answer from the node';auRender();});}
function auGo(a,tag){var fn=a=='apply'?'updapply':(a=='install'?'updinstall':'updcheck'),p=a=='ble'?'ble':'';
fetch('/callfunction/?'+fn+'='+p).then(function(r){return r.json();}).then(function(j){
if(a=='apply'&&j[fn]=='ok'){auFBegin({tag:tag,t0:Date.now(),etaS:auS.etaS||35,upBase:typeof auS.up=='number'?auS.up:undefined,upAt:auUpAt,img:typeof auS.img=='string'?auS.img:undefined});return;}
if(j[fn]!='ok')auLoc=a=='apply'?'Install of '+tag+' failed':'The node did not start the job, try again in a minute';
auInfl=false;auFetch();}).catch(function(){auBusy=false;auLoc='No answer from the node';auRender();});}
function auFBegin(o){auF=o;auFDead=false;auFSet(o);auBusy=false;clearTimeout(auT.poll);clearInterval(auT.tick);auFPct=0;auFMsg='';auFDraw();auT.tick=setInterval(auFTick,1000);}
function auFDraw(){if(!auF)return;var el=(Date.now()-auF.t0)/1000,m=auFMsg||('Installing '+auF.tag+', the node restarts ...');
if(!auFMsg&&el>180+auF.etaS)m='The node is not back yet. Check its display and power; reload this page when it is up.';
auDraw(1,m,'','',auFDead?'Install failed':'Installing now ... '+auFPct+' %','none',1,0);}
function auFTick(){if(!auF)return;if(cpage!='info'||!auEl('au_row')){auPageLeave();return;}
var el=(Date.now()-auF.t0)/1000,p=Math.min(95,Math.floor(el/auF.etaS*100));if(p>auFPct)auFPct=p;auFDraw();
if(el>=Math.max(auF.etaS-10,8)&&Math.floor(el)%2==0)auFProbe();}
function auFProbe(){if(auFInfl)return;auFInfl=true;
// a firmware without the /austatus route answers the Info scaffold (2xx HTML with "cpage=") on /austatus and /ota/state:
// the node is demonstrably up, so that counts as back. Safeboot answers /ota/state with JSON; a redirect page has no "cpage=".
var auSc=false;
fetch('/austatus').then(function(r){if(!r.ok)throw 0;return r.text();}).then(function(t){var j=null;try{j=JSON.parse(t);}catch(e){}
if(j&&typeof j.run=='string'){auFInfl=false;auFJson(j);return;}auSc=t.indexOf('cpage=')>=0;throw 0;}).catch(function(){
fetch('/ota/state').then(function(r){if(!r.ok)throw 0;return r.text();}).then(function(t){var o=null;try{o=JSON.parse(t);}catch(e){}
auFInfl=false;if(o){auFOta(o);return;}if(auSc&&auF){auFDone();return;}throw 0;}).catch(function(){auFInfl=false;});});}
function auFDone(){auFPct=100;auFDraw();auFSet(null);clearInterval(auT.tick);auF=null;auReload();}
function auFRestarted(j){return typeof j.up=='number'&&typeof auF.upBase=='number'&&j.up+10<auF.upBase+(Date.now()-auF.upAt)/1000;}
// run "local": a Safeboot fallback also restarts the node, so with "img" on both sides only a changed image id is success
function auFNewImg(j){if(typeof j.img=='string'&&typeof auF.img=='string')return j.img!=auF.img;return auFRestarted(j);}
function auFJson(j){if(!auF)return;if(j.run==auF.tag||(j.run=='local'&&auFNewImg(j))){auFDone();return;}
auFSet(null);clearInterval(auT.tick);auT.tick=0;auF=null;auFMsg='';
if(j.sbok){auLoc='Install failed, the old firmware is running';}
auApply(j);}
// apply_failed: with fallback_in_ms >= 0 Safeboot counts down to the old firmware; with a negative value or app_valid false it has
// no app to fall back to and waits for an upload (dead end, the line stays; probing goes on so the page recovers by itself)
function auFOta(o){if(!auF||!o)return;auFMsg='';auFDead=false;if(o.reason=='apply_failed'||o.state=='aborted'){
var fb=typeof o.fallback_in_ms=='number'?o.fallback_in_ms:-1;
if(o.app_valid===false||fb<0){auFDead=true;auFMsg='Install failed -- the node stays in Safeboot. Upload a firmware at http://'+location.host+'/update or use the Webflasher';}
else auFMsg='Install failed -- the node returns to the old firmware in '+Math.max(0,Math.ceil(fb/1000))+' s';}
else if(o.state=='applying'&&o.total>0){var p=Math.min(99,Math.floor(o.received*100/o.total));if(p>auFPct)auFPct=p;}
auFDraw();}
// a remembered click counts only for the tag the node still has staged (or already runs); anything else, or a record past its
// time budget while the node answers idle, is dropped and the plain base state is drawn
function auFStale(f,j){if(j.run==f.tag)return false;if(j.staged!=f.tag)return true;
return j.st!='busy'&&(Date.now()-f.t0)/1000>180+(f.etaS||35)+60;}
function auFInit(f){fetch('/austatus').then(function(r){if(!r.ok)throw 0;return r.text();}).then(function(t){var j=null;try{j=JSON.parse(t);}catch(e){}
if(!auShown||auF)return;if(j&&typeof j.run=='string'&&auFStale(f,j)){auFSet(null);auApply(j);return;}auFBegin(f);}).catch(function(){if(auShown&&!auF)auFBegin(f);});}
function auPageLeave(){clearTimeout(auT.poll);clearInterval(auT.tick);auT.poll=0;auT.tick=0;auShown=false;auInfl=false;auFInfl=false;auBusy=false;auS=null;auF=null;auLoc='';auFMsg='';auFDead=false;auFPct=0;}
function auPageInit(){auPageLeave();var b=auEl('au_btn');if(!b)return;auShown=true;b.onclick=auClick;
var f=auFGet();if(f&&f.tag&&f.t0){auFInit(f);return;}auFetch();}
