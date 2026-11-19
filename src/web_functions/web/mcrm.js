var rmShown=false,rmT={tick:0,arm:0,poll:0},rmSel={call:'',slot:-1},rmKnown=Object.create(null),rmSaved=[],rmHeard=[],rmLock=0,rmSrvLock=0,rmSrvMsg='',rmBusy=false,rmArmId='',rmQq=[],rmQp='',rmAuth=false,rmOut=false,rmStat=null,rmWasLocked=false,rmRows=Object.create(null);
var rmCmds={status:'Status',sendpos:'Send position',sendtrack:'Send track',reboot:'Restart',sync:'Re-sync counter',radio:'Radio',sens:'Sensors',name:'Name',atxt:'APRS text',pos:'Position',txq:'TX queue',mbox:'Mailbox',maxhop:'Max hop',mh:'Heard list',net:'Network',upd:'Update',blockcall:'Blocked calls',pins:'Pin assignments',gpio:'Free GPIO'};
var rmUpdT0=0,rmUpdGone=Object.create(null),rmReally='Really? tap again',rmNoAns='The node did not answer the request. Check the connection and try again.',rmNote={t:'',c:''},rmSw=Object.create(null);
function rmCf(l,w){return 'Tap again within 4 seconds to confirm: '+l+' on '+w+'.'+(l.indexOf('Update channel ')==0?' The last check result is cleared and a new check is needed.':'');}
var rmTog={gps:0,track:1,display:2,mesh:3,gateway:4,led:5},rmTogName={gps:'GPS',track:'Track',display:'Display',led:'Light',mesh:'Mesh',gateway:'Gateway'},rmChgModes=['auto','on','off'];function rmSwLbl(n){return n=='ledflash'?'Light flash':rmTogName[n];}
var rmForce=null;
function rmNow(){return Date.now();}
function rmEl(i){return document.getElementById(i);}
function rmTxt(i,t,c){var e=rmEl(i);if(!e)return;e.textContent=t;if(c!==undefined)e.className=c;}
function rmMsg(t,c){rmNote.t=t;rmNote.c='rmmsg '+(c||'');rmRenderLock();}
function rmEnc(s){return encodeURIComponent(s);}
function rmErrText(j){return j&&j.msg?String(j.msg):'The request was refused'+(j&&j.err?' ('+j.err+')':'')+'.';}
function rmAgo(s){return s<120?s+' s':s<7200?Math.round(s/60)+' min':Math.round(s/3600)+' h';}
function rmUp(m){m=parseInt(m,10);if(isNaN(m))return '';return 'up '+(m<120?m+' min':Math.floor(m/60)+' h'+(m%60?' '+(m%60)+' min':''));}
function rmValidCall(c){return typeof c=='string'&&c.length<=9&&/^[A-Z0-9]{2,}-[0-9]{1,2}$/.test(c);}
function rmPwProblem(p){return !p.length?'Enter a password.':p.length>14?'The password can be at most 14 characters.':'';}
function rmKnFor(c){if(!rmKnown[c])rmKnown[c]={sw:[-1,-1,-1,-1,-1,-1],led:false,fl:false,chg:'',ver:'',up:'',bat:'',at:0};return rmKnown[c];}
function rmKn(){return rmKnFor(rmSel.call);}
function rmLockLeft(){var m=Math.max(rmLock,rmSrvLock)-rmNow();return m>0?Math.ceil(m/1000):0;}
function rmLocked(){return rmBusy||rmAuth||rmLockLeft()>0;}
function rmSlotOf(c){for(var i=0;i<rmSaved.length;i++)if(rmSaved[i]&&rmSaved[i].used&&rmSaved[i].call==c)return i;return -1;}
var rmTL=Object.create(null);function rmRowLocked(d){return rmBusy||rmAuth||rmLock>rmNow()||rmTL[d]>rmNow();}
function rmTxp(v){var m=/^(-?\d+)\/(-?\d+)\/(-?\d+)$/.exec(String(v));return m?m[1]+' dBm ('+m[2]+'..'+m[3]+')':String(v)+' dBm';}
var rmPF={ow:'1-Wire',btn:'Button',an:'Analog',wind:'Wind'};
function rmChL(f){var p=f.split(' ');return p[0]=='upd'&&p[1]=='chan'?'Update channel '+(p[2]=='dev'?'DEV':'MAIN'):p[0]=='pins'&&rmPF[p[1]]?'Set '+rmPF[p[1]]+' to '+(p[2]=='0'?(p[1]=='wind'?'off':'board default'):'GPIO '+p[2]):p[0]=='gpio'&&p.length==3?'GPIO '+p[1]+' '+(p[2]=='in'?'input':p[2]):'';}
function rmLabel(f){var p=f.split(' '),n=p[0];if(rmChL(f))return rmChL(f);if(rmCmds[n])return rmCmds[n];if(rmTogName[n])return rmTogName[n]+' '+p[1];if(n=='txpower')return 'TX power '+p[1]+' dBm';if(n=='setout')return 'Pin '+p[1]+' '+p[2];if(n=='chgled')return 'Charge LED'+f.slice(6);return f;}
function rmParseStatus(r){if(typeof r!='string'||r.indexOf('ok v=')!==0)return null;var o={ver:'',up:'',bat:'',sw:[-1,-1,-1,-1,-1,-1],led:false},t=r.split(' '),U='GTDMWL',i,j,k,m,s,ok;
for(i=1;i<t.length;i++){k=t[i];
if(k.indexOf('v=')==0)o.ver=k.substring(2);
else if(k.indexOf('up=')==0)o.up=k.substring(3);
else if(k.indexOf('bat=')==0)o.bat=k.substring(4);
else if(k.indexOf('p=')==0)o.p=rmTxp(k.substring(2));
else if(k.indexOf('rs=')==0)o.rs=k.substring(3);
else if(k.indexOf('s=')==0){s=k.substring(2);if(s.length!=6&&s.length!=5)return null;ok=true;m=[-1,-1,-1,-1,-1,-1];for(j=0;j<s.length;j++){if(s.charAt(j)==U.charAt(j))m[j]=1;else if(s.charAt(j)==U.charAt(j).toLowerCase())m[j]=0;else ok=false;}if(!ok)return null;for(j=0;j<6;j++)if(m[j]>=0)o.sw[j]=m[j];if(s.length==6)o.led=true;}
}
return o;}
function rmApplyEntry(k,e){var c=e.cmd.split(' '),n=c[0],o,m,i;
if(n=='status'){o=rmParseStatus(e.reply);if(!o)return;for(i=0;i<6;i++)if(o.sw[i]>=0)k.sw[i]=o.sw[i];if(o.led)k.led=true;if(o.sw[5]==0)k.fl=false;k.ver=o.ver;k.up=o.up;k.bat=o.bat;k.at=rmNow()-e.ago*1000;}
else if(n=='led'&&(m=/^ok led=(on|off|flash)$/.exec(e.reply))){k.sw[5]=m[1]=='off'?0:1;k.fl=m[1]=='flash';k.led=true;}
else if(n=='chgled'){m=/^ok chgled=(auto|on|off)( rm)?$/.exec(e.reply);if(m)k.chg=m[1];}
else if(rmTog[n]!==undefined){m=/^ok (\w+)=(on|off)$/.exec(e.reply);if(m&&m[1]==n){k.sw[rmTog[n]]=m[2]=='on'?1:0;if(n=='led')k.led=true;}}
}
function rmApplySent(list){var a=list.slice(0).sort(function(x,y){return y.ago-x.ago;}),i,e;
for(i=0;i<a.length;i++){e=a[i];if(e.rep&&e.ver&&e.st=='ok')rmApplyEntry(rmKnFor(String(e.dst)),e);else if(e.rep&&e.ver&&e.st=='err'&&/^chgled\b/.test(e.cmd)&&(e.reply=='err unsupported'||e.reply=='err blocked'))rmKnFor(String(e.dst)).chg='unsupported';}}
function rmOkText(e){var c=e.cmd.split(' '),n=c[0],o,m;
if(n=='status'){o=rmParseStatus(e.reply);return o?'Connected. Version '+o.ver+(o.up!==''?', '+rmUp(o.up):'')+(o.bat!==''?', battery '+o.bat+' %':'')+(o.rs=='1'?', strict security on':'')+'.':e.msg;}
if(n=='sendpos')return 'Done. The node sent its position.';
if(n=='sendtrack')return 'Done. The node sent its track.';
if(n=='reboot')return 'Done. The node is restarting, it is back in about 30 seconds.';
if(n=='sync')return 'Connected. The counters are in step.';
if(n=='led'&&c[1]=='flash')return 'Done. The light is now flashing.';
if(n=='chgled'){m=/^ok chgled=(auto|on|off)( rm)?$/.exec(e.reply);return c[1]?'Done. Charge LED is now '+c[1]+'.':m?'Charge LED is '+m[1]+(m[2]?' (set by remote management)':'')+'.':e.msg||'Done.';}
if(rmTogName[n])return 'Done. '+rmTogName[n]+' is now '+c[1]+'.';
if(n=='pins'||n=='gpio')return c[1]?'Done. '+rmChL(e.cmd)+'.':n=='pins'?'Pin assignments read.':'GPIO state read.';
if(n=='txpower')return 'Done. TX power is now '+c[1]+' dBm.';
if(n=='setout')return 'Done. Pin '+c[1]+' is now '+c[2]+'.';
return rmOkNew(e,c,n)||e.msg||'Done.';}
function rmPost(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b});}
function rmJson(r){if(r.status==401||r.status==403){rmAuthFail();return Promise.reject(0);}return r.json();}
function rmAuthFail(){rmAuth=true;clearTimeout(rmT.poll);rmT.poll=0;var e=rmEl('rm_auth');if(e){e.textContent='Your session has ended. Please log in again (reload this page).';e.style.display='';}rmRender();}
function rmCatch(e,id){if(e===0)return;if(id)rmTxt(id,rmNoAns,'rmmsg rmbad');else rmMsg(rmNoAns,'rmbad');}
var rmQw=null;function rmQClr(){rmQq=[];rmQp='';rmQw=null;}
function rmQDone(){var a,i,e,w=rmQw;if(!w)return true;if(rmNow()-w.t>=60000)return true;if(!rmStat||rmStat.t<w.t)return false;a=rmStat.sent;for(i=0;i<a.length;i++){e=a[i];if(e.dst==w.d&&e.cmd.split(' ')[0]==w.c&&e.ago*1000<=rmStat.t-w.t+5000&&e.st!='queued'&&e.st!='waiting')return true;}return false;}
function rmHintOff(){if(/rmconf/.test(rmNote.c))rmMsg('','');}
function rmDisarm(){clearTimeout(rmT.arm);rmT.arm=0;rmArmId='';rmHintOff();rmRender();}
function rmConfirm(id,hint){if(rmArmId==id){clearTimeout(rmT.arm);rmT.arm=0;rmArmId='';return true;}clearTimeout(rmT.arm);rmArmId=id;rmT.arm=setTimeout(rmDisarm,4000);rmRender();rmMsg(hint,'rmconf');return false;}
function rmBtn(box,label,cls,dis,attrs){var b=document.createElement('button'),k;b.type='button';b.className=cls||'';b.disabled=!!dis;b.textContent=label;for(k in attrs)b.setAttribute(k,attrs[k]);box.appendChild(b);return b;}
function rmTile(box,label,cmd,args,cf,cls,dis){var arm=cf&&rmArmId==cf,a={'data-cmd':cmd,'data-args':args};if(cf)a['data-cf']=cf;return rmBtn(box,arm?rmReally:label,'rmtile '+(cls||'')+(arm?' rmarmed':''),dis,a);}
function rmSwSet(i,st,dis,last,shown){i.disabled=!!dis;i.indeterminate=st<0&&!(shown&&last);i.checked=st<0?last=='on':st==1;}
function rmSwRow(box,n,st,dis,x){var l=document.createElement('label'),i=document.createElement('input'),a=rmArmId==n+' off';i.type='checkbox';i.id='rm_sw_'+n;i.className='rmsw';i.setAttribute('role','switch');i.setAttribute('data-sw',n);
if(n=='mesh'||n=='gateway')i.setAttribute('data-cf',n+' off');l.htmlFor=i.id;l.textContent=a?rmSwLbl(n)+': '+rmReally:rmSwLbl(n);if(x)l.appendChild(rmNew('span','font-small',' ('+x+')'));l.className=a?'rmarmed':'';rmSwSet(i,st,dis,rmSw[rmSel.call+' '+n]);box.appendChild(l);box.appendChild(i);if(!x)box.appendChild(document.createElement('span'));}
function rmRenderTiles(k,dis){var b=rmEl('rm_info'),i,d=[['status','Refresh status'],['sendpos','Send position now'],['sendtrack','Send track now']],o=['display','led','ledflash','mesh','gateway'],p=rmEl('rm_pin').value;
b.textContent='';for(i=0;i<d.length;i++)rmTile(b,d[i][1],d[i][0],'','','',dis);
rmBtn(b,'Re-sync counter','rmtile',dis,{'data-act':'sync',title:'Use this if the node keeps answering that the counters are out of step.'});rmTile(b,'Restart','reboot','','reboot','rmwarn rmtile-end',dis);
b=rmEl('rm_sw');b.textContent='';
for(i=0;i<o.length;i++){if((o[i]=='led'||o[i]=='ledflash')&&!k.led)continue;rmSwRow(b,o[i],o[i]=='ledflash'?(k.sw[5]<0?-1:k.sw[5]==1&&k.fl?1:0):k.sw[rmTog[o[i]]],dis);}
rmChgRow(b,k,dis);
function rmChgRow(box,k,dis){var l=document.createElement('label'),s=document.createElement('select'),o,i,v=rmChgModes.indexOf(k.chg)>=0?k.chg:k.chg=='unsupported'?'':rmSw[rmSel.call+' chgled']||'';
s.id='rm_chgled';s.setAttribute('data-sel','chgled');l.htmlFor=s.id;l.textContent='Charge LED';o=document.createElement('option');o.value='';o.textContent='unknown';s.appendChild(o);
for(i=0;i<3;i++){o=document.createElement('option');o.value=rmChgModes[i];o.textContent=rmChgModes[i];s.appendChild(o);}
s.value=v;s.disabled=!!dis;box.appendChild(l);box.appendChild(s);box.appendChild(rmNew('span','font-small',k.chg=='unsupported'?'not supported by this node':''));}
rmTxt('rm_swnote',k.at?'State from the last answer of '+rmSel.call+', '+rmAgo(Math.round((rmNow()-k.at)/1000))+' ago.':'State unknown until the node answers a status request. Press Refresh status.','font-small');
rmSwSet(rmEl('rm_pinsw'),-1,dis,rmSw[rmSel.call+' '+p],1);}
function rmChip(box,call,hw,age,sv,stale){var b=rmBtn(box,'','rmchip'+(sv?' rmsaved':'')+(stale?' rmstale':'')+(rmSel.call==call?' rmsel':''),false,{'data-act':'pick','data-call':call}),s=document.createElement('b'),i,L=[hw,age];
s.textContent=call;b.appendChild(s);
for(i=0;i<2;i++){s=document.createElement('span');s.className='font-small';s.textContent=L[i];b.appendChild(s);}
var N='http://www.w3.org/2000/svg',v=document.createElementNS(N,'svg'),u=document.createElementNS(N,'use');
s=document.createElement('span');s.className='font-small';v.setAttribute('class','rmlk');u.setAttribute('href',sv?'#rmlk-o':'#rmlk-c');v.appendChild(u);s.appendChild(v);s.appendChild(document.createTextNode(' '+(sv?'key saved':'no key')));b.appendChild(s);}
function rmRenderNode(){var h=rmEl('rm_nodes'),i,x,k,c=rmSel.call,t=rmEl('rm_callchk'),n=0,sv=rmSel.slot>=0,hd=false,seen=Object.create(null),e=rmEl('rm_savednote');
h.textContent='';
for(i=0;i<rmHeard.length;i++){x=rmHeard[i];k=String(x.call);seen[k]=1;if(k==c)hd=true;rmChip(h,k,String(x.hw||''),rmAgo(x.age_s|0)+' ago'+(x.rssi?', '+x.rssi+' dBm':''),rmSlotOf(k)>=0,false);}
for(i=0;i<rmSaved.length;i++){x=rmSaved[i];if(x&&x.used){n++;k=String(x.call);if(!seen[k]){seen[k]=1;rmChip(h,k,'','not heard',true,true);}}}
if(!h.firstChild)h.textContent='No node heard directly yet.';
rmEl('rm_forgetall').style.display=n?'':'none';
t.textContent='';
if(c&&!rmValidCall(c)){t.textContent='Not a call sign. Use letters or digits, a dash and an SSID, for example DK5EN-12.';t.className='font-small rmbad';}
rmEl('rm_pwrow').style.display=sv?'none':'';
e.className='font-small';
e.textContent=sv?'Key saved for '+c+'.'+(hd?'':' Not heard in the last 3 hours, the command may not arrive.')+' ':'Type the password of '+(c||'the node you want to manage')+', or press Remember to save it.';
if(sv)rmBtn(e,'Forget','',false,{'data-act':'forget'});
rmEl('rm_forgetall').textContent=rmArmId=='forgetall'?rmReally:'Forget all';}
function rmRenderLock(){var n=rmLockLeft(),t=rmNote.t,c=rmNote.c;
if(!t){c='rmmsg rmlock';if(rmBusy)t='Sending ...';else if(!rmAuth&&n>0)t=(rmSrvMsg?rmSrvMsg+' ':'')+'Next command possible in '+n+' s.';}
rmTxt('rm_msg',t,c);}
function rmRenderMsgs(dis){var j=rmStat,b=rmEl('rm_msgs'),a,i,e,k,tr,v,x,c,cf,n,seen={},q,d=rmEl('rm_chain'),u=0,s,z;if(!j)return;d.textContent='';
for(i=0;i<j.targets.length;i++){q=j.targets[i];if(rmForce&&q.dst==rmForce.d&&!q.canForce){rmForce=null;rmForceBtn();}if(q.chainMsg)d.textContent=q.dst+': '+q.chainMsg;else if(q.pending)d.textContent=q.dst+': checking the connection first, your command follows.';}
a=j.sent.slice(0).sort(function(x,y){return x.ago-y.ago;});
for(i=0;i<a.length;i++){e=a[i];k=e.dst+'#'+e.ctr+'#'+e.cmd;if(seen[k])continue;if(u>4)break;seen[k]=1;tr=rmRows[k];
if(!tr){tr=rmRows[k]=document.createElement('tr');for(n=0;n<4;n++)tr.appendChild(document.createElement('td'));}
c=tr.children;n=e.cmd.split(' ');v=e.st=='queued'||e.st=='waiting'?'sent':e.st=='noanswer'?'no answer':e.ver&&(e.st=='ok'||e.st=='err')?'verified':'unverified';
x=e.st=='ok'&&v=='verified'?rmOkText(e):(e.msg||'');
q=/^err range (-?\d+)\.\.(-?\d+)$/.exec(e.reply||'');if(q&&e.st=='err')x='Refused: '+e.dst+' accepts '+q[1]+'..'+q[2]+' dBm.';tr.title=e.msg||'';if(e.st=='err'&&/^(pins|gpio)\b/.test(e.cmd)){q={master:rmGOff,refused:'The node refused this pin.',unsupported:'This node does not support that.'}[(e.reply||'').substring(4)];if(q)x=q;}if(e.st=='err'&&/^upd chan /.test(e.cmd||'')&&e.reply=='err blocked')x='This node has no channel switch.';
c[0].textContent=rmAgo(e.ago)+' ago';c[1].textContent=e.dst+' '+rmLabel(e.cmd);c[2].textContent=v;c[2].className=v=='verified'?(e.st=='ok'?'rmok':'rmbad'):v=='sent'?'':'rmbad';c[3].textContent='';z=document.createElement('div');z.textContent=v=='sent'?'':x;c[3].appendChild(z);
if(b.children[u]!==tr)b.insertBefore(tr,b.children[u]||null);u++;}
for(k in rmRows)if(!seen[k]){if(rmRows[k].parentNode)rmRows[k].parentNode.removeChild(rmRows[k]);delete rmRows[k];}}
var rmIn={},rmPK=['lat','lon','alt','src'],rmDefs=[{c:'radio',card:'radio',l:'Radio',f:[['f','Frequency',' MHz'],['sf','Spreading factor',''],['cr','Coding rate','',function(v){return '4/'+v;}],['bw','Bandwidth',' kHz'],['p','TX power','',rmTxp]]},
{c:'sens',card:'radio',l:'Sensors',f:[['t','Temperature',' C'],['h','Humidity',' %'],['p','Pressure',' hPa'],['t2','Second temperature',' C']]},
{c:'name',card:'ident',l:'Name',f:[],w:[['v','Name','name',rmLen.name,'n']]},{c:'atxt',card:'ident',l:'APRS text',f:[],w:[['v','APRS text','atxt',rmLen.atxt,'a']]},
{c:'pos',card:'pos',l:'Position',p:rmPK,f:[['src','Source','',function(v){var m={gps:'from GPS',nofix:'GPS on, no fix',set:'set by hand'};return Object.prototype.hasOwnProperty.call(m,v)?m[v]:v;}]],
w:[['lat','Latitude (+/-)','lat',11],['lon','Longitude (+/-)','lon',11],['alt','Altitude (meter)','alt',5]]},
{c:'txq',card:'queues',l:'TX queue',k:1,v:rmTxqView,t:['tx','Sent','rt','Retransmitted','dr','Dropped']},
{c:'mbox',card:'queues',l:'Mailbox',k:1,u:'This node has no mailbox.',f:[['m','Mode',''],['u','Slots used',''],['b','Bytes',''],['a','Actions this hour','']],t:['st','Stored','dl','Delivered','ak','Acknowledged','dr','Dropped','bl','Blocked','nt','Notified']},
{c:'maxhop',card:'queues',l:'Max hop',k:1,f:[['t','Text messages',''],['p','Position beacons','']]},
{c:'net',card:'radio',l:'Network',k:1,x:1,v:rmNetView,u:'This node has no network settings.'},{c:'upd',card:'radio',l:'Software update',k:1,x:1,a:1,v:rmUpdView,u:'This node cannot update itself.'},
{c:'pins',card:'pins',l:'Pin assignments',a:1,v:rmPinsView,n:'Pin 0 means board default (wind: off). Button pin: set on the node (serial console or web Pins page).'},
{c:'gpio',card:'pins',l:'Free GPIO',a:1,v:rmGpioView,u:'This node has no GPIO control.'},
{c:'blockcall',card:'radio',l:'Blocked calls',f:[],n:'The whole list is replaced. Over remote management it can hold '+rmLen.args+' characters (3 or 4 calls); type none to clear it.',w:[['v','Blocked calls','blockcall',rmLen.args,'b']]}];
function rmClrIn(c){var k;for(k in rmIn)if(k.indexOf('rm_f_'+c+'_')==0)delete rmIn[k];}
function rmChk(k,v){var m;if(v==='')return '';
if(k=='lat'||k=='lon'){m=k=='lat'?90:180;return(/^-?\d{1,3}(\.\d{1,6})?$/.test(v)&&Math.abs(+v)<=m)?false:(k=='lat'?'Latitude':'Longitude')+' must be -'+m+' to '+m+', decimal point, at most 6 decimals.';}
if(k=='alt')return(/^\d{1,5}$/.test(v)&&+v<=40000)?false:'Altitude must be a whole number from 0 to 40000.';
if(k=='blockcall')return v.length>rmLen.args?'Too long.':/^(none|[A-Za-z0-9,-]+)$/i.test(v)?false:'Use calls separated by commas, or the word none.';
return v.length>rmLen[k=='name'?'name':'atxt']?'Too long.':false;}
function rmWUpd(d){var c=rmSel.call,g=!rmValidCall(c)||rmLocked(),j,w,v,r,x,s,n,t,a=[],y;
for(j=0;j<d.w.length;j++)a.push(rmEl('rm_f_'+d.c+'_'+d.w[j][0]).getAttribute('data-r'));
for(j=0;j<d.w.length;j++){w=d.w[j];n='rm_f_'+d.c+'_'+w[0];x=rmEl(n);v=x.value;r=rmChk(w[2],v);t=rmEl(n+'_cnt');if(t)t.textContent=v.length+'/'+w[3];
y=a.slice();y[j]=d.c=='blockcall'?(v.toLowerCase()=='none'?'none':v.toUpperCase()):v;s=rmEl(n+'_set');s.disabled=g||r!==false||(!!d.p&&(+v==a[j]||a.indexOf('')>=0));s.setAttribute('data-args',y.join(' '));s.setAttribute('data-cf',d.c+' '+y.join(' '));rmTxt(n+'_hint',r||'','font-small rmbad');}}
function rmWire(x,d){x.addEventListener('input',function(){rmIn[x.id]=x.value;rmWUpd(d);});}
function rmKv(r,f){var o={},t,i,a;if(typeof r!='string'||r.indexOf('ok ')!==0)return null;t=r.substring(3).split(' ');
for(i=0;i<t.length;i++){if(!f&&/^[na]=/.test(t[i])){o[t[i].charAt(0)]=t.slice(i).join(' ').substring(2);break;}a=t[i].indexOf('=');if(a>0)o[t[i].substring(0,a)]=t[i].substring(a+1);}return o;}
function rmPosKv(p,r){var o={},t,i;if(typeof r!='string'||r.indexOf('ok ')!==0)return null;t=r.substring(3).split(' ');if(t.length!=p.length)return null;for(i=0;i<p.length;i++)o[p[i]]=t[i];return o;}
function rmOkNew(e,c,n){var o=rmKv(e.reply),q=rmPosKv(rmPK,e.reply),t=c.slice(1).join(' '),w=n=='name'?'name':'APRS text';
if(n=='radio'&&o&&o.f)return 'Radio: '+o.f+' MHz, SF '+o.sf+', '+o.bw+' kHz, TX power '+rmTxp(o.p)+'.';
if(n=='sens')return 'Sensors read.';
if(n=='net')return o&&o.mtu?'Network read.':'';if(n=='upd')return rmUpdOk(c,e.reply);
if(n=='name'||n=='atxt'){o=o&&o[n=='name'?'n':'a'];if(t)return 'Done. The '+w+' is now '+(o||t)+'.';return o?'The '+w+' is '+(o=='-'?'empty':o)+'.':'';}
if(n=='blockcall'){o=o&&o.b;if(t)return 'Done. Blocked calls are now '+(o=='-'?'none':(o||t))+'.';return o?'Blocked calls: '+(o=='-'?'none':o)+'.':'';}
if(n=='pos'){if(t)return 'Done. Position set to '+c[1]+', '+c[2]+', '+c[3]+' m.';return q?'Position '+q.lat+', '+q.lon+', '+q.alt+' m, '+rmDefs[4].f[0][3](q.src)+'.':'';}return '';}
function rmKvRows(p,r){var y=p.appendChild(rmNew('div','rmkv')),i;for(i=0;i<r.length;i++){y.appendChild(rmNew('span','font-small',r[i][0]));y.appendChild(rmNew('span','',r[i][1],r[i][2]));}}
function rmNetView(p,o){var w={auto:'automatic',inet:'Internet',hamnet:'HAMNET'},e=o.eff;rmKvRows(p,[['MTU',o.mtu?o.mtu+' bytes':'-','rm_v_net_mtu'],['Server',o.srv||'-','rm_v_net_srv'],['Network',Object.prototype.hasOwnProperty.call(w,o.net)?w[o.net]:o.net||'-','rm_v_net_net'],['Effective uplink',e=='-'?'no IP uplink':e?(w[e]||e):'-','rm_v_net_eff']]);}
function rmUpdOk(c,r){var m;if(c[1]=='chan'){m=/^ok chan (prod|dev)$/.exec(r);return m?'Channel set to '+(m[1]=='dev'?'DEV':'MAIN')+'. Run Check.':'';}
if(c[1]=='check')return r=='ok checking'?'The node is checking for a new version. Read again in about 30 seconds.':'';
if(c[1]=='go'){m=/^ok (installing|applying) (\S+)$/.exec(r);return m?'The node is '+m[1]+' '+m[2]+' and will restart. Read again in a few minutes.':'';}
return /^ok m=/.test(r)?'Update status read.':'';}
function rmUpdView(p,o,v){var c=rmSel.call,s=v&&rmLast(c,'upd chan dev',1),q=v&&rmLast(c,'upd chan prod',1),u=s&&q?(s.ago<q.ago?s:q):s||q,w=u&&u.ago<v.ago&&/^ok chan (prod|dev)$/.exec(u.reply);
if(w)o=Object.assign({},o,{c:w[1],a:'-',n:'0'});var z=function(k){return o[k]===undefined||o[k]=='-'?'':o[k];},tag=z('st')||(o.n=='1'?z('a'):''),g=rmUpdGone[c],t=v&&rmStat?rmStat.t-v.ago*1000-1000:0,dis=!rmValidCall(c)||rmLocked();
var i,k,x,m={off:'off',notify:'notify only',auto:'automatic'},ch={prod:'production',dev:'development'},y=function(k){return o[k]=='1'?'yes':'no';},r,ok;
ok=!!v&&!dis&&/^[A-Za-z0-9._-]{1,23}$/.test(tag)&&o.b=='0'&&o.g=='0'&&t>=rmUpdT0&&!(g&&t<=g);
if(v)rmKvRows(p,[['Mode',m[o.m]||o.m||'-','rm_v_upd_m'],['Channel',ch[o.c]||o.c||'-','rm_v_upd_c'],['Available version',z('a')||'none','rm_v_upd_a'],['Newer than installed',y('n'),'rm_v_upd_n'],['Staged version',z('st')||'none','rm_v_upd_st'],
['Busy',y('b'),'rm_v_upd_b'],['Go pending',y('g'),'rm_v_upd_g'],['Last check',z('ck')?z('ck')+' min ago':'not yet','rm_v_upd_ck'],['Last error',z('e')||'none','rm_v_upd_e']]);
r=p.appendChild(rmNew('div','rmrow rmsplit'));rmBtn(r,'Check','',dis,{'data-cmd':'upd','data-args':'check','id':'rm_upd_check'});rmTile(r,'Update now','upd','go '+tag,'upd go '+tag,'rmwarn rmtile-end',!ok).id='rm_upd_go';
if(rmUpdChan){r=p.appendChild(rmNew('div','rmrow','','rm_upd_chan'));r.appendChild(rmNew('span','font-small','Update channel'));x=!v||dis||o.b!='0'||o.g!='0';
for(i=0;i<2;i++){k=['prod','dev'][i];s=o.c==k;q=rmTile(r,['MAIN (icssw-org)','DEV (DK5EN)'][i],'upd','chan '+k,'upd chan '+k,s?'rmsel':'',x||s);q.id='rm_upd_chan_'+k;q.setAttribute('aria-pressed',s?'true':'false');}}}
function rmWBuild(p,d,o){var j,w,x,k,r,n,b;if(!d.w)return;r=rmNew('div','grid rmf');p.appendChild(r);
for(j=0;j<d.w.length;j++){w=d.w[j];n='rm_f_'+d.c+'_'+w[0];k=rmNew('label','',w[1]+' :');k.htmlFor=n;r.appendChild(k);x=rmNew('input','','',n);x.type='text';x.autocomplete='off';if(d.w.length>1)x.placeholder=w[1];
k=o&&o[w[4]||w[0]];k=k&&k!='-'?k:'';x.setAttribute('data-r',k);x.value=rmIn[n]!==undefined?rmIn[n]:k;r.appendChild(x);rmWire(x,d);
b=rmBtn(r,'','',true,{'data-cmd':d.c,'data-args':'','data-cf':d.c,'aria-label':'Set '+w[1]});b.id=n+'_set';b.appendChild(rmNew('i','btncheckmark'));r.appendChild(rmNew('div','font-small','',n+'_hint'));
if(!d.p)r.appendChild(rmNew('div','font-small rmc','',n+'_cnt'));}rmWUpd(d);}
function rmLast(c,n,x){var a=rmStat?rmStat.sent:[],i,b=null;for(i=0;i<a.length;i++)if(a[i].dst==c&&(x?a[i].cmd==n:a[i].cmd.split(' ')[0]==n)&&(!b||a[i].ago<b.ago)&&a[i].st=='ok'&&a[i].ver)b=a[i];return b;}
var rmMh={on:0,job:'',args:'',fl:0,t:0,pre:'',rows:[],total:0,msg:'',det:null,gen:0,tm:0,ctr:-1};
function rmUns(c,n,v){var a=rmStat?rmStat.sent:[],i;for(i=0;i<a.length;i++)if(a[i].dst==c&&a[i].cmd==n&&a[i].st=='err'&&a[i].reply=='err unsupported'&&(!v||a[i].ago<v.ago))return true;return false;}
function rmMhReset(){clearTimeout(rmMh.tm);rmMh.gen++;rmMh.on=0;rmMh.fl=0;rmMh.job='';rmMh.pre='';rmMh.rows=[];rmMh.total=0;rmMh.msg='';rmMh.det=null;}
function rmNodeChg(n){if(n!=rmSel.call){rmIn={};rmQClr();rmMhReset();rmForce=null;rmForceBtn();}}
function rmMhEnd(m){clearTimeout(rmMh.tm);rmMh.gen++;rmMh.on=0;rmMh.fl=0;rmMh.job='';rmMh.pre='';rmMh.msg=m||'';rmRender();}
function rmMhStop(){rmMhEnd('Stopped.');}
function rmMhGo(job,args){var pr=rmPre();if(rmMh.on)return;if(pr){rmMsg(pr,'rmbad');return;}
if(!rmMh.pre){rmMh.pre=rmSel.slot>=0?'slot='+rmSel.slot+'&call='+rmEnc(rmSel.call):'dst='+rmEnc(rmSel.call)+'&pw='+rmEnc(rmEl('rm_pw').value);rmEl('rm_pw').value='';}
rmMh.on=1;rmMh.job=job;rmMh.args=args;rmMh.msg='';rmMh.det=null;if(job=='list'){rmMh.rows=[];rmMh.total=0;}rmMhStep();rmRender();}
function rmMhStart(){rmMhGo('list','0');}
function rmMhDet(c){c=String(c).trim().toUpperCase();if(rmMh.on)return;if(!rmValidCall(c)){rmMsg('Enter a node call such as DK5EN-12.','rmbad');return;}rmMhGo('det',c);}
function rmMhStep(){var w,e;if(!rmMh.on)return;if(rmMh.fl){e=rmMhFind();if(e)rmMhGot(e);else if(rmNow()-rmMh.t>180000)rmMhEnd('There was no answer from the node.');return;}
w=rmTL[rmSel.call]-rmNow();if(w>0){clearTimeout(rmMh.tm);rmMh.tm=setTimeout(rmMhStep,Math.min(w+30,15000));return;}rmMhSend();}
// Driver match: the sent[] entry must carry the ctr the send answered with. A send answered viaSync gets its
// ctr only later, so rmMh.ctr stays -1 there and rmMhFind falls back to the time-window match (cmd, dst, ago).
function rmMhSend(){var g=rmMh.gen;rmMh.fl=1;rmMh.ctr=-1;rmMh.t=rmNow();rmOut=true;
rmPost('/rmsend',rmMh.pre+'&cmd=mh&args='+rmEnc(rmMh.args)).then(rmJson).then(function(j){if(g!=rmMh.gen||!rmMh.on)return;
if(j&&j.ok){rmMh.ctr=(j.viaSync||typeof j.ctr!='number')?-1:j.ctr;rmPoll();return;}rmMh.fl=0;if(j&&j.err=='busy'&&j.retry>0){rmTL[rmSel.call]=rmNow()+j.retry*1000;rmMhStep();return;}
rmMhEnd(rmErrText(j));}).catch(function(e){if(g==rmMh.gen&&e!==0)rmMhEnd(rmNoAns);});}
function rmMhFind(){var a=rmStat?rmStat.sent:[],i,b=null,e,x=rmNow()-rmMh.t+2000;for(i=0;i<a.length;i++){e=a[i];if(e.dst==rmSel.call&&e.cmd=='mh '+rmMh.args&&(rmMh.ctr<0||e.ctr==rmMh.ctr)&&e.ago*1000<=x&&(!b||e.ago<b.ago)&&(e.st=='ok'||e.st=='err'||e.st=='noanswer'))b=e;}return b;}
function rmMhL(v,l,u,f){return l+': '+((v===undefined||v=='-')?'not present':f?f(v):v+u);}
function rmYn(v){return v=='1'?'yes':'no';}
function rmMhGot(e){var r=e.reply,t,i,o={},k,n,d,x;rmMh.fl=0;
if(e.st=='noanswer'||!e.ver||typeof r!='string')return rmMhEnd('There was no answer from the node.');
if(r.indexOf('err ')==0){k=r.substring(4).split(' ')[0];if(k=='end'&&rmMh.job=='list')return rmMhEnd('End of the list.');return rmMhEnd(k=='unknown'?'That node is not known there.':rmErrText({err:k,msg:e.msg}));}
t=r.split(' ');if(t[0]!='ok')return rmMhEnd('The reply could not be read.');
if(rmMh.job=='det'){for(i=2;i<t.length;i++){k=t[i].indexOf('=');if(k>0)o[t[i].substring(0,k)]=t[i].substring(k+1);}
d=t[1]=='d'?[rmMhL(o.g,'Gateway','',rmYn),rmMhL(o.m,'Mesh','',rmYn),rmMhL(o.r,'RSSI',' dBm'),rmMhL(o.s,'SNR',' dB'),rmMhL((o.la=='-'||o.lo=='-')?'-':o.la+', '+o.lo,'Position',''),rmMhL(o.di,'Distance',' km'),rmMhL(o.a,'Altitude',' m'),rmMhL(o.n,'Its neighbours',''),rmMhL(o.x,'Only it hears',''),rmMhL(o.h,'It hears',''),rmMhL(o.t,'Heard',' min ago')]:
[rmMhL(o.h,'Hops',''),rmMhL(o.k,'Routes',''),rmMhL(o.g,'Via gateway','',rmYn),'Relay: '+((o.rc===undefined||o.rc=='-')?'not known':o.rc),rmMhL(o.t,'Age',' min'),rmMhL(o.v,'Via','',function(v){return v.split(',').join(', ');})];
rmMh.det={c:rmMh.args,l:d};return rmMhEnd('');}
n=rmMh.rows.length;rmMh.total=parseInt(t[1],10)||0;for(i=3;i+1<t.length&&rmMh.rows.length<128;i+=2)rmMh.rows.push({c:t[i],m:t[i+1]});
if(rmMh.rows.length==n||!/^[0-9]+$/.test(t[2]||'-')||rmMh.rows.length>=128)return rmMhEnd('End of the list.');
rmMh.args=t[2];rmMhStep();rmRender();}
function rmCardHd(b,c,L,dis){var h=b.appendChild(rmNew('div','rmrow rmh')),j,v,y=null;h.appendChild(rmNew('span'));for(j=0;j<L.length;j++){v=rmLast(c,L[j]);if(v&&(!y||v.ago<y.ago))y=v;}
h.appendChild(rmNew('span','font-small',y?'last read '+rmAgo(y.ago)+' ago':'not read yet'));rmBtn(h,'Read','',dis,{'data-act':'qread','data-q':L.join(' ')});}
function rmNew(t,c,x,i){var e=document.createElement(t);if(c)e.className=c;if(x)e.textContent=x;if(i)e.id=i;return e;}
function rmMhCard(b,dis){var h=b.appendChild(rmNew('div','rmrow rmsplit')),s=rmNew('span'),w=b.appendChild(rmNew('div','grid rmg rmw')),x,tb,tr,i,r,k;h.appendChild(rmNew('b','','Heard list'));
h.appendChild(rmNew('span','font-small',(rmMh.total||rmMh.rows.length?'Read '+rmMh.rows.length+' of '+rmMh.total+'. ':'')+rmMh.msg,'rm_mh_prog'));h.appendChild(s);
rmBtn(s,'Read','',dis||rmMh.on,{'data-act':'mhgo'});rmBtn(s,'Stop','',!rmMh.on,{'data-act':'mhstop'});
w.appendChild(rmNew('span','','Other node'));x=w.appendChild(rmNew('input','','','rm_f_mh_other'));x.type='text';x.maxLength=9;x.value=rmIn[x.id]||'';x.addEventListener('input',function(){rmIn[this.id]=this.value;});rmBtn(w,'Look up','',dis||rmMh.on,{'data-act':'mhother'});
tr=b.appendChild(rmNew('table','rmtab font-small','','rm_mh')).appendChild(rmNew('thead')).appendChild(rmNew('tr'));for(i=0;i<3;i++)tr.appendChild(rmNew('th','',['node','heard',''][i]));
tb=b.lastChild.appendChild(rmNew('tbody','','','rm_mh_tab'));
for(i=0;i<rmMh.rows.length;i++){r=rmMh.rows[i];tr=tb.appendChild(rmNew('tr'));rmCell(tr,r.c);rmCell(tr,r.m+' min ago');rmBtn(tr.appendChild(rmNew('td')),'Details','',dis||rmMh.on,{'data-act':'mhdet','data-call':r.c});}
x=b.appendChild(rmNew('div',rmMh.det?'rmdet':'','','rm_mh_det'));if(rmMh.det){x.appendChild(rmNew('b','',rmMh.det.c));k=x.appendChild(rmNew('div','rmkv'));
for(i=0;i<rmMh.det.l.length;i++){r=rmMh.det.l[i];tr=r.indexOf(': ');k.appendChild(rmNew('span','font-small',r.slice(0,tr)));k.appendChild(rmNew('span','',r.slice(tr+2)));}}}
function rmTiles(p,c,o,a){var g=rmNew('div','mbx-counters'),i,d;p.appendChild(g);for(i=0;i<a.length;i+=2){d=g.appendChild(rmNew('div','',a[i+1]));d.appendChild(rmNew('b','',o[a[i]]||'-','rm_v_'+c+'_'+a[i]));}}
function rmTxqView(p,o){var m=/^(\d+)\/(\d+)$/.exec(o.q||''),b=m?+m[2]:0,f=b>60?Math.round(m[1]*60/b):m?+m[1]:0,v=['quiet','qrs','qrt'].indexOf(o.bp),x=p.appendChild(rmNew('div','mcq-bar')),s,i;
for(i=0;i<Math.min(b,60);i++)x.appendChild(rmNew('div',i<f?'mcq-cell':'mcq-cell-empty')).style.background=i<f?'#3B7DD8':'';
x=p.appendChild(rmNew('div','font-small',(m?o.q:'-')+' queued, state ','rm_v_txq_q'));s=x.appendChild(rmNew('span',['rmok','','rmbad'][v],v<0?o.bp||'-':['quiet','slow down','hold'][v],'rm_v_txq_bp'));if(v==1)s.style.color='var(--mcsw2)';
x=p.appendChild(rmNew('div','mcq-util-row'));x.appendChild(rmNew('span','mcq-util-label','use '+(isNaN(o.u)?'-':o.u+' %'),'rm_v_txq_u'));
x.appendChild(rmNew('div','mcq-util-track')).appendChild(rmNew('div')).style.cssText='height:100%;width:'+(parseInt(o.u,10)||0)+'%;background:#3B7DD8';}
function rmRenderCards(){var c=rmSel.call,dis=!rmValidCall(c)||rmLocked(),B={},K=['radio','ident','pos','queues','mh','pins'],d,i,j,p,h,v,o,x,y,z,e,f,fa=document.activeElement,fid='';if(!rmEl('rm_card_mh'))return;
if(fa&&fa.tagName=='INPUT'&&/^rm_f_/.test(fa.id)){rmIn[fa.id]=fa.value;fid=fa.id;}
for(i=0;i<K.length;i++){B[K[i]]=rmEl('rm_card_'+K[i]).lastElementChild;B[K[i]].textContent='';}
for(j=0;j<2;j++){rmCardHd(B[['ident','queues'][j]],c,[['name','atxt'],['txq','mbox','maxhop']][j],dis||rmMh.on);if(!j)B.ident.appendChild(rmNew('p','font-small','Stored exactly as typed.'));}
for(i=0;i<rmDefs.length;i++){d=rmDefs[i];p=rmNew('div','','','rm_def_'+d.c);B[d.card].appendChild(p);h=d.card=='ident'?0:rmNew('div',d.card=='queues'?'rmrow rmsplit':'rmrow rmh');if(h){p.appendChild(h);h.appendChild(rmNew('b','',d.p?'':d.l));}
v=rmLast(c,d.c,d.x);o=v?(d.p?rmPosKv(d.p,v.reply):d.k?rmKv(v.reply,1):rmKv(v.reply)):null;
if(h&&d.card=='queues'){h.appendChild(rmNew('span','font-small',v?'last read '+rmAgo(v.ago)+' ago':'not read yet'));rmBtn(h,'Read','',dis||rmMh.on,{'data-act':'qread','data-q':d.c});}
if(h&&d.card!='queues'){h.appendChild(d.p&&o?rmNew('span','font-small',d.f[0][1]+': '+d.f[0][3](o[d.f[0][0]]),'rm_v_'+d.c+'_'+d.f[0][0]):rmNew('span','font-small',v?'last read '+rmAgo(v.ago)+' ago':'not read yet'));
rmBtn(h,'Read','',dis,{'data-cmd':d.c,'data-args':''});}
if(d.v&&(o||d.a))d.v(p,o||{},v);if(o&&d.f&&!d.p){y=p.appendChild(rmNew('div','rmkv'));
for(j=0;j<d.f.length;j++){f=d.f[j];e=o[f[0]];z=(e===undefined||e=='-')?'not present':(f[3]?f[3](e):e)+f[2];y.appendChild(rmNew('span','font-small',f[1]));y.appendChild(rmNew('span','',z,'rm_v_'+d.c+'_'+f[0]));}}if(o&&d.t)rmTiles(p,d.c,o,d.t);
rmWBuild(p,d,o);if(d.n)p.appendChild(rmNew('div','font-small',d.n));
if(d.u&&rmUns(c,d.c,v))p.appendChild(rmNew('div','',d.u));}
y=B.pos.appendChild(rmNew('div','grid rmf'));z=rmKn();rmSwRow(y,'gps',z.sw[0],dis,'enable GPS');rmSwRow(y,'track',z.sw[1],dis,'enable display of SmartBeaconing');rmMhCard(B.mh,dis);if(fid&&rmEl(fid))rmEl(fid).focus();}
var rmGOff='GPIO control is off on the node (serial console: --gpiocontrol on).';
function rmGBit(m,n){var h=typeof m=='string'?m.charAt(m.length-1-(n>>2)):'';return h!==''&&(parseInt(h,16)>>(n&3)&1)==1;}
function rmCtl(p,c,sel,dis){var r=p.appendChild(rmNew('div','rmrow')),a=rmNew('select','','','rm_f_'+c+'_a'),n=rmNew('input','','','rm_f_'+c+'_n'),b,h=rmNew('div','font-small rmbad','','rm_f_'+c+'_hint'),i,o,l,u=function(){var v=n.value,bad=v!==''&&(!(/^(0|[1-9]\d?)$/.test(v)&&+v<=48)||(c=='pins'&&a.value=='an'&&v=='0')),g=c=='pins'?a.value+' '+v:v+' '+a.value;
b.disabled=!!dis||v===''||bad;b.setAttribute('data-args',g);if(c=='pins'){b.setAttribute('data-cf','pins '+g);b.textContent=rmArmId=='pins '+g?'Confirm: set '+rmPF[a.value]+' to '+(+v?'GPIO '+v:(a.value=='wind'?'off':'board default'))+'?':'Set';}h.textContent=bad?(c=='pins'&&a.value=='an'&&v=='0'?'Analog needs a GPIO from 1 to 48 (0 is not accepted).':'GPIO must be a whole number from 0 to 48.'):'';};
l=r.appendChild(rmNew('label','',c=='pins'?'Function':'Action'));l.htmlFor=a.id;r.appendChild(a);
for(i=0;i<sel.length;i++){o=a.appendChild(rmNew('option','',sel[i][1]));o.value=sel[i][0];}
l=r.appendChild(rmNew('label','','GPIO'));l.htmlFor=n.id;r.appendChild(n);n.type='text';n.maxLength=2;n.size=3;n.autocomplete='off';
a.disabled=n.disabled=!!dis;a.value=rmIn[a.id]||sel[0][0];n.value=rmIn[n.id]||'';
b=rmBtn(r,c=='pins'?'Set':'Send','',true,{'data-cmd':c,'data-args':''});b.id='rm_f_'+c+'_go';p.appendChild(h);
a.addEventListener('change',function(){rmIn[a.id]=a.value;u();});n.addEventListener('input',function(){rmIn[n.id]=n.value;u();});u();}
function rmPinsView(p,o){var k,r=[],dis=!rmValidCall(rmSel.call)||rmLocked();
if(o.ow!==undefined)for(k in rmPF)r.push([rmPF[k],o[k]===undefined?'-':o[k]=='0'?'none':'GPIO '+o[k],'rm_v_pins_'+k]);
rmKvRows(p,r);rmCtl(p,'pins',[['ow','1-Wire'],['an','Analog'],['wind','Wind']],dis);}
function rmGpioView(p,o,v){var c=rmSel.call,a=rmStat?rmStat.sent:[],i,e,r=[],n,off=o.m=='0',dis=!rmValidCall(c)||rmLocked();
if(!off)for(i=0;i<a.length;i++){e=a[i];if(e.dst==c&&/^gpio /.test(e.cmd)&&e.st=='err'&&e.reply=='err master'&&(!v||e.ago<v.ago))off=true;}
if(o.m!==undefined)r.push(['GPIO control',o.m=='1'?'on':'off','rm_v_gpio_m']);
if(o.o!==undefined){for(n=0;n<=48;n++)if(rmGBit(o.o,n)||rmGBit(o.i,n))r.push(['GPIO '+n,(rmGBit(o.o,n)?'out':'in')+', '+(rmGBit(o.l,n)?'high':'low'),'rm_v_gpio_'+n]);if(r.length<2)r.push(['Held pins','none','rm_v_gpio_none']);}
rmKvRows(p,r);if(off)p.appendChild(rmNew('div','font-small rmbad',rmGOff,'rm_gpio_off'));
rmCtl(p,'gpio',[['on','on'],['off','off'],['in','input']],dis||off);}
function rmCell(tr,t){var d=document.createElement('td');d.textContent=String(t);tr.appendChild(d);}
function rmRenderLog(){var j=rmStat,b,i,r,tr;if(!j)return;
b=rmEl('rm_log');b.textContent='';for(i=0;i<j.log.length;i++){r=j.log[i];tr=document.createElement('tr');rmCell(tr,rmAgo(r.ago));rmCell(tr,r.src);rmCell(tr,r.ctr);rmCell(tr,r.cmd);rmCell(tr,r.res);b.appendChild(tr);}
rmTxt('rm_cnt','Executed '+j.ok+', rejected '+j.rej+', counter high-water '+j.hwm+'.');}
function rmRenderOff(){var j=rmStat,f=!!j&&!j.on,e=rmEl('rm_off');e.style.display=f?'':'none';e.textContent=f?'Remote management is off on this node. Switch it on under Settings, Remote Admin.':'';}
function rmRender(){if(!rmShown||!rmEl('rm_page'))return;var k=rmKn(),dis=!rmValidCall(rmSel.call)||rmLocked();
rmRenderNode();rmRenderTiles(k,dis);rmRenderLock();rmRenderMsgs();rmRenderCards();rmRenderLog();rmRenderOff();
}
function rmPre(){var c=rmSel.call,p;if(!rmValidCall(c))return 'Choose a node first.';if(rmSel.slot<0){p=rmPwProblem(rmEl('rm_pw').value);if(p)return 'Type the password of '+c+' first. '+p;}return '';}
function rmSendCmd(cmd,args,dc,sl,fc){var call=dc||rmSel.call,ss=dc?sl:rmSel.slot,body,pr=dc||rmQp?'':rmPre(),full=args?cmd+' '+args:cmd;
if(!fc&&(dc?rmRowLocked(dc):rmLocked()))return;rmForce=null;rmTxt('rm_force','');
if(pr){rmMsg(pr,'rmbad');return;}
body='cmd='+rmEnc(cmd)+'&args='+rmEnc(args);
if(ss>=0)body='slot='+ss+'&'+body;
else{body=(rmQp||'dst='+rmEnc(call)+'&pw='+rmEnc(rmEl('rm_pw').value))+'&'+body;rmEl('rm_pw').value='';}if(!rmQq.length)rmQp='';
if(dc)body+='&call='+rmEnc(dc);if(!dc&&ss>=0)body+='&call='+rmEnc(call);if(fc)body+='&force=1';rmBusy=true;rmMsg('','');rmRender();
rmPost('/rmsend',body).then(rmJson).then(function(j){
if(j&&j.ok){rmLock=rmNow()+10000;rmQw={d:call,c:cmd,t:rmNow()};rmOut=true;rmMsg('','');rmPoll();}
else{pr=j&&j.err?String(j.err):'send';rmQClr();rmMsg(pr=='range'&&j.range?'Refused: '+call+' accepts '+j.range+' dBm.':rmErrText(j),'rmbad');if(pr=='busy')rmLock=rmNow()+10000;
if(pr=='limit'&&j.canForce&&!fc&&ss>=0&&call==rmSel.call){rmForce={c:cmd,a:args,d:call,s:ss};rmForceBtn();}}
}).catch(function(e){rmQClr();rmCatch(e);}).then(function(){rmBusy=false;rmRender();});}
function rmForceBtn(){var b=rmEl('rm_force');if(!b)return;b.textContent='';if(rmForce&&rmForce.d==rmSel.call)rmBtn(b,'Try once more','',false,{'data-act':'force'});}
function rmTap(b){var cmd=b.getAttribute('data-cmd'),args=b.getAttribute('data-args')||'',cf=b.getAttribute('data-cf')||'',pr=rmPre();if(!args)rmClrIn(cmd);
if(rmLocked())return;
if(pr){rmMsg(pr,'rmbad');return;}
if(cf&&!rmConfirm(cf,rmCf(rmLabel((cmd+' '+args).trim()),rmSel.call)))return;
if(cmd=='upd'&&args.indexOf('go ')==0)rmUpdGone[rmSel.call]=rmNow();
rmSendCmd(cmd,args);}
function rmPick(c){rmNodeChg(c);rmDisarmQuiet();rmSel.call=c;rmSel.slot=rmSlotOf(c);rmEl('rm_call').value=c;rmRender();}
function rmDisarmQuiet(){clearTimeout(rmT.arm);rmT.arm=0;rmArmId='';}
function rmCallInput(){var c=rmEl('rm_call');c.value=c.value.toUpperCase();rmDisarmQuiet();rmNodeChg(c.value.trim());rmSel.call=c.value.trim();rmSel.slot=rmSlotOf(rmSel.call);rmRender();}
function rmLoadNodes(){fetch('/rmnodes').then(rmJson).then(function(j){rmSaved=(j&&j.nodes)||[];rmSel.slot=rmSlotOf(rmSel.call);rmRender();}).catch(function(e){rmCatch(e,'rm_net');});}
function rmLoadHeard(){fetch('/rmheard').then(rmJson).then(function(j){rmHeard=(j&&j.heard)||[];rmRender();}).catch(function(e){rmCatch(e,'rm_net');});}
function rmNodesPost(body,okText){rmPost('/rmnodes',body).then(rmJson).then(function(j){if(j&&j.ok){rmMsg(okText,'rmok');rmLoadNodes();}else rmMsg(rmErrText(j),'rmbad');}).catch(rmCatch);}
function rmRemember(){var c=rmSel.call,i=rmEl('rm_pw'),pw=i.value,pr=rmPwProblem(pw),s=-1,n;
if(!rmValidCall(c)){rmMsg('Enter a valid call sign first.','rmbad');return;}
if(pr){rmMsg(pr,'rmbad');return;}
for(n=0;n<3;n++)if(!rmSaved[n]||!rmSaved[n].used){s=n;break;}
if(s<0){rmMsg('All 3 places on this node are used. Forget one first.','rmbad');return;}
i.value='';rmNodesPost('act=save&slot='+s+'&call='+rmEnc(c)+'&pw='+rmEnc(pw),c+' is saved.');pw='';}
function rmForget(){if(rmSel.slot<0)return;rmNodesPost('act=del&slot='+rmSel.slot,rmSel.call+' is forgotten.');}
function rmForgetAll(){if(!rmConfirm('forgetall','Tap Forget all again within 4 seconds to remove every saved node.'))return;rmNodesPost('act=forget','All saved nodes are forgotten.');}
function rmQRead(b){var pr=rmPre(),a=b.getAttribute('data-q').split(' '),i;if(rmLocked()||rmMh.on)return;if(pr){rmMsg(pr,'rmbad');return;}for(i=0;i<a.length;i++)rmClrIn(a[i]);rmQp=rmSel.slot<0?'dst='+rmEnc(rmSel.call)+'&pw='+rmEnc(rmEl('rm_pw').value):'';rmQq=a.slice(1);rmSendCmd(a[0],'');}
function rmSwChg(i){var n=i.getAttribute('data-sw'),v=i.checked?'on':'off',cf=v=='off'?(i.getAttribute('data-cf')||''):'',pr=rmPre(),pin=n=='pin',c=pin?'setout':n=='ledflash'?'led':n,a=pin?rmEl('rm_pin').value+' '+v:n=='ledflash'?(i.checked?'flash':'off'):v;
if(rmLocked()||pr){if(pr&&!rmLocked())rmMsg(pr,'rmbad');rmRender();return;}
if(cf&&!rmConfirm(cf,rmCf(rmLabel(c+' '+a),rmSel.call)))return;
rmSw[rmSel.call+' '+(pin?rmEl('rm_pin').value:n)]=v;rmSendCmd(c,a);}
function rmSelChg(s){var v=s.value,pr=rmPre();if(!v||rmLocked()||pr){if(pr&&!rmLocked())rmMsg(pr,'rmbad');rmRender();return;}rmSw[rmSel.call+' chgled']=v;rmSendCmd('chgled',v);}
function rmChange(e){var t=e.target;if(t.id=='rm_pin')rmRender();else if(t.getAttribute&&t.getAttribute('data-sel'))rmSelChg(t);else if(t.getAttribute&&t.getAttribute('data-sw'))rmSwChg(t);}
function rmClick(e){var b=e.target;while(b&&b.tagName!='BUTTON'){if(b.id=='rm_page')return;b=b.parentNode;}
if(!b||b.disabled)return;var a=b.getAttribute('data-act');
if(b.hasAttribute('data-cmd'))rmTap(b);
else if(a=='pick')rmPick(b.getAttribute('data-call'));else if(a=='mhgo')rmMhStart();
else if(a=='mhstop')rmMhStop();else if(a=='mhdet')rmMhDet(b.getAttribute('data-call'));else if(a=='mhother')rmMhDet(rmEl('rm_f_mh_other').value);
else if(a=='sync')rmSendCmd('sync','');
else if(a=='force'&&rmForce&&rmForce.d==rmSel.call)rmSendCmd(rmForce.c,rmForce.a,rmForce.d,rmForce.s,true);
else if(a=='qread')rmQRead(b);
else if(a=='remember')rmRemember();
else if(a=='forget')rmForget();
else if(a=='forgetall')rmForgetAll();
else if(a=='heard')rmLoadHeard();}
function rmGot(j){var i,t,c=rmSel.call;j.sent=j.sent||[];j.log=j.log||[];j.targets=j.targets||[];j.t=rmNow();rmStat=j;rmApplySent(j.sent);
rmOut=false;rmSrvLock=0;rmSrvMsg='';rmTL=Object.create(null);
for(i=0;i<j.sent.length;i++)if(j.sent[i].st=='queued'||j.sent[i].st=='waiting')rmOut=true;
for(i=0;i<j.targets.length;i++){t=j.targets[i];if(t.pending)rmOut=true;rmTL[t.dst]=t.locked?1e15:t.retry>0?rmNow()+t.retry*1000:0;if(t.dst==c){if(t.retry>0)rmSrvLock=rmNow()+t.retry*1000;if(t.locked)rmSrvMsg='Two tries to '+c+' are still unanswered.';}}
rmMhStep();rmRender();}
function rmNext(){clearTimeout(rmT.poll);rmT.poll=0;if(rmShown&&!rmAuth)rmT.poll=setTimeout(rmPoll,rmOut?3000:10000);}
function rmPoll(){if(!rmShown)return;clearTimeout(rmT.poll);rmT.poll=0;
fetch('/rmstatus').then(rmJson).then(function(j){if(!rmShown)return;rmAuth=false;rmTxt('rm_auth','');rmEl('rm_auth').style.display='none';rmTxt('rm_net','');rmGot(j);rmNext();})
.catch(function(e){if(e===0||!rmShown)return;rmTxt('rm_net','No answer from the node, trying again.','rmbad');rmNext();});}
function rmTick(){if((typeof cpage!='undefined'&&cpage!='remote')||!rmEl('rm_page')){rmPageLeave();return;}
var l=rmLocked();if(!l&&rmQq.length&&rmQDone()){if(!rmQp&&rmPre())rmQClr();else rmSendCmd(rmQq.shift(),'');}if(l||rmWasLocked)rmRender();rmWasLocked=l;}
function rmPageLeave(){clearInterval(rmT.tick);clearTimeout(rmT.poll);clearTimeout(rmT.arm);rmT.tick=0;rmT.poll=0;rmT.arm=0;rmShown=false;rmArmId='';rmMhReset();rmQClr();}
function rmPageInit(){var i,p,o;rmPageLeave();if(!rmEl('rm_page'))return;rmShown=true;rmUpdT0=rmNow();rmRows=Object.create(null);rmAuth=false;rmBusy=false;rmArmId='';
p=rmEl('rm_pin');for(i=0;i<16;i++){o=document.createElement('option');o.value=(i<8?'a':'b')+(i%8);o.textContent=o.value;p.appendChild(o);}
rmEl('rm_page').onclick=rmClick;rmEl('rm_page').onchange=rmChange;rmEl('rm_call').oninput=rmCallInput;rmEl('rm_call').value=rmSel.call;
rmT.tick=setInterval(rmTick,1000);rmRender();rmPoll();rmLoadNodes();rmLoadHeard();}
