/**
 * RM web GUI W2b: the Remote page (GET /?page=remote) -- HTML skeleton and the scaffold JS that drives it.
 *
 * Rules the code keeps:
 *  - Sub-pages are injected with innerHTML, scripts in them never run: sub_page_remote() prints HTML and an
 *    inline <style> only; rmScaffoldJs() prints the JS once, from deliver_scaffold().
 *  - Every web_client call is a print/println of ONE string literal of at most 512 bytes (RAK4631: one write is
 *    capped at 2048 B and the rest dropped; nRF52 printf over-reads above 255 B). The only printf is the rmLen
 *    line (two %u from rm_commands.h, far below 64 B so Print::printf stays on the stack).
 *  - The page owns no rule text of its own: error sentences come from the server (msg next to err), the
 *    text limits from rmLen. The JS only pre-checks lengths and the call-sign shape.
 *  - The JS writes all server data with textContent. Passwords go in POST bodies only, the input is cleared
 *    when the request is built, nothing is stored in the browser.
 *  - A host harness extracts the literals below and runs them in jsdom: keep the print/println
 *    string-literal form (adjacent literals are concatenated, only \\ \" \n escapes are used); the printf
 *    arguments must be (unsigned)RM_NAME_MAX / RM_ATXT_MAX, the harness resolves them from the headers.
 *  - Ids are unique per page: read-only values use rm_v_<card>_<key>, inputs and their Set/cnt/hint use
 *    rm_f_<card>_<key>. rmWUpd, rmWire, rmClrIn and rmIn look up rm_f_ only; a shared id makes rmEl() return
 *    the read-only <div> instead of the input.
 *  - The JS carries no comments (they would cost bytes on every page load) and no preprocessor lines.
 */
#include <Arduino.h>
#include "web_rm_page.h"
#include "web_rm_util.h"
#include "web_functions.h"
#include "../rm_commands.h" // RM_NAME_MAX, RM_ATXT_MAX
#include "../remote_cmd.h"  // RM_MAX_ARGS

/** Remote page body: the sub-header, then the static skeleton. The elements are filled by the scaffold JS.
 *  Every card uses the shared collapsablecard markup (CSS and togglecard() live in web_functions.cpp): teaser span,
 *  toggle button and exactly one body div. The class names are short and unique to this page, so no id prefix. */
void sub_page_remote()
{
    _create_meshcom_subheader("Remote");
    web_client.println("<div id=\"content_inner\">\n"
                       "<style>\n"
                       ".rmrow{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:7px 0;}\n"
                       ".rmrow input[type=text],.rmrow input[type=password]{flex:1;min-width:9em;}\n"
                       ".rmh{display:grid;grid-template-columns:9em 1fr auto;}");
    web_client.println(".rmchips{display:flex;flex-wrap:wrap;gap:8px;margin:7px 0;}\n"
                       ".rmchip{flex-direction:column;align-items:flex-start;min-width:7em;text-align:left;}\n"
                       "#content_inner .rmchip{border:solid 2px transparent;}\n"
                       "#content_inner .rmchip.rmsaved,#content_inner .rmchip.rmsaved:hover{background:var(--mclightgreen);}");
    // The tile colour is the key state, the border is the selection; rules carry #content_inner to beat the shared button rule.
    web_client.println("#content_inner .rmchip.rmstale{opacity:.6;}\n"
                       "#content_inner .rmchip.rmsel{border-color:var(--mcgray);}\n"
                       ".rmlk{width:1em;height:1em;vertical-align:-.15em;fill:currentColor;}\n"
                       ".rmsplit{justify-content:space-between;}");
    web_client.println(".rmtiles{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:7px 0;}\n"
                       ".rmtile-end{grid-column:3;}\n"
                       ".rmwarn{background:var(--mclightred);}");
    web_client.println(".rmg{grid-template-columns:minmax(7em,max-content) 1fr max-content;}\n"
                       ".rmg .rmsw{justify-self:start;margin:0;}\n"
                       ".rmg input[type=text],.rmf input[type=text]{width:100%;box-sizing:border-box;}\n"
                       ".rmf{grid-template-columns:9em 1fr 3em;margin:7px 0;}\n"
                       ".rmf>label{grid-column:1;}\n"
                       ".rmf>label[for^=rm_sw]{grid-column:1/3;}");
    web_client.println(".rmf>.rmsw{grid-column:3;justify-self:end;margin:0;}\n"
                       ".rmf>div{grid-column:2;}\n"
                       ".rmf>div:empty{display:none;}\n"
                       ".rmf>.rmc{text-align:right;}");
    web_client.println(".rmkv{display:grid;grid-template-columns:max-content 1fr;gap:2px 10px;font-variant-numeric:tabular-nums;}\n"
                       ".rmdet{border-top:solid 1px #e0e0e0;margin-top:6px;padding-top:4px;}\n"
                       "#rm_mh tr>:nth-child(2){text-align:right;}\n"
                       ".rmtile{min-height:3.4em;justify-content:center;text-align:center;}\n"
                       ".rmarmed{background:var(--mcmidred);font-weight:bold;}\n"
                       "#rm_page button:disabled{opacity:0.45;cursor:default;}\n"
                       ".rmsw:indeterminate{background-color:#d4d4d4;}\n"
                       ".rmsw:indeterminate::before{left:.9em;}");
    web_client.println(".rmok{color:#1a7f37;font-weight:bold;}\n"
                       ".rmbad{color:var(--mcred);font-weight:bold;}\n"
                       ".rmlock{font-weight:bold;}\n"
                       "#rm_msg{min-height:1.4em;}\n"
                       ".rmtab{width:100%;border-collapse:collapse;}\n"
                       ".rmtab th,.rmtab td{padding:2px 5px;text-align:left;vertical-align:top;border-bottom:solid 1px #e0e0e0;}");
    web_client.println("#rm_msgs td:nth-child(2),#rm_msgs td:nth-child(4),#rm_log td:nth-child(3),#rm_log td:nth-child(4),#rm_log td:nth-child(5){overflow-wrap:anywhere;}\n"
                       "#rm_msgs td:nth-child(1),#rm_log td:nth-child(1),.rmtab button{white-space:nowrap;}");
    web_client.println("@media (max-width:600px){.rmtab thead{display:none;}\n"
                       ".rmtab,#rm_msgs,#rm_log{display:block;}\n"
                       ".rmw{grid-template-columns:1fr max-content;}\n"
                       ".rmw>:first-child{grid-column:1/3;}\n"
                       "#rm_msgs tr,#rm_log tr{display:block;padding:3px 0;border-bottom:solid 1px #e0e0e0;}");
    web_client.println("#rm_msgs td,#rm_log td{display:inline-block;border:0;padding:1px 8px 1px 0;}\n"
                       "#rm_msgs td:nth-child(2),#rm_msgs td:nth-child(4),#rm_log td:nth-child(4),#rm_log td:nth-child(5){display:block;}");
    web_client.println("#rm_log td:nth-child(2)::before{content:\"from \";}\n"
                       "#rm_log td:nth-child(3)::before{content:\"ctr \";}}\n"
                       "</style>\n"
                       "<div id=\"rm_page\">\n"
                       "<p id=\"rm_auth\" class=\"rmbad\" style=\"display:none\"></p>\n"
                       "<p id=\"rm_net\" class=\"font-small\"></p>\n"
                       "<p id=\"rm_msg\" class=\"rmmsg\"></p>\n"
                       "<p id=\"rm_force\" class=\"rmmsg\"></p>\n"
                       "<p id=\"rm_chain\" class=\"font-small\"></p>\n"
                       "<p id=\"rm_off\" class=\"font-small\" style=\"display:none\"></p>");
    web_client.println("<div class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Choose a node</label>\n"
                       "<span>Pick the node to manage.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div>");
    // One sprite for the padlock (closed = key missing, open = key saved), referenced by every tile.
    web_client.println("<svg id=\"rm_sprite\" width=\"0\" height=\"0\" style=\"position:absolute\" aria-hidden=\"true\"><symbol id=\"rmlk-c\" viewBox=\"0 0 12 12\"><path d=\"M2 5h8v6H2zM3 5V4a3 3 0 016 0v1H8V4a2 2 0 00-4 0v1z\"/></symbol>"
                       "<symbol id=\"rmlk-o\" viewBox=\"0 0 12 12\"><path d=\"M2 5h8v6H2zM3 4V3a3 3 0 016 0v1H8V3a2 2 0 00-4 0v1z\"/></symbol></svg>");
    web_client.println("<p class=\"font-small\">Nodes heard directly in the last 3 hours and nodes with a saved key. Remote management may be switched off on any of them.</p>\n"
                       "<div id=\"rm_nodes\" class=\"rmchips\"></div>\n"
                       "<div class=\"rmrow rmsplit\"><button type=\"button\" data-act=\"heard\">Refresh list</button><button type=\"button\" id=\"rm_forgetall\" data-act=\"forgetall\">Forget all</button></div>");
    web_client.println("<div class=\"rmrow\"><input type=\"text\" id=\"rm_call\" maxlength=\"9\" autocomplete=\"off\" autocapitalize=\"characters\" placeholder=\"Call sign, for example DK5EN-12\" aria-label=\"Call sign of the node\"></div>\n"
                       "<p id=\"rm_callchk\" class=\"font-small\"></p>");
    web_client.println("<div id=\"rm_pwrow\" class=\"rmrow\"><input type=\"password\" id=\"rm_pw\" maxlength=\"14\" autocomplete=\"new-password\" placeholder=\"Password of that node\" aria-label=\"Password of the node\"><button type=\"button\" data-act=\"remember\">Remember</button></div>\n"
                       "<p id=\"rm_savednote\" class=\"font-small\"></p>");
    web_client.println("</div></div>\n");
    web_client.println("<div class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Messages</label>\n"
                       "<span>Commands sent from this page.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div><table class=\"rmtab font-small\"><thead><tr><th>time</th><th>command</th><th>state</th><th>reply</th></tr></thead><tbody id=\"rm_msgs\"></tbody></table></div></div>\n");
    web_client.println("<div class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Actions</label>\n"
                       "<span>Status, switches, restart.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div><div id=\"rm_info\" class=\"rmtiles\"></div><div id=\"rm_sw\" class=\"grid rmg\"></div><p id=\"rm_swnote\" class=\"font-small\"></p></div></div>\n");
    web_client.println("<div id=\"rm_card_radio\" class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Radio</label>\n"
                       "<span>Radio settings and sensors.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_card_ident\" class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Identity</label>\n"
                       "<span>Name and APRS text.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_card_pos\" class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Position</label>\n"
                       "<span>Position of the node.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_card_queues\" class=\"cardlayout collapsablecard\"><label class=\"cardlabel\">Queues</label>\n"
                       "<span>Open this for TX queue, mailbox and max hop.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_card_mh\" class=\"cardlayout collapsablecard\"><label class=\"cardlabel\">Heard list</label>\n"
                       "<span>Open this for the heard list.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_adv\" class=\"cardlayout collapsablecard\"><label class=\"cardlabel\">Advanced</label>\n"
                       "<span>Output pin, counters, command log.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div>\n"
                       "<p class=\"font-small\">Switch an output pin of the node (only works if the pin is set as an output there).</p>");
    web_client.println("<div class=\"rmrow\"><select id=\"rm_pin\" aria-label=\"Output pin\"></select><label for=\"rm_pinsw\">Output</label><input type=\"checkbox\" role=\"switch\" class=\"rmsw\" id=\"rm_pinsw\" data-sw=\"pin\" disabled></div>\n"
                       "<p id=\"rm_cnt\" class=\"font-small\"></p>");
    web_client.println("<p class=\"font-small\">Commands run on this node</p>\n"
                       "<table class=\"rmtab font-small\"><thead><tr><th>ago</th><th>from</th><th>ctr</th><th>command</th><th>result</th></tr></thead><tbody id=\"rm_log\"></tbody></table>\n"
                       "</div></div>\n"
                       "</div>\n"
                       "</div>");
}

/** Scaffold JS of the Remote page: rmPageInit() / rmPageLeave() are called by loadPage(). Called once from
 *  deliver_scaffold(), inside its <script> block. All global names start with rm. */
void rmScaffoldJs()
{
    web_client.printf("var rmLen={name:%u,atxt:%u,args:%u};\n", (unsigned)RM_NAME_MAX, (unsigned)RM_ATXT_MAX, (unsigned)RM_MAX_ARGS);
    web_client.println("var rmShown=false,rmT={tick:0,arm:0,poll:0},rmSel={call:'',slot:-1},rmKnown=Object.create(null),rmSaved=[],rmHeard=[],rmLock=0,rmSrvLock=0,rmSrvMsg='',rmBusy=false,rmArmId='',rmQq=[],rmQp='',rmAuth=false,rmOut=false,rmStat=null,rmWasLocked=false,rmRows=Object.create(null);");
    web_client.println("var rmCmds={status:'Status',sendpos:'Send position',sendtrack:'Send track',reboot:'Restart',sync:'Re-sync counter',radio:'Radio',sens:'Sensors',name:'Name',atxt:'APRS text',pos:'Position',txq:'TX queue',mbox:'Mailbox',maxhop:'Max hop',mh:'Heard list',net:'Network',upd:'Update',blockcall:'Blocked calls'};");
    web_client.println("var rmUpdT0=0,rmUpdGone=Object.create(null),rmReally='Really? tap again',rmNoAns='The node did not answer the request. Check the connection and try again.',rmNote={t:'',c:''},rmSw=Object.create(null);\n"
                       "function rmCf(l,w){return 'Tap again within 4 seconds to confirm: '+l+' on '+w+'.';}");
    web_client.println("var rmTog={gps:0,track:1,display:2,mesh:3,gateway:4,led:5},rmTogName={gps:'GPS',track:'Track',display:'Display',led:'Light',mesh:'Mesh',gateway:'Gateway'},rmChgModes=['auto','on','off'];function rmSwLbl(n){return n=='ledflash'?'Light flash':rmTogName[n];}");
    web_client.println("var rmForce=null;\n"
                       "function rmNow(){return Date.now();}");
    web_client.println("function rmEl(i){return document.getElementById(i);}\n"
                       "function rmTxt(i,t,c){var e=rmEl(i);if(!e)return;e.textContent=t;if(c!==undefined)e.className=c;}\n"
                       "function rmMsg(t,c){rmNote.t=t;rmNote.c='rmmsg '+(c||'');rmRenderLock();}\n"
                       "function rmEnc(s){return encodeURIComponent(s);}\n"
                       "function rmErrText(j){return j&&j.msg?String(j.msg):'The request was refused'+(j&&j.err?' ('+j.err+')':'')+'.';}\n"
                       "function rmAgo(s){return s<120?s+' s':s<7200?Math.round(s/60)+' min':Math.round(s/3600)+' h';}");
    web_client.println("function rmUp(m){m=parseInt(m,10);if(isNaN(m))return '';return 'up '+(m<120?m+' min':Math.floor(m/60)+' h'+(m%60?' '+(m%60)+' min':''));}\n"
                       "function rmValidCall(c){return typeof c=='string'&&c.length<=9&&/^[A-Z0-9]{2,}-[0-9]{1,2}$/.test(c);}");
    web_client.println("function rmPwProblem(p){return !p.length?'Enter a password.':p.length>14?'The password can be at most 14 characters.':'';}\n"
                       "function rmKnFor(c){if(!rmKnown[c])rmKnown[c]={sw:[-1,-1,-1,-1,-1,-1],led:false,fl:false,chg:'',ver:'',up:'',bat:'',at:0};return rmKnown[c];}\n"
                       "function rmKn(){return rmKnFor(rmSel.call);}\n"
                       "function rmLockLeft(){var m=Math.max(rmLock,rmSrvLock)-rmNow();return m>0?Math.ceil(m/1000):0;}");
    web_client.println("function rmLocked(){return rmBusy||rmAuth||rmLockLeft()>0;}\n"
                       "function rmSlotOf(c){for(var i=0;i<rmSaved.length;i++)if(rmSaved[i]&&rmSaved[i].used&&rmSaved[i].call==c)return i;return -1;}\n");
    web_client.println("var rmTL=Object.create(null);function rmRowLocked(d){return rmBusy||rmAuth||rmLock>rmNow()||rmTL[d]>rmNow();}\n"
                       "function rmTxp(v){var m=/^(-?\\d+)\\/(-?\\d+)\\/(-?\\d+)$/.exec(String(v));return m?m[1]+' dBm ('+m[2]+'..'+m[3]+')':String(v)+' dBm';}\n"
                       "function rmLabel(f){var p=f.split(' '),n=p[0];if(rmCmds[n])return rmCmds[n];if(rmTogName[n])return rmTogName[n]+' '+p[1];if(n=='txpower')return 'TX power '+p[1]+' dBm';if(n=='setout')return 'Pin '+p[1]+' '+p[2];if(n=='chgled')return 'Charge LED'+f.slice(6);return f;}");
    web_client.println("function rmParseStatus(r){if(typeof r!='string'||r.indexOf('ok v=')!==0)return null;var o={ver:'',up:'',bat:'',sw:[-1,-1,-1,-1,-1,-1],led:false},t=r.split(' '),U='GTDMWL',i,j,k,m,s,ok;\n"
                       "for(i=1;i<t.length;i++){k=t[i];\n"
                       "if(k.indexOf('v=')==0)o.ver=k.substring(2);\n"
                       "else if(k.indexOf('up=')==0)o.up=k.substring(3);\n"
                       "else if(k.indexOf('bat=')==0)o.bat=k.substring(4);\n"
                       "else if(k.indexOf('p=')==0)o.p=rmTxp(k.substring(2));\n"
                       "else if(k.indexOf('rs=')==0)o.rs=k.substring(3);");
    web_client.println("else if(k.indexOf('s=')==0){s=k.substring(2);if(s.length!=6&&s.length!=5)return null;ok=true;m=[-1,-1,-1,-1,-1,-1];for(j=0;j<s.length;j++){if(s.charAt(j)==U.charAt(j))m[j]=1;else if(s.charAt(j)==U.charAt(j).toLowerCase())m[j]=0;else ok=false;}if(!ok)return null;for(j=0;j<6;j++)if(m[j]>=0)o.sw[j]=m[j];if(s.length==6)o.led=true;}");
    web_client.println("}\n"
                       "return o;}\n"
                       "function rmApplyEntry(k,e){var c=e.cmd.split(' '),n=c[0],o,m,i;");
    web_client.println("if(n=='status'){o=rmParseStatus(e.reply);if(!o)return;for(i=0;i<6;i++)if(o.sw[i]>=0)k.sw[i]=o.sw[i];if(o.led)k.led=true;if(o.sw[5]==0)k.fl=false;k.ver=o.ver;k.up=o.up;k.bat=o.bat;k.at=rmNow()-e.ago*1000;}\n");
    web_client.println("else if(n=='led'&&(m=/^ok led=(on|off|flash)$/.exec(e.reply))){k.sw[5]=m[1]=='off'?0:1;k.fl=m[1]=='flash';k.led=true;}\n"
                       "else if(n=='chgled'){m=/^ok chgled=(auto|on|off)( rm)?$/.exec(e.reply);if(m)k.chg=m[1];}\n"
                       "else if(rmTog[n]!==undefined){m=/^ok (\\w+)=(on|off)$/.exec(e.reply);if(m&&m[1]==n){k.sw[rmTog[n]]=m[2]=='on'?1:0;if(n=='led')k.led=true;}}\n");
    web_client.println("}");
    web_client.println("function rmApplySent(list){var a=list.slice(0).sort(function(x,y){return y.ago-x.ago;}),i,e;\n"
                       "for(i=0;i<a.length;i++){e=a[i];if(e.rep&&e.ver&&e.st=='ok')rmApplyEntry(rmKnFor(String(e.dst)),e);else if(e.rep&&e.ver&&e.st=='err'&&/^chgled\\b/.test(e.cmd)&&(e.reply=='err unsupported'||e.reply=='err blocked'))rmKnFor(String(e.dst)).chg='unsupported';}}\n");
    web_client.println("function rmOkText(e){var c=e.cmd.split(' '),n=c[0],o,m;\n"
                       "if(n=='status'){o=rmParseStatus(e.reply);return o?'Connected. Version '+o.ver+(o.up!==''?', '+rmUp(o.up):'')+(o.bat!==''?', battery '+o.bat+' %':'')+(o.rs=='1'?', strict security on':'')+'.':e.msg;}\n"
                       "if(n=='sendpos')return 'Done. The node sent its position.';");
    web_client.println("if(n=='sendtrack')return 'Done. The node sent its track.';\n"
                       "if(n=='reboot')return 'Done. The node is restarting, it is back in about 30 seconds.';\n"
                       "if(n=='sync')return 'Connected. The counters are in step.';\n");
    web_client.println("if(n=='led'&&c[1]=='flash')return 'Done. The light is now flashing.';\n");
    web_client.println("if(n=='chgled'){m=/^ok chgled=(auto|on|off)( rm)?$/.exec(e.reply);return c[1]?'Done. Charge LED is now '+c[1]+'.':m?'Charge LED is '+m[1]+(m[2]?' (set by remote management)':'')+'.':e.msg||'Done.';}\n"
                       "if(rmTogName[n])return 'Done. '+rmTogName[n]+' is now '+c[1]+'.';\n"
                       "if(n=='txpower')return 'Done. TX power is now '+c[1]+' dBm.';\n"
                       "if(n=='setout')return 'Done. Pin '+c[1]+' is now '+c[2]+'.';\n"
                       "return rmOkNew(e,c,n)||e.msg||'Done.';}");
    web_client.println("function rmPost(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b});}\n"
                       "function rmJson(r){if(r.status==401||r.status==403){rmAuthFail();return Promise.reject(0);}return r.json();}\n"
                       "function rmAuthFail(){rmAuth=true;clearTimeout(rmT.poll);rmT.poll=0;var e=rmEl('rm_auth');if(e){e.textContent='Your session has ended. Please log in again (reload this page).';e.style.display='';}rmRender();}");
    web_client.println("function rmCatch(e,id){if(e===0)return;if(id)rmTxt(id,rmNoAns,'rmmsg rmbad');else rmMsg(rmNoAns,'rmbad');}\n"
                       "function rmQClr(){rmQq=[];rmQp='';}\nfunction rmHintOff(){if(/rmconf/.test(rmNote.c))rmMsg('','');}\n"
                       "function rmDisarm(){clearTimeout(rmT.arm);rmT.arm=0;rmArmId='';rmHintOff();rmRender();}");
    web_client.println("function rmConfirm(id,hint){if(rmArmId==id){clearTimeout(rmT.arm);rmT.arm=0;rmArmId='';return true;}clearTimeout(rmT.arm);rmArmId=id;rmT.arm=setTimeout(rmDisarm,4000);rmRender();rmMsg(hint,'rmconf');return false;}\n"
                       "function rmBtn(box,label,cls,dis,attrs){var b=document.createElement('button'),k;b.type='button';b.className=cls||'';b.disabled=!!dis;b.textContent=label;for(k in attrs)b.setAttribute(k,attrs[k]);box.appendChild(b);return b;}");
    web_client.println("function rmTile(box,label,cmd,args,cf,cls,dis){var arm=cf&&rmArmId==cf,a={'data-cmd':cmd,'data-args':args};if(cf)a['data-cf']=cf;return rmBtn(box,arm?rmReally:label,'rmtile '+(cls||'')+(arm?' rmarmed':''),dis,a);}");
    web_client.println("function rmSwSet(i,st,dis,last,shown){i.disabled=!!dis;i.indeterminate=st<0&&!(shown&&last);i.checked=st<0?last=='on':st==1;}");
    web_client.println("function rmSwRow(box,n,st,dis,x){var l=document.createElement('label'),i=document.createElement('input'),a=rmArmId==n+' off';i.type='checkbox';i.id='rm_sw_'+n;i.className='rmsw';i.setAttribute('role','switch');i.setAttribute('data-sw',n);");
    web_client.println("if(n=='mesh'||n=='gateway')i.setAttribute('data-cf',n+' off');l.htmlFor=i.id;l.textContent=a?rmSwLbl(n)+': '+rmReally:rmSwLbl(n);if(x)l.appendChild(rmNew('span','font-small',' ('+x+')'));l.className=a?'rmarmed':'';rmSwSet(i,st,dis,rmSw[rmSel.call+' '+n]);box.appendChild(l);box.appendChild(i);if(!x)box.appendChild(document.createElement('span'));}");
    web_client.println("function rmRenderTiles(k,dis){var b=rmEl('rm_info'),i,d=[['status','Refresh status'],['sendpos','Send position now'],['sendtrack','Send track now']],o=['display','led','ledflash','mesh','gateway'],p=rmEl('rm_pin').value;\n"
                       "b.textContent='';for(i=0;i<d.length;i++)rmTile(b,d[i][1],d[i][0],'','','',dis);");
    web_client.println("rmBtn(b,'Re-sync counter','rmtile',dis,{'data-act':'sync',title:'Use this if the node keeps answering that the counters are out of step.'});rmTile(b,'Restart','reboot','','reboot','rmwarn rmtile-end',dis);\n"
                       "b=rmEl('rm_sw');b.textContent='';\n"
                       "for(i=0;i<o.length;i++){if((o[i]=='led'||o[i]=='ledflash')&&!k.led)continue;rmSwRow(b,o[i],o[i]=='ledflash'?(k.sw[5]<0?-1:k.sw[5]==1&&k.fl?1:0):k.sw[rmTog[o[i]]],dis);}\n"
                       "rmChgRow(b,k,dis);");
    web_client.println("function rmChgRow(box,k,dis){var l=document.createElement('label'),s=document.createElement('select'),o,i,v=rmChgModes.indexOf(k.chg)>=0?k.chg:k.chg=='unsupported'?'':rmSw[rmSel.call+' chgled']||'';\n");
    web_client.println("s.id='rm_chgled';s.setAttribute('data-sel','chgled');l.htmlFor=s.id;l.textContent='Charge LED';o=document.createElement('option');o.value='';o.textContent='unknown';s.appendChild(o);\n"
                       "for(i=0;i<3;i++){o=document.createElement('option');o.value=rmChgModes[i];o.textContent=rmChgModes[i];s.appendChild(o);}\n"
                       "s.value=v;s.disabled=!!dis;box.appendChild(l);box.appendChild(s);box.appendChild(rmNew('span','font-small',k.chg=='unsupported'?'not supported by this node':''));}");
    web_client.println("rmTxt('rm_swnote',k.at?'State from the last answer of '+rmSel.call+', '+rmAgo(Math.round((rmNow()-k.at)/1000))+' ago.':'State unknown until the node answers a status request. Press Refresh status.','font-small');\n"
                       "rmSwSet(rmEl('rm_pinsw'),-1,dis,rmSw[rmSel.call+' '+p],1);}");
    web_client.println("function rmChip(box,call,hw,age,sv,stale){var b=rmBtn(box,'','rmchip'+(sv?' rmsaved':'')+(stale?' rmstale':'')+(rmSel.call==call?' rmsel':''),false,{'data-act':'pick','data-call':call}),s=document.createElement('b'),i,L=[hw,age];\n"
                       "s.textContent=call;b.appendChild(s);\n"
                       "for(i=0;i<2;i++){s=document.createElement('span');s.className='font-small';s.textContent=L[i];b.appendChild(s);}");
    web_client.println("var N='http://www.w3.org/2000/svg',v=document.createElementNS(N,'svg'),u=document.createElementNS(N,'use');\n"
                       "s=document.createElement('span');s.className='font-small';v.setAttribute('class','rmlk');u.setAttribute('href',sv?'#rmlk-o':'#rmlk-c');v.appendChild(u);s.appendChild(v);s.appendChild(document.createTextNode(' '+(sv?'key saved':'no key')));b.appendChild(s);}");
    web_client.println("function rmRenderNode(){var h=rmEl('rm_nodes'),i,x,k,c=rmSel.call,t=rmEl('rm_callchk'),n=0,sv=rmSel.slot>=0,hd=false,seen=Object.create(null),e=rmEl('rm_savednote');\n"
                       "h.textContent='';\n"
                       "for(i=0;i<rmHeard.length;i++){x=rmHeard[i];k=String(x.call);seen[k]=1;if(k==c)hd=true;rmChip(h,k,String(x.hw||''),rmAgo(x.age_s|0)+' ago'+(x.rssi?', '+x.rssi+' dBm':''),rmSlotOf(k)>=0,false);}");
    web_client.println("for(i=0;i<rmSaved.length;i++){x=rmSaved[i];if(x&&x.used){n++;k=String(x.call);if(!seen[k]){seen[k]=1;rmChip(h,k,'','not heard',true,true);}}}\n"
                       "if(!h.firstChild)h.textContent='No node heard directly yet.';\n"
                       "rmEl('rm_forgetall').style.display=n?'':'none';\n"
                       "t.textContent='';\n"
                       "if(c&&!rmValidCall(c)){t.textContent='Not a call sign. Use letters or digits, a dash and an SSID, for example DK5EN-12.';t.className='font-small rmbad';}\n"
                       "rmEl('rm_pwrow').style.display=sv?'none':'';");
    web_client.println("e.className='font-small';\n"
                       "e.textContent=sv?'Key saved for '+c+'.'+(hd?'':' Not heard in the last 3 hours, the command may not arrive.')+' ':'Type the password of '+(c||'the node you want to manage')+', or press Remember to save it.';\n"
                       "if(sv)rmBtn(e,'Forget','',false,{'data-act':'forget'});\n"
                       "rmEl('rm_forgetall').textContent=rmArmId=='forgetall'?rmReally:'Forget all';}\n"
                       "function rmRenderLock(){var n=rmLockLeft(),t=rmNote.t,c=rmNote.c;");
    web_client.println("if(!t){c='rmmsg rmlock';if(rmBusy)t='Sending ...';else if(!rmAuth&&n>0)t=(rmSrvMsg?rmSrvMsg+' ':'')+'Next command possible in '+n+' s.';}\n"
                       "rmTxt('rm_msg',t,c);}\n"
                       "function rmRenderMsgs(dis){var j=rmStat,b=rmEl('rm_msgs'),a,i,e,k,tr,v,x,c,cf,n,seen={},q,d=rmEl('rm_chain'),u=0,s,z;if(!j)return;d.textContent='';\n");
    web_client.println("for(i=0;i<j.targets.length;i++){q=j.targets[i];if(rmForce&&q.dst==rmForce.d&&!q.canForce){rmForce=null;rmForceBtn();}if(q.chainMsg)d.textContent=q.dst+': '+q.chainMsg;else if(q.pending)d.textContent=q.dst+': checking the connection first, your command follows.';}\n");
    web_client.println("a=j.sent.slice(0).sort(function(x,y){return x.ago-y.ago;});\n");
    web_client.println("for(i=0;i<a.length;i++){e=a[i];k=e.dst+'#'+e.ctr+'#'+e.cmd;if(seen[k])continue;if(u>4)break;seen[k]=1;tr=rmRows[k];\n"
"if(!tr){tr=rmRows[k]=document.createElement('tr');for(n=0;n<4;n++)tr.appendChild(document.createElement('td'));}\n");
    web_client.println("c=tr.children;n=e.cmd.split(' ');v=e.st=='queued'||e.st=='waiting'?'sent':e.st=='noanswer'?'no answer':e.ver&&(e.st=='ok'||e.st=='err')?'verified':'unverified';\n"
"x=e.st=='ok'&&v=='verified'?rmOkText(e):(e.msg||'');\n"
"q=/^err range (-?\\d+)\\.\\.(-?\\d+)$/.exec(e.reply||'');if(q&&e.st=='err')x='Refused: '+e.dst+' accepts '+q[1]+'..'+q[2]+' dBm.';tr.title=e.msg||'';\n");
    web_client.println("c[0].textContent=rmAgo(e.ago)+' ago';c[1].textContent=e.dst+' '+rmLabel(e.cmd);c[2].textContent=v;c[2].className=v=='verified'?(e.st=='ok'?'rmok':'rmbad'):v=='sent'?'':'rmbad';c[3].textContent='';z=document.createElement('div');z.textContent=v=='sent'?'':x;c[3].appendChild(z);\n");
    web_client.println("if(b.children[u]!==tr)b.insertBefore(tr,b.children[u]||null);u++;}\n");
    web_client.println("for(k in rmRows)if(!seen[k]){if(rmRows[k].parentNode)rmRows[k].parentNode.removeChild(rmRows[k]);delete rmRows[k];}}\n");
    web_client.println("var rmIn={},rmPK=['lat','lon','alt','src'],rmDefs=[{c:'radio',card:'radio',l:'Radio',f:[['f','Frequency',' MHz'],['sf','Spreading factor',''],['cr','Coding rate','',function(v){return '4/'+v;}],['bw','Bandwidth',' kHz'],['p','TX power','',rmTxp]]},\n"
                       "{c:'sens',card:'radio',l:'Sensors',f:[['t','Temperature',' C'],['h','Humidity',' %'],['p','Pressure',' hPa'],['t2','Second temperature',' C']]},\n");
    web_client.println("{c:'name',card:'ident',l:'Name',f:[],w:[['v','Name','name',rmLen.name,'n']]},{c:'atxt',card:'ident',l:'APRS text',f:[],w:[['v','APRS text','atxt',rmLen.atxt,'a']]},\n");
    web_client.println("{c:'pos',card:'pos',l:'Position',p:rmPK,f:[['src','Source','',function(v){var m={gps:'from GPS',nofix:'GPS on, no fix',set:'set by hand'};return Object.prototype.hasOwnProperty.call(m,v)?m[v]:v;}]],\n"
                       "w:[['lat','Latitude (+/-)','lat',11],['lon','Longitude (+/-)','lon',11],['alt','Altitude (meter)','alt',5]]},\n");
    web_client.println("{c:'txq',card:'queues',l:'TX queue',k:1,v:rmTxqView,t:['tx','Sent','rt','Retransmitted','dr','Dropped']},\n"
                       "{c:'mbox',card:'queues',l:'Mailbox',k:1,u:'This node has no mailbox.',f:[['m','Mode',''],['u','Slots used',''],['b','Bytes',''],['a','Actions this hour','']],t:['st','Stored','dl','Delivered','ak','Acknowledged','dr','Dropped','bl','Blocked','nt','Notified']},\n");
    web_client.println("{c:'maxhop',card:'queues',l:'Max hop',k:1,f:[['t','Text messages',''],['p','Position beacons','']]},\n"
                       "{c:'net',card:'radio',l:'Network',k:1,x:1,v:rmNetView,u:'This node has no network settings.'},{c:'upd',card:'radio',l:'Software update',k:1,x:1,a:1,v:rmUpdView,u:'This node cannot update itself.'},\n");
    web_client.println("{c:'blockcall',card:'radio',l:'Blocked calls',f:[],n:'The whole list is replaced. Over remote management it can hold '+rmLen.args+' characters (3 or 4 calls); type none to clear it.',w:[['v','Blocked calls','blockcall',rmLen.args,'b']]}];\n");
    web_client.println("function rmClrIn(c){var k;for(k in rmIn)if(k.indexOf('rm_f_'+c+'_')==0)delete rmIn[k];}\n");
    web_client.println("function rmChk(k,v){var m;if(v==='')return '';\n"
                       "if(k=='lat'||k=='lon'){m=k=='lat'?90:180;return(/^-?\\d{1,3}(\\.\\d{1,6})?$/.test(v)&&Math.abs(+v)<=m)?false:(k=='lat'?'Latitude':'Longitude')+' must be -'+m+' to '+m+', decimal point, at most 6 decimals.';}\n"
                       "if(k=='alt')return(/^\\d{1,5}$/.test(v)&&+v<=40000)?false:'Altitude must be a whole number from 0 to 40000.';\n");
    web_client.println("if(k=='blockcall')return v.length>rmLen.args?'Too long.':/^(none|[A-Za-z0-9,-]+)$/i.test(v)?false:'Use calls separated by commas, or the word none.';\n"
                       "return v.length>rmLen[k=='name'?'name':'atxt']?'Too long.':false;}\n");
    web_client.println("function rmWUpd(d){var c=rmSel.call,g=!rmValidCall(c)||rmLocked(),j,w,v,r,x,s,n,t,a=[],y;\n"
                       "for(j=0;j<d.w.length;j++)a.push(rmEl('rm_f_'+d.c+'_'+d.w[j][0]).getAttribute('data-r'));");
    web_client.println("for(j=0;j<d.w.length;j++){w=d.w[j];n='rm_f_'+d.c+'_'+w[0];x=rmEl(n);v=x.value;r=rmChk(w[2],v);t=rmEl(n+'_cnt');if(t)t.textContent=v.length+'/'+w[3];\n"
                       "y=a.slice();y[j]=d.c=='blockcall'?(v.toLowerCase()=='none'?'none':v.toUpperCase()):v;s=rmEl(n+'_set');s.disabled=g||r!==false||(!!d.p&&(+v==a[j]||a.indexOf('')>=0));s.setAttribute('data-args',y.join(' '));s.setAttribute('data-cf',d.c+' '+y.join(' '));rmTxt(n+'_hint',r||'','font-small rmbad');}}\n");
    web_client.println("function rmWire(x,d){x.addEventListener('input',function(){rmIn[x.id]=x.value;rmWUpd(d);});}\n");
    web_client.println("function rmKv(r,f){var o={},t,i,a;if(typeof r!='string'||r.indexOf('ok ')!==0)return null;t=r.substring(3).split(' ');\n"
                       "for(i=0;i<t.length;i++){if(!f&&/^[na]=/.test(t[i])){o[t[i].charAt(0)]=t.slice(i).join(' ').substring(2);break;}a=t[i].indexOf('=');if(a>0)o[t[i].substring(0,a)]=t[i].substring(a+1);}return o;}\n"
                       "function rmPosKv(p,r){var o={},t,i;if(typeof r!='string'||r.indexOf('ok ')!==0)return null;t=r.substring(3).split(' ');if(t.length!=p.length)return null;for(i=0;i<p.length;i++)o[p[i]]=t[i];return o;}");
    web_client.println("function rmOkNew(e,c,n){var o=rmKv(e.reply),q=rmPosKv(rmPK,e.reply),t=c.slice(1).join(' '),w=n=='name'?'name':'APRS text';\n"
                       "if(n=='radio'&&o&&o.f)return 'Radio: '+o.f+' MHz, SF '+o.sf+', '+o.bw+' kHz, TX power '+rmTxp(o.p)+'.';\n"
                       "if(n=='sens')return 'Sensors read.';\n"
                       "if(n=='net')return o&&o.mtu?'Network read.':'';if(n=='upd')return rmUpdOk(c,e.reply);\n"
                       "if(n=='name'||n=='atxt'){o=o&&o[n=='name'?'n':'a'];if(t)return 'Done. The '+w+' is now '+(o||t)+'.';return o?'The '+w+' is '+(o=='-'?'empty':o)+'.':'';}\n");
    web_client.println("if(n=='blockcall'){o=o&&o.b;if(t)return 'Done. Blocked calls are now '+(o=='-'?'none':(o||t))+'.';return o?'Blocked calls: '+(o=='-'?'none':o)+'.':'';}\n");
    web_client.println("if(n=='pos'){if(t)return 'Done. Position set to '+c[1]+', '+c[2]+', '+c[3]+' m.';return q?'Position '+q.lat+', '+q.lon+', '+q.alt+' m, '+rmDefs[4].f[0][3](q.src)+'.':'';}return '';}\n");
    web_client.println("function rmKvRows(p,r){var y=p.appendChild(rmNew('div','rmkv')),i;for(i=0;i<r.length;i++){y.appendChild(rmNew('span','font-small',r[i][0]));y.appendChild(rmNew('span','',r[i][1],r[i][2]));}}");
    web_client.println("function rmNetView(p,o){var w={auto:'automatic',inet:'Internet',hamnet:'HAMNET'},e=o.eff;rmKvRows(p,[['MTU',o.mtu?o.mtu+' bytes':'-','rm_v_net_mtu'],['Server',o.srv||'-','rm_v_net_srv'],['Network',Object.prototype.hasOwnProperty.call(w,o.net)?w[o.net]:o.net||'-','rm_v_net_net'],['Effective uplink',e=='-'?'no IP uplink':e?(w[e]||e):'-','rm_v_net_eff']]);}");
    web_client.println("function rmUpdOk(c,r){var m;if(c[1]=='check')return r=='ok checking'?'The node is checking for a new version. Read again in about 30 seconds.':'';\n"
                       "if(c[1]=='go'){m=/^ok (installing|applying) (\\S+)$/.exec(r);return m?'The node is '+m[1]+' '+m[2]+' and will restart. Read again in a few minutes.':'';}\n"
                       "return /^ok m=/.test(r)?'Update status read.':'';}");
    web_client.println("function rmUpdView(p,o,v){var c=rmSel.call,z=function(k){return o[k]===undefined||o[k]=='-'?'':o[k];},tag=z('st')||(o.n=='1'?z('a'):''),g=rmUpdGone[c],t=v&&rmStat?rmStat.t-v.ago*1000-1000:0,dis=!rmValidCall(c)||rmLocked();");
    web_client.println("var m={off:'off',notify:'notify only',auto:'automatic'},ch={prod:'production',dev:'development'},y=function(k){return o[k]=='1'?'yes':'no';},r,ok;\n"
                       "ok=!!v&&!dis&&/^[A-Za-z0-9._-]{1,23}$/.test(tag)&&o.b=='0'&&o.g=='0'&&t>=rmUpdT0&&!(g&&t<=g);");
    web_client.println("if(v)rmKvRows(p,[['Mode',m[o.m]||o.m||'-','rm_v_upd_m'],['Channel',ch[o.c]||o.c||'-','rm_v_upd_c'],['Available version',z('a')||'none','rm_v_upd_a'],['Newer than installed',y('n'),'rm_v_upd_n'],['Staged version',z('st')||'none','rm_v_upd_st'],");
    web_client.println("['Busy',y('b'),'rm_v_upd_b'],['Go pending',y('g'),'rm_v_upd_g'],['Last check',z('ck')?z('ck')+' min ago':'not yet','rm_v_upd_ck'],['Last error',z('e')||'none','rm_v_upd_e']]);\n"
                       "r=p.appendChild(rmNew('div','rmrow rmsplit'));rmBtn(r,'Check','',dis,{'data-cmd':'upd','data-args':'check','id':'rm_upd_check'});rmTile(r,'Update now','upd','go '+tag,'upd go '+tag,'rmwarn rmtile-end',!ok).id='rm_upd_go';}\n");
    web_client.println("function rmWBuild(p,d,o){var j,w,x,k,r,n,b;if(!d.w)return;r=rmNew('div','grid rmf');p.appendChild(r);\n"
                       "for(j=0;j<d.w.length;j++){w=d.w[j];n='rm_f_'+d.c+'_'+w[0];k=rmNew('label','',w[1]+' :');k.htmlFor=n;r.appendChild(k);x=rmNew('input','','',n);x.type='text';x.autocomplete='off';if(d.w.length>1)x.placeholder=w[1];\n"
                       "k=o&&o[w[4]||w[0]];k=k&&k!='-'?k:'';x.setAttribute('data-r',k);x.value=rmIn[n]!==undefined?rmIn[n]:k;r.appendChild(x);rmWire(x,d);");
    web_client.println("b=rmBtn(r,'','',true,{'data-cmd':d.c,'data-args':'','data-cf':d.c,'aria-label':'Set '+w[1]});b.id=n+'_set';b.appendChild(rmNew('i','btncheckmark'));r.appendChild(rmNew('div','font-small','',n+'_hint'));\n"
                       "if(!d.p)r.appendChild(rmNew('div','font-small rmc','',n+'_cnt'));}rmWUpd(d);}\n");
    web_client.println("function rmLast(c,n,x){var a=rmStat?rmStat.sent:[],i,b=null;for(i=0;i<a.length;i++)if(a[i].dst==c&&(x?a[i].cmd==n:a[i].cmd.split(' ')[0]==n)&&(!b||a[i].ago<b.ago)&&a[i].st=='ok'&&a[i].ver)b=a[i];return b;}");
    web_client.println("var rmMh={on:0,job:'',args:'',fl:0,t:0,pre:'',rows:[],total:0,msg:'',det:null,gen:0,tm:0,ctr:-1};\nfunction rmUns(c,n,v){var a=rmStat?rmStat.sent:[],i;for(i=0;i<a.length;i++)if(a[i].dst==c&&a[i].cmd==n&&a[i].st=='err'&&a[i].reply=='err unsupported'&&(!v||a[i].ago<v.ago))return true;return false;}\n");
    web_client.println("function rmMhReset(){clearTimeout(rmMh.tm);rmMh.gen++;rmMh.on=0;rmMh.fl=0;rmMh.job='';rmMh.pre='';rmMh.rows=[];rmMh.total=0;rmMh.msg='';rmMh.det=null;}\nfunction rmNodeChg(n){if(n!=rmSel.call){rmIn={};rmQClr();rmMhReset();rmForce=null;rmForceBtn();}}\n");
    web_client.println("function rmMhEnd(m){clearTimeout(rmMh.tm);rmMh.gen++;rmMh.on=0;rmMh.fl=0;rmMh.job='';rmMh.pre='';rmMh.msg=m||'';rmRender();}\nfunction rmMhStop(){rmMhEnd('Stopped.');}\nfunction rmMhGo(job,args){var pr=rmPre();if(rmMh.on)return;if(pr){rmMsg(pr,'rmbad');return;}\nif(!rmMh.pre){rmMh.pre=rmSel.slot>=0?'slot='+rmSel.slot+'&call='+rmEnc(rmSel.call):'dst='+rmEnc(rmSel.call)+'&pw='+rmEnc(rmEl('rm_pw').value);rmEl('rm_pw').value='';}\n");
    web_client.println("rmMh.on=1;rmMh.job=job;rmMh.args=args;rmMh.msg='';rmMh.det=null;if(job=='list'){rmMh.rows=[];rmMh.total=0;}rmMhStep();rmRender();}\nfunction rmMhStart(){rmMhGo('list','0');}\nfunction rmMhDet(c){c=String(c).trim().toUpperCase();if(rmMh.on)return;if(!rmValidCall(c)){rmMsg('Enter a node call such as DK5EN-12.','rmbad');return;}rmMhGo('det',c);}\n");
    web_client.println("function rmMhStep(){var w,e;if(!rmMh.on)return;if(rmMh.fl){e=rmMhFind();if(e)rmMhGot(e);else if(rmNow()-rmMh.t>180000)rmMhEnd('There was no answer from the node.');return;}\n");
    web_client.println("w=rmTL[rmSel.call]-rmNow();if(w>0){clearTimeout(rmMh.tm);rmMh.tm=setTimeout(rmMhStep,Math.min(w+30,15000));return;}rmMhSend();}\n");
    // Driver match: the sent[] entry must carry the ctr the send answered with. A send answered viaSync gets its
    // ctr only later, so rmMh.ctr stays -1 there and rmMhFind falls back to the time-window match (cmd, dst, ago).
    web_client.println("function rmMhSend(){var g=rmMh.gen;rmMh.fl=1;rmMh.ctr=-1;rmMh.t=rmNow();rmOut=true;\nrmPost('/rmsend',rmMh.pre+'&cmd=mh&args='+rmEnc(rmMh.args)).then(rmJson).then(function(j){if(g!=rmMh.gen||!rmMh.on)return;\nif(j&&j.ok){rmMh.ctr=(j.viaSync||typeof j.ctr!='number')?-1:j.ctr;rmPoll();return;}rmMh.fl=0;if(j&&j.err=='busy'&&j.retry>0){rmTL[rmSel.call]=rmNow()+j.retry*1000;rmMhStep();return;}\n");
    web_client.println("rmMhEnd(rmErrText(j));}).catch(function(e){if(g==rmMh.gen&&e!==0)rmMhEnd(rmNoAns);});}\nfunction rmMhFind(){var a=rmStat?rmStat.sent:[],i,b=null,e,x=rmNow()-rmMh.t+2000;for(i=0;i<a.length;i++){e=a[i];if(e.dst==rmSel.call&&e.cmd=='mh '+rmMh.args&&(rmMh.ctr<0||e.ctr==rmMh.ctr)&&e.ago*1000<=x&&(!b||e.ago<b.ago)&&(e.st=='ok'||e.st=='err'||e.st=='noanswer'))b=e;}return b;}\n");
    web_client.println("function rmMhL(v,l,u,f){return l+': '+((v===undefined||v=='-')?'not present':f?f(v):v+u);}\nfunction rmYn(v){return v=='1'?'yes':'no';}\nfunction rmMhGot(e){var r=e.reply,t,i,o={},k,n,d,x;rmMh.fl=0;\nif(e.st=='noanswer'||!e.ver||typeof r!='string')return rmMhEnd('There was no answer from the node.');\n");
    web_client.println("if(r.indexOf('err ')==0){k=r.substring(4).split(' ')[0];if(k=='end'&&rmMh.job=='list')return rmMhEnd('End of the list.');return rmMhEnd(k=='unknown'?'That node is not known there.':rmErrText({err:k,msg:e.msg}));}\nt=r.split(' ');if(t[0]!='ok')return rmMhEnd('The reply could not be read.');\nif(rmMh.job=='det'){for(i=2;i<t.length;i++){k=t[i].indexOf('=');if(k>0)o[t[i].substring(0,k)]=t[i].substring(k+1);}\n");
    web_client.println("d=t[1]=='d'?[rmMhL(o.g,'Gateway','',rmYn),rmMhL(o.m,'Mesh','',rmYn),rmMhL(o.r,'RSSI',' dBm'),rmMhL(o.s,'SNR',' dB'),rmMhL((o.la=='-'||o.lo=='-')?'-':o.la+', '+o.lo,'Position',''),rmMhL(o.di,'Distance',' km'),rmMhL(o.a,'Altitude',' m'),rmMhL(o.n,'Its neighbours',''),rmMhL(o.x,'Only it hears',''),rmMhL(o.h,'It hears',''),rmMhL(o.t,'Heard',' min ago')]:\n");
    web_client.println("[rmMhL(o.h,'Hops',''),rmMhL(o.k,'Routes',''),rmMhL(o.g,'Via gateway','',rmYn),'Relay: '+((o.rc===undefined||o.rc=='-')?'not known':o.rc),rmMhL(o.t,'Age',' min'),rmMhL(o.v,'Via','',function(v){return v.split(',').join(', ');})];\nrmMh.det={c:rmMh.args,l:d};return rmMhEnd('');}\nn=rmMh.rows.length;rmMh.total=parseInt(t[1],10)||0;for(i=3;i+1<t.length&&rmMh.rows.length<128;i+=2)rmMh.rows.push({c:t[i],m:t[i+1]});\n");
    web_client.println("if(rmMh.rows.length==n||!/^[0-9]+$/.test(t[2]||'-')||rmMh.rows.length>=128)return rmMhEnd('End of the list.');\nrmMh.args=t[2];rmMhStep();rmRender();}");
    web_client.println("function rmCardHd(b,c,L,dis){var h=b.appendChild(rmNew('div','rmrow rmh')),j,v,y=null;h.appendChild(rmNew('span'));for(j=0;j<L.length;j++){v=rmLast(c,L[j]);if(v&&(!y||v.ago<y.ago))y=v;}\nh.appendChild(rmNew('span','font-small',y?'last read '+rmAgo(y.ago)+' ago':'not read yet'));rmBtn(h,'Read','',dis,{'data-act':'qread','data-q':L.join(' ')});}");
    web_client.println("function rmNew(t,c,x,i){var e=document.createElement(t);if(c)e.className=c;if(x)e.textContent=x;if(i)e.id=i;return e;}\nfunction rmMhCard(b,dis){var h=b.appendChild(rmNew('div','rmrow rmsplit')),s=rmNew('span'),w=b.appendChild(rmNew('div','grid rmg rmw')),x,tb,tr,i,r,k;h.appendChild(rmNew('b','','Heard list'));");
    web_client.println("h.appendChild(rmNew('span','font-small',(rmMh.total||rmMh.rows.length?'Read '+rmMh.rows.length+' of '+rmMh.total+'. ':'')+rmMh.msg,'rm_mh_prog'));h.appendChild(s);\n"
                       "rmBtn(s,'Read','',dis||rmMh.on,{'data-act':'mhgo'});rmBtn(s,'Stop','',!rmMh.on,{'data-act':'mhstop'});");
    web_client.println("w.appendChild(rmNew('span','','Other node'));x=w.appendChild(rmNew('input','','','rm_f_mh_other'));x.type='text';x.maxLength=9;x.value=rmIn[x.id]||'';x.addEventListener('input',function(){rmIn[this.id]=this.value;});rmBtn(w,'Look up','',dis||rmMh.on,{'data-act':'mhother'});");
    web_client.println("tr=b.appendChild(rmNew('table','rmtab font-small','','rm_mh')).appendChild(rmNew('thead')).appendChild(rmNew('tr'));for(i=0;i<3;i++)tr.appendChild(rmNew('th','',['node','heard',''][i]));\n"
                       "tb=b.lastChild.appendChild(rmNew('tbody','','','rm_mh_tab'));");
    web_client.println("for(i=0;i<rmMh.rows.length;i++){r=rmMh.rows[i];tr=tb.appendChild(rmNew('tr'));rmCell(tr,r.c);rmCell(tr,r.m+' min ago');rmBtn(tr.appendChild(rmNew('td')),'Details','',dis||rmMh.on,{'data-act':'mhdet','data-call':r.c});}");
    web_client.println("x=b.appendChild(rmNew('div',rmMh.det?'rmdet':'','','rm_mh_det'));if(rmMh.det){x.appendChild(rmNew('b','',rmMh.det.c));k=x.appendChild(rmNew('div','rmkv'));\n"
                       "for(i=0;i<rmMh.det.l.length;i++){r=rmMh.det.l[i];tr=r.indexOf(': ');k.appendChild(rmNew('span','font-small',r.slice(0,tr)));k.appendChild(rmNew('span','',r.slice(tr+2)));}}}\n");
    web_client.println("function rmTiles(p,c,o,a){var g=rmNew('div','mbx-counters'),i,d;p.appendChild(g);for(i=0;i<a.length;i+=2){d=g.appendChild(rmNew('div','',a[i+1]));d.appendChild(rmNew('b','',o[a[i]]||'-','rm_v_'+c+'_'+a[i]));}}");
    web_client.println("function rmTxqView(p,o){var m=/^(\\d+)\\/(\\d+)$/.exec(o.q||''),b=m?+m[2]:0,f=b>60?Math.round(m[1]*60/b):m?+m[1]:0,v=['quiet','qrs','qrt'].indexOf(o.bp),x=p.appendChild(rmNew('div','mcq-bar')),s,i;\n"
                       "for(i=0;i<Math.min(b,60);i++)x.appendChild(rmNew('div',i<f?'mcq-cell':'mcq-cell-empty')).style.background=i<f?'#3B7DD8':'';");
    web_client.println("x=p.appendChild(rmNew('div','font-small',(m?o.q:'-')+' queued, state ','rm_v_txq_q'));s=x.appendChild(rmNew('span',['rmok','','rmbad'][v],v<0?o.bp||'-':['quiet','slow down','hold'][v],'rm_v_txq_bp'));if(v==1)s.style.color='#E07B39';\n"
                       "x=p.appendChild(rmNew('div','mcq-util-row'));x.appendChild(rmNew('span','mcq-util-label','use '+(isNaN(o.u)?'-':o.u+' %'),'rm_v_txq_u'));");
    web_client.println("x.appendChild(rmNew('div','mcq-util-track')).appendChild(rmNew('div')).style.cssText='height:100%;width:'+(parseInt(o.u,10)||0)+'%;background:#3B7DD8';}\n");
    web_client.println("function rmRenderCards(){var c=rmSel.call,dis=!rmValidCall(c)||rmLocked(),B={},K=['radio','ident','pos','queues','mh'],d,i,j,p,h,v,o,x,y,z,e,f,fa=document.activeElement,fid='';if(!rmEl('rm_card_mh'))return;\n"
                       "if(fa&&fa.tagName=='INPUT'&&/^rm_f_/.test(fa.id)){rmIn[fa.id]=fa.value;fid=fa.id;}\n"
                       "for(i=0;i<5;i++){B[K[i]]=rmEl('rm_card_'+K[i]).lastElementChild;B[K[i]].textContent='';}");
    web_client.println("for(j=0;j<2;j++){rmCardHd(B[['ident','queues'][j]],c,[['name','atxt'],['txq','mbox','maxhop']][j],dis||rmMh.on);if(!j)B.ident.appendChild(rmNew('p','font-small','Stored exactly as typed.'));}");
    web_client.println("for(i=0;i<rmDefs.length;i++){d=rmDefs[i];p=rmNew('div','','','rm_def_'+d.c);B[d.card].appendChild(p);h=d.card=='ident'?0:rmNew('div',d.card=='queues'?'rmrow rmsplit':'rmrow rmh');if(h){p.appendChild(h);h.appendChild(rmNew('b','',d.p?'':d.l));}\n");
    web_client.println("v=rmLast(c,d.c,d.x);o=v?(d.p?rmPosKv(d.p,v.reply):d.k?rmKv(v.reply,1):rmKv(v.reply)):null;\n"
                       "if(h&&d.card!='queues'){h.appendChild(d.p&&o?rmNew('span','font-small',d.f[0][1]+': '+d.f[0][3](o[d.f[0][0]]),'rm_v_'+d.c+'_'+d.f[0][0]):rmNew('span','font-small',v?'last read '+rmAgo(v.ago)+' ago':'not read yet'));\n"
                       "rmBtn(h,'Read','',dis,{'data-cmd':d.c,'data-args':''});}");
    web_client.println("if(d.v&&(o||d.a))d.v(p,o||{},v);if(o&&d.f&&!d.p){y=p.appendChild(rmNew('div','rmkv'));\n"
                       "for(j=0;j<d.f.length;j++){f=d.f[j];e=o[f[0]];z=(e===undefined||e=='-')?'not present':(f[3]?f[3](e):e)+f[2];y.appendChild(rmNew('span','font-small',f[1]));y.appendChild(rmNew('span','',z,'rm_v_'+d.c+'_'+f[0]));}}if(o&&d.t)rmTiles(p,d.c,o,d.t);");
    web_client.println("rmWBuild(p,d,o);if(d.n)p.appendChild(rmNew('div','font-small',d.n));\n"
                       "if(d.u&&rmUns(c,d.c,v))p.appendChild(rmNew('div','',d.u));}\n"
                       "y=B.pos.appendChild(rmNew('div','grid rmf'));z=rmKn();rmSwRow(y,'gps',z.sw[0],dis,'enable GPS');rmSwRow(y,'track',z.sw[1],dis,'enable display of SmartBeaconing');rmMhCard(B.mh,dis);if(fid&&rmEl(fid))rmEl(fid).focus();}");
    web_client.println("function rmCell(tr,t){var d=document.createElement('td');d.textContent=String(t);tr.appendChild(d);}\n"
"function rmRenderLog(){var j=rmStat,b,i,r,tr;if(!j)return;\n");
    web_client.println("b=rmEl('rm_log');b.textContent='';for(i=0;i<j.log.length;i++){r=j.log[i];tr=document.createElement('tr');rmCell(tr,rmAgo(r.ago));rmCell(tr,r.src);rmCell(tr,r.ctr);rmCell(tr,r.cmd);rmCell(tr,r.res);b.appendChild(tr);}\n");
    web_client.println("rmTxt('rm_cnt','Executed '+j.ok+', rejected '+j.rej+', counter high-water '+j.hwm+'.');}\n"
                       "function rmRenderOff(){var j=rmStat,f=!!j&&!j.on,e=rmEl('rm_off');e.style.display=f?'':'none';e.textContent=f?'Remote management is off on this node. Switch it on under Settings, Remote Admin.':'';}\n"
                       "function rmRender(){if(!rmShown||!rmEl('rm_page'))return;var k=rmKn(),dis=!rmValidCall(rmSel.call)||rmLocked();\n"
                       "rmRenderNode();rmRenderTiles(k,dis);rmRenderLock();rmRenderMsgs();rmRenderCards();rmRenderLog();rmRenderOff();");
    web_client.println("}\n"
"function rmPre(){var c=rmSel.call,p;if(!rmValidCall(c))return 'Choose a node first.';if(rmSel.slot<0){p=rmPwProblem(rmEl('rm_pw').value);if(p)return 'Type the password of '+c+' first. '+p;}return '';}");
    web_client.println("function rmSendCmd(cmd,args,dc,sl,fc){var call=dc||rmSel.call,ss=dc?sl:rmSel.slot,body,pr=dc||rmQp?'':rmPre(),full=args?cmd+' '+args:cmd;\n"
                       "if(!fc&&(dc?rmRowLocked(dc):rmLocked()))return;rmForce=null;rmTxt('rm_force','');\n"
                       "if(pr){rmMsg(pr,'rmbad');return;}\n");
    web_client.println("body='cmd='+rmEnc(cmd)+'&args='+rmEnc(args);\n"
                       "if(ss>=0)body='slot='+ss+'&'+body;\n"
                       "else{body=(rmQp||'dst='+rmEnc(call)+'&pw='+rmEnc(rmEl('rm_pw').value))+'&'+body;rmEl('rm_pw').value='';}if(!rmQq.length)rmQp='';\n"
                       "if(dc)body+='&call='+rmEnc(dc);if(!dc&&ss>=0)body+='&call='+rmEnc(call);if(fc)body+='&force=1';rmBusy=true;rmMsg('','');rmRender();\n"
                       "rmPost('/rmsend',body).then(rmJson).then(function(j){");
    web_client.println("if(j&&j.ok){rmLock=rmNow()+10000;rmOut=true;rmMsg('','');rmPoll();}\n");
    web_client.println("else{pr=j&&j.err?String(j.err):'send';rmQClr();rmMsg(pr=='range'&&j.range?'Refused: '+call+' accepts '+j.range+' dBm.':rmErrText(j),'rmbad');if(pr=='busy')rmLock=rmNow()+10000;\n"
                       "\n");
    web_client.println("if(pr=='limit'&&j.canForce&&!fc&&ss>=0&&call==rmSel.call){rmForce={c:cmd,a:args,d:call,s:ss};rmForceBtn();}}\n");
    web_client.println("}).catch(function(e){rmQClr();rmCatch(e);}).then(function(){rmBusy=false;rmRender();});}\n"
                       "function rmForceBtn(){var b=rmEl('rm_force');if(!b)return;b.textContent='';if(rmForce&&rmForce.d==rmSel.call)rmBtn(b,'Try once more','',false,{'data-act':'force'});}\n"
                       "function rmTap(b){var cmd=b.getAttribute('data-cmd'),args=b.getAttribute('data-args')||'',cf=b.getAttribute('data-cf')||'',pr=rmPre();if(!args)rmClrIn(cmd);");
    web_client.println("if(rmLocked())return;\n"
                       "if(pr){rmMsg(pr,'rmbad');return;}\n"
                       "if(cf&&!rmConfirm(cf,rmCf(rmLabel((cmd+' '+args).trim()),rmSel.call)))return;\n"
                       "if(cmd=='upd'&&args.indexOf('go ')==0)rmUpdGone[rmSel.call]=rmNow();\n"
                       "rmSendCmd(cmd,args);}\n");
    web_client.println("function rmPick(c){rmNodeChg(c);rmDisarmQuiet();rmSel.call=c;rmSel.slot=rmSlotOf(c);rmEl('rm_call').value=c;rmRender();}\n"
                       "function rmDisarmQuiet(){clearTimeout(rmT.arm);rmT.arm=0;rmArmId='';}\n"
                       "function rmCallInput(){var c=rmEl('rm_call');c.value=c.value.toUpperCase();rmDisarmQuiet();rmNodeChg(c.value.trim());rmSel.call=c.value.trim();rmSel.slot=rmSlotOf(rmSel.call);rmRender();}");
    web_client.println("function rmLoadNodes(){fetch('/rmnodes').then(rmJson).then(function(j){rmSaved=(j&&j.nodes)||[];rmSel.slot=rmSlotOf(rmSel.call);rmRender();}).catch(function(e){rmCatch(e,'rm_net');});}\n"
                       "function rmLoadHeard(){fetch('/rmheard').then(rmJson).then(function(j){rmHeard=(j&&j.heard)||[];rmRender();}).catch(function(e){rmCatch(e,'rm_net');});}");
    web_client.println("function rmNodesPost(body,okText){rmPost('/rmnodes',body).then(rmJson).then(function(j){if(j&&j.ok){rmMsg(okText,'rmok');rmLoadNodes();}else rmMsg(rmErrText(j),'rmbad');}).catch(rmCatch);}\n"
                       "function rmRemember(){var c=rmSel.call,i=rmEl('rm_pw'),pw=i.value,pr=rmPwProblem(pw),s=-1,n;\n"
                       "if(!rmValidCall(c)){rmMsg('Enter a valid call sign first.','rmbad');return;}\n"
                       "if(pr){rmMsg(pr,'rmbad');return;}\n"
                       "for(n=0;n<3;n++)if(!rmSaved[n]||!rmSaved[n].used){s=n;break;}");
    web_client.println("if(s<0){rmMsg('All 3 places on this node are used. Forget one first.','rmbad');return;}\n"
                       "i.value='';rmNodesPost('act=save&slot='+s+'&call='+rmEnc(c)+'&pw='+rmEnc(pw),c+' is saved.');pw='';}\n"
                       "function rmForget(){if(rmSel.slot<0)return;rmNodesPost('act=del&slot='+rmSel.slot,rmSel.call+' is forgotten.');}");
    web_client.println("function rmForgetAll(){if(!rmConfirm('forgetall','Tap Forget all again within 4 seconds to remove every saved node.'))return;rmNodesPost('act=forget','All saved nodes are forgotten.');}");
    web_client.println("function rmQRead(b){var pr=rmPre(),a=b.getAttribute('data-q').split(' '),i;if(rmLocked()||rmMh.on)return;if(pr){rmMsg(pr,'rmbad');return;}for(i=0;i<a.length;i++)rmClrIn(a[i]);rmQp=rmSel.slot<0?'dst='+rmEnc(rmSel.call)+'&pw='+rmEnc(rmEl('rm_pw').value):'';rmQq=a.slice(1);rmSendCmd(a[0],'');}");
    web_client.println("function rmSwChg(i){var n=i.getAttribute('data-sw'),v=i.checked?'on':'off',cf=v=='off'?(i.getAttribute('data-cf')||''):'',pr=rmPre(),pin=n=='pin',c=pin?'setout':n=='ledflash'?'led':n,a=pin?rmEl('rm_pin').value+' '+v:n=='ledflash'?(i.checked?'flash':'off'):v;");
    web_client.println("if(rmLocked()||pr){if(pr&&!rmLocked())rmMsg(pr,'rmbad');rmRender();return;}\n"
                       "if(cf&&!rmConfirm(cf,rmCf(rmLabel(c+' '+a),rmSel.call)))return;\n"
                       "rmSw[rmSel.call+' '+(pin?rmEl('rm_pin').value:n)]=v;rmSendCmd(c,a);}\n");
    web_client.println("function rmSelChg(s){var v=s.value,pr=rmPre();if(!v||rmLocked()||pr){if(pr&&!rmLocked())rmMsg(pr,'rmbad');rmRender();return;}rmSw[rmSel.call+' chgled']=v;rmSendCmd('chgled',v);}\n"
                       "function rmChange(e){var t=e.target;if(t.id=='rm_pin')rmRender();else if(t.getAttribute&&t.getAttribute('data-sel'))rmSelChg(t);else if(t.getAttribute&&t.getAttribute('data-sw'))rmSwChg(t);}\n");
    web_client.println("function rmClick(e){var b=e.target;while(b&&b.tagName!='BUTTON'){if(b.id=='rm_page')return;b=b.parentNode;}\n"
                       "if(!b||b.disabled)return;var a=b.getAttribute('data-act');");
    web_client.println("if(b.hasAttribute('data-cmd'))rmTap(b);\n"
                       "else if(a=='pick')rmPick(b.getAttribute('data-call'));else if(a=='mhgo')rmMhStart();");
    web_client.println("else if(a=='mhstop')rmMhStop();else if(a=='mhdet')rmMhDet(b.getAttribute('data-call'));else if(a=='mhother')rmMhDet(rmEl('rm_f_mh_other').value);\n"
                                              "else if(a=='sync')rmSendCmd('sync','');\n"
                       "else if(a=='force'&&rmForce&&rmForce.d==rmSel.call)rmSendCmd(rmForce.c,rmForce.a,rmForce.d,rmForce.s,true);\n");
    web_client.println("else if(a=='qread')rmQRead(b);\n"
                       "else if(a=='remember')rmRemember();\n"
                       "else if(a=='forget')rmForget();");
    web_client.println("else if(a=='forgetall')rmForgetAll();\n"
                       "else if(a=='heard')rmLoadHeard();}\n"
                       "function rmGot(j){var i,t,c=rmSel.call;j.sent=j.sent||[];j.log=j.log||[];j.targets=j.targets||[];j.t=rmNow();rmStat=j;rmApplySent(j.sent);\n"
                       "rmOut=false;rmSrvLock=0;rmSrvMsg='';rmTL=Object.create(null);\n"
                       "for(i=0;i<j.sent.length;i++)if(j.sent[i].st=='queued'||j.sent[i].st=='waiting')rmOut=true;");
    web_client.println("for(i=0;i<j.targets.length;i++){t=j.targets[i];if(t.pending)rmOut=true;rmTL[t.dst]=t.locked?1e15:t.retry>0?rmNow()+t.retry*1000:0;if(t.dst==c){if(t.retry>0)rmSrvLock=rmNow()+t.retry*1000;if(t.locked)rmSrvMsg='Two tries to '+c+' are still unanswered.';}}\n"
                       "rmMhStep();rmRender();}\n"
                       "function rmNext(){clearTimeout(rmT.poll);rmT.poll=0;if(rmShown&&!rmAuth)rmT.poll=setTimeout(rmPoll,rmOut?3000:10000);}\n"
                       "function rmPoll(){if(!rmShown)return;clearTimeout(rmT.poll);rmT.poll=0;");
    web_client.println("fetch('/rmstatus').then(rmJson).then(function(j){if(!rmShown)return;rmAuth=false;rmTxt('rm_auth','');rmEl('rm_auth').style.display='none';rmTxt('rm_net','');rmGot(j);rmNext();})\n"
                       ".catch(function(e){if(e===0||!rmShown)return;rmTxt('rm_net','No answer from the node, trying again.','rmbad');rmNext();});}");
    web_client.println("function rmTick(){if((typeof cpage!='undefined'&&cpage!='remote')||!rmEl('rm_page')){rmPageLeave();return;}\n"
                       "var l=rmLocked();if(!l&&rmQq.length){if(!rmQp&&rmPre())rmQClr();else rmSendCmd(rmQq.shift(),'');}if(l||rmWasLocked)rmRender();rmWasLocked=l;}");
    web_client.println("function rmPageLeave(){clearInterval(rmT.tick);clearTimeout(rmT.poll);clearTimeout(rmT.arm);rmT.tick=0;rmT.poll=0;rmT.arm=0;rmShown=false;rmArmId='';rmMhReset();rmQClr();}\n"
                       "function rmPageInit(){var i,p,o;rmPageLeave();if(!rmEl('rm_page'))return;rmShown=true;rmUpdT0=rmNow();rmRows=Object.create(null);rmAuth=false;rmBusy=false;rmArmId='';\n"
                       "p=rmEl('rm_pin');for(i=0;i<16;i++){o=document.createElement('option');o.value=(i<8?'a':'b')+(i%8);o.textContent=o.value;p.appendChild(o);}");
    web_client.println("rmEl('rm_page').onclick=rmClick;rmEl('rm_page').onchange=rmChange;rmEl('rm_call').oninput=rmCallInput;rmEl('rm_call').value=rmSel.call;\n"
                       "rmT.tick=setInterval(rmTick,1000);rmRender();rmPoll();rmLoadNodes();rmLoadHeard();}");
}
