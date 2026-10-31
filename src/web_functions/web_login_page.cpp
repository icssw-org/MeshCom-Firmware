/**
 * WEB-AUTH-01: the web login page and the scaffold JS that drives it (wire format: Web-API_documentation.txt, section "Login").
 *
 * Rules the code keeps:
 *  - Plain http has no crypto.subtle, so the JS carries its own compact SHA-256 / HMAC-SHA256. The password never
 *    leaves the browser: login() fetches a nonce, sends HMAC(password, nonce) and the node checks it.
 *  - Every web_client call is a println of ONE string literal of at most 512 bytes (RAK4631: one write is capped
 *    at 2048 B and the rest dropped; nRF52 printf over-reads above 255 B). Adjacent literals count as one.
 *  - The JS carries no comments (they would cost bytes on every page load) and uses single quotes only.
 *  - tools/webgui_login_test.js extracts the literals between the LOGIN-JS / LOGIN-HTML markers and runs them in
 *    jsdom: keep the println string-literal form (only \\ \" \n escapes).
 *  - Every request of the page carries `X-MC: 1` (fetch wrapper of the scaffold), which web_guard.h requires.
 */
#include <Arduino.h>
#include "web_functions.h"
#include "web_rm_util.h" // extern web_client

/** Scaffold JS: mcSha256, mcHmacHex, login, logout, mcLoginFocus. Printed inside the scaffold <script>, no tags here. */
void webLoginScaffoldJs()
{
    // LOGIN-JS-BEGIN
    web_client.println("function mcR(x,n){return x>>>n|x<<32-n;}\n"
                       "function mcSha256(m){var H=[],K=[],w=[],i,j,p=2,n=0,l=m.length,a,b,c,d,e,f,g,h,t,u,s;"
                       "for(;n<64;p++){for(i=2;i*i<=p&&p%i;i++);if(i*i>p){if(n<8)H[n]=Math.pow(p,.5)*4294967296|0;K[n++]=Math.pow(p,1/3)*4294967296|0;}}"
                       "m=m.slice();m.push(128);while(m.length%64!=56)m.push(0);"
                       "m.push(0,0,0,l>>>29,l>>>21&255,l>>>13&255,l>>>5&255,l<<3&255);");
    web_client.println("for(j=0;j<m.length;j+=64){for(i=0;i<64;i++){if(i<16)w[i]=m[j+4*i]<<24|m[j+4*i+1]<<16|m[j+4*i+2]<<8|m[j+4*i+3];"
                       "else{s=w[i-15];u=w[i-2];w[i]=w[i-16]+(mcR(s,7)^mcR(s,18)^s>>>3)+w[i-7]+(mcR(u,17)^mcR(u,19)^u>>>10)|0;}}"
                       "a=H[0];b=H[1];c=H[2];d=H[3];e=H[4];f=H[5];g=H[6];h=H[7];");
    web_client.println("for(i=0;i<64;i++){t=h+(mcR(e,6)^mcR(e,11)^mcR(e,25))+(e&f^~e&g)+K[i]+w[i]|0;"
                       "u=(mcR(a,2)^mcR(a,13)^mcR(a,22))+(a&b^a&c^b&c)|0;h=g;g=f;f=e;e=d+t|0;d=c;c=b;b=a;a=t+u|0;}"
                       "t=[a,b,c,d,e,f,g,h];for(i=0;i<8;i++)H[i]=H[i]+t[i]|0;}"
                       "t=[];for(i=0;i<32;i++)t.push(H[i>>2]>>>24-8*(i&3)&255);return t;}");
    web_client.println("function mcUtf8(s){return Array.from(unescape(encodeURIComponent(s)),function(c){return c.charCodeAt(0);});}\n"
                       "function mcHmacHex(k,s){var i,a=[],b=[],o='';k=mcUtf8(k);if(k.length>64)k=mcSha256(k);"
                       "for(i=0;i<64;i++){a.push((k[i]||0)^54);b.push((k[i]||0)^92);}"
                       "k=mcSha256(b.concat(mcSha256(a.concat(mcUtf8(s)))));"
                       "for(i=0;i<32;i++)o+=(k[i]+256).toString(16).slice(1);return o;}");
    web_client.println("function mcLoginMsg(t){var e=document.getElementById('loginerr');if(e)e.textContent=t;}\n"
                       "function mcLoginErr(r){return r.text().then(function(b){var c='',p=document.getElementById('nodepassword');"
                       "try{c=JSON.parse(b).err;}catch(x){}"
                       "mcLoginMsg(c=='badpw'?'Wrong password':c=='throttled'?'Too many attempts, wait a minute':'Login failed, try again');"
                       "if(p){p.value='';p.focus();}});}");
    web_client.println("function login(pwd){if(!pwd){mcLoginMsg('Enter the password');return;}"
                       "fetch('/?loginnonce').then(function(r){if(r.status!=200)return mcLoginErr(r);"
                       "return r.text().then(function(n){n=n.trim();"
                       "return fetch('/?login='+n+'.'+mcHmacHex(pwd,n)).then(function(r){"
                       "if(r.status==200)window.location.reload(true);else return mcLoginErr(r);});});})"
                       ".catch(function(){mcLoginMsg('Login failed, try again');});}");
    web_client.println("function logout(){var r=function(){window.location.reload(true);};fetch('/?logout').then(r,r);}\n"
                       "function mcLoginFocus(){var p=document.getElementById('nodepassword');if(p)p.focus();}");
    // LOGIN-JS-END
}

/**
 * ###########################################################################################################################
 * delivers a page to enter the nodes web password
 */
void sub_page_login()
{
    _create_meshcom_subheader("Login required");
    // LOGIN-HTML-BEGIN
    web_client.println("<div id=\"content_inner\">");
    web_client.println("<div class=\"cardlayout\">");
    web_client.println("<label class=\"cardlabel\">Enter Password</label>");
    web_client.println("<form class=\"grid\" onsubmit=\"login(document.getElementById('nodepassword').value);return false;\">");
    web_client.println("<span>In order to continue, you need to log in.</span>");
    web_client.println("<input type=\"password\" maxlength=\"20\" size=\"20\" id=\"nodepassword\" name=\"nodepassword\" "
                       "autocomplete=\"current-password\" autofocus>");
    web_client.println("<button type=\"submit\" style=\"justify-self:self-end;\">Login</button>");
    web_client.println("<span id=\"loginerr\" style=\"color:#c0392b;min-height:1.2em;\"></span>");
    web_client.println("</form></div></div>");
    // LOGIN-HTML-END
    web_client.println(); // The HTTP response ends with another blank line
}
