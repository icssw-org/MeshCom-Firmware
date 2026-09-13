/**
 *  @author      Ralph Weich (DD5RW)
 *  @date        2025-12-03
 */
#include <Arduino.h>

#include <configuration.h>
#include <debugconf.h>
#include "web_functions.h"
#include <mheard_functions.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <time.h>
#include <lora_setchip.h>
#include <rtc_functions.h>
#include <time_functions.h>
#include <spectral_scan.h>
#include <maxhop.h>         // CS-02: drop-down values for the text hop limit
#include <config_json.h>   // CS-03: config download/upload as one JSON object
#include <ArduinoJson.h>    // JSN-01: call_function()/setparam()/getparam() JSON escaping
#include <txring_functions.h> // WQ-01: LoRa queue panel -- txRingPrioCounts()
#include <setlog_lines.h>      // WQ-01: LoRa queue panel -- setlogDedupWindowMin()
#include "track_warning.h"    // TRK-01: Warnhinweis-Text neben dem Track-Switch

#include "web_UIComponents.h"
#include "web_setup.h"
#include "web_nodefunctioncalls.h"
#include "web_commonServer.h"


CommonWebServer web_server(80);
CommonWebClient web_client;

void web_client_html(CommonWebClient web_client);


String web_header;

unsigned long web_currentTime = millis(); // Current time
unsigned long web_previousTime = 0;       // Previous time
#define WEB_TIMEOUT_TIME 2000             // Define timeout time in milliseconds (example: 2000ms = 2s)
bool bweb_server_running = false;

// password check
char web_ip[10][20] = {0};
long web_ip_passwd_time[10] = {0};

extern double mheardLat[MAX_MHEARD];
extern double mheardLon[MAX_MHEARD];
extern int mheardAlt[MAX_MHEARD];

double dlat;
double dlon;
char clat;
char clon;

bool bMDNSOK=true;

/**
 * ###########################################################################################################################
 * initialize the Web Server
 */
void startWebserver()
{
    // esp32loop() calls this every pass while iWlanWait == 0; without an IP
    // the debug line came out ~125 times per second (TM-21). Log the state
    // once, again only after it changed.
    static bool s_noIpLogged = false;

    if (bweb_server_running)
        return;
    #ifdef HAS_ETHERNET
        if (meshcom_settings.node_netmode == 0)
        {
            if (strlen(meshcom_settings.node_ip) < 7 && !bWIFIAP)
            {
                if(bDEBUG && !s_noIpLogged)
                {
                    Serial.print("[WEB]...no ip set :");
                    Serial.println(meshcom_settings.node_ip);
                    s_noIpLogged = true;
                }

                stopWebserver();
                return;
            }
        }
    #else
        if (strlen(meshcom_settings.node_ip) < 7 && !bWIFIAP)
        {
            if(bDEBUG && !s_noIpLogged)
            {
                Serial.print("[WEB]...no ip set :");
                Serial.println(meshcom_settings.node_ip);
                s_noIpLogged = true;
            }

            stopWebserver();
            return;
        }
    #endif

    s_noIpLogged = false;               // IP is set now; log again if it goes away

#ifdef ESP32
    // Check if WiFi is actually connected or AP is active before trying to start MDNS
    bool is_connected = (WiFi.status() == WL_CONNECTED);
    bool is_ap_active = (WiFi.getMode() == WIFI_MODE_AP) || (WiFi.getMode() == WIFI_MODE_APSTA);

    #ifdef HAS_ETHERNET
    bool ethernet_mode = (meshcom_settings.node_netmode == 1);
    #else
    bool ethernet_mode = false;
    #endif

    if (!is_connected && !is_ap_active && !ethernet_mode)
    {
        return;
    }

    web_server.stop();

    MDNS.end();

    // Set up mDNS responder:
    // - first argument is the domain name, in this example
    //   the fully-qualified domain name is "esp32.local"
    // - second argument is the IP address to advertise
    //   we send our IP address on the WiFi network
    if(bMDNSOK)
    {
        if (!MDNS.begin(meshcom_settings.node_call))
        {
            Serial.print(getTimeString());
            Serial.println("[Web]...Error setting up MDNS responder!");
            
            stopWebserver();

            bMDNSOK=false;

            return;
        }
        else
        {
            if (bDEBUG)
            {
                Serial.print(getTimeString());
                Serial.println("[Web]...mDNS responder started");
            }
        }
    }

    web_server.begin();


#else
    if (web_server.server_port[1] == 0)
    {
        web_server.begin();
    }
#endif
    bweb_server_running = true;

    if (bDEBUG)
    {
        Serial.print(getTimeString());
        Serial.println("[Web]...WEBServer started");
    }
}

/**
 * ###########################################################################################################################
 * Stop the Web Server
 */
void stopWebserver()
{
#ifdef ESP32
    MDNS.end();
    web_server.stop();
#endif
    bweb_server_running = false;

    if (bDEBUG)
    {
        Serial.print(getTimeString());
        Serial.println("[Web]...WEBServer stopped");
    }
}

/**
 * ###########################################################################################################################
 * loop function for Web Server
 */
void loopWebserver()
{
    if (!bweb_server_running)
        return;

    #ifdef HAS_ETHERNET
    if (meshcom_settings.node_netmode == 0)
    {
        if (strlen(meshcom_settings.node_ip) < 7)
        {
            return;
        }
    }
    #else
    if (strlen(meshcom_settings.node_ip) < 7)
    {
        return;
    }
    #endif

    web_client = web_server.available(); // Create a client connection.

    // HTML Page formating
    if (web_client)
    {
        // If a new client connects,
        web_client_html(web_client);
    }

    // Close the connection
    web_client.stop();
}

/**
 * ###########################################################################################################################
 * Web Client Handler
 */
void web_client_html(CommonWebClient web_client)
{
    IPAddress web_ip_now = web_client.remoteIP();
    char c_web_ip_now[20];
    snprintf(c_web_ip_now, sizeof(c_web_ip_now), "%i.%i.%i.%i", web_ip_now[0], web_ip_now[1], web_ip_now[2], web_ip_now[3]);

    if(bDEBUG) {
        Serial.print("[Web]...Client IP: ");
    if(bDEBUG) {
        Serial.print("[Web]...Client IP: ");
        Serial.println(web_ip_now);
    }
    }
    bool bPasswordOk = false;

    // check password used
    int inext_free = -1;
    int iwebid = -1;

    if (strlen(meshcom_settings.node_webpwd) > 0)
    {
        for (int iwid = 0; iwid < 10; iwid++)
        {
            // check passwort Time expired 4h
            if((uint32_t)(millis() - (uint32_t)web_ip_passwd_time[iwid]) >= (1000U * 60 * 60 * 4))
            {
                web_ip_passwd_time[iwid] = 0;
                memset(web_ip[iwid], 0x00, sizeof(web_ip[iwid]));
            }

            // check timeout
            if (web_ip_passwd_time[iwid] > 0)
            {
                if (bDEBUG)
                    Serial.printf("iwid:%i web_ip[iwid]:%s %s\n", iwid, web_ip[iwid], c_web_ip_now);

                if (is_equ(web_ip[iwid], c_web_ip_now))
                {
                    bPasswordOk = true;
                    web_ip_passwd_time[iwid] = millis();
                    iwebid = iwid;
                }
            }
            else
            {
                if (inext_free < 0)
                    inext_free = iwid;
            }
        }

        if(inext_free < 0)
        {
            Serial.print(getTimeString());
            Serial.printf(" WEBServer not free IP/Password table\n");
        }
        else
        {
            if (!bPasswordOk)
            {
                String strGetPassword = work_webpage(true, inext_free);

                if (is_equ(strGetPassword.c_str(), meshcom_settings.node_webpwd))
                {
                    Serial.print(getTimeString());
                    Serial.printf(" WEBServer Password OK IP:<%s pos:%i>\n", c_web_ip_now, inext_free);

                    snprintf(web_ip[inext_free], sizeof(web_ip[inext_free]), "%s", c_web_ip_now);
                    web_ip_passwd_time[inext_free] = millis();
                    bPasswordOk = true;
                    iwebid = inext_free;
                }
                else
                {
                    Serial.print(getTimeString());
                    Serial.printf(" WEBServer Password not found IP:<%s> show LOGIN\n", c_web_ip_now);
                }
            }
        }
    }
    else
        bPasswordOk = true;
        
    // no connection via password or no password need
    if (!bPasswordOk)
    {
        work_webpage(true, inext_free);
    }
    else
    {
        work_webpage(false, iwebid);
    }
}

/**
 * ###########################################################################################################################
 * CS-03: config download / upload
 *
 * One buffer serves both directions -- the export never runs while an upload
 * body is being read. The worst-case export (every string field filled to its
 * limit) measures about 3.1 kB, so CONFIG_JSON_MAX is roughly a factor of two
 * of headroom and at the same time the hard cap on what an upload may send.
 *
 * On the heap, NOT in BSS: 6 kB of static buffer overflows dram0_0_seg on the
 * plain ESP32 (E22-DevKitC links with ~4 kB to spare). It is allocated for the
 * duration of one /config.json or POST /config request and released again --
 * both are rare, operator-triggered requests.
 */
static char *web_cfg_buf = NULL;

static bool web_cfg_alloc(void)
{
    if (web_cfg_buf == NULL)
        web_cfg_buf = (char *)malloc(CONFIG_JSON_MAX + 1);

    if (web_cfg_buf != NULL)
        web_cfg_buf[0] = '\0';

    return web_cfg_buf != NULL;
}

static void web_cfg_free(void)
{
    if (web_cfg_buf != NULL)
    {
        free(web_cfg_buf);
        web_cfg_buf = NULL;
    }
}

/**
 * Reads the request BODY.
 *
 * work_webpage() has only ever read the request HEADER -- it stops at the
 * blank line and answers. A file upload is the first thing here that carries
 * a body, so this is the reader for it: bounded by the Content-Length the
 * caller picked out of the header, hard-capped at CONFIG_JSON_MAX, and given
 * up on after WEB_TIMEOUT_TIME without a byte so a lying Content-Length
 * cannot pin the loop.
 *
 * @return bytes read into web_cfg_buf (NUL-terminated), or negative:
 *         -1 no/!invalid Content-Length, -2 too large, -3 short read.
 */
static int web_read_body(long content_length)
{
    web_cfg_buf[0] = '\0';

    if (content_length <= 0)
        return -1;

    if (content_length > (long)CONFIG_JSON_MAX)
        return -2;

    long          got = 0;
    unsigned long last = millis();

    while (got < content_length && (millis() - last) <= WEB_TIMEOUT_TIME)
    {
        yield();

        if (web_client.available())
        {
            int c = web_client.read();
            if (c < 0)
                break;

            web_cfg_buf[got++] = (char)c;
            last = millis();
        }
        else if (!web_client.connected())
        {
            break;
        }
    }

    web_cfg_buf[got] = '\0';

    return (got == content_length) ? (int)got : -3;
}

/**
 * GET /config.json -- the whole configuration as a downloadable file.
 *
 * Sends its own header instead of send_http_header(): only this response
 * carries a Content-Disposition, which is what turns a browser navigation
 * into a download instead of a page full of JSON.
 */
static void sub_config_download(void)
{
    if (!web_cfg_alloc())
    {
        send_http_header(422, RESPONSE_TYPE_JSON);
        web_client.println("{\"error\":\"out of memory\"}");
        return;
    }

    size_t n = configExportJson(web_cfg_buf, CONFIG_JSON_MAX);

    if (n == 0)
    {
        web_cfg_free();
        send_http_header(422, RESPONSE_TYPE_JSON);
        web_client.println("{\"error\":\"config export did not fit the buffer\"}");
        return;
    }

    /* the callsign goes into a filename -- keep it to characters that cannot
     * break out of the quoted header value or of a directory */
    char fname[16];
    size_t fi = 0;
    for (const char *p = meshcom_settings.node_call; *p && fi < sizeof(fname) - 1; p++)
    {
        char c = *p;
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '-')
            fname[fi++] = c;
    }
    fname[fi] = '\0';
    if (fi == 0)
        snprintf(fname, sizeof(fname), "node");

    web_client.printf("HTTP/1.1 200 OK \n");
    web_client.println("Content-type:application/json");
    web_client.printf("Content-Disposition: attachment; filename=\"meshcom-%s.json\"\n", fname);
    web_client.printf("Content-Length: %u\n", (unsigned int)n);
    web_client.println("Access-Control-Allow-Origin: *");
    web_client.println("Connection: close");
    web_client.println("Cache-Control: no-cache, no-store, must-revalidate");
    web_client.println("Pragma: no-cache");
    web_client.println("Expires: 0");
    web_client.println();

    web_client.print(web_cfg_buf);

    web_cfg_free();
}

/**
 * POST /config -- restore a configuration file and reboot once.
 *
 * Nothing is written unless configImportJson() accepted every value; on
 * failure the answer is a 400 with the reason and NVRAM is untouched. On
 * success the response is flushed and the connection closed BEFORE the reset,
 * otherwise the browser never sees the answer. Reboot pattern copied from
 * commandAction()'s --cleanflash/--reboot (command_functions.cpp).
 */
static void sub_config_upload(long content_length)
{
    char err[160] = {0};

    if (!web_cfg_alloc())
    {
        send_http_header(400, RESPONSE_TYPE_TEXT);
        web_client.printf("<p>upload rejected: out of memory</p>");
        return;
    }

    int n = web_read_body(content_length);

    if (n < 0)
    {
        web_cfg_free();
        send_http_header(400, RESPONSE_TYPE_TEXT);
        web_client.printf("<p>upload rejected: %s</p>",
                          (n == -2) ? "file too large" : (n == -1) ? "no Content-Length" : "incomplete upload");
        /* own marker: these codes are the body reader's, not configImportJson()'s */
        Serial.printf("[CONFIG];upload;rc;%d;len;%ld\n", n, content_length);
        return;
    }

    int rc = configImportJson(web_cfg_buf, (size_t)n, err, sizeof(err));

    web_cfg_free();

    if (rc != 0)
    {
        send_http_header(400, RESPONSE_TYPE_TEXT);
        web_client.printf("<p>config import failed: %s</p>", err);
        return;
    }

    save_settings();

    send_http_header(200, RESPONSE_TYPE_TEXT);
    web_client.printf("<html><body><h3>config imported</h3><p>%s</p><p>the node reboots now.</p></body></html>", err);
    web_client.flush();
    web_client.stop();

    Serial.printf("[CONFIG];reboot;after;import\n");

    delay(2000);

    #ifdef ESP32
        ESP.restart();
    #endif

    #if defined NRF52_SERIES
        NVIC_SystemReset();     // resets the device
    #endif
}

/**
 * ###########################################################################################################################
 * Handle Web requests and call the matching sub function
 */
String work_webpage(bool bget_password, int webid)
{
    static char web_header_collect[1024];        // BSS statt Heap
    static uint16_t web_header_collect_len = 0;
    // ...
    web_header_collect_len = 0;
    web_header_collect[0] = '\0';

    web_currentTime = millis();
    web_previousTime = web_currentTime;
    String password_message = "";
    String web_currentLine = ""; // make a String to hold incoming data from the client

    // CS-03: an upload needs its body length, and the header may well be
    // longer than web_header_collect[]. Picked off the line as it completes,
    // so it does not depend on that 1 kB window.
    long web_content_length = -1;

    if (bDEBUG)
    {
        #ifdef ESP
        Serial.printf("%s;[HEAP]Cnew;%d;(free)\n", getTimeString().c_str(), ESP.getFreeHeap());
        #endif
        Serial.println("New Client."); // print a message out in the serial port
    }
       
    while (web_client.connected() && (web_currentTime - web_previousTime) <= WEB_TIMEOUT_TIME)
    { // loop while the client's connected
        yield();
        web_currentTime = millis();
        if (web_client.available())
        {
            // if there's bytes to read from the client,
            char c = web_client.read(); // read a byte, then

            if (bDEBUG)
                Serial.write(c); // print it out the serial monitor

            if (web_header_collect_len < sizeof(web_header_collect) - 1)
            {
                web_header_collect[web_header_collect_len++] = c;
                web_header_collect[web_header_collect_len] = '\0';
            }

            if (c == '\n')
            {
                // if the byte is a newline character
                // if the current line is blank, you got two newline characters in a row.
                // that's the end of the client HTTP request, so send a response:
                if (web_currentLine.length() == 0)
                {
                    web_header = web_header_collect;
                    
                    // Serial.println(web_header);

                    // user sends authentication
                    if (web_header.indexOf("/?nodepassword") >= 0)
                    {
                        web_header = web_header.substring(web_header.indexOf("/?nodepassword=") + 15, web_header.indexOf("HTTP/1.1"));
                        web_header.trim();
                        if (web_header.length() == 0)
                        {
                            web_ip_passwd_time[webid] = 0; // logging out
                            if (bDEBUG)
                                Serial.println("WebUI requested logout");
                        }
                        else
                        {
                            password_message = decodeURLPercentCoding(web_header); // try logging in using password
                            if (bDEBUG)
                                Serial.println("WebUI requested login");
                        }
                        send_http_header(200, RESPONSE_TYPE_TEXT);
                    }

                    // in every other case, we check if authentication is required but not yet provided
                    if (bget_password)
                    {
                        // user requested a page ? Send a login page instead.
                        if (web_header.indexOf("/?page=") >= 0)
                        {
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_login();
                        }
                        // user requested anything else (but not just "/")? Send a 401 - Unauthorized as answear
                        else if (web_header.indexOf("/") >= 0 && web_header.indexOf("HTTP/1.1") > web_header.indexOf("/") + 2)
                        {
                            send_http_header(401, RESPONSE_TYPE_TEXT);
                        }
                        else
                        {
                            deliver_scaffold(true);
                        }
                    }
                    else
                    {

                        if (web_header.indexOf("/callfunction/") >= 0)
                        { // user requested to invoke a function
                            // ### !!function will generate a HTML header itself
                            call_function(web_header);
                        }
                        else if (web_header.indexOf("/?sendmessage") >= 0)
                        { // user requested to send a message to the mesh
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            send_message(web_header);
                        }
                        else if (web_header.indexOf("/?getmessages") >= 0)
                        { // user requested to retrieve the stored messages
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_content_messages();
                        }
                        else if (web_header.indexOf("/setparam/") >= 0)
                        { // user requested to set a parameter
                            // ### !!function will generate a HTML header itself
                            setparam(web_header);
                        }
                        else if (web_header.indexOf("/getparam/") >= 0)
                        { // user requested to get a parameter
                            // ### !!function will generate a HTML header itself
                            getparam(web_header);
                        }
                        else if (web_header.indexOf("/config.json") >= 0)
                        { // CS-03: download the configuration as a file
                            // ### !!function will generate a HTML header itself
                            sub_config_download();
                        }
                        else if (web_header.indexOf("POST /config") >= 0)
                        { // CS-03: restore a configuration file, then reboot
                            // ### !!function will generate a HTML header itself
                            sub_config_upload(web_content_length);
                        }
                        else if (web_header.indexOf("/?page=setup") >= 0)
                        { // user requested the position page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_setup();
                        }
                        else if (web_header.indexOf("/?page=position") >= 0)
                        { // user requested the position page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_position();
                        }
                        else if (web_header.indexOf("/?page=wx") >= 0)
                        { // user requested the weather page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_wx();
                        }
                        else if (web_header.indexOf("/?page=mheard") >= 0)
                        { // user requested the mheard page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_mheard();
                        }
                        else if (web_header.indexOf("/?page=messages") >= 0)
                        { // user requested the messages page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_messages();
                        }
                        else if (web_header.indexOf("/?page=rxlog") >= 0)
                        { // user requested the rx log page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_rxlog();
                        }
                        else if (web_header.indexOf("/?page=path") >= 0)
                        { // user requested the path page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_path();
                        }
                        else if (web_header.indexOf("/?page=spectrum") >= 0)
                        { // user requested the path page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_spectrum();
                        }
                        else if (web_header.indexOf("/?page=mcp23017") >= 0)
                        { // user requested the path page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_mcp23017();
                        }
                        else if (web_header.indexOf("/?page=mcp23017") >= 0)
                        { // user requested the path page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_mcp23017();
                        }
                        else if (web_header.indexOf("/?page=info") >= 0)
                        { // user requested the info page
                            send_http_header(200, RESPONSE_TYPE_TEXT);
                            sub_page_info();
                        }
                        else if (web_header.indexOf("/?page=") >= 0)
                        { // user requested a page we do not know
                            send_http_header(404, RESPONSE_TYPE_TEXT);
                            sub_page_unknown();
                        }
                        else
                        {
                            deliver_scaffold(bget_password);
                        }
                    } // if (bget_password)

                    web_client.stop();
                }
                else
                { // if you got a newline, then clear currentLine
                    if (web_content_length < 0)
                    {
                        String hdr_name = web_currentLine;
                        hdr_name.toLowerCase();
                        if (hdr_name.startsWith("content-length:"))
                            web_content_length = web_currentLine.substring(15).toInt();
                    }

                    web_currentLine = "";
                }
            }
            else if (c != '\r')
            {                         // if you got anything else but a carriage return character,
                web_currentLine += c; // add it to the end of the currentLine
            }
        }
    }

    if (bDEBUG)
    {
        Serial.println("Client disconnected.");
        #ifdef ESP
        Serial.printf("%s;[HEAP]Cdis;%d;(free)\n", getTimeString().c_str(), ESP.getFreeHeap());
        Serial.println("");
        #endif
    }

    return password_message;
}

/**
 * ###########################################################################################################################
 * decodes Percent-Coded Strings (e.g. URLs and its parameters)
 */
String decodeURLPercentCoding(String input)
{
    input.replace("+", " ");

    input.replace("%C2%A3", "£");
    input.replace("%C2%B0", "°");

    input.replace("%C3%A4", "ä");
    input.replace("%C3%B6", "ö");
    input.replace("%C3%BC", "ü");

    input.replace("%C3%84", "Ä");
    input.replace("%C3%96", "Ö");
    input.replace("%C3%9C", "Ü");

    input.replace("%C3%B2", "ò");
    input.replace("%C3%A0", "à");
    input.replace("%C3%B9", "ù");
    input.replace("%C3%A8", "è");
    input.replace("%C3%A9", "é");

    input.replace("%C3%9F", "ß");

    input.replace("%0D%0A", "-");

    input.replace("%21", "!");
    input.replace("%23", "#");
    input.replace("%24", "$");
    input.replace("%25", "%");
    input.replace("%26", "&");
    input.replace("%27", "'");
    input.replace("%28", "(");
    input.replace("%29", ")");
    input.replace("%2A", "*");
    input.replace("%2B", "+");
    input.replace("%2C", ",");
    input.replace("%2F", "/");
    input.replace("%3A", ":");
    input.replace("%3B", ";");
    input.replace("%3D", "=");
    input.replace("%3F", "?");
    input.replace("%40", "@");
    input.replace("%5B", "[");
    input.replace("%5D", "]");

    input.replace("%20", " ");
    input.replace("%22", ""
                         "");
    input.replace("%2D", "-");
    input.replace("%2E", ".");
    input.replace("%3C", "<");
    input.replace("%3E", ">");
    input.replace("%5C", "\\");
    input.replace("%5E", "^");
    input.replace("%5F", "_");
    input.replace("%60", "`");
    input.replace("%7B", "{");
    input.replace("%7C", "|");
    input.replace("%7D", "}");
    input.replace("%7E", "~");

    input.replace("%09", "");

    return input;
}

/**
 * ###########################################################################################################################
 * delivers the WebUI scaffold including the info page
 */
void deliver_scaffold(bool bget_password)
{
    send_http_header(200, RESPONSE_TYPE_TEXT);

    // Serial.println("Sending Scaffold");
    web_client.println("<!DOCTYPE html>\n<html>\n<head>\n");
    web_client.println("<meta charset=\"utf-8\">\n");
    web_client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n");
    web_client.printf("<title>%s - Meshcom</title>\n",  meshcom_settings.node_call);

    // ECMA-Script/Javascript
    web_client.println("<script type=\"text/javascript\">\n");
    // these variables will hold the last loaded page name and sender in order to force a refresh
    web_client.println("cpage=\"info\";csender=undefined;\nsetInterval(autorefresh,10000);");
    // This function will be called in intervalls - can be used to auto-refresh content depending on what page is loaded
    web_client.println("function autorefresh() {if(cpage=='messages')updateMessages();if(cpage=='wx')loadPage('wx',csender,false);if(cpage=='position')loadPage('position',csender,false);if(cpage=='mheard')loadPage('mheard',csender,false);if(cpage=='path')loadPage('path',csender,false);if(cpage=='rxlog')loadPage('rxlog',csender,false);};");
    // this function is used for login and logout
    web_client.println("function login(pwd){var xhttp = new XMLHttpRequest(); xhttp.onreadystatechange=function(){if(this.readyState==4 && this.status==200){window.location.reload(true);}};xhttp.open(\"GET\",\"?nodepassword=\"+pwd,true);xhttp.send();}\n");
    // this function is used to load content depending on the navigation button pressed
    web_client.println("function loadPage(page,sender,useSpinner) {cpage=page;csender=sender;if(useSpinner){document.getElementById(\"content_layer\").innerHTML=\"<span class=\\\"loader\\\"></span>\"};var xhttp = new XMLHttpRequest(); xhttp.onreadystatechange=function(){if(this.readyState==4 && this.status==200){document.getElementById(\"content_layer\").innerHTML=this.responseText;mcRenderQueue();if(page=='messages' && document.getElementById('messages_panel'))mcRenderHistory();}};xhttp.open(\"GET\",\"?page=\"+page,true);xhttp.send();Array.from(document.querySelectorAll('.nav_button.nbactive ')).forEach((el) => el.classList.remove('nbactive')); sender.classList.add('nbactive');}\n");
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
    web_client.println("function sendMessage() {var xhttp=new XMLHttpRequest();xhttp.onreadystatechange=function(){if(this.readyState==4 && this.status==200 && this.responseText.indexOf(\"sendmessage ok\")>=0){var sc=document.getElementById(\"sendcall\");if(!/^[0-9]+$/.test(sc.value))sc.value=\"\"; document.getElementById(\"messagetext\").value=\"\"; updateCharsLeft();}};xhttp.open(\"GET\",\"/?sendmessage&tocall=\"+encodeURIComponent(document.getElementById(\"sendcall\").value)+\"&message=\"+encodeURIComponent(document.getElementById(\"messagetext\").value),true);xhttp.send();}\n");
    // this functions is counting and displaying the amount of chars left that the user can use to write a message
    web_client.println("function updateCharsLeft() {let maxlength=149;if(document.getElementById(\"sendcall\").value.length>0) {maxlength-=(document.getElementById(\"sendcall\").value.length)+2;}let msglength=document.getElementById(\"messagetext\").value.length;if(msglength>maxlength){document.getElementById(\"messagetext\").value=document.getElementById(\"messagetext\").value.substring(0,maxlength);msglength=maxlength;}document.getElementById(\"indicator_charsleft\").innerHTML=maxlength-msglength;}\n");
    // MC-msg-history: BLEtoPhoneBuff/MAX_RING is only 20 slots and is shared
    // with positions and acks, so a handful of new messages can push an old
    // message out of the node's own ring within minutes. The browser tab
    // keeps every message it has seen for the life of the page in
    // mcHistory/mcSeen (capped at MC_HIST_MAX, oldest dropped first) so
    // switching Info -> Messages -> Info -> Messages does not lose messages
    // that scrolled out of the node's ring. This is browser memory only --
    // node RAM is unchanged, and persisting the history to localStorage is
    // deferred (not implemented in this round).
    web_client.println("var mcSeen={};");
    web_client.println("var mcHistory=[];");
    web_client.println("var MC_HIST_MAX=200;");
    // MC-msg-tabs: this and the rest of the mcTab* functions live in the
    // scaffold rather than sub_page_messages() because sub-pages are
    // injected via innerHTML and their own <script> tags never run.
    web_client.println("var mcTabSel='all';");
    web_client.println("try{var mcTabStored=localStorage.getItem('mcTab');if(mcTabStored!=null)mcTabSel=mcTabStored;}catch(e){}");
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
    web_client.println("var mcReadIds={};");
    web_client.println("var mcWm={};");
    web_client.println("try{var mcWmStored=localStorage.getItem('mcWm');if(mcWmStored!=null)mcWm=JSON.parse(mcWmStored);}catch(e){}if(typeof mcWm!=='object' || mcWm===null)mcWm={};");
    web_client.println("function mcSaveWm(){try{localStorage.setItem('mcWm',JSON.stringify(mcWm));}catch(e){}}");
    web_client.println("function mcTabMatchKey(key,dst){if(key=='all')return true;if(key=='*')return dst=='*';if(key=='dm')return dst!='*' && !/^[0-9]+$/.test(dst);return dst==key;}");
    web_client.println("function mcTabMatch(dst){return mcTabMatchKey(mcTabSel,dst);}");
    web_client.println("function mcUnreadCount(key){var wm=mcWm[key]||0;var n=0;for(var i=0;i<mcHistory.length;i++){var m=mcHistory[i];if(m.rx && mcTabMatchKey(key,m.dst) && !mcReadIds[m.id] && m.ts>wm)n++;}return n;}");
    // seeds the watermark of any tab key that has never had one (first visit
    // to this browser, or a group number just added in setup) to the newest
    // ts already known, so first load never shows a wall of unread badges
    web_client.println("function mcSeedWm(){var btns=document.querySelectorAll('#mctabs .mctab');if(btns.length==0)return;var maxTs=0;for(var i=0;i<mcHistory.length;i++){if(mcHistory[i].ts>maxTs)maxTs=mcHistory[i].ts;}var changed=false;for(var j=0;j<btns.length;j++){var key=btns[j].getAttribute('data-tab');if(!(key in mcWm)){mcWm[key]=maxTs;changed=true;}}if(changed)mcSaveWm();}");
    // marks every history entry on the currently selected tab as read; viewing
    // 'all' counts as having read every tab, so its watermark propagates to
    // every configured tab key, not only 'all' itself
    web_client.println("function mcMarkRead(){if(document.visibilityState==='hidden')return;var maxTs=0;var changed=false;for(var i=0;i<mcHistory.length;i++){var m=mcHistory[i];if(mcTabMatchKey(mcTabSel,m.dst)){if(!mcReadIds[m.id]){mcReadIds[m.id]=true;changed=true;}if(m.ts>maxTs)maxTs=m.ts;}}if(maxTs>(mcWm[mcTabSel]||0)){mcWm[mcTabSel]=maxTs;changed=true;}if(mcTabSel=='all'){var btns=document.querySelectorAll('#mctabs .mctab');for(var j=0;j<btns.length;j++){var key=btns[j].getAttribute('data-tab');if(maxTs>(mcWm[key]||0)){mcWm[key]=maxTs;changed=true;}}}if(changed)mcSaveWm();}");
    // the tab bar is re-rendered by the server on every loadPage('messages'),
    // so badges are painted here on every call, never once at page load
    web_client.println("function mcApplyTab(){var panel=document.getElementById('messages_panel');if(!panel)return;var els=panel.querySelectorAll('.message[data-dst]');for(var i=0;i<els.length;i++){els[i].hidden=!mcTabMatch(els[i].getAttribute('data-dst'));}mcSeedWm();mcMarkRead();var btns=document.querySelectorAll('#mctabs .mctab');for(var j=0;j<btns.length;j++){var key=btns[j].getAttribute('data-tab');btns[j].classList.toggle('mctab-on',key==mcTabSel);var label=key=='all'?'All':(key=='dm'?'DM':key);var cnt=mcUnreadCount(key);btns[j].innerHTML=label+(cnt>0?' <span class=\"mcbadge\">'+cnt+'</span>':'');btns[j].classList.toggle('mctab-new',cnt>0);}}");
    web_client.println("function mcTab(btn){mcTabSel=btn.getAttribute('data-tab');try{localStorage.setItem('mcTab',mcTabSel);}catch(e){}var sc=document.getElementById('sendcall');if(sc){if(/^[0-9]+$/.test(mcTabSel))sc.value=mcTabSel;else if(mcTabSel=='*')sc.value='';}if(typeof updateCharsLeft==='function' && sc)updateCharsLeft();mcApplyTab();}");
    // messages that arrive while the tab is hidden must not count as read
    // until the operator actually comes back to look at them
    web_client.println("document.addEventListener('visibilitychange',function(){if(cpage=='messages')mcApplyTab();});");
    web_client.println("function mcHistIndex(id){for(var i=0;i<mcHistory.length;i++){if(mcHistory[i].id==id)return i;}return -1;}");
    web_client.println("function mcMergeEntry(el){var id=el.getAttribute('data-id');var html=el.outerHTML;var dst=el.getAttribute('data-dst')||'';var ts=parseInt(el.getAttribute('data-ts'))||0;var rx=el.classList.contains('message-received');var idx=mcHistIndex(id);if(idx<0){mcHistory.push({id:id,html:html,dst:dst,ts:ts,rx:rx});mcSeen[id]=true;if(mcHistory.length>MC_HIST_MAX){var dropped=mcHistory.shift();delete mcSeen[dropped.id];}}else{mcHistory[idx].html=html;mcHistory[idx].dst=dst;mcHistory[idx].ts=ts;mcHistory[idx].rx=rx;}}");
    web_client.println("function mcRemovePlaceholder(panel){var kids=panel.children;for(var i=kids.length-1;i>=0;i--){if(kids[i].tagName=='P')panel.removeChild(kids[i]);}}");
    // this function is an ayncronous loader that is used to update the received messages without re-loading the whole page, it will re-call itself after a timeout as long as the message-page is displayed
    // it merges the response into mcHistory/mcSeen instead of overwriting the panel outright, so a message already on screen keeps its DOM position when only its ack mark changed
    web_client.println("function mcProcessMessages(text,panel){var tmpl=document.createElement('template');tmpl.innerHTML=text;var els=tmpl.content.querySelectorAll('.message[data-id]');for(var i=0;i<els.length;i++){var el=els[i];var id=el.getAttribute('data-id');var existed=mcSeen.hasOwnProperty(id);mcMergeEntry(el);if(existed){var old=panel.querySelector('.message[data-id=\"'+id+'\"]');if(old)old.replaceWith(el);}else{panel.appendChild(el);}}mcRemovePlaceholder(panel);if(mcHistory.length==0)panel.innerHTML='<p>No messages available.</p>';if(typeof mcApplyTab==='function')mcApplyTab();}");
    web_client.println("function updateMessages() {var xhttp=new XMLHttpRequest();xhttp.onreadystatechange=function(){if(this.readyState==4 && this.status==200){var panel=document.getElementById('messages_panel');if(panel!=null)mcProcessMessages(decodeURIComponent(this.responseText),panel);}};setTimeout(function(){xhttp.open('GET','/?getmessages',true);xhttp.send();},1000);}\n");
    // rebuilds #messages_panel from mcHistory when the messages page is (re-)injected by loadPage(); first merges the server-rendered entries already sitting in the panel into mcHistory (same dedupe as mcProcessMessages) so nothing the server just sent is lost, then renders the full remembered history in order
    web_client.println("function mcRenderHistory(){var panel=document.getElementById('messages_panel');if(!panel)return;var els=panel.querySelectorAll('.message[data-id]');for(var i=0;i<els.length;i++){mcMergeEntry(els[i]);}var html='';for(var j=0;j<mcHistory.length;j++){html+=mcHistory[j].html;}panel.innerHTML=html.length>0?html:'<p>No messages available.</p>';if(typeof mcApplyTab==='function')mcApplyTab();}");
    //  this function sends a parameter:value request to the backend
    // TRK-01: bei Erfolg (returncode==0, die Seite wird hier NICHT neu geladen) den Warnhinweis
    // "<id>_warn" live ein-/ausblenden -- generisch ueber param, damit kuenftige Switches denselben Mechanismus erben
    web_client.println("function setvalue(param,value,refresh) {fetch(\"/setparam/?\"+param+\"=\"+encodeURIComponent(value)).then(function(response){return response.json();}).then(function(jsonResponse){if(jsonResponse['returncode']==1)alert(\"Value could not be set.\");if(jsonResponse['returncode']==2)alert(\"Parameter unknown to node.\");if(jsonResponse['returncode']==0){var w=document.getElementById(param+\"_warn\");if(w)w.style.display=(value==\"on\")?\"\":\"none\";}if(jsonResponse['returncode']>0){loadPage(cpage,csender,false)}if(refresh)loadPage(cpage,csender,false);});}\n");
    // this function invokes a function call to the backend passing the function name and an optional parameter (e.g. sendpos)
    web_client.println("function callfunction(functionname,functionparameter){fetch(\"/callfunction/?\"+functionname+\"=\"+functionparameter).then(function(response){return response.json();}).then(function (jsonResponse) {/*Nothing todo yet.*/})}\n");
    // CS-03: config restore. Lives here and not in the setup page, because the
    // sub-pages are injected with innerHTML -- a <script> inside them never runs.
    web_client.println("function uploadconfig(){var e=document.getElementById(\"cfgfile\");if(!e||e.files.length==0){alert(\"choose a config file first\");return;}if(!confirm(\"Restore this configuration and reboot the node?\"))return;var r=new FileReader();r.onload=function(){fetch(\"/config\",{method:\"POST\",headers:{\"Content-Type\":\"application/json\"},body:r.result}).then(function(x){return x.text().then(function(t){var m=t.replace(/<[^>]*>/g,\" \").trim();if(x.ok){alert(\"config imported: \"+m);}else{alert(\"import failed: \"+m);}});}).catch(function(err){alert(\"upload failed: \"+err);});};r.readAsText(e.files[0]);}\n");

    // This function is used to toggle a css class so setup cars can collapse / expand
    web_client.println("function togglecard(element){element.parentElement.classList.toggle(\"cardopen\");}");

    // WQ-01: LoRa Queue panel on the rxlog page. rxlog is fetched by loadPage()
    // and injected with innerHTML, which never runs a <script> tag it carries,
    // so the rendering logic has to live here in the scaffold instead and be
    // invoked from loadPage() after each fragment swap (that covers both the
    // initial page load and the 10s autorefresh). The fragment itself only
    // emits an empty #mcq div carrying data-* attributes; all markup below is
    // built from those. JS strings use single quotes and HTML attribute
    // values are written unquoted (none of them ever contain a space) so
    // nothing here needs quote-escaping in the C string literals.
    web_client.println("var mcQueueOpen=true;");
    web_client.println("function mcQueueToggle(btn){mcQueueOpen=!mcQueueOpen;var b=document.getElementById('mcq');if(b)b.hidden=!mcQueueOpen;if(btn)btn.textContent=mcQueueOpen?'hide':'show';}");
    web_client.println("function mcRenderQueue(){");
    web_client.println("var d=document.getElementById('mcq');");
    web_client.println("if(!d)return;");
    web_client.println("var p=[0,0,0,0,0,0];");
    web_client.println("for(var i=1;i<=5;i++){p[i]=parseInt(d.getAttribute('data-p'+i))||0;}");
    web_client.println("var ring=parseInt(d.getAttribute('data-ring'))||0;");
    web_client.println("var used=parseInt(d.getAttribute('data-p0'))||0;");
    web_client.println("var bp=parseInt(d.getAttribute('data-bp'))||0;");
    web_client.println("var bpname=d.getAttribute('data-bpname')||'';");
    web_client.println("var qrs=parseInt(d.getAttribute('data-qrs'))||0;");
    web_client.println("var qrsf=parseInt(d.getAttribute('data-qrsf'))||qrs;");
    web_client.println("var qrt=parseInt(d.getAttribute('data-qrt'))||0;");
    web_client.println("var win=parseInt(d.getAttribute('data-win'))||0;");
    web_client.println("var rx=parseInt(d.getAttribute('data-rx'))||0;");
    web_client.println("var tx=parseInt(d.getAttribute('data-tx'))||0;");
    web_client.println("var intv=parseInt(d.getAttribute('data-int'))||300;");
    web_client.println("var newid=parseInt(d.getAttribute('data-newid'))||0;");
    web_client.println("var dup=parseInt(d.getAttribute('data-dup'))||0;");
    web_client.println("var dwin=parseInt(d.getAttribute('data-dwin'))||0;");
    web_client.println("var age=parseInt(d.getAttribute('data-age'))||0;");
    web_client.println("var empty=ring-used;");
    web_client.println("if(empty<0)empty=0;");
    web_client.println("var qrsPct=ring>0?(qrs/ring*100):0;");
    web_client.println("var qrsfPct=ring>0?(qrsf/ring*100):0;");
    web_client.println("var qrtPct=ring>0?(qrt/ring*100):0;");
    web_client.println("var colors=['#A2182F','#E07B39','#3B7DD8','#6FA96F','#9E9E9E'];");
    web_client.println("var names=['crit','high','normal','low','bg'];");
    web_client.println("var html='';");
    web_client.println("html+='<div class=font-bold>TX ring</div>';");
    web_client.println("html+='<div class=mcq-barwrap><div class=mcq-bar>';");
    web_client.println("for(var pr=1;pr<=5;pr++){for(var c=0;c<p[pr];c++){html+='<div class=mcq-cell style=background:'+colors[pr-1]+'></div>';}}");
    web_client.println("for(var c2=0;c2<empty;c2++){html+='<div class=mcq-cell-empty></div>';}");
    web_client.println("html+='</div>';");
    web_client.println("html+='<div class=mcq-tick-faint style=left:'+qrsPct+'%></div>';");
    web_client.println("html+='<div class=mcq-tick style=left:'+qrsfPct+'%></div>';");
    web_client.println("html+='<div class=mcq-tick style=left:'+qrtPct+'%></div>';");
    web_client.println("html+='</div>';");
    web_client.println("html+='<div class=mcq-ticklabels>';");
    web_client.println("html+='<span class=font-small style=position:absolute;left:'+qrsfPct+'%;transform:translateX(-50%)>QRS</span>';");
    web_client.println("html+='<span class=font-small style=position:absolute;left:'+qrtPct+'%;transform:translateX(-50%)>QRT</span>';");
    web_client.println("html+='</div>';");
    web_client.println("html+='<div class=mcq-legend>'+used+'/'+ring+' queued&nbsp;|&nbsp;';");
    web_client.println("for(var pr2=1;pr2<=5;pr2++){html+='<span class=mcq-swatch style=background:'+colors[pr2-1]+'></span>'+names[pr2-1]+' '+p[pr2]+' ';}");
    web_client.println("html+='</div>';");
    web_client.println("var bpcolor=(bp==0)?'#3B9E4F':((bp==1)?'#E07B39':'#A2182F');");
    web_client.println("html+='<div class=font-bold>Back-pressure: <span style=color:'+bpcolor+'>'+bpname+'</span></div>';");
    web_client.println("html+='<div class=font-small>(QRS forecast at&nbsp;'+qrsf+' for your next msgs, line&nbsp;&ge;'+qrs+', QRT at&nbsp;&ge;'+qrt+' of '+ring+')</div>';");
    web_client.println("if(win==0){");
    web_client.println("html+='<div class=font-bold>Dedup window: n/a (no completed 5-min window yet)</div>';");
    web_client.println("}else if(dwin==0){");
    web_client.println("html+='<div class=font-bold>Dedup window: n/a (no new ids in the last window)</div>';");
    web_client.println("}else{");
    web_client.println("var dwc=(dwin<40||dwin>48)?'#E07B39':'inherit';");
    web_client.println("html+='<div class=font-bold>Dedup window:&nbsp;&asymp;<span style=color:'+dwc+'>'+dwin+'</span> min</div>';");
    web_client.println("html+='<div class=font-small>('+newid+' new ids, '+dup+' dups in last '+Math.round(intv/60)+' min; safe corridor 40-48 min)</div>';");
    web_client.println("}");
    web_client.println("html+='<div class=font-bold>Channel utilisation (last 5 min)</div>';");
    web_client.println("if(win==0){");
    web_client.println("html+='<div class=font-small>n/a</div>';");
    web_client.println("}else{");
    web_client.println("var rxPct=rx/(intv*1000)*100;if(rxPct>100)rxPct=100;");
    web_client.println("var txPct=tx/(intv*1000)*100;if(txPct>100)txPct=100;");
    web_client.println("var totPct=(rx+tx)/(intv*1000)*100;if(totPct>100)totPct=100;");
    web_client.println("html+='<div class=mcq-util-row><span class=mcq-util-label>rx '+rxPct.toFixed(1)+'%</span><div class=mcq-util-track><div style=height:100%;width:'+rxPct+'%;background:#3B7DD8></div></div></div>';");
    web_client.println("html+='<div class=mcq-util-row><span class=mcq-util-label>tx '+txPct.toFixed(1)+'%</span><div class=mcq-util-track><div style=height:100%;width:'+txPct+'%;background:#A2182F></div></div></div>';");
    web_client.println("html+='<div class=font-small>total '+totPct.toFixed(1)+'% ('+age+' s ago)</div>';");
    web_client.println("}");
    web_client.println("d.innerHTML=html;");
    web_client.println("var btn=document.getElementById('mcqtogglebtn');");
    web_client.println("d.hidden=!mcQueueOpen;");
    web_client.println("if(btn)btn.textContent=mcQueueOpen?'hide':'show';");
    web_client.println("}");

    web_client.println("</script>\n\n");

    // css style definitions
    web_client.println("<style>\n");

    // basic definitions
    web_client.println(":root {--mcbg:#FFFFFF;--mcgray:#252323;--mcred:#A2182F;--mclightred:#FCEDF0;--mcmidred:#FFA5B4;--mclightblue:#CADFEA;--mclightgreen:#CBFBD4;--widthfactor:1.0;}\n");
    web_client.println("@media screen and (max-width:600px) {* {font-size:12px;} :root{--widthfactor:0.7;}}\n");
    web_client.println("@media screen and (min-width:601px) {* {font-size:14px;} :root{--widthfactor:0.7;}}\n");
    web_client.println("@media screen and (min-width:801px) {* {font-size:14px;} :root{--widthfactor:1.0;}}\n");
    web_client.println("body {background:var(--mcbg);padding:0px;margin:0px 0px;height:100%;width:100%;-webkit-font-smoothing:antialiased;-moz-osx-font-smoothing:grayscale;color:var(--mcgray);font-family:sans-serif;}\n");
    web_client.println(".font-small {font-size:x-small;}\n");
    web_client.println(".font-large {font-size:large;}\n");
    web_client.println(".font-xlarge {font-size:x-large;}\n");
    web_client.println(".font-bold {font-weight:bold;}\n");
    web_client.println(".no-wrap {white-space:nowrap;}\n");
    web_client.println(".mw-600 {max-width:600px;}");

    // nav-bar definitions
    web_client.println("#nav_layer {height:100%;width:calc(60px*var(--widthfactor));background-color:var(--mcgray);position:fixed !important;overflow:auto;top:0px;}\n");
    web_client.println("#mc_logo {width:100%;aspect-ratio:1/1;margin:0px auto 75px auto;padding:5px 0px;display:block;background-color:#000;}\n");
    web_client.println(".nav_button {background-color:var(--mcgray);border:solid 6px var(--mcgray);box-shadow:none;border-radius:10px 0px 0px 10px;cursor:pointer; width:100%; aspect-ratio:1/1; margin:calc(20px*var(--widthfactor)) auto; display:block;}\n");
    web_client.println(".nav_button>svg {display:block;width:90%;height:90%;margin:auto auto;}\n");
    web_client.println(".nav_button.nbactive {background-color:var(--mcbg);-webkit-transition:0.5s;transition:0.5s; border-right-color:var(--mcbg);}\n");
    web_client.println(".nav_button.nbactive>svg, .nav_button.nbactive>img {filter:invert(1);}\n");
    web_client.println(".nav_button.extra_space {margin-top:40px !important;}\n");
    web_client.println("select,input,textarea {font-family:monospace;border-radius:7px;padding:3px 5px;box-sizing:border-box;border:solid 1px #7c7c7c;-webkit-transition:0.5s;transition:0.5s;}\n");
    web_client.println("select:focus,input:focus,textarea:focus {border:solid 1px var(--mcred);background-color:var(--mclightred);outline:none;}\n");

    // header (red bar with Call, Time, Date, etc) definitions
    web_client.println("#head_layer {margin:0px 0px 0px calc(60px*var(--widthfactor)); padding:10px 30px 10px 0px;text-align:right;top:0px;overflow:hidden;background-color:#a2182f;color:#ffffff;width:auto;height:39px;}\n");
    web_client.println("#head_layer>p {margin:0px 0px;}\n");

    // loader animation (the circling animation when content is loaded)
    web_client.println(".loader {display:block;margin:150px auto 50px auto;width:100px;height:100px;border:5px solid #FFF;border-bottom-color:#FF3D00;border-radius:50%;box-sizing:border-box;animation:rotation 1s linear infinite;}\n");
    web_client.println("@keyframes rotation{0%{transform:rotate(0deg);}100%{transform:rotate(360deg);}}\n");

    // content definitions
    web_client.println("#content_title {padding:0px 30px 0px 0px;margin:0px 0px 10px 0px;text-align:right;top:0px;overflow:hidden;background-color:#FCEDF0;}\n");
    web_client.println("#content_layer {margin-left:calc(60px*var(--widthfactor));top:0px;overflow:hidden;}");
    web_client.println("#content_inner {margin:20px 10px 30px 4%;}\n");

    // content definitions -> tables
    web_client.println("#content_inner > table {border-collapse:separate;overflow:hidden;border-spacing:0;border:solid 1px #7c7c7c;border-radius:10px;width:100%;background:#f9f9f9;}\n");
    web_client.println("#content_inner > table thead {background-color:var(--mcgray);color:#ffffff;border-radius:10px;}\n");
    web_client.println("#content_inner > table tr:nth-child(even) {background-color:#f0f0f0;}\n");
    web_client.println("#content_inner > table td {vertical-align:top;padding:0.5em 0.5em 0.5em 0.5em;border-bottom:solid 1px #e0e0e0;}\n");

    // content definitions -> anchors
    web_client.println("#content_inner a {color:var(--mcred);}\n");

    // content definitions -> buttons
    web_client.println("#content_inner button {background-color:var(--mclightred);border:solid 1px var(--mcgray);padding:3px 10px;text-align:center;cursor:pointer;display:inline-flex;align-items:center;border-radius:5px;}\n");
    web_client.println("#content_inner button:hover {background-color:var(--mcmidred);}\n");
    web_client.println(".btncheckmark {width:20px;height:20px;margin:auto;background:center no-repeat url(\"data:image/svg+xml;base64,PHN2ZyB2aWV3Qm94PSIwIDAgNDggNDgiIHZlcnNpb249IjEiIHhtbG5zPSJodHRwOi8vd3d3LnczLm9yZy8yMDAwL3N2ZyIgZW5hYmxlLWJhY2tncm91bmQ9Im5ldyAwIDAgNDggNDgiIGZpbGw9IiMwMDAwMDAiPjxnIHN0cm9rZS13aWR0aD0iMCI+PC9nPjxnIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIgc3Ryb2tlLWxpbmVqb2luPSJyb3VuZCI+PC9nPjxnPjxwb2x5Z29uIGZpbGw9IiM0M0EwNDciIHBvaW50cz0iNDAuNiwxMi4xIDE3LDM1LjcgNy40LDI2LjEgNC42LDI5IDE3LDQxLjMgNDMuNCwxNC45Ij48L3BvbHlnb24+PC9nPjwvc3ZnPg==\");}\n");
    web_client.println(".btnrefresh {width:20px;height:20px;margin:auto;background:center no-repeat url(\"data:image/svg+xml;base64,PHN2ZyB2aWV3Qm94PSIwIDAgMjQgMjQiIGZpbGw9Im5vbmUiIHhtbG5zPSJodHRwOi8vd3d3LnczLm9yZy8yMDAwL3N2ZyI+PGcgc3Ryb2tlLXdpZHRoPSIwIj48L2c+PGcgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiBzdHJva2UtbGluZWpvaW49InJvdW5kIj48L2c+PGc+PHBhdGggZD0iTTIxIDNWOE0yMSA4SDE2TTIxIDhMMTggNS4yOTE2OEMxNi40MDc3IDMuODY2NTYgMTQuMzA1MSAzIDEyIDNDNy4wMjk0NCAzIDMgNy4wMjk0NCAzIDEyQzMgMTYuOTcwNiA3LjAyOTQ0IDIxIDEyIDIxQzE2LjI4MzIgMjEgMTkuODY3NSAxOC4wMDggMjAuNzc3IDE0IiBzdHJva2U9IiM0M0EwNDciIHN0cm9rZS13aWR0aD0iMiIgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiBzdHJva2UtbGluZWpvaW49InJvdW5kIj48L3BhdGg+IDwvZz48L3N2Zz4=\");}\n");

    // content definitions -> message-page
    web_client.println("#messagetext {width:100%;}\n");
    web_client.println("#sendcall {min-width:8em;}\n");

    web_client.println(".message {--r:1em;--t:1.5em;max-width:300px;padding:15px;margin-bottom:20px;border-inline:var(--t) solid #0000; border-radius:calc(var(--r) + var(--t))/var(--r);mask:radial-gradient(100% 100% at var(--_p) 0,#0000 99%,#000 102%) var(--_p) 100%/var(--t) var(--t) no-repeat, linear-gradient(#000 0 0) padding-box;color:#000;}\n");
    web_client.println(".message>div>p {padding:0px;margin:0px;}\n");
    web_client.println(".message-received{--_p:0;border-bottom-left-radius:0 0;place-self:start;background-color:var(--mclightgreen);border-color:var(--mclightgreen);}\n");
    web_client.println(".message-send{--_p:100%;border-bottom-right-radius:0 0;place-self:end;background-color:var(--mclightblue);border-color:var(--mclightblue);}\n");

    // content definitions -> commonly used cards
    web_client.println(".cardlayout {min-height:20px;border:solid 1px var(--mcgray);border-radius:7px;padding:7px;padding-top:16px;margin:30px 0px;position:relative;max-width:600px;}\n");
    web_client.println(".cardlabel {position:absolute;font-weight:bold;margin:0 5px;left:6px;top:-1px;transform:translateY(-50%);z-index:10;background-color:#fff;padding:0 2px;}\n");

    web_client.println(".grid {display:grid;align-items:center;grid-column-gap:8px;grid-row-gap:7px;margin:7px;}\n");
    web_client.println(".grid3{grid-template-columns:minmax(min-content,400px) minmax(min-content,150px) minmax(min-content,50px)}\n");
    web_client.println(".grid2 {grid-template-columns:minmax(min-content,550px) minmax(min-content,50px)}\n");
    web_client.println(".flex-auto-wrap {display:flex;flex-direction:row;flex-wrap:wrap;gap:10px;}\n");
    web_client.println(".flex-auto-wrap>* {flex:1;}\n");

    // content definitions -> commonly used inputs
    web_client.println("input:where([type=\"checkbox\"][role=\"switch\"]) {-webkit-appearance:none;-moz-appearance:none;appearance:none;position:relative;font-size:inherit;width:2.5em;height:1.2em;box-sizing:content-box;border-radius:1em;vertical-align:text-bottom;margin:auto;color:inherit;background-color:rgb(212, 212, 212);}\n");
    web_client.println("input:where([type=\"checkbox\"][role=\"switch\"]):checked {background-color:rgb(102, 214, 102);}\n");
    web_client.println("input:where([type=\"checkbox\"][role=\"switch\"])::before {content:\"\";position:absolute;top:50%;left:0;transform:translate(0,-50%);box-sizing:border-box;width:1.1em;height:1.1em;margin:0 0.15em;border-radius:50%;background:var(--mcbg);-webkit-transition:0.5s;transition:0.5s;}\n");
    web_client.println("input:where([type=\"checkbox\"][role=\"switch\"]):checked::before {left:1.8em;}\n");

    // content definitions -> setup collapsable cards

    web_client.println(".collapsablecard>.cardtoggle {position:absolute;right:8px;top:-1px;transform:translateY(-50%);z-index:10;width:36px;height:36px;border-radius:50% !important;disjplay:none;}\n");
    web_client.println(".collapsablecard>.cardtoggle>i {padding:4px;margin:auto;border:solid black;border-width:2px 0 0 2px;display:block;-webkit-transform:rotate(-135deg);transform:rotate(-135deg);-webkit-transition:0.5s;transition:0.5s;}\n");
    web_client.println(".cardopen>.cardtoggle>i {-webkit-transform: rotate(45deg);transform: rotate(45deg);}\n");
    web_client.println(".collapsablecard>div {max-height:0px;-webkit-transition:opacity .15s .0s,max-height .25s .10s;transition:opacity .15s .0s,max-height .25s .10s,margin .0s .50s;	opacity:0.0;overflow:hidden;margin:0px;}\n");
    web_client.println(".cardopen>div {-webkit-transition:opacity .15s .10s,max-height .25s .0s;transition:opacity .15s .10s,max-height .25s .0s;max-height:1000px;opacity:1;margin:7px;}\n");
    web_client.println(".cardopen>span:first-of-type {display:none;}\n");

    // content definitions -> WQ-01 LoRa Queue panel (rxlog page)
    web_client.println(".mcq-toggle {position:absolute;right:8px;top:-1px;transform:translateY(-50%);z-index:10;border:solid 1px var(--mcgray);background:#fff;border-radius:5px;padding:1px 8px;cursor:pointer;}\n");
    web_client.println(".mcq-barwrap {position:relative;}\n");
    web_client.println(".mcq-bar {display:flex;flex-direction:row;gap:1px;height:14px;margin:4px 0 2px 0;}\n");
    web_client.println(".mcq-cell {flex:1;height:14px;}\n");
    web_client.println(".mcq-cell-empty {flex:1;height:14px;background:#ECECEC;border:1px solid #d0d0d0;box-sizing:border-box;}\n");
    web_client.println(".mcq-tick {position:absolute;top:0;bottom:0;width:1px;background:#000;opacity:0.5;}\n");
    web_client.println(".mcq-tick-faint {position:absolute;top:0;bottom:0;width:1px;background:#000;opacity:0.15;}\n");
    web_client.println(".mcq-ticklabels {position:relative;height:12px;font-size:x-small;margin:0 0 8px 0;}\n");
    web_client.println(".mcq-legend {font-size:x-small;margin:0 0 8px 0;}\n");
    web_client.println(".mcq-swatch {display:inline-block;width:8px;height:8px;margin:0 3px 0 6px;border-radius:2px;vertical-align:middle;}\n");
    web_client.println(".mcq-util-row {display:flex;align-items:center;gap:6px;margin:2px 0;}\n");
    web_client.println(".mcq-util-label {display:inline-block;min-width:60px;}\n");
    web_client.println(".mcq-util-track {flex:1;height:10px;background:#ECECEC;border-radius:4px;overflow:hidden;}\n");

    // content definitions -> message-page tab bar
    web_client.println("#mctabs {margin:6px 0;}\n");
    web_client.println("#content_inner .mctab {display:inline-flex;align-items:center;border:solid 1px var(--mcgray);background-color:var(--mcbg);border-radius:5px;padding:2px 8px;margin-right:4px;cursor:pointer;}\n");
    web_client.println("#content_inner .mctab-new {background-color:var(--mclightgreen);}\n");
    web_client.println("#content_inner .mctab-on {background-color:var(--mclightblue);}\n");
    web_client.println(".mcbadge {font-size:x-small;font-weight:bold;margin-left:4px;}\n");

    web_client.println("</style>\n\n");

    // scaffold body
    web_client.println("</head>\n<body>\n");
    web_client.println("<div id=\"nav_layer\">\n");
    // MC Logo (Image as base64 and split to several print() to adress limited print buffer
    web_client.print("<svg id=\"mc_logo\" width=\"48px\" height=\"48px\" viewBox=\"0 0 270.93332 270.93332\" xmlns=\"http://www.w3.org/2000/svg\"><g transform=\"translate(21.572702,-6.7150369)\">");
    web_client.print("<path style=\"fill:none;stroke:#ff0000;stroke-width:6.00001;stroke-dasharray:none\" d=\"M 43.312893,84.905251 C 59.073749,118.8832 128.87181,227.98106 128.87181,227.98106 L 136.6499,64.641293 36.353556,");
    web_client.print("184.58753 192.73397,156.95486 Z M 30.622336,75.899048 25.505175,189.50001 124.98278,237.806 206.65266,155.52206 146.06547,48.880441 c 0,0 -75.488902,17.574879 -115.443134,27.018607 z\"/>");
    web_client.print("<path style=\"fill:#000;stroke:#ff0000;stroke-width:6\" d=\"M 55.701471,187.15109 A 29.524067,29.524067 0 0 1 26.177404,216.67516 29.524067,29.524067 0 0 1 -3.3466625,187.15109 29.524067,29.524067 0 0 1 ");
    web_client.print("26.177404,157.62703 29.524067,29.524067 0 0 1 55.701471,187.15109 Z m 98.954169,47.625 a 29.524067,29.524067 0 0 1 -29.52407,29.52407 29.524067,29.524067 0 0 1 -29.524068,-29.52407 29.524067,29.524067 0 0 ");
    web_client.print("1 29.524068,-29.52406 29.524067,29.524067 0 0 1 29.52407,29.52406 z m 78.58126,-80.16874 a 29.524067,29.524067 0 0 1 -29.52407,29.52406 29.524067,29.524067 0 0 1 -29.52407,-29.52406 29.524067,29.524067 0 0 1 ");
    web_client.print("29.52407,-29.52407 29.524067,29.524067 0 0 1 29.52407,29.52407 z M 175.02855,49.303177 A 29.524067,29.524067 0 0 1 145.50449,78.827244 29.524067,29.524067 0 0 1 115.98042,49.303177 29.524067,29.524067 0 0 1 ");
    web_client.print("145.50449,19.77911 29.524067,29.524067 0 0 1 175.02855,49.303177 Z M 60.993137,76.290672 A 29.524067,29.524067 0 0 1 31.46907,105.81474 29.524067,29.524067 0 0 1 1.9450035,76.290672 29.524067,29.524067 0 0 1 ");
    web_client.print("31.46907,46.766605 29.524067,29.524067 0 0 1 60.993137,76.290672 Z\"/><circle style=\"fill:#ff0000;fill-opacity:1;stroke:none;stroke-width:6.35477;stroke-dasharray:none\" id=\"mclogo_circle\" cx=\"108.51074\" ");
    web_client.print("cy=\"139.66409\" r=\"52.916668\"/><text xml:space=\"preserve\" style=\"font-size:63.5px;text-align:start;writing-mode:lr-tb;direction:ltr;text-anchor:start;display:inline;fill:#ffffff;fill-opacity:1;");
    web_client.print("stroke:#ffffff;stroke-width:6.00001;stroke-dasharray:none\" x=\"63.16748\" y=\"161.0486\"><tspan style=\"font-style:normal;font-variant:normal;font-weight:bold;font-stretch:normal;font-size:63px;");
    web_client.println("font-family:sans-serif;stroke:none;stroke-width:6\" x=\"63.16748\" y=\"161.0486\">4.0</tspan></text></g></svg>");
    
    

    web_client.println("<Button class=\"nav_button nbactive\" onclick=\"loadPage('info',this,true)\"><svg viewBox=\"-0.5 0 25 25\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g> <path d=\"M12 21.5C17.1086 21.5 21.25 17.3586 21.25 12.25C21.25 7.14137 17.1086 3 12 3C6.89137 3 2.75 7.14137 2.75 12.25C2.75 17.3586 6.89137 21.5 12 21.5Z\" stroke=\"#ffffff\" stroke-width=\"1.5\" stroke-linecap=\"round\" stroke-linejoin=\"round\"></path> <path d=\"M12.9309 8.15005C12.9256 8.39231 12.825 8.62272 12.6509 8.79123C12.4767 8.95974 12.2431 9.05271 12.0008 9.05002C11.8242 9.04413 11.6533 8.98641 11.5093 8.884C11.3652 8.7816 11.2546 8.63903 11.1911 8.47415C11.1275 8.30927 11.1139 8.12932 11.152 7.95675C11.19 7.78419 11.278 7.6267 11.405 7.50381C11.532 7.38093 11.6923 7.29814 11.866 7.26578C12.0397 7.23341 12.2192 7.25289 12.3819 7.32181C12.5446 7.39072 12.6834 7.506 12.781 7.65329C12.8787 7.80057 12.9308 7.97335 12.9309 8.15005ZM11.2909 16.5301V11.1501C11.2882 11.0556 11.3046 10.9615 11.3392 10.8736C11.3738 10.7857 11.4258 10.7057 11.4922 10.6385C11.5585 10.5712 11.6378 10.518 11.7252 10.4822C11.8126 10.4464 11.9064 10.4286 12.0008 10.43C12.094 10.4299 12.1863 10.4487 12.272 10.4853C12.3577 10.5218 12.4352 10.5753 12.4997 10.6426C12.5642 10.7099 12.6143 10.7895 12.6472 10.8767C12.6801 10.9639 12.6949 11.0569 12.6908 11.1501V16.5301C12.6908 16.622 12.6727 16.713 12.6376 16.7979C12.6024 16.8828 12.5508 16.96 12.4858 17.025C12.4208 17.09 12.3437 17.1415 12.2588 17.1767C12.1738 17.2119 12.0828 17.23 11.9909 17.23C11.899 17.23 11.8079 17.2119 11.723 17.1767C11.6381 17.1415 11.5609 17.09 11.4959 17.025C11.4309 16.96 11.3793 16.8828 11.3442 16.7979C11.309 16.713 11.2909 16.622 11.2909 16.5301Z\" fill=\"#ffffff\"></path> </g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('messages',this,true)\"><svg viewBox=\"0 0 24 24\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g> <path d=\"M7 9H17M7 13H17M21 20L17.6757 18.3378C17.4237 18.2118 17.2977 18.1488 17.1656 18.1044C17.0484 18.065 16.9277 18.0365 16.8052 18.0193C16.6672 18 16.5263 18 16.2446 18H6.2C5.07989 18 4.51984 18 4.09202 17.782C3.71569 17.5903 3.40973 17.2843 3.21799 16.908C3 16.4802 3 15.9201 3 14.8V7.2C3 6.07989 3 5.51984 3.21799 5.09202C3.40973 4.71569 3.71569 4.40973 4.09202 4.21799C4.51984 4 5.0799 4 6.2 4H17.8C18.9201 4 19.4802 4 19.908 4.21799C20.2843 4.40973 20.5903 4.71569 20.782 5.09202C21 5.51984 21 6.0799 21 7.2V20Z\" stroke=\"#ffffff\" stroke-width=\"2\" stroke-linecap=\"round\" stroke-linejoin=\"round\"></path> </g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('wx',this,true)\"><svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"0 0 512 512\" xml:space=\"preserve\" fill=\"#000000\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g > <style type=\"text/css\"> .st0{fill:#ffffff;} </style> <g> <path class=\"st0\" d=\"M115.958,269.922c16.999-10.12,36.842-15.916,58.04-15.916c2.556,0,5.127,0.078,7.682,0.234 c7.199-24.681,20.957-46.355,39.203-63.12c-3.49-39.437-36.562-70.32-76.879-70.32c-42.647,0-77.207,34.56-77.207,77.199 C66.798,230.766,87.194,258.719,115.958,269.922z\"></path> <rect x=\"135.652\" y=\"54.002\" class=\"st0\" width=\"16.696\" height=\"45.911\"></rect> <polygon class=\"st0\" points=\"102.184,108.88 79.232,69.116 64.772,77.467 87.724,117.232 \"></polygon> <polygon class=\"st0\" points=\"15.114,133.233 54.878,156.185 63.23,141.726 23.466,118.774 \"></polygon> <polygon class=\"st0\" points=\"45.919,189.654 0,189.654 0,206.35 45.919,206.342 \"></polygon> <polygon class=\"st0\" points=\"15.114,262.77 23.466,277.23 63.23,254.27 54.878,239.811 \"></polygon> <rect x=\"240.478\" y=\"114.523\" transform=\"matrix(0.4998 0.8661 -0.8661 0.4998 243.5358 -146.7501)\" class=\"st0\" width=\"16.694\" height=\"45.913\"></rect> <polygon class=\"st0\" points=\"223.228,77.467 208.776,69.116 185.817,108.88 200.269,117.232 \"></polygon> <path class=\"st0\" d=\"M431.997,298c-0.031,0-0.062,0.008-0.101,0.008c0.054-1.332,0.101-2.665,0.101-4.004 C431.997,229.932,380.064,178,316,178c-60.012,0-109.382,45.575-115.388,104.006c-8.414-2.602-17.342-4.005-26.614-4.005 C124.294,278.001,84,318.295,84,368c0,49.704,40.294,89.998,89.998,89.998h257.999c44.182,0,80.003-35.814,80.003-79.995 C512,333.814,476.178,298,431.997,298z\"></path> </g> </g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('position',this,true)\"><svg viewBox=\"0 0 512 512\" xmlns=\"http://www.w3.org/2000/svg\" fill=\"#ffffff\" stroke=\"#ffffff\"><g stroke-width=\"0\"></g><g istroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g><path fill=\"#ffffff\" d=\"M256 17.108c-75.73 0-137.122 61.392-137.122 137.122.055 23.25 6.022 46.107 11.58 56.262L256 494.892l119.982-274.244h-.063c11.27-20.324 17.188-43.18 17.202-66.418C393.122 78.5 331.73 17.108 256 17.108zm0 68.56a68.56 68.56 0 0 1 68.56 68.562A68.56 68.56 0 0 1 256 222.79a68.56 68.56 0 0 1-68.56-68.56A68.56 68.56 0 0 1 256 85.67z\"></path></g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('mheard',this,true)\"><svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"0 0 32 32\" xml:space=\"preserve\" fill=\"#000000\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g> <style type=\"text/css\"> .linesandangles_een{fill:#ffffff;} </style> <path class=\"linesandangles_een\" d=\"M25,13c0,3.348-2.208,7.455-4.286,9.618c-0.527,0.549-0.902,1.188-1.299,1.863 C18.447,26.131,17.35,28,14,28c-3.616,0-5.077-2.068-6.043-3.437c-0.238-0.337-0.464-0.657-0.664-0.856l1.414-1.414 C9.028,22.614,9.301,23,9.59,23.41C10.49,24.683,11.42,26,14,26c2.205,0,2.796-1.007,3.69-2.531 c0.417-0.711,0.891-1.517,1.581-2.236C21.064,19.366,23,15.687,23,13c0-3.86-3.14-7-7-7s-7,3.14-7,7H7c0-4.962,4.038-9,9-9 S25,8.038,25,13z M12,17h-1v2h1c1.206,0,3-0.799,3-3c0-1.639-0.994-2.5-2-2.833v-0.161C13.006,12.503,13.177,10,16,10 s2.994,2.503,3,3.005L20,13h1c0-1.729-1.045-5-5-5s-5,3.271-5,5l0.014,1.975L11.988,15C12.45,15.012,13,15.195,13,16 S12.45,16.988,12,17z\"></path> </g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('path',this,true)\"><svg viewBox=\"0 0 16 16\" xmlns=\"http://www.w3.org/2000/svg\" fill=\"none\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g><path fill=\"#ffffff\" fill-rule=\"evenodd\" d=\"M13 0a3 3 0 00-1.65 5.506 7.338 7.338 0 01-.78 1.493c-.22.32-.472.635-.8 1.025a1.509 1.509 0 00-.832.085 12.722 12.722 0 00-1.773-1.124c-.66-.34-1.366-.616-2.215-.871a1.5 1.5 0 10-2.708 1.204c-.9 1.935-1.236 3.607-1.409 5.838a1.5 1.5 0 101.497.095c.162-2.07.464-3.55 1.25-5.253.381-.02.725-.183.979-.435.763.23 1.367.471 1.919.756a11.13 11.13 0 011.536.973 1.5 1.5 0 102.899-.296c.348-.415.64-.779.894-1.148.375-.548.665-1.103.964-1.857A3 3 0 1013 0zm-1.5 3a1.5 1.5 0 113 0 1.5 1.5 0 01-3 0z\" clip-rule=\"evenodd\"></path></g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('rxlog',this,true)\"><svg viewBox=\"0 0 32 32\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" xmlns:sketch=\"http://www.bohemiancoding.com/sketch/ns\" fill=\"#ffffff\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g> <title>book-album</title> <desc>Created with Sketch Beta.</desc><defs></defs><g stroke=\"none\" stroke-width=\"1\" fill=\"none\" fill-rule=\"evenodd\" sketch:type=\"MSPage\"> <g sketch:type=\"MSLayerGroup\" transform=\"translate(-412.000000, -99.000000)\" fill=\"#ffffff\"> <path d=\"M442,124 C442,125.104 441.073,125.656 440,126 C440,126 434.557,127.515 429,128.977 L429,104 L440,101 C441.104,101 442,101.896 442,103 L442,124 L442,124 Z M427,128.998 C421.538,127.53 416,126 416,126 C414.864,125.688 414,125.104 414,124 L414,103 C414,101.896 414.896,101 416,101 L427,104 L427,128.998 L427,128.998 Z M440,99 C440,99 434.211,100.594 428.95,102 C428.291,102.025 427.627,102 426.967,102 C421.955,100.656 416,99 416,99 C413.791,99 412,100.791 412,103 L412,124 C412,126.209 413.885,127.313 416,128 C416,128 421.393,129.5 426.967,131 L428.992,131 C434.612,129.5 440,128 440,128 C442.053,127.469 444,126.209 444,124 L444,103 C444,100.791 442.209,99 440,99 L440,99 Z\" sketch:type=\"MSShapeGroup\"> </path> </g> </g> </g></svg></Button>\n");
    web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('spectrum',this,true)\"><svg viewBox=\"0 0 24 24\" xmlns=\"http://www.w3.org/2000/svg\"><g><path d=\"M13,11v4M9,7v8m8-6v6\" style=\"fill:none;stroke:#ffffff;stroke-linecap:round;stroke-linejoin:round;stroke-width:2;\"></path><path d=\"M3,19H21M5,3V21\" style=\"fill:none;stroke:#ffffff;stroke-linecap:round;stroke-linejoin:round;stroke-width:2;\"></path></g></svg></Button>\n");
    if(bMCP23017) {
        web_client.println("<Button class=\"nav_button\" onclick=\"loadPage('mcp23017',this,true)\"><svg width=\"48\" height=\"48\" version=\"1.1\" viewBox=\"0 0 12.7 12.7\" xmlns=\"http://www.w3.org/2000/svg\"><g fill=\"none\" stroke=\"#ffffff\" stroke-linejoin=\"round\"><rect x=\".78644\" y=\".78644\" width=\"11.127\" height=\"5.4354\" stroke-width=\"1.0\"/><g stroke-linecap=\"round\" stroke-width=\"1.0\"><path d=\"m0.79071 8.2113h11.119\"/><path d=\"m0.79679 9.7926h11.119\"/><path d=\"m0.79679 11.38h11.119\"/></g></g></svg></Button>\n");
    }

    // Setup Button (Image as base64 and split t0 several print() so RAK can handle it)
    web_client.print("<Button class=\"nav_button\" onclick=\"loadPage('setup',this,true)\"><img src=\"data:image/svg+xml;base64,PHN2ZyB2aWV3Qm94PSIwIDAgMjQgMjQiIGZpbGw9Im5vbmUiIHhtbG5zPSJodHRwOi8vd3d3LnczLm9yZy8yMDAwL3N2ZyI+PGcgc3Ryb2tlLXdpZHRoPSIwIj48L2c+PGcgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiBzdHJva2UtbGluZWpvaW49InJvdW5kIj48L2c+PGc+PGNpcmNsZSBjeD0iMTIiIGN5PSIxMiIgcj0iMyIgc3Ryb2tlPSIjZmZmZmZmIiBzdHJva2Utd2lkdGg9IjEuNSI+PC9jaXJjbGU+PHBhdGggZD0iTTEzLjc2NTQgMi4xNTIyNEMxMy4zOTc4IDIgMTIuOTMxOSAyIDEyIDJDMTEuMDY4MSAyIDEwLjYwMjIgMiAxMC4yMzQ2IDIuMTUyMjRDOS43NDQ1NyAyLjM1NTIzIDkuMzU1MjIgMi43NDQ1OCA5LjE1MjIzIDMuMjM0NjNDOS4wNTk1NyAzLjQ1ODM0IDkuMDIzMyAzLjcxODUgOS4wMDkxMSA0LjA5Nzk5QzguOTg4MjYgNC42NTU2OCA4LjcwMjI2IDUuMTcxODkgOC4yMTg5NCA1LjQ1MDkzQzcuNzM1NjQgNS43Mjk5NiA3LjE0NTU5IDUuNzE5NTQgNi42NTIxOSA1LjQ1ODc2QzYuMzE2NDUgNS4yODEzIDYuMDczMDEgNS4xODI2MiA1LjgzMjk0IDUuMTUxMDJDNS4zMDcwNCA1LjA4MTc4IDQuNzc1MTggNS4yMjQyOSA0LjM1NDM2IDUuNTQ3MkM0LjAzODc0IDUuNzg5MzggMy44MDU3NyA2LjE5MjkgMy4zMzk4MyA2Ljk5OTkzQzIuODczODkgNy44MDY5NyAyLjY0MDkyIDguMjEwNDggMi41ODg5OSA4LjYwNDkxQz");
    web_client.print("IuNTE5NzYgOS4xMzA4IDIuNjYyMjcgOS42NjI2NiAyLjk4NTE4IDEwLjA4MzVDMy4xMzI1NiAxMC4yNzU2IDMuMzM5NyAxMC40MzcgMy42NjExOSAxMC42MzlDNC4xMzM4IDEwLjkzNiA0LjQzNzg5IDExLjQ0MTkgNC40Mzc4NiAxMkM0LjQzNzgzIDEyLjU1ODEgNC4xMzM3NSAxMy4wNjM5IDMuNjYxMTggMTMuMzYwOEMzLjMzOTY1IDEzLjU2MjkgMy4xMzI0OCAxMy43MjQ0IDIuOTg1MDggMTMuOTE2NUMyLjY2MjE3IDE0LjMzNzMgMi41MTk2NiAxNC44NjkxIDIuNTg4OSAxNS4zOTVDMi42NDA4MiAxNS43ODk0IDIuODczNzkgMTYuMTkzIDMuMzM5NzMgMTdDMy44MDU2OCAxNy44MDcgNC4wMzg2NSAxOC4yMTA2IDQuMzU0MjYgMTguNDUyN0M0Ljc3NTA4IDE4Ljc3NTYgNS4zMDY5NCAxOC45MTgxIDUuODMyODQgMTguODQ4OUM2LjA3Mjg5IDE4LjgxNzMgNi4zMTYzMiAxOC43MTg2IDYuNjUyMDQgMTguNTQxMkM3LjE0NTQ3IDE4LjI4MDQgNy43MzU1NiAxOC4yNyA4LjIxODkgMTguNTQ5QzguNzAyMjQgMTguODI4MSA4Ljk4ODI2IDE5LjM0NDMgOS4wMDkxMSAxOS45MDIxQzkuMDIzMzEgMjAuMjgxNSA5LjA1OTU3IDIwLjU0MTcgOS4xNTIyMyAyMC43NjU0QzkuMzU1MjIgMjEuMjU1NCA5Ljc0NDU3IDIxLjY0NDggMTAuMjM0NiAyMS44NDc4QzEwLjYwMjIgMjIgMTEuMDY4MSAyMiAxMiAyMkMxMi45MzE5IDIyIDEzLjM5NzggMjIgMTMuNzY1NCAyMS44NDc4QzE0LjI1NTQgMjEuNjQ0OCAxNC42NDQ4IDIxLjI1NTQgMTQuODQ3NyAyMC43NjU0QzE0Ljk0MDQgM");
    web_client.print("jAuNTQxNyAxNC45NzY3IDIwLjI4MTUgMTQuOTkwOSAxOS45MDJDMTUuMDExNyAxOS4zNDQzIDE1LjI5NzcgMTguODI4MSAxNS43ODEgMTguNTQ5QzE2LjI2NDMgMTguMjY5OSAxNi44NTQ0IDE4LjI4MDQgMTcuMzQ3OSAxOC41NDEyQzE3LjY4MzYgMTguNzE4NiAxNy45MjcgMTguODE3MiAxOC4xNjcgMTguODQ4OEMxOC42OTI5IDE4LjkxODEgMTkuMjI0OCAxOC43NzU2IDE5LjY0NTYgMTguNDUyN0MxOS45NjEyIDE4LjIxMDUgMjAuMTk0MiAxNy44MDcgMjAuNjYwMSAxNi45OTk5QzIxLjEyNjEgMTYuMTkyOSAyMS4zNTkxIDE1Ljc4OTQgMjEuNDExIDE1LjM5NUMyMS40ODAyIDE0Ljg2OTEgMjEuMzM3NyAxNC4zMzcyIDIxLjAxNDggMTMuOTE2NEMyMC44Njc0IDEzLjcyNDMgMjAuNjYwMiAxMy41NjI4IDIwLjMzODcgMTMuMzYwOEMxOS44NjYyIDEzLjA2MzkgMTkuNTYyMSAxMi41NTggMTkuNTYyMSAxMS45OTk5QzE5LjU2MjEgMTEuNDQxOCAxOS44NjYyIDEwLjkzNjEgMjAuMzM4NyAxMC42MzkyQzIwLjY2MDMgMTAuNDM3MSAyMC44Njc1IDEwLjI3NTcgMjEuMDE0OSAxMC4wODM1QzIxLjMzNzggOS42NjI3MyAyMS40ODAzIDkuMTMwODcgMjEuNDExMSA4LjYwNDk3QzIxLjM1OTIgOC4yMTA1NSAyMS4xMjYyIDcuODA3MDMgMjAuNjYwMiA3QzIwLjE5NDMgNi4xOTI5NyAxOS45NjEzIDUuNzg5NDUgMTkuNjQ1NyA1LjU0NzI3QzE5LjIyNDkgNS4yMjQzNiAxOC42OTMgNS4wODE4NSAxOC4xNjcxIDUuMTUxMDlDMTcuOTI3MSA1LjE4MjY5IDE3LjY4MzcgNS4y");
    web_client.println("ODEzNiAxNy4zNDc5IDUuNDU4OEMxNi44NTQ1IDUuNzE5NTkgMTYuMjY0NCA1LjczMDAyIDE1Ljc4MTEgNS40NTA5NkMxNS4yOTc3IDUuMTcxOTEgMTUuMDExNyA0LjY1NTY2IDE0Ljk5MDkgNC4wOTc5NEMxNC45NzY3IDMuNzE4NDggMTQuOTQwNCAzLjQ1ODMzIDE0Ljg0NzcgMy4yMzQ2M0MxNC42NDQ4IDIuNzQ0NTggMTQuMjU1NCAyLjM1NTIzIDEzLjc2NTQgMi4xNTIyNFoiIHN0cm9rZT0iI2ZmZmZmZiIgc3Ryb2tlLXdpZHRoPSIxLjUiPjwvcGF0aD48L2c+PC9zdmc+\"></Button>\n");

    web_client.println("<Button class=\"nav_button extra_space\" onclick=\"if(confirm('Node will reboot, are you sure?')){callfunction('reboot','');}\"><svg viewBox=\"0 0 16 16\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\" stroke=\"#FFFFFF\" stroke-width=\"0.032\"></g><g><path d=\"m 8 0 c -0.550781 0 -1 0.449219 -1 1 v 5 c 0 0.550781 0.449219 1 1 1 s 1 -0.449219 1 -1 v -5 c 0 -0.550781 -0.449219 -1 -1 -1 z m -7 1 l 2.050781 2.050781 c -2.117187 2.117188 -2.652343 5.355469 -1.332031 8.039063 c 1.324219 2.683594 4.214844 4.238281 7.179688 3.851562 c 2.96875 -0.386718 5.367187 -2.625 5.960937 -5.554687 c 0.59375 -2.933594 -0.75 -5.929688 -3.335937 -7.433594 c -0.476563 -0.28125 -1.089844 -0.117187 -1.367188 0.359375 s -0.117188 1.089844 0.359375 1.367188 c 1.851563 1.078124 2.808594 3.207031 2.382813 5.3125 c -0.421876 2.101562 -2.128907 3.691406 -4.253907 3.96875 c -2.128906 0.273437 -4.183593 -0.828126 -5.128906 -2.753907 s -0.566406 -4.226562 0.949219 -5.742187 l 1.535156 1.535156 v -4.003906 c 0 -0.519532 -0.449219 -0.996094 -1 -0.996094 z m 0 0 \" fill=\"#FFFFFF\"></path></g></svg></Button>\n");

    // if the suer has set a password, we deliver a logout button
    if (strlen(meshcom_settings.node_webpwd) > 0)
    {
        web_client.println("<Button class=\"nav_button\" onclick=\"if(confirm('Logging out, are you sure?')){login('')};\"><svg fill=\"#ffffff\" viewBox=\"0 0 24 24\" xmlns=\"http://www.w3.org/2000/svg\"><g stroke-width=\"0\"></g><g stroke-linecap=\"round\" stroke-linejoin=\"round\"></g><g><path d=\"M7.707,8.707,5.414,11H17a1,1,0,0,1,0,2H5.414l2.293,2.293a1,1,0,1,1-1.414,1.414l-4-4a1,1,0,0,1,0-1.414l4-4A1,1,0,1,1,7.707,8.707ZM21,1H13a1,1,0,0,0,0,2h7V21H13a1,1,0,0,0,0,2h8a1,1,0,0,0,1-1V2A1,1,0,0,0,21,1Z\"></path></g></svg></Button>\n");
    }
    web_client.printf("</div>\n<div id=\"head_layer\"><p class=\"font-small\">Meshcom 4.0 %s%s</p><p class=\"font-bold\">%s</p></div>\n</div>\n", SOURCE_VERSION, SOURCE_VERSION_WEB_SUB, meshcom_settings.node_call);
    web_client.println("<div id=\"content_layer\">\n");

    // initial content
    if (bget_password)
    {
        sub_page_login();
    }
    else
    {
        sub_page_info();
    }

    web_client.println("</div>\n</body>\n</html>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers a page telling the user that the requested page is not known
 */
void sub_page_unknown()
{
    _create_meshcom_subheader("Oh no...(404)");
    web_client.println("<div id=\"content_inner\">");
    web_client.println("<p>The page you have requested is not known.</p>");
    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers a page to enter the nodes web password
 */
void sub_page_login()
{
    _create_meshcom_subheader("Login required");
    web_client.println("<div id=\"content_inner\">");
    web_client.println("<div class=\"cardlayout\">");
    web_client.println("<label class=\"cardlabel\">Enter Password</label>");
    web_client.println("<div class=\"grid\">");
    web_client.println("<span>In order to continue, you need to log in.</span>");
    web_client.println("<input type=\"password\" maxlength=\"20\" size=\"20\" id=\"nodepassword\" name=\"nodepassword\">");
    web_client.println("<button onclick=\"login(document.getElementById('nodepassword').value)\" style=\"justify-self:self-end;\">Login</button");
    web_client.println("</div></div></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * Increments i by 1 within a modular range.
 */
static int increment_mod(int i, int n) {
    return (i + 1) % n;
}

/**
 * ###########################################################################################################################
 * escapes a string for safe inclusion in HTML output (WEB-03a: mesh-derived strings such as
 * message payloads, callsigns and paths are attacker-controlled and must not reach innerHTML raw)
 */
static String htmlEscape(const String &input)
{
    String out;
    out.reserve(input.length());
    for (unsigned int i = 0; i < input.length(); i++)
    {
        char c = input.charAt(i);
        switch (c)
        {
            case '&':  out += "&amp;";  break;
            case '<':  out += "&lt;";   break;
            case '>':  out += "&gt;";   break;
            case '"':  out += "&quot;"; break;
            case '\'': out += "&#39;";  break;
            default:   out += c;        break;
        }
    }
    return out;
}

/**
 * ###########################################################################################################################
 * resolves the NTP server the node actually uses (WEB-01): own override if set, else the same
 * default selection udp_functions.cpp:1495-1536 applies when connecting. No clean read-only
 * extern exposes that resolved choice, so the selection is duplicated here (read-only mirror).
 */
static String getEffectiveNtpServer()
{
    if (strlen(meshcom_settings.node_ownntp) >= 7)
        return String(meshcom_settings.node_ownntp);

    IPAddress webNtpIp;
    webNtpIp.fromString(meshcom_settings.node_ip);

    bool bHamnet = (webNtpIp[0] == 44 || meshcom_settings.node_hamnet_only == 1);

    if (bHamnet)
        return (webNtpIp[1] == 143) ? "44.143.0.9 (default)" : "44.148.224.123 (default)";
    else
        return "pool.ntp.org (default)";
}

/**
 * ###########################################################################################################################
 * delivers the rxlog-page to be injected into the scaffold
 */
void sub_page_rxlog()
{
    int iRead = RAWLoRaRead;
    _create_meshcom_subheader("RX Log");

    // WQ-01: LoRa Queue panel. This fragment only carries data-* attributes;
    // mcRenderQueue() (scaffold JS, see deliver_scaffold()) turns them into
    // the bars/text, because a <script> tag injected via innerHTML never
    // runs. stat_last_window_ms == 0 means no 5-min window has completed
    // since boot -- data-win covers that for the JS side.
    uint8_t mcqPrio[6] = {0};
    txRingPrioCounts(mcqPrio);
    uint32_t mcqDedupWin = setlogDedupWindowMin(stat_last_window.newid, PRIO_STAT_INTERVAL_S, MAX_DEDUP_RING);
    uint32_t mcqAgeS = 0;
    if (stat_last_window_ms != 0)
    {
        mcqAgeS = (millis() - stat_last_window_ms) / 1000UL;
    }

    // WQ-01: the card lives inside #content_inner so it shares the 4 % left
    // margin of the log lines; fixed 600 px wide (max-width:100% keeps it on
    // a phone screen), it does not scale with the page.
    web_client.println("<div id=\"content_inner\" class=\"logoutput\">");
    web_client.println("<div class=\"cardlayout\" style=\"width:600px;max-width:100%;box-sizing:border-box;\">");
    web_client.println("<label class=\"cardlabel\">LoRa Queue</label>");
    web_client.println("<button id=\"mcqtogglebtn\" class=\"mcq-toggle\" onclick=\"mcQueueToggle(this)\">hide</button>");
    web_client.printf("<div id=\"mcq\" data-ring=\"%u\" data-p0=\"%u\" data-p1=\"%u\" data-p2=\"%u\" data-p3=\"%u\"\n",
                       (unsigned int)MAX_RING, (unsigned int)mcqPrio[0], (unsigned int)mcqPrio[1], (unsigned int)mcqPrio[2], (unsigned int)mcqPrio[3]);
    web_client.printf(" data-p4=\"%u\" data-p5=\"%u\" data-bp=\"%d\" data-bpname=\"%s\" data-qrs=\"%d\" data-qrsf=\"%d\" data-qrt=\"%d\"\n",
                       (unsigned int)mcqPrio[4], (unsigned int)mcqPrio[5], bpCurrentState(), bpStateName(), bpQrsThreshold(), bpQrsForecast((int)mcqPrio[0]), bpRefuseThreshold());
    web_client.printf(" data-win=\"%d\" data-rx=\"%lu\" data-tx=\"%lu\" data-int=\"%d\" data-newid=\"%lu\"\n",
                       (stat_last_window_ms != 0) ? 1 : 0, (unsigned long)stat_last_window.rx_ms, (unsigned long)stat_last_window.tx_ms,
                       (int)PRIO_STAT_INTERVAL_S, (unsigned long)stat_last_window.newid);
    web_client.printf(" data-dup=\"%lu\" data-dedup=\"%u\" data-dwin=\"%lu\" data-age=\"%lu\"></div>\n",
                       (unsigned long)stat_last_window.dup, (unsigned int)MAX_DEDUP_RING, (unsigned long)mcqDedupWin, (unsigned long)mcqAgeS);
    web_client.println("</div>");

    web_client.println("<div style=\"overflow:scroll;\">");
    do
    {
        // WQ-01: normal text size (was font-small) -- the page uses three sizes only:
        // title, normal (log lines, panel text), small (legend, notes, tick labels).
        web_client.printf("<p class=\"no-wrap\"><%i>%s</p>\n", iRead, ringbufferRAWLoraRX[iRead]);
        iRead = increment_mod(iRead, MAX_LOG);
    } while (RAWLoRaRead != iRead);
    web_client.println("</div></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the wx-page to be injected into the scaffold
 */
void sub_page_wx()
{
    bool bBMX = (bBMEON || bBMPON);
    char cbme[10] = {0};
    if (bBMX)
        snprintf(cbme, sizeof(cbme), " (%s)", (bmx_found ? "found" : "error"));

    char c680[10] = {0};
    if (bBME680ON)
        snprintf(c680, sizeof(c680), " (%s)", (bme680_found ? "found" : "error"));

    char c811[10] = {0};
    if (bMCU811ON)
        snprintf(c811, sizeof(c811), " (%s)", (mcu811_found ? "found" : "error"));

    _create_meshcom_subheader("WX Information");
    web_client.println("<div id=\"content_inner\">");
    web_client.println("<table class=\"table mw-600\">");
    web_client.println("<thead><tr class=\"font-bold\"><td>Item</td><td>Value</td></tr></thead>");
    web_client.printf("<tr><td>BME(P)280</td><td>%s %s</td></tr>\n", (bBMX ? "on" : "off"), cbme);
    web_client.printf("<tr><td>BME680</td><td>%s %s</td></tr>\n", (bBME680ON ? "on" : "off"), c680);
    web_client.printf("<tr><td>MCU811</td><td>%s %s</td></tr>\n", (bMCU811ON ? "on" : "off"), c811);
    web_client.printf("<tr><td>LPS33</td><td>%s (RAK)</td></tr>\n", (bLPS33 ? "on" : "off"));
    web_client.printf("<tr><td>1-Wire</td><td>%s (GPIO %i)</td></tr>\n", (bONEWIRE ? "on" : "off"), meshcom_settings.node_owgpio);
    web_client.printf("<tr><td>Temperature</td><td>%.1f &deg;C</td></tr>\n", meshcom_settings.node_temp);
    web_client.printf("<tr><td>Tout</td><td>%.1f &deg;C</td></tr>\n", meshcom_settings.node_temp2);
    web_client.printf("<tr><td>Humidity</td><td>%.1f%% rH</td></tr>\n", meshcom_settings.node_hum);
    web_client.printf("<tr><td>QFE</td><td>%.1f hPa</td></tr>\n", meshcom_settings.node_press);
    web_client.printf("<tr><td>QNH</td><td>%.1f hPa</td></tr>\n", meshcom_settings.node_press_asl);
    web_client.printf("<tr><td>Altitude asl</td><td>%i m</td></tr>\n", meshcom_settings.node_press_alt);
    web_client.printf("<tr><td>Gas</td><td>%.1f kOhm</td></tr>\n", meshcom_settings.node_gas_res);
    web_client.printf("<tr><td>eCO2</td><td>%.1f ppm</td></tr>\n", meshcom_settings.node_co2);
    #if defined(NTC_PIN) && defined(FAN_CTRL) // BOARD_TBEAM_1W
    web_client.printf("<tr><td>NTC / FAN %s</td><td>%.1f &deg;C</td></tr>\n", digitalRead(FAN_CTRL) ? "on" : "off", meshcom_settings.node_ntctemp);
    #endif
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    web_client.println("</table></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the position-page to be injected into the scaffold
 */
void sub_page_position()
{
    _create_meshcom_subheader("Position Information");
    web_client.println("<div id=\"content_inner\">");
    web_client.println("<table class=\"table mw-600\">");
    web_client.println("<thead><tr class=\"font-bold\"><td>Item</td><td>Value</td></tr></thead>");
    web_client.printf("<tr><td>Latitude</td><td>%.4lf %c</td></tr>\n", meshcom_settings.node_lat, meshcom_settings.node_lat_c);
    web_client.printf("<tr><td>Longitude</td><td>%.4lf %c</td></tr>\n", meshcom_settings.node_lon, meshcom_settings.node_lon_c);
    web_client.printf("<tr><td>Altitude</td><td>%i</td></tr>\n", meshcom_settings.node_alt);
    web_client.printf("<tr><td>Satellites</td><td>%i - %s - HDOP %i</td></tr>\n", (int)posinfo_satcount, (posinfo_fix ? "fix" : "nofix"), (int)fposinfo_hdop);
    web_client.printf("<tr><td>Rate</td><td>%i</td></tr>\n", (int)posinfo_interval);
    web_client.printf("<tr><td>Next</td><td>%i sec</td></tr>\n", (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis()) / 1000));
    web_client.printf("<tr><td>Distance</td><td>%.0lf m</td></tr>\n", posinfo_distance);
    web_client.printf("<tr><td>Dir (current)</td><td>%i&deg;</td></tr>\n", (int)posinfo_direction);
    web_client.printf("<tr><td>Dir (last)</td><td>%i&deg;</td></tr>\n", (int)posinfo_last_direction);
    web_client.printf("<tr><td>Date</td><td>%i.%02i.%02i %02i:%02i:%02i %s</td></tr>\n", meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, getTimeZone().c_str());
    web_client.printf("<tr><td>Symbol</td><td>%c %c</td></tr>\n", meshcom_settings.node_symid, meshcom_settings.node_symcd);
    web_client.printf("<tr><td>GPS</td><td>%s</td></tr>\n", (bGPSON ? "on" : "off"));
    web_client.printf("<tr><td>Track</td><td>%s</td></tr>\n", (bDisplayTrack ? "on" : "off"));
    web_client.printf("<tr><td></td><td><button onclick=\"callfunction('sendpos','')\"><i class=\"btnrefresh\"></i>Send Position</button></td></tr>\n");
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    web_client.println("</table></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the mheard-page to be injected into the scaffold
 */
void sub_page_mheard()
{
    mheardLine mheardLine;
    bool isShowing = false;
    _create_meshcom_subheader("MHeard Information");
    web_client.println("<div id=\"content_inner\">");

    for (int iset = 0; iset < MAX_MHEARD; iset++)
    {
        if (mheardCalls[iset][0] != 0x00)
        {
            if (mheardFreshMs(iset, 3UL * 60UL * 60UL * 1000UL)) // 3h (NC-02: monoton, nicht Wanduhr)
                isShowing = true;
            decodeMHeard(mheardBuffer[iset], mheardLine);
            web_client.printf("<div class=\"cardlayout\">\n");
            web_client.printf("<label class=\"cardlabel\"><a href=\"https://aprs.fi/?call=%s\" target=\"_blank\">%s</a> <span class=\"font-small\">(%s %s)</span></label>", mheardCalls[iset], mheardCalls[iset], mheardLine.mh_date.c_str(), mheardLine.mh_time.c_str());
            web_client.printf("<div class=\"flex-auto-wrap\">");
            web_client.printf("<div><span class=\"font-bold\">Type:</span><br><span>%s</span></div>", getPayloadType(mheardLine.mh_payload_type));
            web_client.printf("<div><span class=\"font-bold\">Hardware:</span><br><span>%s</span></div>", getHardwareLong(mheardLine.mh_hw).c_str());
            web_client.printf("<div><span class=\"font-bold\">Mod:</span><br><span>%01X/%01X</span></div>", (mheardLine.mh_mod >> 4), (mheardLine.mh_mod & 0x0f));
            web_client.printf("<div><span class=\"font-bold\">RSSI:</span><br><span>%4idBm</span></div>", mheardLine.mh_rssi);
            web_client.printf("<div><span class=\"font-bold\">SNR:</span><br><span>%4idB</span></div>", mheardLine.mh_snr);
            web_client.printf("<div><span class=\"font-bold\">Dist:</span><br><span>%5.1lf</span></div>", mheardLine.mh_dist);
            web_client.printf("<div><span class=\"font-bold\">NCNT:</span><br><span>%2i</span></div>", mheardLine.mh_ncount);            

            dlat = mheardLat[iset];
            clat = 'N';
            if(dlat < 0)
            {
                dlat = dlat * (-1);
                clat = 'S';
            }
            dlon = mheardLon[iset];
            clon = 'E';
            if(dlon < 0)
            {
                dlon = dlon * (-1);
                clon = 'W';
            }

            web_client.printf("<div><span class=\"font-bold\">Lat:</span><br><span>%c%06.3lf</span></div>", clat, dlat);
            web_client.printf("<div><span class=\"font-bold\">Lon:</span><br><span>%c%07.3lf</span></div>", clon, dlon);
            web_client.printf("<div><span class=\"font-bold\">Alt:</span><br><span>%4i</span></div>", mheardAlt[iset]);
            web_client.printf("</div></div>");
        }
    }
    if (!isShowing)
        web_client.println("<p>No Nodes heard so far.</p>"); // no nodes available? Tell the user
    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the path-page to be injected into the scaffold
 */
void sub_page_path()
{
    bool isShowing = false;
    mheardLine mheardLine;
    _create_meshcom_subheader("Path Information");
    web_client.println("<div id=\"content_inner\">");
    for (int iset = 0; iset < MAX_MHPATH; iset++)
    {
        if (mheardPathCalls[iset][0] != 0x00)
        {
            if (mheardPathFreshMs(iset, 3UL * 60UL * 60UL * 1000UL)) // 3h (NC-02: monoton, nicht Wanduhr)
            { // 3h
                isShowing = true;
                unsigned long lt = mheardPathEpoch[iset] + (long)(meshcom_settings.node_utcoff * 3600.0);
                web_client.printf("<div class=\"cardlayout\">\n");
                web_client.printf("<label class=\"cardlabel\"><a href=\"https://aprs.fi/?call=%s\" target=\"_blank\">%s</a> <span class=\"font-small\">(%s)</span></label>", mheardPathCalls[iset], mheardPathCalls[iset], convertUNIXtoString(lt).substring(5).c_str());
                web_client.printf("<div class=\"flex-auto-wrap\">");
                web_client.printf("<div><span class=\"font-bold\">Source Path: </span><span>%01u%s/%s</span></div>", (mheardPathLen[iset] & 0x7F), ((mheardPathLen[iset] & 0x80) ? "G" : " "), mheardPathBuffer1[iset]);
                web_client.printf("</div></div>");
            }
        }
    }
    if (!isShowing)
        web_client.println("No Paths available so far.");
    web_client.println("</div></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the message-page to be injected into the scaffold
 * Includes all messages received/stored so far. To update message while using the Message Page, there is another callback function
 * (sub_content_messages) that will only deliver the message block
 */
void sub_page_messages()
{
    _create_meshcom_subheader("Messages");
    web_client.println("<div id=\"content_inner\">");

    // tab bar filtering the panel below by data-dst; mcTab()/mcApplyTab() in
    // the scaffold do the actual filtering (sub-page <script> never runs)
    web_client.println("<div id=\"mctabs\">");
    web_client.println("<button class=\"mctab mctab-on\" data-tab=\"all\" onclick=\"mcTab(this)\">All</button>");
    web_client.println("<button class=\"mctab\" data-tab=\"*\" onclick=\"mcTab(this)\">*</button>");
    for (int i = 0; i < (int)sizeof(meshcom_settings.node_gcb) / (int)sizeof(meshcom_settings.node_gcb[0]); i++)
    {
        if (meshcom_settings.node_gcb[i] > 0 && meshcom_settings.node_gcb[i] < 100000)
        {
            web_client.printf("<button class=\"mctab\" data-tab=\"%i\" onclick=\"mcTab(this)\">%i</button>\n", meshcom_settings.node_gcb[i], meshcom_settings.node_gcb[i]);
        }
    }
    web_client.println("<button class=\"mctab\" data-tab=\"dm\" onclick=\"mcTab(this)\">DM</button>");
    web_client.println("</div>");

    // this is where the asynchronous received messages will be displayed
    web_client.println("<div id=\"messages_panel\" class=\"mw-600\">");
    sub_content_messages(); // deliver all known messages
    web_client.println("</div>");

    // the input panel for new messages
    web_client.println("<table class=\"mw-600\"><tbody><tr>");
    web_client.println("<td><label for=\"sendcall\" class=\"font-small font-bold\">DM Call (or empty):</label></td>");
    web_client.println("<td><input type=\"text\" id=\"sendcall\" name=\"sendcall\" maxlength=\"9\" size=\"9\" oninput=\"updateCharsLeft()\";></td>");
    web_client.println("</tr><tr>");
    web_client.println("<td><label for=\"messagetext\" class=\"font-small font-bold\">Message:</label><p class=\"font-small\"><span id=\"indicator_charsleft\">149</span> chars left</p></td>");
    web_client.println("<td><textarea id=\"messagetext\" name=\"messagetext\" maxlength=\"149\" rows=\"5\" cols=\"40\" oninput=\"updateCharsLeft()\";></textarea></td>");
    web_client.println("</tr><tr>");
    web_client.println("<td><button onclick=\"updateMessages()\"><i class=\"btnrefresh\"></i>Update</button></td></td><td><button id=\"sendmessage\" onclick=\"sendMessage(); updateCharsLeft(); updateMessages();\"><i class=\"btncheckmark\"></i>Send</button></td>");
    web_client.println("</tr></tbody></table>");
    web_client.println("</div>");

    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the setup-page to be injected into the scaffold
 *
 */
void sub_page_setup()
{
    _create_meshcom_subheader("Node Setup");
    web_client.println("<div id=\"content_inner\">");

    // Manual Command Section
    web_client.println("<div class=\"cardlayout\">\n");
    web_client.println("<label class=\"cardlabel\">Manual Command</label>\n");
    web_client.println("<div class=\"grid\">");
    web_client.println("<span>Enter manual command:</span>");
    web_client.println("<input type=\"text\" id=\"manualcommand\" maxlength=\"40\" size=\"20\" style=\"width:100%\">");
    web_client.println("<button onclick=\"setvalue('manualcommand', document.getElementById('manualcommand').value,false); document.getElementById('manualcommand').value='';\" style=\"justify-self:self-end;\">send command</button>");
    web_client.println("</div></div>");

    // Common Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard cardopen\">");
    web_client.println("<label class=\"cardlabel\">Common Settings</label>");
    web_client.println("<span>Open this for common settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");
    _create_setup_textinput_element("nodecall", "Call-Sign", meshcom_settings.node_call, "AB1CDE-12", "setcall", 9, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("viacall", "Via-Call-Sign", meshcom_settings.node_via, "AB1CDE-12", "setvia", 9, false, false); // create Textinput-Element including Label and Button

    web_client.println("<label for=\"country\">Country</label>");
    web_client.println("<select id=\"country\" name=\"country\">");
    for (int ic = 0; ic < 21; ic++)
    {
        if (getCountry(ic) != "none")
        {
            web_client.printf("\t<option value=\"%i\" %s>%s</option>\n", ic, (ic == meshcom_settings.node_country) ? "selected" : "", getCountry(ic).c_str());
        }
    }
    web_client.println("</select>");
    web_client.println("<button onclick=\"setvalue('setctry', document.getElementById('country').value,false)\"><i class=\"btncheckmark\"></i></button>");

    _create_setup_textinput_element("txpower", "TX Power", String(meshcom_settings.node_power), "15", "txpower", 2, false, false);                    // create Textinput-Element including Label and Button

    // CS-02: Max-Hop als Drop-down. Angeboten werden 4/3/2; hat --maxhop einen
    // Wert ausserhalb dieser Liste gesetzt (1, 5, 6), steht er zusaetzlich drin,
    // damit die Seite den echten Zustand zeigt statt ihn stillschweigend zu
    // aendern. Die Liste baut maxhop.h, dieselbe Quelle wie die serielle Pruefung.
    {
        int hops[MAXHOP_OPTION_MAX];
        int nhops = maxHopOptionList(meshcom_settings.max_hop_text, hops, MAXHOP_OPTION_MAX);

        web_client.println("<label for=\"maxhop\">Max Hop</label>");
        web_client.println("<select id=\"maxhop\" name=\"maxhop\">");
        for (int ih = 0; ih < nhops; ih++)
        {
            web_client.printf("\t<option value=\"%i\" %s>%i</option>\n", hops[ih], (hops[ih] == meshcom_settings.max_hop_text) ? "selected" : "", hops[ih]);
        }
        web_client.println("</select>");
        web_client.println("<button onclick=\"setvalue('maxhop', document.getElementById('maxhop').value,false)\"><i class=\"btncheckmark\"></i></button>");
    }

    _create_setup_textinput_element("utcoffset", "UTC Offset", String(meshcom_settings.node_utcoff, 1).c_str(), "1.0", "utcoffset", 4, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("maxv", "max. Voltage", String(meshcom_settings.node_maxv, 3), "4.125", "maxv", 5, false, false);                 // create Textinput-Element including Label and Button

    web_client.println("</div><div class=\"grid grid2\">");

    _create_setup_switch_element("display", "Display", "keep display active permanently", !bDisplayOff);        // create Switch-Element inclucing Label and Description
    //NOT USED _create_setup_switch_element("small", "small Display", "reduce content for small displays", bSMALLDISPLAY); // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("volt", "Voltage", "show batt. voltage, not percent", bDisplayVolt);           // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("mesh", "Mesh", "enable mesh/forwarding of received LoRa messages", bMESH);    // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("via", "Via", "enable via", bVIA);    // create Switch-Element inclucing Label and Description

// We support OTA only for ESP based devices, not RAK
#ifdef ESP32
// We support OTA only for ESP based devices, not RAK
#ifdef ESP32
    web_client.println("<span>Reboot into OTA Updater</span>");
    web_client.println("<button onclick=\"if(confirm('Node will reboot to OTA Updater, are you sure?')){callfunction('otaupdate', '');setTimeout(function(){window.location.reload();},10000);}\"><i class=\"btncheckmark\"></i></button>");
#endif
#endif

    web_client.println("</div></div>");

    // IP Network Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">IP Network Settings</label>");
    web_client.println("<span>Open this for network-specific settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");

    #ifndef BOARD_RAK4630
    _create_setup_textinput_element("wifissid", "SSID", String(meshcom_settings.node_ssid), "wifi-name", "setssid", 50, false, true);  // create Textinput-Element including Label and Button
    _create_setup_textinput_element("wifipassword", "WiFi Password", String(meshcom_settings.node_pwd), "", "setpwd", 50, true, true); // create Textinput-Element including Label and Button
    #endif

    _create_setup_textinput_element("ownip", "fixed IP", String(meshcom_settings.node_ip), "192.168.2.100", "setownip", 50, false, true);        // create Textinput-Element including Label and Button
    _create_setup_textinput_element("ownsn", "Subnet Mask", String(meshcom_settings.node_subnet), "255.255.255.0", "setownms", 50, false, true); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("owngw", "Gateway", String(meshcom_settings.node_gw), "192.168.2.1", "setowngw", 50, false, true);           // create Textinput-Element including Label and Button
    _create_setup_textinput_element("owndns", "DNS", String(meshcom_settings.node_dns), "192.168.2.1", "setowndns", 50, false, true);             // create Textinput-Element including Label and Button
    _create_setup_textinput_element("ownntp", "NTP", String(meshcom_settings.node_ownntp), "192.168.2.1", "setownntp", 50, false, true);          // create Textinput-Element including Label and Button

    _create_setup_textinput_element("extudp", "ext. UDP IP", String(meshcom_settings.node_extern), "192.168.100.100", "extudpip", 50, false, false); // create Textinput-Element including Label and Button

    web_client.println("</div><div class=\"grid grid2\">");
    #if defined(HAS_ETHERNET)
    _create_setup_switch_element("netmode", "Ethernet Mode", "switch between WiFi and Ethernet", meshcom_settings.node_netmode == 1);
    #endif
    _create_setup_switch_element("extudp", "ext UDP", "enable ext. UDP", bEXTUDP); // create Switch-Element inclucing Label and Description
    #ifndef BOARD_RAK4630
    _create_setup_switch_element("netconsole", "net console", "enable net console (port 2323, HMAC auth)", bNETCONSOLE); // create Switch-Element inclucing Label and Description
    #endif
    _create_setup_switch_element("gateway", "Gateway", "enable gateway", bGATEWAY);   // create Switch-Element inclucing Label and Description

    web_client.println("</div></div>");

    // Position Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Position Settings</label>");
    web_client.println("<span>Open this for position-specific settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");

    _create_setup_textinput_element("latitude", "Position Latitude (+/-)", String(meshcom_settings.node_lat, 8), "48.26940877", "setlat", 12, false, false);   // create Textinput-Element including Label and Button
    _create_setup_textinput_element("longitude", "Position Longitude (+/-)", String(meshcom_settings.node_lon, 8), "16.40922749", "setlon", 12, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("altitude", "Position Altitude (meter)", String(meshcom_settings.node_alt), "300", "setalt", 5, false, false);             // create Textinput-Element including Label and Button

    web_client.println("</div><div class=\"grid grid2\">");

    _create_setup_switch_element("gps", "GPS", "enable GPS", bGPSON);                                  // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("track", "Track", "enable display of SmartBeaconing", bDisplayTrack, TRACK_WARNING_TEXT, bDisplayTrack); // create Switch-Element inclucing Label and Description; TRK-01: Warnhinweis neben dem Switch

    web_client.println("</div></div>");

    // APRS Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">APRS Settings</label>");
    web_client.println("<span>Open this for APRS-specific settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");

    _create_setup_textinput_element("nametext", "Operator Name", String(meshcom_settings.node_name), "aprsname", "setname", 25, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aprstext", "APRS Comment", String(meshcom_settings.node_atxt), "aprstext", "atxt", 25, false, false);    // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aprssymbol", "APRS Group", String(meshcom_settings.node_symid), "S", "symid", 1, false, false);      // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aprsgroup", "APRS Symbol", String(meshcom_settings.node_symcd), "/", "symcd", 1, false, false);        // create Textinput-Element including Label and Button

    web_client.println("</div></div>");

    // Hardware Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">External Hardware Settings</label>");
    web_client.println("<span>Open this for hardware-related settings like sensors.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid3\">");

    _create_setup_textinput_element("owgpio", "1-Wire GPIO", String(meshcom_settings.node_owgpio), "36", "onewiregpio", 3, false, false); // create Textinput-Element including Label and Button

    web_client.println("</div><div class=\"grid grid2\">");

    _create_setup_switch_element("onewire", "1-Wire", "enable 1-Wire capability", bONEWIRE); // create Switch-Element inclucing Label and Description

    web_client.println("</div>");
    web_client.println("<div class=\"grid grid3\">");

    int iButtonPin = 0;
    #ifdef BUTTON_PIN
        iButtonPin = BUTTON_PIN;
    #else
        iButtonPin = 99;
    #endif

    if(meshcom_settings.node_button_pin > 0)
        iButtonPin = meshcom_settings.node_button_pin;

    _create_setup_textinput_element("ubgpio", "Userbutton GPIO", String(iButtonPin), "0", "buttongpio", 3, false, false); // create Textinput-Element including Label and Button

    web_client.println("</div>");
    web_client.println("<div class=\"grid grid2\">");

    _create_setup_switch_element("button", "Userbutton", "enable user-button", bButtonCheck); // create Switch-Element inclucing Label and Description

    web_client.println("</div>");

    web_client.println("<div class=\"grid grid3\">");

    #if defined(ANALOG_PIN)
    _create_setup_textinput_element("angpio", "Analog GPIO", String(meshcom_settings.node_analog_pin), "2", "angpio", 3, false, false);                // create Textinput-Element including Label and Button
    _create_setup_textinput_element("afactor", "Analog Factor", String(meshcom_settings.node_analog_faktor, 6), "1.000", "afactor", 10, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aslope", "Analog Slope", String(meshcom_settings.node_analog_slope, 6), "1.000", "aslope", 10, false, false); // create Textinput-Element including Label and Button
    _create_setup_textinput_element("aoffset", "Analog Offset", String(meshcom_settings.node_analog_offset, 6), "0", "aoffset", 3, false, false); // create Textinput-Element including Label and Button
    #endif

    web_client.println("</div>");
    web_client.println("<div class=\"grid grid2\">");

    #if defined(ANALOG_PIN)
    _create_setup_switch_element("analogcheck", "Analog", "enable analog GPIO measurement", bAnalogCheck); // create Switch-Element inclucing Label and Description
    #endif
    _create_setup_switch_element("bmp", "BMP280", "enable BMP280 sensor", bBMPON);                         // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("bme", "BME280", "enable BME280 sensor", bBMEON);                         // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("680", "BME680", "enable BME680 sensor", bBME680ON);                      // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("811", "MCU811", "enable MCU811 sensor", bMCU811ON);                      // create Switch-Element inclucing Label and Description
    _create_setup_switch_element("ina226", "INA226", "enable INA226 sensor", bINA226ON);                   // create Switch-Element inclucing Label and Description
    #if defined(ENABLE_AHT20)
    _create_setup_switch_element("aht20", "AHT20", "enable AHT20 sensor", bAHT20ON);                       // create Switch-Element inclucing Label and Description
    #endif
    #if defined(ENABLE_SHT21)
    _create_setup_switch_element("sht21", "SHT21", "enable SHT21 sensor", bSHT21ON);                       // create Switch-Element inclucing Label and Description
    #endif
    #if defined(ENABLE_SOFTSER)
    _create_setup_switch_element("softser", "SoftSer", "enable software serial console", bSOFTSERON);      // create Switch-Element inclucing Label and Description
    #endif

    web_client.println("</div>");
    
    web_client.println("<div class=\"grid grid3\">");
    _create_setup_textinput_element("tempoffi", "Indoor Temp Offset", String(meshcom_settings.node_tempi_off), "0.0", "tempoffsetindoor", 3, false, false);                // create Textinput-Element including Label and Button
    _create_setup_textinput_element("tempoffa", "Outdoor Temp Offset", String(meshcom_settings.node_tempo_off), "0.0", "tempoffsetoutdoor", 3, false, false); // create Textinput-Element including Label and Button
    web_client.println("</div>");

    web_client.println("</div>");

    // Groups Settings Section
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Groups / Messaging</label>");
    web_client.println("<span>Open this for messaging-groups-related settings.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid2\">");

    for (int i = 0; i < (int)sizeof(meshcom_settings.node_gcb) / (int)sizeof(meshcom_settings.node_gcb[0]); i++)
    {
        web_client.printf("<label for=\"grp%i\">Listen to Group:</label>\n", i);
        web_client.printf("<input type=\"text\" id=\"grp%i\" value=\"%i\" maxlength=\"5\" size=\"20\" placeholder=\"0\"/>\n", i, meshcom_settings.node_gcb[i]);
    }
    web_client.println("<i></i>");
    web_client.println("<button onclick=\"setvalue('setgrc', (document.getElementById('grp0').value+';'+document.getElementById('grp1').value+';'+document.getElementById('grp2').value+';'+document.getElementById('grp3').value+';'+document.getElementById('grp4').value+';'+document.getElementById('grp5').value),false)\"><i class=\"btncheckmark\"></i></button>");

    _create_setup_switch_element("nomsgall", "No MSG All", "do not show messages send to all", bNoMSGtoALL); // create Switch-Element inclucing Label and Description

    web_client.println("</div></div>");

    // Config Backup / Restore Section (CS-03)
    // The download is a plain navigation to /config.json -- the response
    // carries a Content-Disposition, so the browser saves it instead of
    // rendering it. uploadconfig() lives in the scaffold's script block
    // (this page is injected with innerHTML, an inline <script> would not run).
    web_client.println("<div class=\"cardlayout collapsablecard\">");
    web_client.println("<label class=\"cardlabel\">Config Backup / Restore</label>");
    web_client.println("<span>Open this to save the whole node configuration to a file, or to restore it.</span>\n");
    web_client.println("<button class=\"cardtoggle\" onclick=\"togglecard(this);\"><i></i></button>\n");
    web_client.println("<div class=\"grid grid2\">");
    web_client.println("<span><b>The file contains secrets in clear text</b> &ndash; the WiFi password, the web password and the BT code. Keep it like a password store.</span><i></i>");
    web_client.println("<span>Download the configuration as JSON</span>");
    web_client.println("<button onclick=\"window.location='/config.json';\">download</button>");
    web_client.println("<span>Restore a configuration file. The node checks the layout version and the checksum, applies all values at once and then reboots.</span><i></i>");
    web_client.println("<input type=\"file\" id=\"cfgfile\" accept=\".json,application/json\">");
    web_client.println("<button onclick=\"uploadconfig();\">restore</button>");
    web_client.println("</div></div>");

    web_client.println("</div>");
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * This will only deliver the preformatted messages to be loaded asyncronous into the WebUI scaffold
 */
// The ring has no reader cursor of its own. toPhoneRead only advances when a
// BLE client with an active "hello" session drains a slot, which happens
// within ~100 ms of the write -- following toPhoneRead here reliably finds
// an empty window while a phone is connected. toPhoneWrite always points at
// the oldest surviving slot (the writer fills it, then wraps toPhoneWrite
// forward), so scan the full ring from there instead. Upstream origin
// 87c6c200.
void sub_content_messages()
{
    int rendered = 0;
    int iStart = toPhoneWrite; // snapshot: the writer may advance it while we scan

    if (bDEBUG)
        Serial.printf("toPhoneWrite:%i\n", iStart);

    for (int i = 0; i < MAX_RING; i++)
    {
        int iRead = (iStart + i) % MAX_RING;

        if (BLEtoPhoneBuff[iRead][0] == 0) // 0 = slot never written since reboot
            continue;

        if (bDEBUG)
            Serial.printf("iRead:%i [1]:%02X\n", iRead, BLEtoPhoneBuff[iRead][1]);

        uint8_t toPhoneBuff[MAX_MSG_LEN_PHONE] = {0}; // we need to insert the first byte text msg flag
        uint8_t blelen = BLEtoPhoneBuff[iRead][0];    // MAXIMUM PACKET Length over BLE is 245 (MTU=247 bytes), two get lost, otherwise we need to split it up!

        if (BLEtoPhoneBuff[iRead][1] == 0x91)
        { // Mheard
          // memcpy(toPhoneBuff, BLEtoPhoneBuff[iRead]+1, blelen-1);
        }
        else if (BLEtoPhoneBuff[iRead][1] == 0x44)
        { // Data Message (JSON)
          // memcpy(toPhoneBuff, BLEtoPhoneBuff[iRead]+1, blelen);
        }
        else if (blelen >= 4 && (size_t)(blelen - 4) <= sizeof(toPhoneBuff))
        { // Text Message and Position
            uint8_t tbuffer[5];
            unsigned long unix_time = 0;
            char timestamp[21];
            String ccheck = "";

            memcpy(toPhoneBuff, BLEtoPhoneBuff[iRead] + 1, blelen - 4);
            memcpy(tbuffer, BLEtoPhoneBuff[iRead] + 1 + (blelen - 4), 4);
            unix_time = (tbuffer[0] << 24) | (tbuffer[1] << 16) | (tbuffer[2] << 8) | tbuffer[3];
            time_t unix_t = (time_t)(unix_time + (long)(meshcom_settings.node_utcoff * 60 * 60));
            struct tm *oldt = gmtime(&unix_t);
            strftime(timestamp, 20, "%Y-%m-%d %H:%M:%S", oldt);
            struct aprsMessage aprsmsg;
            uint8_t msg_type_b_lora = decodeAPRS(toPhoneBuff, blelen, aprsmsg); // print which message type we got
            int icheck = checkOwnTx(aprsmsg.msg_id);

            if (icheck >= 0)
            {
                if (own_msg_id[icheck][4] == 1)
                { // 00...not heard, 01...heard, 02...ACK
                    ccheck = "&#x2713&nbsp;";
                }

                if (own_msg_id[icheck][4] == 2)
                { // 00...not heard, 01...heard, 02...ACK
                    ccheck = "&#x2611;&nbsp;";
                }
            }

            // Textmessage
            if (msg_type_b_lora == 0x3A)
            {
                // {CET} time beacons sit in the ring for the phone app's clock
                // sync; they are not operator traffic and would light the tab
                // badges on every beacon, so the web list skips them
                if (aprsmsg.msg_payload.indexOf(":ack") < 1 && !aprsmsg.msg_payload.startsWith("{CET}"))
                {
                    String msgtxt = aprsmsg.msg_payload;
                    if (bDEBUG)
                        Serial.printf("aprsmsg.msg_source_call.c_str():%s, aprsmsg.msg_gateway_call.c_str():%s, aprsmsg.msg_destination_call.c_str():%s, aprsmsg.msg_payload.c_str():%s\n", aprsmsg.msg_source_call.c_str(), aprsmsg.msg_source_last.c_str(), aprsmsg.msg_destination_call.c_str(), aprsmsg.msg_payload.c_str());

                    if (msgtxt.indexOf('{') > 0)
                        msgtxt = aprsmsg.msg_payload.substring(0, msgtxt.indexOf('{'));

                    // WEB-03a: mesh-derived strings (payload, path, callsigns) are attacker-controlled -- escape before HTML output
                    String msgtxt_esc = htmlEscape(msgtxt);
                    String msg_source_path_esc = htmlEscape(aprsmsg.msg_source_path);
                    String msg_destination_path_esc = htmlEscape(aprsmsg.msg_destination_path);

                    // own messages (source == us): the browser's DM tab keys on the destination call
                    if (is_equ(meshcom_settings.node_call, aprsmsg.msg_source_call.c_str()))
                    {
                        String dst_esc = htmlEscape(aprsmsg.msg_destination_call);

                        web_client.printf("<div class=\"message message-send\" data-id=\"%u\" data-dst=\"%s\" data-ts=\"%lu\"><div>", aprsmsg.msg_id, dst_esc.c_str(), unix_time);

                        web_client.printf("<p class=\"font-small font-bold\">%s", ccheck.c_str());
                        web_client.printf("<a target=\"_blank\" href=\"https://aprs.fi/?call=%s\">%s</a>", msg_source_path_esc.c_str(), msg_source_path_esc.c_str());
                        web_client.printf("%s%s</p>", (char *)">", msg_destination_path_esc.c_str());

                        web_client.printf("<p class=\"font-small font-bold\">%s</p>", timestamp);
                        web_client.printf("<p class=\"font-normal\">%s</p>", msgtxt_esc.c_str());
                        web_client.printf("</div></div>");
                    }
                    // messages by others: "*" and group numbers key on the destination call as-is,
                    // a DM to us keys on the source call so the DM tab shows both directions
                    else
                    {
                        bool isGroupDst = is_equ(aprsmsg.msg_destination_call.c_str(), "*");
                        if (!isGroupDst && aprsmsg.msg_destination_call.length() > 0)
                        {
                            isGroupDst = true;
                            for (unsigned int ci = 0; ci < aprsmsg.msg_destination_call.length(); ci++)
                            {
                                if (!isDigit(aprsmsg.msg_destination_call.charAt(ci)))
                                {
                                    isGroupDst = false;
                                    break;
                                }
                            }
                        }
                        String dst_esc = htmlEscape(isGroupDst ? aprsmsg.msg_destination_call : aprsmsg.msg_source_call);

                        web_client.printf("<div class=\"message message-received\" data-id=\"%u\" data-dst=\"%s\" data-ts=\"%lu\"><div>", aprsmsg.msg_id, dst_esc.c_str(), unix_time);

                        web_client.printf("<p class=\"font-small font-bold\">%s", ccheck.c_str());
                        web_client.printf("<a target=\"_blank\" href=\"https://aprs.fi/?call=%s\">%s</a>", msg_source_path_esc.c_str(), msg_source_path_esc.c_str());
                        web_client.printf("%s%s</p>", (char *)">", msg_destination_path_esc.c_str());

                        web_client.printf("<p class=\"font-small font-bold\">%s</p>", timestamp);
                        web_client.printf("<p class=\"font-normal\">%s</p>", msgtxt_esc.c_str());
                        web_client.printf("</div></div>");
                    }

                    rendered++;
                }
            }
        }
    }

    if (rendered == 0)
    {
        web_client.printf("<p>No messages available.</p>");
    }

    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * delivers the ispectrumpage to be injected into the scaffold
 */
void sub_page_spectrum()
{
    _create_meshcom_subheader("Spectrum Scan");
    web_client.println("<div id=\"content_inner\">");
#if defined(SX1262X) || defined(SX126X) || defined(SX1262_V3) || defined(SX1262_E290) || defined(USING_SX1262)

    meshcom_settings.node_specstart = 432.0;
    meshcom_settings.node_specend = 434.0;
    meshcom_settings.node_specstep = 0.025;
    meshcom_settings.node_specsamples = 2048;

    if(getFreq() > 860)
    {
        meshcom_settings.node_specstart = 863.0;
        meshcom_settings.node_specend = 870.0;
        meshcom_settings.node_specstep = 0.100;
        meshcom_settings.node_specsamples = 2048;
    }

    float spec_curr_freq = meshcom_settings.node_specstart; // scan start frequency

    uint16_t step_pixel_width = 10;  // the amout of pixel we use for a single frequency step
    uint16_t step_pixel_height = 10; // the amout of pixel we use for a single frequency step

    uint16_t num_fsteps = roundf((meshcom_settings.node_specend - spec_curr_freq) / meshcom_settings.node_specstep);
    uint16_t current_fStep = 0;                                       // current iteration  counter
    uint16_t start_x = 60;                                            // x-position where the diagramm starts
    uint16_t start_y = 10;                                            // y-position where the diagramm starts
    uint16_t end_x = start_x + ((num_fsteps + 1) * step_pixel_width); // calculate the end. Use one more fstep as the last frequency step also starts a scan and giving a result
    uint16_t end_y = start_y + (RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE * step_pixel_height);

    uint16_t own_freq_marker_width = round(((meshcom_settings.node_bw / 1000) / meshcom_settings.node_specstep) * step_pixel_width);
    uint16_t own_freq_marker_center = start_x + (((meshcom_settings.node_freq - spec_curr_freq) / meshcom_settings.node_specstep) * step_pixel_width);
    uint16_t own_freq_marker_start = own_freq_marker_center - (own_freq_marker_width / 2);



    #if defined(BOARD_T_DECK_PRO)
        web_client.println("<p>unable to initialize spectrum scan</p>");
    #else
    if (sx126x_spectral_init_scan(spec_curr_freq) != RADIOLIB_ERR_NONE)
    {
        web_client.println("<p>unable to initialize spectrum scan</p>");
    }
    else
    #endif
    {
        web_client.printf("<svg viewbox=\"0, 0, %d, %d\" id=\"spectrum_display\">", end_x + 40, end_y + 60);

        web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:black;stroke-width:1\"/>\n", start_x, end_y, end_x, end_y);                    // X-Line at bottom
        web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:black;stroke-width:1\"/>\n", start_x, start_y, start_x, end_y);                // Y-Line at left
        web_client.printf("<text x=\"%d\" y=\"%d\" style=\"font-size: 12px; color: black;\">Freq [MHz]</text>\n", start_x, end_y + 40);                                // caption for X-Line (frequency)
        web_client.printf("<text x=\"0\" y=\"0\" f style=\"font-size: 12px; color: black;\" transform=\"translate(10, %d) rotate(-90)\")>RSSI [dBm]</text>\n", end_y); // caption for Y-Line (power bins)

        web_client.printf("<rect width=\"%d\" height=\"%d\" x=\"%d\" y=\"%d\" fill=\"rgba(0, 110, 129, 0.2)\" />", own_freq_marker_width, end_y - start_y, own_freq_marker_start, start_y); // mark the frequency we are on
        web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:rgba(114, 0, 129, 0.2); stroke-width:1\"/>\n", own_freq_marker_center, start_y, own_freq_marker_center, end_y);

        // draw y axis titles and lines
        for (uint8_t i = 0; i < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; i++)
        {
            if (i % 3 == 0)
            {                                                                                                                                                                                                            // print frequency value every 3 steps
                web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:grey; stroke-width:1\"/>\n", start_x - 10, start_y + (i * step_pixel_height), end_x, start_y + (i * step_pixel_height)); // axis lines for Y axis
                web_client.printf("<text x=\"%d\" y=\"%d\" style=\"font-size: 12px; color: black;\">-%d</text>\n", start_x - 40, start_y + (i * step_pixel_height) + 6, 11 + (i * 4));                                   // axis title for Y-axis (power)
            }
        }

        #if not defined(BOARD_T_DECK_PRO)
        while (spec_curr_freq < meshcom_settings.node_specend)
        {                                                                                                 // loop through the frequency range
            uint16_t *res = sx126x_spectral_scan_freq(spec_curr_freq, meshcom_settings.node_specsamples); // get spectrum analysis for this given frequency
            for (uint8_t i = 0; i < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; i++)
            { // loop through the power bins (usually 33)
                if (res[i] > 0)
                {                                                               // did the scan found anything above zero?
                    float alpha = (res[i] / meshcom_settings.node_specsamples); // calculate the alpha-value (transperancy) ranging 0.0 - 1.0
                    if (alpha < 0.2)
                        alpha = 0.2; // use a minimum of alpha. Smaller values might be hard to see
                    web_client.printf("<rect width=\"%d\" height=\"%d\" x=\"%d\" y=\"%d\" fill=\"rgba(0,0,0,%0.3f)\"/>", step_pixel_width, step_pixel_height, start_x + (current_fStep * step_pixel_width), start_y + (i * step_pixel_height), alpha);
                }
            }

            // draw scale values
            if (current_fStep % 5 == 0)                                                                                                                                                                                                      // draw axis line every 5 steps
                web_client.printf("<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:grey; stroke-width:1\"/>\n", start_x + (current_fStep * step_pixel_width), end_y, start_x + (current_fStep * step_pixel_width), end_y + 10); // axis lines for X-axis
            if (current_fStep % 10 == 0)                                                                                                                                                                                                     // draw axis title every 10 steps
                web_client.printf("<text x=\"%d\" y=\"%d\" style=\"font-size: 12px; color: black;\">%.3f</text>\n", start_x + (current_fStep * step_pixel_width) - 10, end_y + 25, spec_curr_freq);                                          // axis title for X-axis (frequency)

            delay(50); // lets wait for a moment (the example code used 100ms but 50ms seems to work, too)
            yield();   // this loop runs for a long time, pet the watchdog and give other tasks a chance to operate
            current_fStep++;
            spec_curr_freq += meshcom_settings.node_specstep;
        }
        #endif

        web_client.println("</svg>");

        #if not defined(BOARD_T_DECK_PRO)
        sx126x_spectral_finish_scan(); // finish scan, return to normale lora operation
        #endif
    }

    web_client.printf("<p>The Node scans the frequency range (%.3fMHz - %.3fMHz) in small steps using a given scan bandwith (%.3fMHz).</p>\n", meshcom_settings.node_specstart, meshcom_settings.node_specend, meshcom_settings.node_specstep);
    web_client.printf("<p>It performs %i measurements each step and puts them into %i power-bins. That is why you will see several measurement-dots for each step. The darker the dot, the more measurements have been put into that certain power bin.</p>", meshcom_settings.node_specsamples, RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE);
#else
    web_client.println("<p>Spectrum scan is not supported on this device.</p>");
#endif
    web_client.println("</div>");
}

/**
 * ###########################################################################################################################
 * delivers the info-page to be injected into the scaffold
 */
void sub_page_info()
{
    _create_meshcom_subheader("Node Information");
    web_client.println("<div id=\"content_inner\">");

    web_client.println("<table class=\"table mw-600\">");
    web_client.println("<thead><tr class=\"font-bold\"><td>Item</td><td>Value</td></tr></thead>");

    web_client.printf("<tr><td>Firmware</td><td>Meshcom %-4.4s%s<br>(build: %s / %s)<br>(flash-version %i)</td></tr>\n", SOURCE_VERSION, SOURCE_VERSION_WEB_SUB, __DATE__, __TIME__, meshcom_settings.node_fversion);
    web_client.printf("<tr><td>Start Date</td><td>%s</td></tr>\n", meshcom_settings.node_update);
    web_client.printf("<tr><td>Call</td><td>%s</td></tr>\n", meshcom_settings.node_call);
    web_client.printf("<tr><td>Hardware</td><td>%s</td></tr>\n", getHardwareLong(BOARD_HARDWARE).c_str());
    web_client.printf("<tr><td>UTC offset</td><td>%.1f [%s]</td></tr>\n", meshcom_settings.node_utcoff, cTimeSource);
    // BAT-01: global_batt==0.0 is the established "no reading" convention (grounded pin, or
    // the ADC-path no-battery detection in batt_functions.cpp) -- same check the on-device
    // displays already use, see loop_functions.cpp.
    if(global_batt == 0.0)
        web_client.printf("<tr><td>Battery</td><td>USB (no battery)</td></tr>\n");
    else
        web_client.printf("<tr><td>Battery</td><td>%.3fV (%d%%) max %.3fV</td></tr>\n", global_batt / 1000.0, global_proz, meshcom_settings.node_maxv);
    web_client.printf("<tr><td>Settings</td><td>");
    web_client.printf("Gateway: %s<br>", (bGATEWAY ? "on" : "off"));
    if (!bAnalogCheck)
        web_client.printf("Analog: off<br>");
    else if (meshcom_settings.node_analog_pin <= 0 || meshcom_settings.node_analog_pin >= 99)
        web_client.printf("Analog: on (GPIO not set, measurement paused)<br>");
    else
        web_client.printf("Analog: on (GPIO %i)<br>", meshcom_settings.node_analog_pin);
    web_client.printf("Mesh: %s<br>", (bMESH ? "on" : "off"));
    web_client.printf("Routing: %s<br>", (bVIA ? "on" : "off"));
    web_client.printf("Button: %s<br>", (bButtonCheck ? "on" : "off"));
    web_client.printf("Debug: %s<br>", (bDEBUG ? "on" : "off"));
    web_client.printf("Debug LoRa: %s<br>", (bLORADEBUG ? "on" : "off"));
    web_client.printf("Debug GPS: %s<br>", (iGPSDEBUG ? "on" : "off"));
    web_client.printf("Debug WX: %s<br>", (bWXDEBUG ? "on" : "off"));
    web_client.printf("Debug BLE: %s<br>", (bBLEDEBUG ? "on" : "off"));
    web_client.printf("</td></tr>\n");
    web_client.printf("<tr><td>APRS text</td><td>%s</td></tr>\n", meshcom_settings.node_atxt);
    web_client.printf("<tr><td>Mesh settings</td><td>");
    web_client.printf("max_hop_text: %i<br>", meshcom_settings.max_hop_text);
    web_client.printf("max_hop_pos: %i<br>", meshcom_settings.max_hop_pos);
    web_client.printf("</td></tr>\n");
    web_client.printf("<tr><td>Country</td><td>%s</td></tr>\n", getCountry(meshcom_settings.node_country).c_str());
    web_client.printf("<tr><td>Frequency</td><td>%.4f MHz</td></tr>\n", getFreq());
    web_client.printf("<tr><td>Bandwidth</td><td>%.0f kHz</td></tr>\n", getBW());
    web_client.printf("<tr><td>Spreading Factor (SF)</td><td>%i</td></tr>\n", getSF());
    web_client.printf("<tr><td>Coding Rate (CR)</td><td>%i</td></tr>\n", getCR());
    web_client.printf("<tr><td>TX Power</td><td>%i dBm (%.2f mW)</td></tr>\n", getPower(), 1000 * powf(10, ((float)getPower() - 30) / 10));

    #if defined(HAS_ETHERNET)
    web_client.printf("<tr><td>Netmode</td><td>%s</td></tr>\n", (meshcom_settings.node_netmode == 1 ? "Ethernet" : "WiFi"));
    #endif

    #ifndef BOARD_RAK4630
    if(meshcom_settings.node_netmode == 0)
    {
        if (bWIFIAP)
            web_client.printf("<tr><td>WiFi SSID</td><td>%s</td></tr>\n", cBLEName);
        else
        {
            web_client.printf("<tr><td>WiFi SSID</td><td>%s</td></tr>\n", meshcom_settings.node_ssid);
            // WEB-02: shows which AP the node associated with when several APs share the same SSID (mesh sets)
            web_client.printf("<tr><td>WiFi BSSID</td><td>%s</td></tr>\n", WiFi.BSSIDstr().c_str());
        }

        web_client.printf("<tr><td>WiFi AP</td><td>%s</td></tr>\n", (bWIFIAP ? "yes" : "no"));
        web_client.printf("<tr><td>WiFi RSSI</td><td>%i</td></tr>\n", WiFi.RSSI()); 
        web_client.printf("<tr><td>WiFi POWER SET</td><td>%i dBm\n</td></tr>\n", WiFi.getTxPower()/4);
    }
    #endif

    // wenn WIFI unterbrochen wird
    // if(meshcom_settings.node_hasIPaddress && strcmp(meshcom_settings.node_ip, "0.0.0.0") == 0)
    //    meshcom_settings.node_hasIPaddress = false;
    //    web_client.printf("<tr><td><b>hasIpAddress</b></td><td>%s</td></tr>\n", (meshcom_settings.node_hasIPaddress?"yes":"no"));
    
    web_client.printf("<tr><td>hasIpAddress</td><td>%s</td></tr>\n", (meshcom_settings.node_hasIPaddress ? "yes" : "no"));

    if (meshcom_settings.node_hasIPaddress)
    {
        web_client.printf("<tr><td>IP address</td><td>%s</td></tr>\n", meshcom_settings.node_ip);
        web_client.printf("<tr><td>SUB-MASK</td><td>%s</td></tr>\n", meshcom_settings.node_subnet);

        if (!bWIFIAP)
        {
            web_client.printf("<tr><td>GW address</td><td>%s</td></tr>\n", meshcom_settings.node_gw);
            web_client.printf("<tr><td>DNS address</td><td>%s</td></tr>\n", meshcom_settings.node_dns);
            web_client.printf("<tr><td>NTP address</td><td>%s</td></tr>\n", getEffectiveNtpServer().c_str());
        }
    
    }

    if (bINA226ON)
    {
        web_client.println("<tr><td>INA226</td><td>");
        web_client.printf("vBUS<: %.2fV<br>", meshcom_settings.node_vbus);
        web_client.printf("vSHUNT: %.2fmV<br>", meshcom_settings.node_vshunt);
        web_client.printf("vCURRENT: %.1fmA<br>", meshcom_settings.node_vcurrent);
        web_client.printf("vPOWER: %.1fmW<br>", meshcom_settings.node_vpower);
        web_client.printf("</td></tr>\n");
    }

    if (bAnalogCheck)
    {
        web_client.println("<tr><td>Analog</td><td>");
        if (meshcom_settings.node_analog_pin <= 0 || meshcom_settings.node_analog_pin >= 99)
            web_client.printf("ANALOG GPIO: not set (measurement paused)<br>");
        else
            web_client.printf("ANALOG GPIO: %i<br>", meshcom_settings.node_analog_pin);
        web_client.printf("Factor: %.4fV<br>", meshcom_settings.node_analog_faktor);
        web_client.printf("Value: %.2fV<br>", fAnalogValue);
        web_client.printf("</td></tr>\n");
    }

    #if defined ENABLE_RTC
        if (bRTCON)
        {
            web_client.printf("<tr><td>RTC UTC Date/Time</td><td>%s</td></tr>\n", getStringRTCNow().c_str());
        }
    #endif

    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    // web_client.printf("<tr><td></td><td></td></tr>\n", );
    web_client.println("</table></div>");
    web_client.println(); // The HTTP response ends with another blank line
}

void sub_page_mcp23017()
{

    char onclick[100];
    char caption[40];
    char id[40];
    char value[40];

    uint16_t t_io = meshcom_settings.node_mcp17io;
    uint16_t t_out = meshcom_settings.node_mcp17out;
    uint16_t t_in = meshcom_settings.node_mcp17in;

    _create_meshcom_subheader("MCP23017 Status");
    web_client.println("<div id=\"content_inner\">");

    web_client.println("<table class=\"table mw-600\">");
    web_client.println("<colgroup>");
    web_client.println("<col style=\"width: 10%;\">");
    web_client.println("<col style=\"width: 16%;\">");
    web_client.println("<col style=\"width: 49%;\">");
    web_client.println("<col style=\"width: 15%;\">");
    web_client.println("<col style=\"width: 10%;\">");
    web_client.println("</colgroup>");
    web_client.println("<thead><tr class=\"font-bold\"><td>PORT</td><td>In/Out</td><td>Name</td><td>Status</td><td>Set</td></tr></thead>");


    if(bDEBUG)
    {
        Serial.printf("t_out = %i\n", t_out);
        Serial.printf("t_in = %i\n", t_in);
    }


    for (int io = 0; io < 16; io++)
    {
        bool bOut = false;
        if ((t_io & 0x0001) == 0x0001)
            bOut = true;

        bool bOutValue = false;
        if ((t_out & 0x0001) == 0x0001)
            bOutValue = true;

        bool bInValue = false;
        if ((t_in & 0x0001) == 0x0001)
            bInValue = true;

        char cAB = 'B';
        int iAB = io - 8;
        if (io < 8)
        {
            cAB = 'A';
            iAB = io;
        }

        if(bDEBUG)
            Serial.printf("Port %c%i has mask %i and t_in %i and t_out %i\n",cAB, iAB, t_io, t_in, t_out);

        web_client.printf("<tr><td>[%c%i]</td><td>", cAB, iAB);
        snprintf(onclick, 100, "setvalue('mcpio%c%i','%s',true)", cAB, iAB, bOut ?"in":"out");
        snprintf(caption, 4, "%s",  bOut ?"out":"in");
        uic_button(&web_client, onclick, caption);
        web_client.println("</td><td>");

        snprintf(id, 40, "mcpname%c%i", cAB, iAB);
        snprintf(value, 100, "%s", meshcom_settings.node_mcp17t[io]);
        uic_input(&web_client, id, (char*)"", value);

        snprintf(onclick, 100, "setvalue('mcpname%c%i', document.getElementById('mcpname%c%i').value,true);", cAB, iAB, cAB, iAB);
        snprintf(caption, 4,  "set");
        uic_button(&web_client, onclick, caption);

        web_client.println("</td>");

        if (bOut)
        {
                web_client.printf("<td>%s</td><td>", (bOutValue ? "HIGH" : "LOW"));
                snprintf(onclick, 100, "setvalue('mcpout%c%i','%s',true)", cAB, iAB, (bOutValue ? "off" : "on"));
                snprintf(caption, 5,  "%s", (bOutValue ? "LOW" : "HIGH"));
                uic_button(&web_client, onclick, caption);
                web_client.println("</td></tr>");
        }
        else
        {
            if (meshcom_settings.node_mcp17t[io][0] == 0x00)
                web_client.printf("<td>%s</td><td></td></tr>\n", (bInValue ? "HIGH" : "LOW"));
            else
                web_client.printf("<td><b>%s</b></td><td></td></tr>\n", (bInValue ? "HIGH" : "LOW"));
        }

        t_io >>= 1;
        t_out >>= 1;
        t_in >>= 1;
    }

    web_client.println("<tr><td colspan=\"5\">");
    snprintf(onclick, 100, "setvalue('mcpclear','',true)");
    snprintf(caption, 10,  "%s", "clear all");
    uic_button(&web_client, onclick, caption);
    web_client.println("</td></tr></table></div>");
}

/**
 * ###########################################################################################################################
 * Sends a valid HTTP header. Takes care of the status code.
 * We may need to set more status_text cases once we need them.
 */
void send_http_header(uint16_t http_status_code, uint8_t content_type)
{
    String status_text = "";
    switch (http_status_code)
    {
    case 200:
        status_text = "OK";
        break; // use this when ever a request was successful
    case 400:
        status_text = "Bad Request";
        break; // use this when a request body was rejected (CS-03 config upload)
    case 401:
        status_text = "Unauthorized";
        break; // use this when ever a request was successful
    case 404:
        status_text = "Not Found";
        break; // use this if a request was not known
    case 422:
        status_text = "Unprocessable Entity";
        break; // use this if a parameter was not processable (e.g. out of bounds or somehow wrong)
    default:
        status_text = "Unknown"; // fallback
    }

    // Serial.println("STATUS_TEXT="+status_text);
    web_client.printf("HTTP/1.1 %i %s \n", http_status_code, status_text.c_str());
    if (content_type == RESPONSE_TYPE_JSON)
        web_client.println("Content-type:application/json");
    else
        web_client.println("Content-type:text/html");
    web_client.println("Access-Control-Allow-Origin: *"); // tell modern browsers that CORS is okay for us
    web_client.println("Access-Control-Allow-Methods: GET, POST, OPTIONS");
    web_client.println("Access-Control-Allow-Headers: access-control-allow-headers,access-control-allow-methods,access-control-allow-origin, Origin, Content-Type, Accept");
    web_client.println("Connection: close");                                  // tell broser that the connection will be closed (in opposite to keep-alive)
    web_client.println("Cache-Control: no-cache, no-store, must-revalidate"); // set caching policy
    web_client.println("Pragma: no-cache");                                   // Disable caching or request/respinse
    web_client.println("Expires: 0");                                         // set cache expiring time to 0
    web_client.println();                                                     // two CR-LF marks the end of the header
}

/**
 * Creates the Sub-Header containing the Title and the Date/Time when that page was created
 */
void _create_meshcom_subheader(String title)
{
    web_client.printf("<div id=\"content_title\"><p class=\"font-small\">%i-%02i-%02i&nbsp;%02i:%02i:%02i&nbsp;%s&nbsp;[%s]</p><h1 class=\"font-bold\">%s</h1></div>", meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, getTimeZone().c_str(), cTimeSource, title.c_str());
}

/**
 * ###########################################################################################################################
 * Creates a common setup text-input elementsrc/web_functions/web_functions.cpp src/web_functions/web_setup.cpp including label and apply-button
 * Parameter inputValue is chosen to be String so conversions from integers, floats, etc is easier.
 *
 * @param id the DOM id of that element. Must be unique.
 * @param labelText the text in the label
 * @param inputValue the current value of that field
 * @param placeHolder the placeholder that is displayed if the textinput is empty
 * @param parameterName the name of the parameter wich is used to identify the parameter
 * @param maxlength the maximum allowed input length
 * @param isPassword if set to TRUE, the input will be a password-type input
 */
void _create_setup_textinput_element(const char id[], const char labelText[], String inputValue, const char placeHolder[], const char  parameterName[], uint8_t maxlength, bool isPassword, bool needConfirm){
    char onclick[100]; 
    char caption[100];
    snprintf(onclick, 100, "setvalue('%s', document.getElementById('%s').value,false)", parameterName, id);
    snprintf(caption, 100, "<i class=\"btncheckmark\"></i>");

    web_client.printf("<label for=\"%s\">%s :</label>\n", id, labelText);
    web_client.printf("<input type=\"%s\" name=\"%s\" id=\"%s\" value=\"%s\" maxlength=\"%i\" size=\"10\" placeholder=\"%s\">\n", isPassword ? "password" : "text", id, id, inputValue.c_str(), maxlength, placeHolder);

    if(needConfirm) {
        char confirm[200];
        snprintf(confirm, sizeof(confirm), "Are you sure you want to set &quot;%s&quot; to &quot;'+document.getElementById('%s').value+'&quot;?", labelText, id);
        uic_button(&web_client, onclick, caption, confirm);
    } else {
         uic_button(&web_client, onclick, caption);
    }
}


/**
 * ###########################################################################################################################
 * Creates a common setup switch-input element including label and description
 *
 * @param id the DOM id of that element. Must be unique.
 * @param labelText the text in the label
 * @param descriptionText the smaller text in brackets
 * @param checked TRUE, if the switch should be displayed as activated
 * @param warnText TRK-01: optionaler Warnhinweis-Text neben dem Switch; nullptr = kein Hinweis
 * @param warnVisible TRK-01: TRUE, wenn der Warnhinweis beim Seitenaufbau sichtbar sein soll
 */
void _create_setup_switch_element(const char id[], const char labelText[], const char descriptionText[], bool checked, const char warnText[], bool warnVisible)
{
    web_client.printf("<label for=\"%s\">%s <span class=\"font-small\">(%s)</span>", id, labelText, descriptionText);
    if (warnText != nullptr)
    { // TRK-01: zweiter Span mit der id "<id>_warn", damit setvalue() ihn live umschalten kann
        web_client.printf("<span id=\"%s_warn\" class=\"font-small\" style=\"color:var(--mcred)%s\"> %s</span>", id, warnVisible ? "" : ";display:none", warnText);
    }
    web_client.println("</label>");
    web_client.printf("<input type=\"checkbox\" role=\"switch\" id=\"%s\" %s onchange=\"setvalue(this.id,this.checked?'on':'off',false)\"/>\n", id, checked ? "checked" : "");
}

/**
 * ###########################################################################################################################
 * handles incoming sendmessage request
 * ToDo: this might be better located anywhere else - maybe nodefunctioncalls?
 * ToDo: switch response to JSON
 */
void send_message(String web_header)
{
    if ((web_header.indexOf("&tocall=") >= 0) && (web_header.indexOf("&message=") >= 0))
    { // check if all neccessary parameters are there
        if (web_header.lastIndexOf(" HTTP/1.1") >= 0)
        {
            web_header = web_header.substring(0, web_header.lastIndexOf(" HTTP/1.1")); // remove last occurance of " HTTP/1.1" wich is path of the html header
        }
        String tocall = web_header.substring(web_header.indexOf("&tocall=") + 8, web_header.indexOf("&message"));
        String message = web_header.substring(web_header.indexOf("&message=") + 9);

        // replace Percent-Coded Chars with its original Char (e.g.: replace %20 with SPACE)
        tocall = decodeURLPercentCoding(tocall);
        message = decodeURLPercentCoding(message);

        tocall.trim();
        message.trim();

        tocall.toUpperCase();

        // BP-09: default outcome string, overridden below by the actual
        // sendMessage() result. The page's own sendMessage() JS (further
        // below, search for "onreadystatechange") tests this response for
        // "sendmessage ok" and only clears the input fields on a match, so
        // a refused or dropped message leaves the operator's text on screen.
        // Keep the four strings and that test in sync.
        const char *bp_result_text = "sendmessage ok";

        if (message.length() > 0)
        {
            if (tocall.length() > 0)
                message = ":{" + tocall + "}" + message;
            // snprintf(message_text, sizeof(message_text), ":{%s}%s", message_call.c_str(), message.c_str());
            else
                message = ":" + message;
            // snprintf(message_text, sizeof(message_text), ":%s", message.c_str());
            // force whole Message to have a maximum length of 150 chars including a destination callsign
            if (message.length() > 150)
                message = message.substring(0, 150);

            // We expect a char array instead of a String, so we need to convert the string to char array
            // ToDo: We might think about changing everything to String instead of Char Array later.
            // That might be easier to handle, we won't waste memory and we cannot forget the NULL-Char at the end.
            char message_text[200];
            strncpy(message_text, message.c_str(), sizeof(message_text)); // c_str() automagically adds a NULL-character to the end

            int iml = strlen(message_text);
            // new massage has to be different from previous message
            if (iml > 0)
            {
                hasMsgFromPhone = true;
                setMsgOrigin(ORIGIN_WEB);   // BP-01: notice goes back to the web GUI
                int bp_rc = sendMessage(message_text, iml);
                setMsgOrigin(ORIGIN_NONE);
                hasMsgFromPhone = false;
                // Serial.print("Message send: ");
                // Serial.println(message_text);

                switch (bp_rc)
                {
                    case BP_SEND_REFUSED: bp_result_text = "sendmessage refused"; break;
                    case BP_SEND_DROPPED: bp_result_text = "sendmessage dropped"; break;
                    case BP_SEND_INVALID: bp_result_text = "sendmessage invalid"; break;
                    default:              bp_result_text = "sendmessage ok";      break;
                }
            }
        }

        web_client.println(bp_result_text);
    }
    else
    {
        web_client.println("sendmessage failed");
    }
    web_client.println(); // The HTTP response ends with another blank line
}

/**
 * ###########################################################################################################################
 * Tries to execute a Web-API function call.
 * This will return a JSON containing the result of the function execution ("ok" or "failed")
 */
void call_function(String web_header)
{
    web_header = web_header.substring(web_header.indexOf("/callfunction/?") + 15, web_header.indexOf("HTTP/1.1"));
    web_header.trim();
    funCallStruct functionData;

    if (web_header.indexOf("=") > 0)
    {
        functionData.functionName = decodeURLPercentCoding(web_header.substring(0, web_header.indexOf("=")));
        functionData.functionParameter = decodeURLPercentCoding(web_header.substring(web_header.indexOf("=") + 1));
    }
    else
    {
        functionData.functionName = decodeURLPercentCoding(web_header);
        functionData.functionParameter = "";
    }

    webFunctionCall(&functionData); // try to execute that command

    send_http_header(functionData.returnCode == WF_RETURNCODE_OKAY ? 200 : 422, RESPONSE_TYPE_JSON); // send header, either 200 if command was executed or 422 if not

    // JSN-01: functionName is percent-decoded, attacker-controlled query input
    // and used to land here unescaped via a raw %s -- a '"' or '\' in it broke
    // the response. Build through a JsonDocument instead: ArduinoJson escapes
    // both the key and the value on serialise.
    JsonDocument doc;
    doc[functionData.functionName] = (functionData.returnCode == WF_RETURNCODE_OKAY) ? "ok" : "failed"; // {"functionName":"ok|failed"}
    serializeJson(doc, web_client);
    web_client.println();
}

/**
 * ###########################################################################################################################
 * Set the Value of a parameter (if known)
 * This will return a JSON containing the result like {"parameterName":"parameterValue"} using a HTTP return code (e.g. 200 or 422)
 */
void setparam(String web_header)
{

    web_header = web_header.substring(web_header.indexOf("/setparam/?") + 11, web_header.indexOf(" HTTP/1.1"));
    String param_name = decodeURLPercentCoding(web_header.substring(0, web_header.indexOf("=")));
    String param_value = decodeURLPercentCoding(web_header.substring(web_header.indexOf("=") + 1));
    param_name.toLowerCase();

    setupStruct setupData;
    setupData.paramName = param_name;
    setupData.paramValue = param_value;
    setupData.returnCode = 255;
    setupData.returnValue = "";

    webSetup_setParam(&setupData);

    //Serial.printf("pName: %s\n", setupData.paramName);
    //Serial.printf("pValue: %s\n", setupData.paramValue);
    //Serial.printf("retcode: %i\n", setupData.returnCode);
    //Serial.printf("retvalue: %s\n", setupData.returnValue);

    if (setupData.returnCode == WS_RETURNCODE_OKAY)
        send_http_header(200, RESPONSE_TYPE_JSON);
    else
        send_http_header(422, RESPONSE_TYPE_JSON);

    // JSN-01: paramName/returnValue are unescaped user input (SSID, password,
    // node_atxt free text, the static IP fields, ...) landing here via a raw
    // %s -- a '"' broke the response. Build via ArduinoJson so both key and
    // value are escaped on serialise. Example: {"returncode":1,"setcall":"AB1CDE-12"}
    JsonDocument doc;
    doc["returncode"] = setupData.returnCode;
    doc[setupData.paramName] = setupData.returnValue;
    serializeJson(doc, web_client);
    web_client.println();
}

/**
 * ###########################################################################################################################
 * Get the Value of a parameter (if known)
 * This will return a JSON containing the result like {"parameterName":"parameterValue"} using a HTTP return code (e.g. 200 or 422)
 */
void getparam(String web_header)
{
    // CS-04, zwei Fehler in zwei Zeilen -- beide auf DK5EN-98 nachgestellt:
    //
    //  1. Gesucht wurde "/setparam/?" statt "/getparam/?". indexOf() liefert
    //     dann -1, und substring(-1+11 = 10, ...) schneidet den Namen aus der
    //     falschen Stelle des Headers: aus "GET /getparam/?gateway HTTP/1.1"
    //     wurde der Parametername "ram/?gateway" -> immer returncode 2.
    //  2. Bei vorhandenem "=" wurde substring(indexOf("=")) genommen, also der
    //     Teil AB dem Gleichheitszeichen. Der Kommentar sagt das Gegenteil
    //     ("We only do need the parameter Name"): "/getparam/?gateway=" ergab
    //     den Namen "=". Richtig ist substring(0, indexOf("=")).
    //
    // Damit war JEDES Lesen ueber die Web-API defekt, waehrend /setparam/
    // funktionierte -- die Web-GUI faellt es nicht auf, weil sie ihre Werte aus
    // den gerenderten Seiten nimmt, nicht ueber /getparam/.
    web_header = web_header.substring(web_header.indexOf("/getparam/?") + 11, web_header.indexOf(" HTTP/1.1"));
    if (web_header.indexOf("=") > 0)
    {
        web_header = web_header.substring(0, web_header.indexOf("=")); // maybe there is an unintended "=" or anything more. We only do need the parameter Name.
    }

    web_header.trim();

    String param_name = decodeURLPercentCoding(web_header);
    param_name.toLowerCase();

    setupStruct setupData;
    setupData.paramName = param_name;
    setupData.paramValue = "";
    setupData.returnCode = 255;
    setupData.returnValue = "";

    webSetup_getParam(&setupData);

    // Serial.printf("pName: %s\n", setupData.paramName);
    // Serial.printf("pValue: %s\n", setupData.paramValue);
    // Serial.printf("retcode: %i\n", setupData.returnCode);
    // Serial.printf("retvalue: %s\n", setupData.returnValue);

    if (setupData.returnCode == WS_RETURNCODE_OKAY)
        send_http_header(200, RESPONSE_TYPE_JSON);
    else
        send_http_header(422, RESPONSE_TYPE_JSON);

    // JSN-01: same fix as setparam() above -- build via ArduinoJson instead
    // of an unescaped %s. Example: {"returncode":1,"setcall":"AB1CDE-12"}
    JsonDocument doc;
    doc["returncode"] = setupData.returnCode;
    doc[setupData.paramName] = setupData.returnValue;
    serializeJson(doc, web_client);
    web_client.println();
}