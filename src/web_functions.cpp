#include <Arduino.h>

#include <configuration.h>
#include <debugconf.h>

#include <web_functions.h>

#include <command_functions.h>
#include <mheard_functions.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>

#include <time.h>

#include <lora_setchip.h>

#include <rtc_functions.h>

#ifdef ESP32
    // WIFI
    #include <WiFi.h>
    #include <WiFiClient.h>
    #include <ESPmDNS.h>

    // WebServer
    WiFiServer web_server(80);
    
    WiFiClient web_client;

    void web_client_html(WiFiClient web_client);
#else
    #include <SPI.h> 
    #include <RAK13800_W5100S.h> // Click to install library: http://librarymanager/All#RAK13800-W5100S
    
    EthernetServer web_server(80);

    EthernetClient web_client;

    void web_client_html(EthernetClient web_client);
#endif

String web_header;

//CALL
String message_call="";

// Current time
unsigned long web_currentTime = millis();
// Previous time
unsigned long web_previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long web_timeoutTime = 2000;

int idx_text=0;
int idx_text_call_end=0;
int idx_text_end=0;

char message_text[200];

int web_page_state = 0;

bool bweb_server_running = false;

// WEBSERVER
void startWebserver()
{
    if(bweb_server_running)
        return;

    if(!meshcom_settings.node_hasIPaddress)
        return;


#ifdef ESP32

    web_server.stop();

    // Set up mDNS responder:
    // - first argument is the domain name, in this example
    //   the fully-qualified domain name is "esp32.local"
    // - second argument is the IP address to advertise
    //   we send our IP address on the WiFi network
    if (!MDNS.begin(meshcom_settings.node_call))
    {
        Serial.println("Error setting up MDNS responder!");
        return;
    }
    
    if(bDEBUG)
        Serial.println("mDNS responder started");
        
#endif

    web_server.begin();

    bweb_server_running = true;
}

void stopWebserver()
{
    bweb_server_running = false;
}

void loopWebserver()
{
    if(!bweb_server_running)
        return;

    if(!meshcom_settings.node_hasIPaddress)
        return;

    web_client = web_server.available(); // Create a client connection.

    // HTML Page formating
    if (web_client)
    {                             // If a new client connects,
        web_client_html(web_client);
    }

    // Close the connection
    web_client.stop();
}

#ifdef ESP32
    void web_client_html(WiFiClient web_client)
#else
    void web_client_html(EthernetClient web_client)
#endif
{
    web_header="";

    web_currentTime = millis();
    web_previousTime = web_currentTime;

    if(bDEBUG)
        Serial.println("New Client.");          // print a message out in the serial port

    String web_currentLine = "";                // make a String to hold incoming data from the client

    while (web_client.connected() && web_currentTime - web_previousTime <= web_timeoutTime)
    {  // loop while the client's connected
        web_currentTime = millis();
        if (web_client.available())
        {
            // if there's bytes to read from the client,
            char c = web_client.read();             // read a byte, then
            
            if(bDEBUG)
                Serial.write(c);                    // print it out the serial monitor
            
            web_header += c;

            if (c == '\n')
            {                    // if the byte is a newline character
                // if the current line is blank, you got two newline characters in a row.
                // that's the end of the client HTTP request, so send a response:
                if (web_currentLine.length() == 0) {
                // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                // and a content-type so the client knows what's coming, then a blank line:
                web_client.println("HTTP/1.1 200 OK");
                web_client.println("Content-type:text/html");
                web_client.println("Connection: close");
                web_client.println();
                
                bool bPhoneReady = false;
                if (isPhoneReady == 1)
                    bPhoneReady = true;

                bool bRefresh = false;

                // turns the Button on and off
                if (web_header.indexOf("GET /display/on") >= 0)
                {
                    commandAction((char*)"--display on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /display/off") >= 0)
                {
                    commandAction((char*)"--display off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /gps/on") >= 0)
                {
                    commandAction((char*)"--gps on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /gps/off") >= 0)
                {
                    commandAction((char*)"--gps off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /track/on") >= 0)
                {
                    commandAction((char*)"--track on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /track/off") >= 0)
                {
                    commandAction((char*)"--track off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /button/on") >= 0)
                {
                    commandAction((char*)"--button on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /button/off") >= 0)
                {
                    commandAction((char*)"--button off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /bmp/on") >= 0)
                {
                    commandAction((char*)"--bmp on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /bmp/off") >= 0)
                {
                    commandAction((char*)"--bmp off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /bme/on") >= 0)
                {
                    commandAction((char*)"--bme on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /bme/off") >= 0)
                {
                    commandAction((char*)"--bme off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /bme680/on") >= 0)
                {
                    commandAction((char*)"--680 on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /bme680/off") >= 0)
                {
                    commandAction((char*)"--680 off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /mcu/on") >= 0)
                {
                    commandAction((char*)"--811 on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /mcu/off") >= 0)
                {
                    commandAction((char*)"--811 off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /mesh/on") >= 0)
                {
                    commandAction((char*)"--mesh on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /mesh/off") >= 0)
                {
                    commandAction((char*)"--mesh off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /reboot") >= 0)
                {
                    commandAction((char*)"--reboot", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /onewire/on") >= 0)
                {
                    commandAction((char*)"--onewire on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /onewire/off") >= 0)
                {
                    commandAction((char*)"--onewire off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /ina226/on") >= 0)
                {
                    commandAction((char*)"--226 on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /ina226/off") >= 0)
                {
                    commandAction((char*)"--226 off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /smalldisplay/on") >= 0)
                {
                    commandAction((char*)"--small on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /smalldisplay/off") >= 0)
                {
                    commandAction((char*)"--small off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /nopos/on") >= 0)
                {
                    commandAction((char*)"--gateway nopos", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /nopos/off") >= 0)
                {
                    commandAction((char*)"--gateway pos", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /softser/on") >= 0)
                {
                    commandAction((char*)"--softser on", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /softser/off") >= 0)
                {
                    commandAction((char*)"--softser off", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /volt/on") >= 0)
                {
                    commandAction((char*)"--volt", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /volt/off") >= 0)
                {
                    commandAction((char*)"--proz", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /mcp/on/") >= 0)
                {
                    int ipos=web_header.indexOf("GET /mcp/on/");
                    char cBefehl[30];
                    sprintf(cBefehl, "--setout %s on", web_header.substring(ipos+12, ipos+14).c_str());
                    commandAction(cBefehl, bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /mcp/off/") >= 0)
                {
                    int ipos=web_header.indexOf("GET /mcp/off/");
                    char cBefehl[30];
                    sprintf(cBefehl, "--setout %s off", web_header.substring(ipos+13, ipos+15).c_str());
                    commandAction(cBefehl, bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /mcptype/IN/") >= 0)
                {
                    int ipos=web_header.indexOf("GET /mcptype/IN/");
                    char cBefehl[30];
                    sprintf(cBefehl, "--setio %s OUT", web_header.substring(ipos+16, ipos+18).c_str());
                    commandAction(cBefehl, bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /mcptype/OUT/") >= 0)
                {
                    int ipos=web_header.indexOf("GET /mcptype/OUT/");
                    char cBefehl[30];

                    sprintf(cBefehl, "--setout %s OFF", web_header.substring(ipos+17, ipos+19).c_str());
                    commandAction(cBefehl, bPhoneReady);

                    sprintf(cBefehl, "--setio %s IN", web_header.substring(ipos+17, ipos+19).c_str());
                    commandAction(cBefehl, bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /reboot") >= 0)
                {
                    commandAction((char*)"--reboot", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /sendpos") >= 0)
                {
                    if(bDisplayTrack)
                        commandAction((char*)"--sendpos", bPhoneReady);
                    else
                        commandAction((char*)"--sendtrack", bPhoneReady);
                }
                else
                if (web_header.indexOf("GET /info") >= 0)
                {
                    web_page_state=0;
                }
                else
                if (web_header.indexOf("GET /pos") >= 0)
                {
                    web_page_state=1;
                }
                else
                if (web_header.indexOf("GET /mheard") >= 0)
                {
                    web_page_state=2;
                }
                else
                if (web_header.indexOf("GET /wx") >= 0)
                {
                    web_page_state=3;
                }
                else
                if (web_header.indexOf("GET /setup") >= 0)
                {
                    web_page_state=4;
                }
                else
                if (web_header.indexOf("GET /refresh") >= 0)
                {
                    web_page_state=5;
                    bRefresh=true;
                }
                else
                if (web_header.indexOf("GET /mhrefresh") >= 0)
                {
                    web_page_state=2;
                    bRefresh=true;
                }
                else
                if (web_header.indexOf("GET /logrefresh") >= 0)
                {
                    web_page_state=6;
                    bRefresh=true;
                }
                else
                if (web_header.indexOf("GET /ssrefresh") >= 0)
                {
                    web_page_state=8;
                    bRefresh=true;
                }
                else
                if (web_header.indexOf("GET /message") >= 0)
                {
                    web_page_state=5;
                }
                else
                if (web_header.indexOf("GET /mcpstatus") >= 0)
                {
                    web_page_state=7;
                }
                else
                if (web_header.indexOf("GET /softser") >= 0)
                {
                    web_page_state=8;
                }
                else
                if (web_header.indexOf("GET /mclear") >= 0)
                {
                    web_page_state=5;
                    toPhoneRead = toPhoneWrite; // message ringbuffer clear
                }
                else
                if (web_header.indexOf("GET /logclear") >= 0)
                {
                    web_page_state=6;
                    RAWLoRaRead=RAWLoRaWrite;
                }
                else
                if (web_header.indexOf("GET /ssclear") >= 0)
                {
                    web_page_state=8;
                    strSOFTSER_BUF="";
                }
                else
                if (web_header.indexOf("GET /mhclear") >= 0)
                {
                    web_page_state=2;
                    for(int iset=0; iset<MAX_MHEARD; iset++)
                    {
                        mheardCalls[iset][0] = 0x00;
                    }
                }
                else
                if (web_header.indexOf("GET /logprint") >= 0)
                {
                    web_page_state=6;
                    bRefresh=true;
                }

                idx_text_end=web_header.indexOf(" HTTP/1.1");

                web_header = web_header.substring(0, idx_text_end);

                // Check Parameter
                // GET /message?sendcall=ok&sendmassage=xx HTTP/1.1
                if (web_header.indexOf("?sendcall=") >= 0)
                {
                    idx_text=web_header.indexOf("?sendcall=") + 10;
                    idx_text_call_end=web_header.indexOf("&");

                    int iend=idx_text_end;
                    if(idx_text_call_end > 0)
                        iend = idx_text_call_end;

                    if(iend > 0 && iend > idx_text)
                        message_call = hex2ascii(web_header.substring(idx_text, iend));

                    message_call.trim();
                    message_call.toUpperCase();

                    //?message=ok HTTP/1.1
                    if (web_header.indexOf("&sendmessage=") > 0)
                    {
                        idx_text=web_header.indexOf("&sendmessage=") + 13;

                        //CALL
                        String message="";

                        if(idx_text_end < 0)
                            message = hex2ascii(web_header.substring(idx_text));
                        else
                            message = hex2ascii(web_header.substring(idx_text, idx_text_end+1));

                        if(message.length() > 0)
                        {
                            if(message_call.length() > 0)
                                sprintf(message_text, ":{%s}%s", message_call.c_str(), message.c_str());
                            else
                                sprintf(message_text, ":%s", message.c_str());
                            
                            
                            int iml=strlen(message_text);
                            if(iml>150)
                            {
                                iml=150;
                                message_text[iml]=0x00;
                            }

                            hasMsgFromPhone=true;

                            sendMessage(message_text, iml);

                            hasMsgFromPhone=false;
                        }

                        message_call="";
                    }
                }
                else
                //?message=ok HTTP/1.1
                if (web_header.indexOf("?nodecall=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--setcall %s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?owgpio=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--onewire gpio %s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?maxv=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--maxv %s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?listento0=") >= 0)
                {
                    String strListen = web_header.substring(web_header.indexOf("?listento0=")+9, web_header.indexOf(" HTTP"));
                    strListen += "&";
                    char cListen[80];
                    sprintf(cListen, "%s", strListen.c_str());

                    // Read each command pair 
                    char* command = strtok(cListen, "&");
                    while (command != 0)
                    {
                        //Serial.printf("command:%s\n", command);
                        
                        // Split the command in two values
                        char* separator = strchr(command, '=');
                        if (separator != 0)
                        {
                            // Actually split the string in 2: replace '=' with 0
                            *separator = 0;
                            int lindex = atoi(command);
                            ++separator;
                            int lgroup = atoi(separator);

                            meshcom_settings.node_gcb[lindex] = lgroup;
                        }
                        
                        // Find the next command in input string
                        command = strtok(0, "&");
                    }

                    commandAction((char*)"--save", bPhoneReady);
                }
                else
                if (web_header.indexOf("?ss0=") >= 0)
                {
                    String strListen = web_header.substring(web_header.indexOf("?ss0=")+3, web_header.indexOf(" HTTP"));
                    strListen += "&";
                    char cListen[80];
                    sprintf(cListen, "%s", strListen.c_str());

                    // Read each command pair 
                    char* command = strtok(cListen, "&");
                    while (command != 0)
                    {
                        //Serial.printf("command:%s\n", command);
                        
                        // Split the command in two values
                        char* separator = strchr(command, '=');
                        if (separator != 0)
                        {
                            // Actually split the string in 2: replace '=' with 0
                            *separator = 0;
                            int lindex = atoi(command);
                            ++separator;
                            int lgroup = atoi(separator);

                            if(lindex == 0)
                                meshcom_settings.node_ss_rx_pin = lgroup;
                            else
                            if(lindex == 1)
                                meshcom_settings.node_ss_tx_pin = lgroup;
                            else
                            if(lindex == 2)
                                meshcom_settings.node_ss_baud = lgroup;
                        }
                        
                        // Find the next command in input string
                        command = strtok(0, "&");
                    }

                    commandAction((char*)"--save", bPhoneReady);

                    commandAction((char*)"--softser on", bPhoneReady);
                }
                else
                if (web_header.indexOf("?sstext=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--softser send %s\r", message.c_str());

                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?passwd=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--passwd %s\r", message.c_str());

                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?txpower=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--txpower %s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?utcoff=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--utcoff %s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?utcdate=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    setRTCNow(message);
                }
                else
                if (web_header.indexOf("?aprstext=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--atxt %s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?latidude=") >= 0)
                {
                    idx_text=web_header.indexOf("latidude=") + 9;
                    idx_text_call_end=web_header.indexOf("&longitude");

                    String message="";

                    int iend=idx_text_end;
                    if(idx_text_call_end > 0)
                        iend = idx_text_call_end;

                    if(iend > 0 && iend > idx_text)
                        message = hex2ascii(web_header.substring(idx_text, iend));

                    sprintf(message_text, "--setlat %s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);

                    if (web_header.indexOf("&longitude=") >= 0)
                    {
                        idx_text=web_header.indexOf("longitude=") + 10;
                        idx_text_call_end=web_header.indexOf("&altitude");

                        String message="";

                        int iend=idx_text_end;
                        if(idx_text_call_end > 0)
                            iend = idx_text_call_end;

                        if(iend < 0)
                            message = hex2ascii(web_header.substring(idx_text));
                        else
                            message = hex2ascii(web_header.substring(idx_text, iend));

                        sprintf(message_text, "--setlon %s", message.c_str());

                        commandAction(message_text, bPhoneReady);
                    }

                    if (web_header.indexOf("&altitude=") >= 0)
                    {
                        idx_text=web_header.indexOf("altitude=") + 9;

                        String message="";

                        if(idx_text_end < 0)
                            message = hex2ascii(web_header.substring(idx_text));
                        else
                            message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                        sprintf(message_text, "--setalt %s", message.c_str());

                        commandAction(message_text, bPhoneReady);
                    }
                }
                else
                if (web_header.indexOf("?country=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--setctry %s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?aprsgroup=") >= 0)
                {
                    if(web_header.indexOf("=1") > 0)
                        sprintf(message_text, "--symid /");
                    else
                        sprintf(message_text, "--symid \\");
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                if (web_header.indexOf("?aprssymbol=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "--symcd %-1.1s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                //?message=ok HTTP/1.1
                if (web_header.indexOf("?command=") >= 0)
                {
                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(message_text, "%s", message.c_str());
                    
                    commandAction(message_text, bPhoneReady);
                }
                else
                //?message=ok HTTP/1.1
                if (web_header.indexOf("?mcp") >= 0)
                {
                    int ipos=web_header.indexOf("?mcp");
                    
                    int idx = web_header.substring(ipos+5, ipos+6).toInt();
                    if(web_header.substring(ipos+4, ipos+5) == "B")
                        idx=idx+8;

                    idx_text=web_header.indexOf("=") + 1;

                    String message="";

                    if(idx_text_end <= 0)
                        message = hex2ascii(web_header.substring(idx_text));
                    else
                        message = hex2ascii(web_header.substring(idx_text, idx_text_end));

                    sprintf(meshcom_settings.node_mcp17t[idx], "%s", message.c_str());

                    save_settings();
                }

                // Display the HTML web page
                web_client.println("<!DOCTYPE html><html>");
                web_client.println("<head><meta charset=\"utf-8\" name=\"viewport\" content=\"width=device-width, initial-scale=1\">");

                if(bRefresh)
                    web_client.println("<meta http-equiv=\"Refresh\" content=\"10\">");

                web_client.println("<link rel=\"icon\" href=\"data:,\">");
                // CSS to style the on/off buttons 
                // Feel free to change the background-color and font-size attributes to fit your preferences
                web_client.println("<style>html { font-family: Helvetica; font-size: 16px; color:#a2182f; display: inline-block; margin: 0px auto; text-align: left;}");
                
                web_client.println("@media screen and (max-width: 600px) {");
                web_client.println("button, table, th, td {font-size: 10px;}");
                web_client.println("h3 {font-size: 20px;}");
                web_client.println("}");

                web_client.println("@media screen and (min-width: 601px; max-width: 800px) {");
                web_client.println("button, table, th, td {font-size: 14px;}");
                web_client.println("h3 {font-size: 20px;}");
                web_client.println("}");

                web_client.println("@media screen and (min-width: 801px) {");
                web_client.println("button, table, th, td {font-size: 16px;}");
                web_client.println("h3 {font-size: 24px;}");
                web_client.println("}");

                // Button Style
                web_client.println(".button { border: none; color:  #a2182f; height:26px; width:100%; padding: 1px 1px;");
                web_client.println("text-decoration: none; margin: 2px; cursor: pointer; border-radius: 8px;}");
                web_client.println(".button2 {background-color:  #a2182f; color: white;}");

                web_client.println(".table {background-color: #FCEDF0; width: max(25%, min(801px, 100%));margin-bottom: 0px;}");
                web_client.println(".table, th, td {border: 1px solid white; border-collapse: collapse;}");
                web_client.println(".table2 {background-color: white;}");
                web_client.println(".tableconsole {font-family: Lucida Console; font-size: 14px;}");
                web_client.println(".td2 {background-color: white;}");

                web_client.println("input[type=submit] {background-color: #a2182f; color: white; padding: 3px 10px;}");

                web_client.println("</style></head>");
                
                // Web Page Heading
                web_client.printf("<body><h3 style=\"margin-bottom: 0px;\">MeshCom 4.0 &nbsp;&nbsp;%s<br />%i-%02i-%02i&nbsp;%02i:%02i:%02i&nbsp;LT</h3>\n", meshcom_settings.node_call,
                    meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);

                web_client.println("<table class=\"table\">");
                web_client.println("<colgroup>");
                web_client.println("<col style=\"width: 25%;\">");
                web_client.println("<col style=\"width: 25%;\">");
                web_client.println("<col style=\"width: 25%;\">");
                web_client.println("<col style=\"width: 25%;\">");
                web_client.println("</colgroup><tr>\n");
                web_client.println("<td><a href=\"#anchor_button\"><button class=\"button button2\"<b>COMMANDS</b></button></a></td>");
                web_client.println("</tr></table>");

                // POS
                if(web_page_state == 1)
                {
                    web_client.println("<table class=\"table\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("<col style=\"width: 75%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.printf("<tr><td style=\"width:40px\"><b>LAT&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</b></td><td>%.4lf %c</td></tr><tr><td><b>LON</b></td><td>%.4lf %c</td></tr><tr><td><b>ALT</b></td><td>%i</td></tr><tr><td><b>SAT</b></td><td>%i - %s - HDOP %i</td></tr>\n",
                    meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,(int)posinfo_satcount, (posinfo_fix?"fix":"nofix"), posinfo_hdop);

                    web_client.printf("<tr><td><b>RATE</b></td><td>%i</td></tr><tr><td><b>NEXT</b></td><td>%i sec</td></tr><tr><td><b>DIST</b></td><td>%im</td></tr><tr><td><b>DIRn:</b></td><td>%i°</td></tr><tr><td><b>DIRo</b></td><td>%i°</td></tr>\n",
                    (int)posinfo_interval, (int)(((posinfo_timer + (posinfo_interval * 1000)) - millis())/1000), posinfo_distance, (int)posinfo_direction, (int)posinfo_last_direction);
                    
                    web_client.printf("<tr><td><b>DATE</b></td><td>%i.%02i.%02i %02i:%02i:%02i LT</td></tr>\n",
                    meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day,meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);
                    web_client.printf("<tr><td><b>SYMB</b></td><td>%c %c</td></tr><tr><td><b>GPS</b></td><td>%s</td></tr><tr><td><b>Track</b></td><td>%s</td></tr>\n", meshcom_settings.node_symid, meshcom_settings.node_symcd, (bGPSON?"on":"off"), (bDisplayTrack?"on":"off"));

                    web_client.println("</table>");
                }
                else
                // MHEARD
                if(web_page_state == 2)
                {
                    web_client.println("<table class=\"table\">");

                    web_client.printf("<tr><th>LHeard call</th><th>date</th><th>time</th><th>LHEARD hardware</th><th>mod</th><th>rssi</th><th>snr</th><th>dist</th><th>pl</th><th>m</th></tr>\n");

                    mheardLine mheardLine;

                    for(int iset=0; iset<MAX_MHEARD; iset++)
                    {
                        if(mheardCalls[iset][0] != 0x00)
                        {
                            if((mheardEpoch[iset]+60*60*6) > getUnixClock())
                            {
                                web_client.printf("<tr><td>%-10.10s</td>", mheardCalls[iset]);
                                
                                decodeMHeard(mheardBuffer[iset], mheardLine);

                                web_client.printf("<td>%-10.10s</td>", mheardLine.mh_date.c_str());
                                web_client.printf("<td>%-8.8s</td>", mheardLine.mh_time.c_str());
                                //web_client.printf("<td>%-3.3s</td>", getPayloadType(mheardLine.mh_payload_type));
                                web_client.printf("<td>%-13.13s</td>", getHardwareLong(mheardLine.mh_hw).c_str());
                                web_client.printf("<td>%3i</td>", mheardLine.mh_mod);
                                web_client.printf("<td>%4i</td>", mheardLine.mh_rssi);
                                web_client.printf("<td>%4i</td>", mheardLine.mh_snr);
                                web_client.printf("<td>%5.1lf</td>", mheardLine.mh_dist);
                                web_client.printf("<td>%i</td>", mheardLine.mh_path_len);
                                web_client.printf("<td>%i</td></tr>\n", mheardLine.mh_mesh);
                            }
                        }
                    }

                    web_client.println("</table>");
                }
                else
                // WX
                if(web_page_state == 3)
                {
                    web_client.println("<table class=\"table\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("<col style=\"width: 75%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.printf("<tr><td><b>BME(P)280</b></td><td>%s</td><tr><td><b>BME680</b></td><td>%s</td><tr><td><b>MCU811</b></td><td>%s</td><tr><td><b>LPS33</b></td><td>%s (RAK)</td><tr><td><b>ONEWIRE</b></td><td>%s (%i)</td><tr>\n",
                    (bBMEON?"on":"off"), (bBME680ON?"on":"off"), (bMCU811ON?"on":"off"), (bLPS33?"on":"off"), (bONEWIRE?"on":"off"), meshcom_settings.node_owgpio);
                    web_client.printf("<td><b>TEMP</b></td><td>%.1f °C</td><tr><td><b>TOUT</b></td><td>%.1f °C</td><tr><td><b>HUM</b></td><td>%.1f%% rH</td><tr><td><b>QFE</b></td><td>%.1f hPa</td><tr><td><b>QNH</b></td><td>%.1f hPa</td><tr>\n",
                    meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_hum, meshcom_settings.node_press, meshcom_settings.node_press_asl);
                    web_client.printf("<td><b>ALT asl</b></td><td>%i m</td><tr><td><b>GAS</b></td><td>%.1f kOhm</td><tr><td><b>eCO2</b></td><td>%.0f ppm</td></tr>\n",
                    meshcom_settings.node_press_alt, meshcom_settings.node_gas_res, meshcom_settings.node_co2);
                    web_client.println("</table>");
                }
                else
                // SETUP
                if(web_page_state == 4)
                {
                    web_client.println("<table class=\"table\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("<col style=\"width: 75%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>NODE Call</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.printf("<input type=\"text\" value=\"%s\" maxlength=\"9\" size=\"9\" id=\"nodecall\" name=\"nodecall\">\n", meshcom_settings.node_call);
                    web_client.println("<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>COUNTRY:</b></label>");
                    web_client.println("</td><td>\n");

                    web_client.println("<select id=\"country\" name=\"country\">");

                    for(int ic=0;ic<21 ;ic++)
                    {
                        if(getCountry(ic) != "none")
                        {
                            web_client.printf("<option value=\"%i\"", ic);

                            if(ic == meshcom_settings.node_country)
                                web_client.printf(" selected");
    
                            web_client.printf(">%s</option>", getCountry(ic).c_str());
                        }
                    }
                    web_client.println("&nbsp;<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>TX-Power dBm:</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"2\" size=\"3\" id=\"txpower\" name=\"txpower\">\n", getPower());
                    web_client.println("<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>UTC-Offset:</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.printf("<input type=\"text\" value=\"%.1f\" maxlength=\"5\" size=\"4\" id=\"utcoff\" name=\"utcoff\">\n", meshcom_settings.node_utcoff);
                    web_client.println("<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    if(bRTCON)
                    {
                        web_client.println("<form action=\"/#\">");
                        web_client.println("<tr><td>\n");
                        web_client.println("<label for=\"fname\"><b>UTC-Date/Time:</b></label>");
                        web_client.println("</td><td>\n");
                        web_client.printf("<input type=\"text\" value=\"%s\" maxlength=\"19\" size=\"19\" id=\"utcdate\" name=\"utcdate\">\n", getStringRTCNow().c_str());
                        web_client.println("<input type=\"submit\" value=\"set\">");
                        web_client.println("</td></tr>\n");
                        web_client.println("</form>");
                    }

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>POS-LAT (+/-):</b></label>");
                    web_client.println("</td><td>\n");
                    double lat=meshcom_settings.node_lat;
                    if(meshcom_settings.node_lat_c == 'S')
                        lat = lat * -1.0;
                    web_client.printf("<input type=\"text\" value=\"%.4lf\" maxlength=\"25\" size=\"15\" id=\"latidude\" name=\"latidude\">\n", lat);

                    web_client.println("</td></tr><tr><td>\n");

                    web_client.println("<label for=\"fname\"><b>POS-LON (+/-):</b></label>");
                    web_client.println("</td><td>\n");
                    double lon=meshcom_settings.node_lon;
                    if(meshcom_settings.node_lon_c == 'W')
                        lon = lon * -1.0;
                    web_client.printf("<input type=\"text\" value=\"%.4lf\" maxlength=\"15\" size=\"15\" id=\"longitude\" name=\"longitude\">\n", lon);
                    
                    web_client.println("</td></tr><tr><td>\n");

                    web_client.println("<label for=\"fname\"><b>POS-Altitude:</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"15\" size=\"15\" id=\"altitude\" name=\"altitude\">\n", meshcom_settings.node_alt);
                    web_client.println("<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>APRS-Text:</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.printf("<input type=\"text\" value=\"%s\" maxlength=\"25\" size=\"25\" id=\"aprstext\" name=\"aprstext\">\n", meshcom_settings.node_atxt);
                    web_client.println("<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>APRS-GROUP:</b></label>");
                    web_client.println("</td><td>\n");

                    if(meshcom_settings.node_symid == '/')
                        web_client.println("<select id=\"aprsgroup\" name=\"aprsgroup\"><option value=\"1\" selected>/</option><option value=\"2\">\\</option>");
                    else
                        web_client.println("<select id=\"aprsgroup\" name=\"aprsgroup\"><option value=\"1\">/</option><option value=\"2\" selected>\\</option>");
                    web_client.println("&nbsp;<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>APRS-SYMBOL:</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.printf("<input type=\"text\" value=\"%c\" maxlength=\"1\" size=\"2\" id=\"aprssymbol\" name=\"aprssymbol\">\n", meshcom_settings.node_symcd);
                    web_client.println("<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>ONEWIRE-PIN:</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"2\" size=\"2\" id=\"owgpio\" name=\"owgpio\">\n", meshcom_settings.node_owgpio);
                    web_client.println("<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>MAXV:</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.printf("<input type=\"text\" value=\"%.3f\" maxlength=\"5\" size=\"5\" id=\"maxv\" name=\"maxv\">\n", meshcom_settings.node_maxv);
                    web_client.println("<input type=\"submit\" value=\"set\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>LISTEN-TO:</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"4\" size=\"4\" id=\"listento0\" name=\"listento0\">\n", meshcom_settings.node_gcb[0]);
                    web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"4\" size=\"4\" id=\"listento1\" name=\"1\">\n", meshcom_settings.node_gcb[1]);
                    web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"4\" size=\"4\" id=\"listento2\" name=\"2\">\n", meshcom_settings.node_gcb[2]);
                    web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"4\" size=\"4\" id=\"listento3\" name=\"3\">\n", meshcom_settings.node_gcb[3]);
                    web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"4\" size=\"4\" id=\"listento4\" name=\"4\">\n", meshcom_settings.node_gcb[4]);
                    web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"4\" size=\"4\" id=\"listento5\" name=\"5\">\n", meshcom_settings.node_gcb[5]);
                    web_client.println("<input type=\"submit\" value=\"send\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form>");

                    if(bSOFTSERON)
                    {
                        web_client.println("<form action=\"/#\">");
                        web_client.println("<tr><td>\n");
                        web_client.println("<label for=\"fname\"><b>SS RX/TX/BAUD:</b></label>");
                        web_client.println("</td><td>\n");
                        web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"2\" size=\"4\" id=\"ssrx\" name=\"ss0\">\n", meshcom_settings.node_ss_rx_pin);
                        web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"2\" size=\"4\" id=\"sstx\" name=\"1\">\n", meshcom_settings.node_ss_tx_pin);
                        web_client.printf("<input type=\"text\" value=\"%i\" maxlength=\"4\" size=\"5\" id=\"ssbd\" name=\"2\">\n", meshcom_settings.node_ss_baud);
                        web_client.println("<input type=\"submit\" value=\"set\">");
                        web_client.println("</td></tr>\n");
                        web_client.println("</form>");
                    }

                    if(bMCP23017)
                    {
                        web_client.println("<form action=\"/#\">");
                        web_client.println("<tr><td>\n");
                        web_client.println("<label for=\"fname\"><b>MCP PASSWD:</b></label>");
                        web_client.println("</td><td>\n");
                        web_client.printf("<input type=\"text\" value=\"%s\" maxlength=\"14\" size=\"8\" id=\"passwd\" name=\"passwd\">\n", meshcom_settings.node_passwd);
                        web_client.println("<input type=\"submit\" value=\"set\">");
                        web_client.println("</td></tr>\n");
                        web_client.println("</form>");
                    }

                    web_client.println("<form action=\"/#\">");
                    web_client.println("<tr><td>\n");
                    web_client.println("<label for=\"fname\"><b>COMMAND:</b></label>");
                    web_client.println("</td><td>\n");
                    web_client.println("<input type=\"text\" maxlength=\"50\" size=\"30\" id=\"command\" name=\"command\">");
                    web_client.println("<input type=\"submit\" value=\"send\">");
                    web_client.println("</td></tr>\n");
                    web_client.println("</form></table>");
                }
                else
                // MESSAGE
                if(web_page_state == 5)
                {
                    int iRead = toPhoneRead;

                    web_client.println("<table class=\"table\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.println("<tr><th>last messages</th><th colspan=\"3\"></th></tr>");

                    if(bDEBUG)
                        Serial.printf("toPhoneWrite:%i toPhoneRead:%i\n", toPhoneWrite, toPhoneRead);

                    while(toPhoneWrite != iRead)
                    {
                        if(bDEBUG)
                            Serial.printf("iRead:%i [1]:%02X\n", iRead, BLEtoPhoneBuff[iRead][1]);

                        // we need to insert the first byte text msg flag
                        uint8_t toPhoneBuff [MAX_MSG_LEN_PHONE] = {0};
                        // MAXIMUM PACKET Length over BLE is 245 (MTU=247 bytes), two get lost, otherwise we need to split it up!
                        uint8_t blelen = BLEtoPhoneBuff[iRead][0];

                        //Mheard
                        if(BLEtoPhoneBuff[iRead][1] == 0x91)
                        {
                            //memcpy(toPhoneBuff, BLEtoPhoneBuff[iRead]+1, blelen-1);
                        }
                        else 
                        // Data Message (JSON)
                        if(BLEtoPhoneBuff[iRead][1] == 0x44)
                        {		
                            //memcpy(toPhoneBuff, BLEtoPhoneBuff[iRead]+1, blelen);	
                        } 
                        else
                        // Text Message and Position
                        {
                            memcpy(toPhoneBuff, BLEtoPhoneBuff[iRead] + 1, blelen - 4);

                            uint8_t tbuffer[5];
                            memcpy(tbuffer, BLEtoPhoneBuff[iRead] + 1 + (blelen - 4), 4);
                            
                            unsigned long unix_time=0;

                            unix_time = (tbuffer[0] << 24) | (tbuffer[1] << 16) | (tbuffer[2] << 8) | tbuffer[3];
                            
                            time_t unix_t = (time_t)(unix_time + (long)(meshcom_settings.node_utcoff * 60 * 60));

                            struct tm *oldt = gmtime(&unix_t);

                            char timestamp[21];

                            strftime(timestamp, 20, "%Y-%m-%d %H:%M:%S", oldt);
                        
                            //Serial.printf("Timestamp:<%s>\n", timestamp);

                            struct aprsMessage aprsmsg;

                            // print which message type we got
                            uint8_t msg_type_b_lora = decodeAPRS(toPhoneBuff, blelen, aprsmsg);

                            int icheck = checkOwnTx(toPhoneBuff+1);

                            String ccheck="";
                            if(own_msg_id[icheck][4] == 1)   // 00...not heard, 01...heard, 02...ACK
                            {
                                ccheck="&#x2713&nbsp;";
                            }

                            if(own_msg_id[icheck][4] == 2)   // 00...not heard, 01...heard, 02...ACK
                            {
                                ccheck="&#x2611;&nbsp;";
                            }

                            // Textmessage
                            if(msg_type_b_lora == 0x3A)
                            {
                                if(aprsmsg.msg_payload.indexOf(":ack") < 1)
                                {
                                    if(bDEBUG)
                                        Serial.printf("aprsmsg.msg_source_call.c_str():%s, aprsmsg.msg_gateway_call.c_str():%s, aprsmsg.msg_destination_call.c_str():%s, aprsmsg.msg_payload.c_str():%s\n", aprsmsg.msg_source_call.c_str(), aprsmsg.msg_source_last.c_str(), aprsmsg.msg_destination_call.c_str(), aprsmsg.msg_payload.c_str());

                                    String msgtxt = aprsmsg.msg_payload;
                                    if(msgtxt.indexOf('{') > 0)
                                        msgtxt = aprsmsg.msg_payload.substring(0, msgtxt.indexOf('{'));

                                    if(strcmp(meshcom_settings.node_call, aprsmsg.msg_source_call.c_str()) == 0)
                                        web_client.printf("<tr><td class=\"td2\"></td><td colspan=\"3\"><small>%s<br /><b>%s%s%s%s</b><br /></small><b>%s</b></td></tr>\n", timestamp, ccheck.c_str(), aprsmsg.msg_source_path.c_str(), (char*)">", aprsmsg.msg_destination_path.c_str(), msgtxt.c_str());
                                    else
                                        web_client.printf("<tr><td colspan=\"3\"><small>%s<br /><b>%s%s%s%s</b><br /></small><b>%s</b></td><td class=\"td2\"></td></tr>\n", timestamp, ccheck.c_str(), aprsmsg.msg_source_path.c_str(), (char*)">", aprsmsg.msg_destination_path.c_str(), msgtxt.c_str());
                                }
                            }
                        }

                        iRead++;
                        if (iRead >= MAX_RING)
                        iRead = 0;
                    }

                    web_client.println("</table>");

                    web_client.println("<table class=\"table\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("<col style=\"width: 75%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.println("<form action=\"?\">");

                    web_client.println("<tr><td>");
                    web_client.println("<label for=\"fname\"><b>DM Call (or empty):</b></label>");
                    web_client.println("</td><td>");
                    web_client.println("<input type=\"text\" id=\"sendcall\" name=\"sendcall\" maxlength=\"9\" size=\"9\">");
                    web_client.println("</td></tr><tr><td>");
                    web_client.println("<label for=\"fname\"><b>Message:</b></label>");
                    web_client.println("</td><td>");
                    web_client.println("<textarea id=\"sendmessage\" name=\"sendmessage\" maxlength=\"150\" rows=\"5\" cols=\"40\"></textarea>");
                    web_client.println("</td></tr><tr><td></td><td>");
                    web_client.println("<input type=\"submit\" value=\"send\">");
                    web_client.println("</td></tr>");
                    web_client.println("</form>");

                    web_client.println("</table>");
                }
                else
                // LOGPRINT
                if(web_page_state == 6)
                {
                    int iRead = RAWLoRaRead;

                    web_client.println("<table class=\"tableconsole\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 100%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.println("<tr><th>LoRa-LOG</th></tr>");

                    while(RAWLoRaWrite != iRead)
                    {
                        web_client.printf("<tr><td><nobr><small>%s</small></nobr></td></tr>\n", ringbufferRAWLoraRX[iRead]);

                        iRead++;
                        if (iRead >= MAX_LOG)
                            iRead = 0;
                    }

                    web_client.println("</table>");
                }
                else
                // MCP-STATUS
                if(web_page_state == 7)
                {
                    web_client.println("<table class=\"table\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 10%;\">");
                    web_client.println("<col style=\"width: 16%;\">");
                    web_client.println("<col style=\"width: 49%;\">");
                    web_client.println("<col style=\"width: 15%;\">");
                    web_client.println("<col style=\"width: 10%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.printf("<tr><th>PORT</th><th>MCP-23017</th><th>%s</th><th>STATUS</th><th>SET</th></tr>\n", (bMCP23017?"active":"offline"));

                    uint16_t t_io = meshcom_settings.node_mcp17io;
                    uint16_t t_out = meshcom_settings.node_mcp17out;
                    uint16_t t_in = meshcom_settings.node_mcp17in;

                    for(int io=0; io<16; io++)
                    {
                        bool bOut=false;
                        if((t_io & 0x0001) == 0x0001)
                            bOut=true;

                        bool bOutValue=false;
                        if((t_out & 0x0001) == 0x0001)
                            bOutValue=true;

                        bool bInValue=false;
                        if((t_in & 0x0001) == 0x0001)
                            bInValue=true;

                        char cAB='B';
                        int iAB=io-8;
                        if(io < 8)
                        {
                            cAB='A';
                            iAB=io;
                        }

                        web_client.printf("<tr><td>[%c%i]</td>", cAB, iAB);
                        web_client.printf("<td><a href=\"/mcptype/%s/%c%i\"><button class=\"button button2\"<b>%s</b></button></a></td>", (bOut?"OUT":"IN"), cAB, iAB, (bOut?"OUT":"IN"));
                        web_client.println("<form action=\"/#\">");
                        web_client.printf("<td><input type=\"text\" value=\"%s\" maxlength=\"16\" size=\"16\" id=\"mcp%c%i\" name=\"mcp%c%i\">\n", meshcom_settings.node_mcp17t[io], cAB, iAB, cAB, iAB);
                        web_client.println("<input type=\"submit\" value=\"set\"></td>");
                        web_client.println("</form>");

                        if(bOut)
                        {
                            if(bOutValue)
                                web_client.printf("<td>%s</td><td><a href=\"/mcp/off/%c%i\"><button class=\"button button2\"<b>ON</b></button></a></td></tr>\n",  (bOutValue?"OFF ":"ON  "), cAB, iAB);
                            else
                                web_client.printf("<td>%s</td><td><a href=\"/mcp/on/%c%i\"><button class=\"button button2\"<b>OFF</b></button></a></td></tr>\n",  (bOutValue?"OFF ":"ON  "), cAB, iAB);
                        }
                        else
                        {
                            if(meshcom_settings.node_mcp17t[io][0] == 0x00)
                                web_client.printf("<td>%s</td><td></td></tr>\n",  (bInValue?"HIGH":"LOW "));
                            else
                                web_client.printf("<td><b>%s</b></td><td></td></tr>\n",  (bInValue?"HIGH":"LOW "));
                        }

                        t_io >>= 1;
                        t_out >>= 1;
                        t_in >>= 1;

                    }

                    web_client.println("</table>");
                }
                else
                // SOFTSER
                if(web_page_state == 8)
                {
                    web_client.println("<table class=\"table\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 100%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.println("<tr><th>last message</th></tr>");

                    // SOFTSER-Message
                    web_client.printf("<tr><td><textarea cols='100' rows='30'>%s</textarea></td></tr>\n", strSOFTSER_BUF.c_str());

                    web_client.println("</table>");

                    web_client.println("<table class=\"table\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("<col style=\"width: 75%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.println("<form action=\"?\">");

                    web_client.println("<tr><td>");
                    web_client.println("<label for=\"fname\"><b>Message:</b></label>");
                    web_client.println("</td><td>");
                    web_client.println("<textarea id=\"sstext\" name=\"sstext\" maxlength=\"50\" rows=\3\" cols=\"40\"></textarea>");
                    web_client.println("</td></tr><tr><td></td><td>");
                    web_client.println("<input type=\"submit\" value=\"send\">");
                    web_client.println("</td></tr>");
                    web_client.println("</form>");

                    web_client.println("</table>");
                }
                else
                // INFO
                {
                    web_client.println("<table class=\"table\">");

                    web_client.println("<colgroup>");
                    web_client.println("<col style=\"width: 25%;\">");
                    web_client.println("<col style=\"width: 75%;\">");
                    web_client.println("</colgroup>\n");

                    web_client.printf("<tr><td style=\"width:40px\"><b>Firmware</b></td><td>MeshCom %s %-4.4s%-1.1s</td><tr><td><b>Call</b></td><td>%s ...%s</td></tr><tr><td><b>UTC-OFF</b></td><td>%.1f</td></tr>\n",
                        SOURCE_TYPE, SOURCE_VERSION, SOURCE_VERSION_SUB,meshcom_settings.node_call, getHardwareLong(BOARD_HARDWARE).c_str(), meshcom_settings.node_utcoff);
                    
                    web_client.printf("<tr><td><b>BATT</b></td><td>%.2f V %d %% max %.2f V</td></tr><tr><td><b>Setting</b></td><td>GATEWAY %s %s ...MESH %s</td></tr><tr><td></td><td>BUTTON  %s ...DEBUG %s</td></tr>\n",
                        global_batt/1000.0, global_proz, meshcom_settings.node_maxv, (bGATEWAY?"on":"off"), (bGATEWAY_NOPOS?"nopos":""), (bMESH?"on":"off"), (bButtonCheck?"on":"off"), (bDEBUG?"on":"off"));
                    
                    
                    web_client.printf("<tr><td></td><td>LORADEBUG %s ...GPSDEBUG  %s</td></tr><tr><td></td><td>WXDEBUG %s ... BLEDEBUG %s</td></tr><tr><td><b>APRS-TXT</b></td><td>%s</td></tr>\n",
                        (bLORADEBUG?"on":"off"), (bGPSDEBUG?"on":"off"), (bWXDEBUG?"on":"off"), (bBLEDEBUG?"on":"off"), meshcom_settings.node_atxt);

                    web_client.printf("<tr><td><b>MESH-Settings</b></td><td>max_hop_text %i</td></tr><tr><td></td><td>max_hop_pos %i</td></tr>\n", meshcom_settings.max_hop_text, meshcom_settings.max_hop_pos);

                    web_client.printf("<tr><td><b>COUNTRY</b></td><td>%s</td></tr><tr><td><b>FREQ</b></td><td>%.4f MHz</td></tr><tr><td><b>BW</b></td><td>%.0f kHz</td></tr><tr><td><b>SF</b></td><td>%i</td></tr><tr><td><b>CR</b></td><td>4/%i</td></tr><tr><td><b>TXPWR</b></td><td>%i dBm</td></tr>\n",
                        getCountry(meshcom_settings.node_country).c_str(), getFreq(), getBW(), getSF(), getCR(), getPower());

                    web_client.println("<tr><td>&nbsp;</td><td>&nbsp;</td></tr>");

                    #ifndef BOARD_RAK4630
                        if(bWIFIAP)
                        {
                            web_client.printf("<tr><td><b>SSID</b></td><td>%s</td></tr>\n", cBLEName);
                        }
                        else
                        {
                            web_client.printf("<tr><td><b>SSID</b></td><td>%s</td></tr>\n", meshcom_settings.node_ssid);
                        }
                        //web_client.printf("<tr><td><b>PASSWORD</b></td><td>%s</td></tr>\n", meshcom_settings.node_pwd);
                        web_client.printf("<tr><td><b>WIFI-AP</b></td><td>%s</td></tr>\n", (bWIFIAP?"yes":"no"));
                    #endif
                    web_client.printf("<tr><td><b>hasIpAddress</b></td><td>%s</td></tr>\n", (meshcom_settings.node_hasIPaddress?"yes":"no"));

                    if(meshcom_settings.node_hasIPaddress)
                    {
                        web_client.printf("<tr><td><b>IP address</b></td><td>%s</td></tr>\n", meshcom_settings.node_ip);
                        if(!bWIFIAP)
                        {
                            web_client.printf("<tr><td><b>GW address</b></td><td>%s</td></tr>\n", meshcom_settings.node_gw);
                            web_client.printf("<tr><td><b>DNS address</b></td><td>%s</td></tr>\n", meshcom_settings.node_dns);
                        }
                        web_client.printf("<tr><td><b>SUB-MASK</b></td><td>%s</td></tr>\n", meshcom_settings.node_subnet);
                    }

                    if(bINA226ON)
                    {
                        web_client.println("<tr><td>&nbsp;</td><td>&nbsp;</td></tr>");
                        web_client.println("<tr><td>INA226</td><td>&nbsp;</td></tr>");
                        web_client.printf("<tr><td><b>vBUS</b></td><td><b>%.2f V</b></td></tr>\n", meshcom_settings.node_vbus);
                        web_client.printf("<tr><td><b>vSHUNT</b></td><td>%.2f mV</td></tr>\n", meshcom_settings.node_vshunt);
                        web_client.printf("<tr><td><b>vCURRENT</b></td><td>%.1f mA</td></tr>\n", meshcom_settings.node_vcurrent);
                        web_client.printf("<tr><td><b>vPOWER</b></td><td>%.1f mW</td></tr>\n", meshcom_settings.node_vpower);
                    }

                    if(bRTCON)
                    {
                        web_client.println("<tr><td>&nbsp;</td><td>&nbsp;</td></tr>");
                        web_client.println("<tr><td><b>RTC</b></td><td>&nbsp;</td></tr>");
                        web_client.printf("<tr><td><b>UTC-Date/Time</b></td><td>%s</td></tr>\n", getStringRTCNow().c_str());
                    }

                    web_client.println("</table>");
                }

                web_client.println("<table class=\"table table2\">");

                web_client.println("<colgroup>");
                web_client.println("<col style=\"width: 25%;\">");
                web_client.println("<col style=\"width: 25%;\">");
                web_client.println("<col style=\"width: 25%;\">");
                web_client.println("<col style=\"width: 25%;\">");
                web_client.println("</colgroup>\n");

                // SETUP
                if(web_page_state == 4)
                {
                    web_client.println("<tr><td><b>&nbsp;SETUP BUTTONS</b></td></tr><tr>");

                    if (bDisplayOff)
                    {
                        web_client.println("<td><a href=\"/display/on\"><button class=\"button\"><b>DISPLAY</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/display/off\"><button class=\"button button2\"><b>DISPLAY</b></button></a></td>");
                    } 

                    // GPS Button
                    if (bGPSON)
                    {
                        web_client.println("<td><a href=\"/gps/off\"><button class=\"button button2\"><b>GPS</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/gps/on\"><button class=\"button\"><b>GPS</b></button></a></td>");
                    } 

                    // TRACK ON
                    if (bDisplayTrack)
                    {
                        web_client.println("<td><a href=\"/track/off\"><button class=\"button button2\"><b>TRACK</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/track/on\"><button class=\"button\"><b>TRACK</b></button></a></td>");
                    }

                    // BUTON ON
                    if (bButtonCheck)
                    {
                        web_client.println("<td><a href=\"/button/off\"><button class=\"button button2\"><b>BUTTON</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/button/on\"><button class=\"button\"><b>BUTTON</b></button></a></td></tr>");
                    }

                    // NEXT LINE
                    // BME280 ON
                    if (bBMEON)
                    {
                        web_client.println("<tr><td><a href=\"/bme/off\"><button class=\"button button2\"><b>BME 280</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<tr><td><a href=\"/bme/on\"><button class=\"button\"><b>BME 280</b></button></a></td>");
                    }

                    // BME280 ON
                    if (bBMPON)
                    {
                        web_client.println("<td><a href=\"/bmp/off\"><button class=\"button button2\"><b>BMP 280</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/bmp/on\"><button class=\"button\"><b>BMP 280</b></button></a></td>");
                    }

                    // BME680 ON
                    if (bBME680ON)
                    {
                        web_client.println("<td><a href=\"/bme680/off\"><button class=\"button button2\"><b>BME 680</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/bme680/on\"><button class=\"button\"><b>BME 680</b></button></a></td>");
                    }

                    // MCU811 ON
                    if (bMCU811ON)
                    {
                        web_client.println("<td><a href=\"/mcu/off\"><button class=\"button button2\"<b>MCU 811</b></button></a></td></tr>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/mcu/on\"><button class=\"button\"><b>MCU 811</b></button></a></td></tr>");
                    }

                    // MESH ON
                    if (bMESH)
                    {
                        web_client.println("<tr><td><a href=\"/mesh/off\"><button class=\"button button2\"<b>MESH</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<tr><td><a href=\"/mesh/on\"><button class=\"button\"><b>MESH</b></button></a></td>");
                    }

                    // ONEWIRE ON
                    if (bONEWIRE)
                    {
                        web_client.println("<td><a href=\"/onewire/off\"><button class=\"button button2\"<b>ONEWIRE</b></button></a></td>");
                    }
                    else
                    {
                        web_client.printf("<td><a href=\"/onewire/on\"><button class=\"button\"><b>ONEWIRE (%i)</b></button></a></td>\n", meshcom_settings.node_owgpio);
                    }

                    // VOLT/PROZ
                    if ((meshcom_settings.node_sset & 0x0001) == 0x0001)
                    {
                        web_client.println("<td><a href=\"/volt/off\"><button class=\"button button2\"<b>VOLT</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/volt/on\"><button class=\"button\"><b>VOLT</b></button></a></td>");
                    }

                    // INA226
                    if (bINA226ON)
                    {
                        web_client.println("<td><a href=\"/ina226/off\"><button class=\"button button2\"<b>INA226</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/ina226/on\"><button class=\"button\"><b>INA226</b></button></a></td></tr>");
                    }

                    // GATEWAY_NOPOS
                    if (bGATEWAY_NOPOS)
                    {
                        web_client.println("<tr><td><a href=\"/nopos/off\"><button class=\"button button2\"<b>GW NOPOS</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<tr><td><a href=\"/nopos/on\"><button class=\"button\"><b>GW NOPOS</b></button></a></td>");
                    }

                    // SMALLDISPLAY
                    if (bSMALLDISPLAY)
                    {
                        web_client.println("<td><a href=\"/smalldisplay/off\"><button class=\"button button2\"<b>SMALL</b></button></a></td>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/smalldisplay/on\"><button class=\"button\"><b>SMALL</b></button></a></td>");
                    }

                    // SOFTSER
                    if (bSOFTSERON)
                    {
                        web_client.println("<td><a href=\"/softser/off\"><button class=\"button button2\"<b>SOFTSER</b></button></a></td></tr>");
                    }
                    else
                    {
                        web_client.println("<td><a href=\"/softser/on\"><button class=\"button\"><b>SOFTSER</b></button></a></td></tr>");
                    }
                }

                web_client.println("<p style=\"margin: 0px;\" id=\"anchor_button\">&nbsp;</p>");

                if(web_page_state == 2) //MHEARD TAB
                {
                    web_client.println("<td><a href=\"/mhclear\"><button class=\"button\"><b>M.CLEAR</b></button></a></td>");
                    web_client.println("<td><a href=\"/mhrefresh\"><button class=\"button\"><b>REFRESH</b></button></a></td>");
                    web_client.println("</tr>");
                }

                if(web_page_state == 5) //Message TAB
                {
                    web_client.println("<td><a href=\"/mclear\"><button class=\"button\"><b>M.CLEAR</b></button></a></td>");
                    web_client.println("<td><a href=\"/refresh/#anchor_button\"><button class=\"button\"><b>REFRESH</b></button></a></td>");
                    web_client.println("</tr>");
                }

                if(web_page_state == 6) //LOG TAB
                {
                    web_client.println("<td><a href=\"/logclear\"><button class=\"button\"><b>L.CLEAR</b></button></a></td>");
                    web_client.println("<td><a href=\"/logrefresh\"><button class=\"button\"><b>REFRESH</b></button></a></td>");
                    web_client.println("</tr>");
                }

                if(web_page_state == 8) //SOFTSER TAB
                {
                    web_client.println("<td><a href=\"/ssclear\"><button class=\"button\"><b>S.CLEAR</b></button></a></td>");
                    web_client.println("<td><a href=\"/ssrefresh\"><button class=\"button\"><b>REFRESH</b></button></a></td>");
                    web_client.println("</tr>");
                }

                // NEXT INFO
                web_client.println("<tr><td><a href=\"/info\"><button class=\"button\"><b>INFO</b></button></a></td>");        //page 0
                web_client.println("<td><a href=\"/pos\"><button class=\"button\"><b>POS</b></button></a></td>");              //page 1
                web_client.println("<td><a href=\"/wx\"><button class=\"button\"><b>WX</b></button></a></td>");                //page 3
                web_client.println("<td><a href=\"/mheard\"><button class=\"button\"><b>MHEARD</b></button></a></td></tr>");   //page 2

                web_client.println("<tr><td><a href=\"/setup\"><button class=\"button\"><b>SETUP</b></button></a></td>");      //page 4
                web_client.println("<td><a href=\"/message\"><button class=\"button\"><b>MESSAGE</b></button></a></td>");      //page 5
                
                web_client.println("<td><a href=\"/logprint\"><button class=\"button\"><b>RX-LOG</b></button></a></td>");       //page 6
                web_client.println("<td><a href=\"/sendpos\"><button class=\"button\"><b>SENDPOS</b></button></a></td></tr>");

                if(bMCP23017)
                    web_client.println("<tr><td><a href=\"/mcpstatus\"><button class=\"button\"><b>MCP-STATUS</b></button></a></td>");       //page 7

                if(bSOFTSERON)
                    web_client.println("<td><a href=\"/softser\"><button class=\"button\"><b>SOFTSER</b></button></a></td>");   // page 8

                // REBOOT
                web_client.println("<td><a href=\"/reboot\"><button class=\"button\"><b>REBOOT</b></button></a></td></tr>");

                web_client.println("</table>");

                web_client.println("</body></html>");
                
                // The HTTP response ends with another blank line
                web_client.println();
                // Break out of the while loop

                break;
                }
                else
                { // if you got a newline, then clear currentLine
                web_currentLine = "";
                }
            }
            else
            if (c != '\r')
            {  // if you got anything else but a carriage return character,
                web_currentLine += c;      // add it to the end of the currentLine
            }
        }
    }
  
    if(bDEBUG)
    {
        Serial.println("Client disconnected.");
        Serial.println("");
    }
}

// HTTP functions
String hex2ascii(String ustring)
{
    ustring.replace("+", " ");

    /*
    string.replace("%F0%9F%98%80", ":-)");
    string.replace("%F0%9F%91%8D", "(Y)");
    string.replace("%F0%9F%98%AC", ";-#");
    */

    char pbuff[200];
    char nbuff[200];
    char dbuff[3];
    int ihex=0;

    sprintf(pbuff, "%s", ustring.c_str());

    int in=0;
    int il=0;

    memset(nbuff, 0x00, 200);

    // %F0%XX%XX%XX
    for(int ip=0; ip<(int)ustring.length(); ip++)
    {
        if(il > 0)
        {
            il--;
        }
        else
        if(memcmp(pbuff+ip, "%F0%", 4) == 0)
        {
            nbuff[in] = 0xF0;
            in++;

            for(int ih=0;ih<3;ih++)
            {
                memset(dbuff, 0x00, 3);
                memcpy(dbuff, pbuff+ip+4+(3*ih), 2);
                sscanf(dbuff, "%X", &ihex);

                nbuff[in] = ihex;
                in++;
            }

            il=11;
        }
        if(memcmp(pbuff+ip, "%EF%", 4) == 0)
        {
            nbuff[in] = 0xEF;
            in++;

            for(int ih=0;ih<2;ih++)
            {
                memset(dbuff, 0x00, 3);
                memcpy(dbuff, pbuff+ip+4+(3*ih), 2);
                sscanf(dbuff, "%X", &ihex);

                nbuff[in] = ihex;
                in++;
            }

            il=8;
        }
        else
        if(memcmp(pbuff+ip, "%E2%", 4) == 0)
        {
            nbuff[in] = 0xE2;
            in++;

            for(int ih=0;ih<2;ih++)
            {
                memset(dbuff, 0x00, 3);
                memcpy(dbuff, pbuff+ip+4+(3*ih), 2);
                sscanf(dbuff, "%X", &ihex);

                nbuff[in] = ihex;
                in++;
            }

            il=8;
        }
        else
        {
            nbuff[in] = pbuff[ip];
            in++;
        }

    }

    String string = nbuff;

    string.replace("%C2%A3", "£");
    string.replace("%C2%B0", "°");

    string.replace("%C3%A4", "ä");
    string.replace("%C3%B6", "ö");
    string.replace("%C3%BC", "ü");

    string.replace("%C3%84", "Ä");
    string.replace("%C3%96", "Ö");
    string.replace("%C3%9C", "Ü");

    string.replace("%C3%9F", "ß");

    string.replace("%0D%0A", "-");

    string.replace("%21", "!");
    string.replace("%23", "#");
    string.replace("%24", "$");
    string.replace("%25", "%");
    string.replace("%26", "&");
    string.replace("%27", "'");
    string.replace("%28", "(");
    string.replace("%29", ")");
    string.replace("%2A", "*");
    string.replace("%2B", "+");
    string.replace("%2C", ",");
    string.replace("%2F", "/");
    string.replace("%3A", ":");
    string.replace("%3B", ";");
    string.replace("%3D", "=");
    string.replace("%3F", "?");
    string.replace("%40", "@");
    string.replace("%5B", "[");
    string.replace("%5D", "]");

    string.replace("%20", " ");
    string.replace("%22", """");
    string.replace("%2D", "-");
    string.replace("%2E", ".");
    string.replace("%3C", "<");
    string.replace("%3E", ">");
    string.replace("%5C", "\\");
    string.replace("%5E", "^");
    string.replace("%5F", "_");
    string.replace("%60", "`");
    string.replace("%7B", "{");
    string.replace("%7C", "|");
    string.replace("%7D", "}");
    string.replace("%7E", "~");

    string.replace("%09", "");

    return string;
}
