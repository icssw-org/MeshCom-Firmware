/**
 * This is a wrapper class for both, the WiFiWebserver and WiFiClient as well as the EthernetServer and EthernetClient.
 * Using this, both are available as a common class so we cann use and pass them without using compiler macros.
 */

#ifndef _COMMONWEBSERVER_H_
#define _COMMONWEBSERVER_H_

#ifdef ESP32
    #include <WiFi.h>
    #include <WiFiClient.h>
    #include <ESPmDNS.h>

    class CommonWebClient : public WiFiClient {
        using WiFiClient::WiFiClient;
    };

    class CommonWebServer : public WiFiServer {
        using WiFiServer::WiFiServer;
        public:
            CommonWebClient available() {
                WiFiClient c = WiFiServer::available();
                CommonWebClient* cwc = static_cast<CommonWebClient*>(&c);
                return *cwc;
            }
    };
#else
    #include <SPI.h>
    #include <RAK13800_W5100S.h> // Click to install library: http://librarymanager/All#RAK13800-W5100S

    class CommonWebClient : public EthernetClient {
        using EthernetClient::EthernetClient;
    };

    class CommonWebServer : public EthernetServer {
        using EthernetServer::EthernetServer;
        public:
              CommonWebClient available() {
                EthernetClient c = EthernetServer::available();
                CommonWebClient* cwc = static_cast<CommonWebClient*>(&c);
                return *cwc;
            }
    };
#endif

#endif