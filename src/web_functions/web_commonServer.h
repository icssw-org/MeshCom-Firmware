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

    // RULE: no data members here. CommonWebServer::available() static_casts a base client
    // and returns a copy; all copies share the single send buffer in web_txbuf.cpp.
    // write()/stop() are overridden to route output through the bounded send buffer
    // (web_txbuf.h); flush() is deliberately NOT touched (Arduino flush = discard RX).
    class CommonWebClient : public WiFiClient {
        using WiFiClient::WiFiClient;
        typedef WiFiClient Base;
        public:
            using WiFiClient::write;
            size_t write(uint8_t b) override;
            size_t write(const uint8_t *buf, size_t size) override;
            void stop() override;
            void txEnd(); // flush the send buffer and release it; idempotent
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

    // RULE: no data members here. CommonWebServer::available() static_casts a base client
    // and returns a copy; all copies share the single send buffer in web_txbuf.cpp.
    // write()/stop() are overridden to route output through the bounded send buffer
    // (web_txbuf.h); flush() is NOT touched: here it waits for the W5100S TX buffer to drain,
    // and our unsent bytes are still in the send buffer until stop()/txEnd().
    class CommonWebClient : public EthernetClient {
        using EthernetClient::EthernetClient;
        typedef EthernetClient Base;
        public:
            using EthernetClient::write;
            size_t write(uint8_t b) override;
            size_t write(const uint8_t *buf, size_t size) override;
            void stop() override;
            void txEnd(); // flush the send buffer and release it; idempotent
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