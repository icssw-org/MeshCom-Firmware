#ifdef HAS_ETHERNET

#include "esp32_eth.h"
#include "esp32_flash.h"
#include <configuration.h>

void EspETH::initethDHCP()
{
    Serial.println("[ETH] init");

    ETH.begin(
        ETH_PHY_W5500,
        1,
        ETH_CS_PIN,
        ETH_INT_PIN,
        ETH_RST_PIN,
        SPI3_HOST,
        ETH_SCLK_PIN,
        ETH_MISO_PIN,
        ETH_MOSI_PIN
    );

    delay(1000);

    Serial.println(ETH.linkUp() ? "[ETH] LINK UP" : "[ETH] LINK DOWN");

    IPAddress cfg_ip;
    cfg_ip.fromString(meshcom_settings.node_ownip);

    // -----------------------------
    // STATIC CONFIGURATION
    // -----------------------------
    if(cfg_ip != IPAddress(0,0,0,0))
    {
        IPAddress gw;
        IPAddress mask;
        IPAddress dns;

        gw.fromString(meshcom_settings.node_owngw);
        mask.fromString(meshcom_settings.node_ownms);
        dns.fromString(meshcom_settings.node_owndns);

        ETH.config(cfg_ip, gw, mask, dns);

        Serial.print("[ETH] STATIC IP: ");
        Serial.println(cfg_ip);

        snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip), "%s", meshcom_settings.node_ownip);
        snprintf(meshcom_settings.node_gw, sizeof(meshcom_settings.node_gw), "%s", meshcom_settings.node_owngw);
        snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns), "%s", meshcom_settings.node_owndns);
        snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet), "%s", meshcom_settings.node_ownms);

        meshcom_settings.node_hasIPaddress = true;
        Serial.printf("[ETH] node_hasIPaddress=%d\n", meshcom_settings.node_hasIPaddress);
    }
    else
    {
        // -----------------------------
        // DHCP MODE
        // -----------------------------
        Serial.println("[ETH] DHCP mode");

        IPAddress ip;

        for(int i=0;i<50;i++)
        {
            ip = ETH.localIP();

            if(ip[0] != 0)
                break;

            delay(200);
        }

        if(ip[0] != 0)
        {
            Serial.print("[ETH] DHCP IP: ");
            Serial.println(ip);

            meshcom_settings.node_hasIPaddress = true;

            snprintf(meshcom_settings.node_ip, sizeof(meshcom_settings.node_ip),
                "%i.%i.%i.%i", ip[0], ip[1], ip[2], ip[3]);

            IPAddress gw = ETH.gatewayIP();
            IPAddress dns = ETH.dnsIP();
            IPAddress mask = ETH.subnetMask();

            snprintf(meshcom_settings.node_gw, sizeof(meshcom_settings.node_gw),
                "%i.%i.%i.%i", gw[0], gw[1], gw[2], gw[3]);

            snprintf(meshcom_settings.node_dns, sizeof(meshcom_settings.node_dns),
                "%i.%i.%i.%i", dns[0], dns[1], dns[2], dns[3]);

            snprintf(meshcom_settings.node_subnet, sizeof(meshcom_settings.node_subnet),
                "%i.%i.%i.%i", mask[0], mask[1], mask[2], mask[3]);
        }
        else
        {
            Serial.println("[ETH] DHCP failed -> fallback 192.168.4.1");

            IPAddress ip(192,168,4,1);
            IPAddress gw(192,168,4,1);
            IPAddress mask(255,255,255,0);

            ETH.config(ip, gw, mask);

            meshcom_settings.node_hasIPaddress = true;
        }
    }

    Udp.begin(EXTERN_PORT);
}

void EspETH::initethfixIP()
{
    Serial.println("[ETH] init FIX-IP");

    ETH.begin();

    delay(2000);

    IPAddress ip = ETH.localIP();

    if(ip[0] != 0)
    {
        Serial.print("[ETH] IP: ");
        Serial.println(ip);
        meshcom_settings.node_hasIPaddress = true;
    }
    else
    {
        Serial.println("[ETH] IP error");
        meshcom_settings.node_hasIPaddress = true;
    }

    Udp.begin(EXTERN_PORT);
}

bool EspETH::sendUDP(uint8_t *buffer, uint16_t rx_buf_size)
{
    Udp.beginPacket(udp_dest_addr, EXTERN_PORT);

    for(int i=0;i<rx_buf_size;i++)
    {
        Udp.write(buffer[i]);
    }

    return Udp.endPacket();
}

int EspETH::checkUDP()
{
    int packetSize = Udp.parsePacket();

    if(packetSize <= 0)
        return -1;

    return 0;
}

#endif