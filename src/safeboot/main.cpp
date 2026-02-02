// SPDX-License-Identifier: GPL-3.0-or-later
/*
* Copyright (C) 2023-2024 Mathieu Carbou
*/

#include "ElegantOTA.h"
#include <WiFi.h>
 
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
 
#include <esp_ota_ops.h>
#include <esp_partition.h>
 
#include <Preferences.h>
#include "../esp32/esp32_flash.h"
 
#define TAG "SafeBoot"
 
const unsigned int port = 80;
AsyncWebServer webServer(port);
String hostname = "MeshCom-OTA";
 
extern Preferences preferences;
extern s_meshcom_settings meshcom_settings;
 
bool updateInProgress = false;  // Flag to indicate if an update is in progress
 
unsigned int ota_timeout_timer = 0; // Timer to check if OTA was started. If not, reboot to app/ota partition
unsigned int last_timer_update = 0; // Timer to update the last time the OTA was updated
int wait_ota_timeout = 180 * 1000; // OTA Timeout in seconds
boolean reboot_after_cancel = false; // Reboot after cancelling OTA if no update was started
 
void startMDNS();
 
 
void wifiConnect() {

  // read wlan credentials from flash
  init_flash();

  const char *ssid = meshcom_settings.node_ssid;
  const char *pass = meshcom_settings.node_pwd;
  bool bWEBSERVER = meshcom_settings.node_sset2 & 0x0040;
  bool bGATEWAY = meshcom_settings.node_sset & 0x1000;
  bool bWIFIAP = meshcom_settings.node_sset2 & 0x0080;

  Serial.printf("\nNVS Flash Settings:\n");
  Serial.printf("Callsign: %s\n", meshcom_settings.node_call);
  Serial.printf("Wifi SSID: %s\n", ssid);
  Serial.printf("Webserver: %d\n", bWEBSERVER);
  Serial.printf("Gateway: %d\n", bGATEWAY);
  Serial.printf("WIFI AP: %d\n", bWIFIAP);

  

  // Set the hostname from the callsign. If the callsign is not set, use the default hostname
  if (!((meshcom_settings.node_call[0] == 0x00) || (memcmp(meshcom_settings.node_call, "none", 4) == 0) || (memcmp(meshcom_settings.node_call, "XX0XXX", 6) == 0) || (memcmp(meshcom_settings.node_call, "XX0XXX-00", 9) == 0)))
  {
    hostname = meshcom_settings.node_call;
  }

  // When there is no SSID or WIFI-AP is enabled, start AP
  if (strcmp(ssid, "none") == 0 || bWIFIAP)
  {
    Serial.println("\nStarting Wifi AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(hostname);
    delay(300);
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    // start mDNS responder
    startMDNS();
    return;
  }

   WiFi.mode(WIFI_STA);
   WiFi.disconnect(true);

   // Static IP settings
  IPAddress node_ip = IPAddress(0,0,0,0);
  IPAddress node_gw = IPAddress(0,0,0,0);
  IPAddress node_ms = IPAddress(0,0,0,0);
  IPAddress node_dns = IPAddress(0,0,0,0);

  if (strlen(meshcom_settings.node_ownip) >= 7 && strlen(meshcom_settings.node_owngw) >= 7 && strlen(meshcom_settings.node_ownms) >= 7 && bWIFIAP == false)
  {
    Serial.printf("Static IP settings:\n");
    Serial.printf("IP: %s\n", meshcom_settings.node_ownip);
    Serial.printf("GW: %s\n", meshcom_settings.node_owngw);
    Serial.printf("MS: %s\n", meshcom_settings.node_ownms);
    Serial.printf("DNS: %s\n", meshcom_settings.node_owndns);

    // Set your Static IP address
    node_ip.fromString(meshcom_settings.node_ownip);
    // Set your Gateway IP address
    node_gw.fromString(meshcom_settings.node_owngw);
    // Set your Gateway IP mask
    node_ms.fromString(meshcom_settings.node_ownms);
    // Set your DNS IP
    if (strlen(meshcom_settings.node_owndns) >= 7)
      node_dns.fromString(meshcom_settings.node_owndns);
    else
      node_dns.fromString("8.8.8.8");

    // Configures static IP address
    if (!WiFi.config(node_ip, node_gw, node_ms, node_dns))
    {
      Serial.println("[Error] STA Failed to configure static IP!");
    }
  }

   delay(500);
 
   // Scan for AP with best RSSI
   int nrAps = WiFi.scanNetworks();
   int best_rssi = -200;
   int best_idx = -1;
   for (int i = 0; i < nrAps; ++i)
   {
      
      if(strcmp(WiFi.SSID(i).c_str(), ssid) == 0)
      {
         Serial.printf("SSID: %s CHAN: %d RSSI: %d BSSID: ", WiFi.SSID(i).c_str(), (int) WiFi.channel(i), (int) WiFi.RSSI(i));
         uint8_t *bssid = WiFi.BSSID(i);
         for (byte i = 0; i < 6; i++){
           Serial.print(*bssid++, HEX);
           if (i < 5) Serial.print(":");
         }
         Serial.println("");
         if(WiFi.RSSI(i) > best_rssi)
         {
           best_rssi = WiFi.RSSI(i);
           best_idx = i;
         }
      }
   }	
 
   if(best_idx == -1)
   {
     // ESP32 - force connecting (in case of hidden ssid or out of range atm)
     Serial.printf("-> try connecting to SSID: %s \n",ssid);	
     WiFi.mode(WIFI_STA);
     
     if(strcmp(pass, "none") == 0)
       WiFi.begin(ssid, NULL);
     else
       WiFi.begin(ssid, pass);
   
   }
   else
   {
     // ESP32 - connecting to strongest ssid
     Serial.printf("-> connecting to CHAN: %d BSSID: ",(int) WiFi.channel(best_idx));	
     uint8_t *bssid = WiFi.BSSID(best_idx);
     for (byte i = 0; i < 6; i++){
       Serial.print(*bssid++, HEX);
       if (i < 5) Serial.print(":");
       }
     Serial.println("");
     WiFi.mode(WIFI_STA);
     
     if(strcmp(pass, "none") == 0)
       WiFi.begin(ssid, NULL, WiFi.channel(best_idx), WiFi.BSSID(best_idx),true);
     else
       WiFi.begin(ssid, pass, WiFi.channel(best_idx), WiFi.BSSID(best_idx),true);
   }
   delay(500);
  
   
 
   Serial.println("Connecting to WiFi");
 
   int iWlanWait = 0;
 
   while(WiFi.status() != WL_CONNECTED)
   {
     delay(1000);
     iWlanWait++;
     Serial.print(".");
     if(iWlanWait == 5) WiFi.reconnect();
 
     if(iWlanWait > 15)
     {
       // Start AP
       Serial.println("\nStarting AP");
       WiFi.mode(WIFI_AP);
       WiFi.softAP(hostname);
       delay(300);
       Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
       return;
     }
   }
 
   Serial.println("\nConnected to WiFi");
   Serial.print("IP Address: ");
   Serial.println(WiFi.localIP());
   // start mDNS responder
   startMDNS();
   return;
 }
 
 
 
 // start mdns responder and set hostname and tcp service
 void startMDNS()
 {
   if (!MDNS.begin(hostname.c_str()))
   {
     Serial.println("Error setting up MDNS responder!");
   }
   Serial.println("mDNS responder started");
   if(MDNS.addService("http", "tcp", port))
   {
     Serial.println("mDNS http service added");
   }
   else
   {
     Serial.println("Error setting up mDNS service!");
   }
 }
 
 
 // set partition to ota_0 and reboot
 void setBootPartition_APP()
 {
   const esp_partition_t *partition = esp_partition_find_first(esp_partition_type_t::ESP_PARTITION_TYPE_APP, esp_partition_subtype_t::ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
   if (partition)
   {
     esp_ota_set_boot_partition(partition);
   } 
   else
   {
     Serial.println("Error setting boot partition!");
   }
 }
 
 
 // ElegantOTA Callbacks
 unsigned long ota_progress_millis = 0;
 
 void onOTAStart() {
   // Log when OTA has started
   updateInProgress = true;
   Serial.println("OTA update started!");
 }
 
 void onOTAProgress(size_t current, size_t final) {
   // Log every 1 second
   if (millis() - ota_progress_millis > 1000) {
     ota_progress_millis = millis();
     Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
   }
 }
 
 void onOTAEnd(bool success)
 {
   // Log when OTA has finished
   if (success)
   {
     Serial.println("OTA update finished successfully!");
     // Set next boot partition
     setBootPartition_APP();
   }
   else
   {
     Serial.println("There was an error during OTA update!");
   }
 }
 
 
 
 void setup() {
 
   Serial.begin(115200);
   // whait for serial
   delay(1000);
   Serial.println("\n-----------------------------");
   Serial.println("OTA UDATE started");
 
   // Connect to saved ssid or as fallback spawn an AP
   wifiConnect();
 
   // Start ElegantOTA
   ElegantOTA.clearAuth();
   ElegantOTA.setAutoReboot(true);
   ElegantOTA.begin(&webServer);
   // ElegantOTA callbacks
   ElegantOTA.onStart(onOTAStart);
   ElegantOTA.onProgress(onOTAProgress);
   ElegantOTA.onEnd(onOTAEnd);
 
   // Start web server
   webServer.rewrite("/", "/update");
   webServer.onNotFound([](AsyncWebServerRequest* request) {
     request->redirect("/");
   });
 
   //endpoint for canceling the update. Only works if the update has not started yet
   webServer.on("/ota/cancel", HTTP_GET, [](AsyncWebServerRequest *request) {
     if(updateInProgress)
     {
       request->send(400, "text/plain", "OTA update in progress. Cannot cancel.");
     }
     else
     {
       request->send(200, "text/plain", "OTA update canceled.");
       reboot_after_cancel = true;
     }
   });
 
   webServer.begin();
 
   ota_timeout_timer = millis();
   last_timer_update = millis();
 
 }
 
 void loop() {
   ElegantOTA.loop();
   // Check if OTA was started. If not, reboot to app/ota partition
   if((!updateInProgress && (ota_timeout_timer > wait_ota_timeout)) || reboot_after_cancel)
   {
     Serial.println("OTA Start Timeout. Rebooting to app partition.");
     setBootPartition_APP();
     delay(1000);
     ESP.restart();
   } 
   
   if (millis() - last_timer_update > 1000)
   {
     ota_timeout_timer = millis();
     last_timer_update = millis();
   }
 }