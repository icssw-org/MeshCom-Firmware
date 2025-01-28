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

void startMDNS();


void wifiConnect() {
  
  // read wlan credentials from flash
	init_flash();

  const char* ssid = meshcom_settings.node_ssid;
  const char* pass = meshcom_settings.node_pwd;
  bool bWEBSERVER = meshcom_settings.node_sset2 & 0x0040;
  bool bGATEWAY =  meshcom_settings.node_sset & 0x1000;
  bool bWIFIAP = meshcom_settings.node_sset2 & 0x0080;

  Serial.printf("\nNVS Flash Settings:\n");
  Serial.printf("Wifi SSID: %s\n", ssid);
  Serial.printf("Webserver: %d\n", bWEBSERVER);
  Serial.printf("Gateway: %d\n", bGATEWAY);
  Serial.printf("WIFI AP: %d\n", bWIFIAP);


  // When there is no SSID or WIFI-AP is enabled, start AP
  if(strcmp(ssid, "none") == 0 || bWIFIAP)
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
  Serial.println("Rebooting...");
  delay(300);
  ESP.restart();
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
    updateInProgress = false;
    Serial.println("OTA update finished successfully!");
    // Set next boot partition
    setBootPartition_APP();
  }
  else
  {
    updateInProgress = false;
    Serial.println("There was an error during OTA update!");
  }
}



void setup() {

  Serial.begin(115200);
  // whait for serial
  delay(1000);
  Serial.println("\n-----------------------------");
  Serial.println("SafeBoot started");

  // Connect to saved ssid or as fallback spawn an AP
  wifiConnect();

  // Start ElegantOTA
  ElegantOTA.clearAuth();
  ElegantOTA.setAutoReboot(false);
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
      setBootPartition_APP();
    }
  });

  webServer.begin();

}

void loop() {
  ElegantOTA.loop();
}
