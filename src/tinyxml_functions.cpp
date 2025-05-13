#include <configuration.h>

#include <Arduino.h>

#if defined(ENABLE_XML)

#include <tinyxml_functions.h>

#include <tinyxml2.h>

using namespace tinyxml2;

XMLDocument xmlDocument;

void testTinyXML()
{
char * testDocument = (char*)"<StationDataList><StationData stationId=\"0077234567\" name=\"DemoStationNetDL500\" timezone=\"+01:00\"><ChannelData channelId=\"0050\" name=\"Wassertemperatur\" unit=\"&#176;C\"><Values><VT t=\"2025-04-22T13:00:00\">25.2</VT><VT>35.2</VT></Values></ChannelData><ChannelData channelId=\"0065\" name=\"Batteriespannung\" unit=\"V\"><Values><VT>2.2</VT><VT>3.2</VT></Values></ChannelData></StationData></StationDataList>";
/*
char * testDocument = (char*)"<StationDataList><StationData stationId=\"0077234567\" name=\"DemoStationNetDL500\" timezone=\"+01:00\"> \
<StationInfo time=\"2025-04-22T13:04:19\" firmware=\"V3080\" configtime=\"2025-04-09T05:49:48\" paramtime=\"2025-04-09T05:49:48\" batteryVoltage=\"13.34\" temperature=\"28.62\" deviceType=\"OTT netDL 500\" providerName=\"none\" gsmSignal=\"0\" ipAddress=\"0.0.0.0\" transmissionCycle=\"360\" transmissionOffset=\"0\" configuredTransmissionCycle=\"360\" /> \
<ChannelData channelId=\"0060\" name=\"Wasserstand\" unit=\"cm\" samplingInterval=\"300\" storageInterval=\"300\" configuredSamplingInterval=\"300\" configuredStorageInterval=\"300\"  > \
<Values> \
<VT t=\"2025-04-22T12:00:00\">28.3</VT> \
<VT t=\"2025-04-22T12:05:00\">28.4</VT> \
<VT t=\"2025-04-22T12:10:00\">28.4</VT> \
<VT t=\"2025-04-22T12:15:00\">28.4</VT> \
<VT t=\"2025-04-22T12:20:00\">28.4</VT> \
<VT t=\"2025-04-22T12:25:00\">28.4</VT> \
<VT t=\"2025-04-22T12:30:00\">28.4</VT> \
<VT t=\"2025-04-22T12:35:00\">28.4</VT> \
<VT t=\"2025-04-22T12:40:00\">28.4</VT> \
<VT t=\"2025-04-22T12:45:00\">28.4</VT> \
<VT t=\"2025-04-22T12:50:00\">28.4</VT> \
<VT t=\"2025-04-22T12:55:00\">28.5</VT> \
<VT t=\"2025-04-22T13:00:00\">28.5</VT> \
</Values> \
</ChannelData> \
<ChannelData channelId=\"0065\" name=\"Wassertemperatur\" unit=\"&#176;C\" samplingInterval=\"300\" storageInterval=\"300\" configuredSamplingInterval=\"300\" configuredStorageInterval=\"300\"  > \
<Values> \
<VT t=\"2025-04-22T12:05:00\">22.0</VT> \
<VT t=\"2025-04-22T12:10:00\">25.0</VT> \
<VT t=\"2025-04-22T12:15:00\">24.0</VT> \
<VT t=\"2025-04-22T12:20:00\">23.0</VT> \
<VT t=\"2025-04-22T12:25:00\">27.0</VT> \
<VT t=\"2025-04-22T12:30:00\">27.0</VT> \
<VT t=\"2025-04-22T12:35:00\">27.0</VT> \
<VT t=\"2025-04-22T12:40:00\">26.0</VT> \
<VT t=\"2025-04-22T12:45:00\">25.0</VT> \
<VT t=\"2025-04-22T12:50:00\">25.0</VT> \
<VT t=\"2025-04-22T12:55:00\">22.0</VT> \
<VT t=\"2025-04-22T13:00:00\">25.0</VT> \
</Values> \
</ChannelData> \
<ChannelData channelId=\"0050\" name=\"Batteriespannung\" unit=\"V\" samplingInterval=\"300\" storageInterval=\"300\" configuredSamplingInterval=\"300\" configuredStorageInterval=\"300\"  > \
<Values> \
<VT t=\"2025-04-22T12:00:00\">13.2</VT> \
<VT t=\"2025-04-22T12:05:00\">13.2</VT> \
<VT t=\"2025-04-22T12:10:00\">13.2</VT> \
<VT t=\"2025-04-22T12:15:00\">13.3</VT> \
<VT t=\"2025-04-22T12:20:00\">13.2</VT> \
<VT t=\"2025-04-22T12:25:00\">13.2</VT> \
<VT t=\"2025-04-22T12:30:00\">13.3</VT> \
<VT t=\"2025-04-22T12:35:00\">13.2</VT> \
<VT t=\"2025-04-22T12:40:00\">13.2</VT> \
<VT t=\"2025-04-22T12:45:00\">13.2</VT> \
<VT t=\"2025-04-22T12:50:00\">13.2</VT> \
<VT t=\"2025-04-22T12:55:00\">13.4</VT> \
<VT t=\"2025-04-22T13:00:00\">13.4</VT> \
</Values> \
</ChannelData> \
</StationData> \
</StationDataList>";
*/
  decodeTinyXML(testDocument);

}

////////////////////////////////////////////////////////
// SOFTSER APP=1 DECODE
extern String strSOFTSERAPP_ID;
extern String strSOFTSERAPP_NAME;
extern String strSOFTSERAPP_PEGEL;
extern String strSOFTSERAPP_PEGEL2;
extern String strSOFTSERAPP_TEMP;
extern String strSOFTSERAPP_BATT;

String strChannelId="";

void decodeTinyXML(char* document)
{
  if(xmlDocument.Parse(document)!= XML_SUCCESS)
  {
    Serial.println("Error parsing"); 
    return; 
  }

  strSOFTSERAPP_BATT = "";
  strSOFTSERAPP_TEMP = "";
  strSOFTSERAPP_PEGEL = "";
  strSOFTSERAPP_PEGEL2 = "";

  XMLNode * root = xmlDocument.FirstChild();

  XMLElement * station = root->FirstChildElement("StationData");

  while(station != NULL)
  {
    strSOFTSERAPP_ID = station->Attribute("stationId");
    Serial.printf("Station...%s\n", strSOFTSERAPP_ID.c_str());

    strSOFTSERAPP_NAME = station->Attribute("name");
    Serial.printf("Station Name...%s\n", strSOFTSERAPP_NAME.c_str());

    Serial.println(station->Attribute("timezone"));

    Serial.print("  while ChannelData:");
    Serial.print(station->ChildElementCount("ChannelData"));

    XMLElement * channel = station->FirstChildElement("ChannelData");

    while(channel != NULL)
    {
      Serial.print(" ");
      
      strChannelId = channel->Attribute("channelId");

      Serial.print(channel->Attribute("channelId"));
      Serial.print(" ");
      Serial.print(channel->Attribute("name"));
      Serial.print(" ");
      Serial.println(channel->Attribute("unit"));

      Serial.print("   while values:");
      Serial.println(channel->ChildElementCount("Values"));

      XMLElement * values = channel->FirstChildElement("Values");

      while(values != NULL)
      {
        Serial.print("    while VT:");
        Serial.println(values->ChildElementCount("VT"));

        XMLElement * vt = values->LastChildElement("VT");

        while(vt != NULL)
        {
          Serial.print("    VAL:");
          Serial.print(vt->Attribute("t"));
          Serial.print(" ");

          float val;

          vt->QueryFloatText(&val);

          Serial.println(val);

          char cval[10];
          snprintf(cval, sizeof(cval), "%.1f", val);

          if(strChannelId == "0050")
            strSOFTSERAPP_BATT = cval;
          else
          if(strChannelId == "0065")
            strSOFTSERAPP_TEMP = cval;
          else
          if(strChannelId == "0060")
            strSOFTSERAPP_PEGEL = cval;
          else
          if(strChannelId == "0061")
            strSOFTSERAPP_PEGEL2 = cval;

      
          vt = vt->NextSiblingElement("VT");
        }

        Serial.println("   next Values");

        values = values->NextSiblingElement("Values");
      }


      Serial.println("  next ChannelData");

      channel = channel->NextSiblingElement("ChannelData");
    }

    Serial.println(" next StationData");

    station = station->NextSiblingElement("StationData");
  }

  Serial.println("next StationDataList");

}

#endif