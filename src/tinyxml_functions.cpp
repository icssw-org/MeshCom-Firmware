#include <configuration.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>

#include <Arduino.h>

#if defined(ENABLE_XML)

#include <tinyxml_functions.h>

#include <tinyxml2.h>

using namespace tinyxml2;

XMLDocument xmlDocument;

void testTinyXML()
{
//char * testDocument = (char*)"<StationDataList><StationData stationId=\"0077234567\" name=\"DemoStationNetDL500\" timezone=\"+01:00\"><ChannelData channelId=\"0050\" name=\"Wassertemperatur\" unit=\"&#176;C\"><Values><VT t=\"2025-04-22T13:00:00\">25.2</VT><VT>35.2</VT></Values></ChannelData><ChannelData channelId=\"0065\" name=\"Batteriespannung\" unit=\"V\"><Values><VT>2.2</VT><VT>3.2</VT></Values></ChannelData></StationData></StationDataList>";


String testDocument = "<StationDataList><StationData stationId=\"0077234567\" name=\"DemoStationNetDL500\" timezone=\"+01:00\"> \
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
<VT t=\"2025-04-22T13:00:00\">30.1</VT> \
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
<VT t=\"2025-04-22T13:00:00\">22.7</VT> \
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
<VT t=\"2025-04-22T13:00:00\">12.9</VT> \
</Values> \
</ChannelData> \
</StationData> \
</StationDataList>";

  decodeTinyXML(testDocument);

}

////////////////////////////////////////////////////////
// SOFTSER APP=1 DECODE
extern String strTELE_PARM;
extern String strTELE_UNIT;
extern String strTELE_VALUES;
extern String strTELE_DATETIME;
extern String strTELE_CH_ID;
extern String strTELE_UTCOFF;

String strChannelId="";

bool decodeTinyXML(String document)
{
  if(bSOFTSERDEBUG)Serial.println("decodeTinyXML started....");

//  if(bSOFTSERDEBUG)Serial.println(document);

  if(xmlDocument.Parse(document.c_str())!= XML_SUCCESS)
  {
    Serial.println("[APP]...Error parsing"); 
    return false; 
  }

  strTELE_PARM = "";
  strTELE_UNIT = "";
  strTELE_VALUES = "";
  strTELE_DATETIME = "";
  strTELE_CH_ID = "";
  strTELE_UTCOFF = "";

  XMLNode * root = xmlDocument.FirstChild();

  XMLElement * station = root->FirstChildElement("StationData");

  while(station != NULL)
  {

    strSOFTSERAPP_ID = station->Attribute("stationId");
    if(bSOFTSERDEBUG)if(bSOFTSERDEBUG)Serial.printf("Station...%s\n", strSOFTSERAPP_ID.c_str());

    strSOFTSERAPP_NAME = station->Attribute("name");
    if(bSOFTSERDEBUG)Serial.printf("Station Name...%s\n", strSOFTSERAPP_NAME.c_str());

    strTELE_UTCOFF = station->Attribute("timezone");
    if(bSOFTSERDEBUG)Serial.printf("Station timezone...%s\n", strTELE_UTCOFF.c_str());

    if(bSOFTSERDEBUG)Serial.print("  while ChannelData:");
    if(bSOFTSERDEBUG)Serial.print(station->ChildElementCount("ChannelData"));

    XMLElement * channel = station->FirstChildElement("ChannelData");

    while(channel != NULL)
    {
      if(bSOFTSERDEBUG)Serial.print(" ");
      
      strChannelId = channel->Attribute("channelId");

      if(strTELE_PARM.length() > 0)
        strTELE_PARM.concat(",");
      strTELE_PARM.concat(strChannelId.substring(2));
      strTELE_PARM.concat(" ");
      strTELE_PARM.concat(channel->Attribute("name"));

      if(strTELE_CH_ID.length() > 0)
        strTELE_CH_ID.concat(",");
      strTELE_CH_ID.concat(strChannelId.substring(2));

      if(bSOFTSERDEBUG)
      {
        Serial.print(strChannelId);
        Serial.print(" ");
        Serial.print(channel->Attribute("name"));
        Serial.print(" ");
      }

      if(strTELE_UNIT.length() > 0)
        strTELE_UNIT.concat(",");
      strTELE_UNIT.concat(channel->Attribute("unit"));

      if(bSOFTSERDEBUG)Serial.println(channel->Attribute("unit"));

      if(bSOFTSERDEBUG)
      {
        Serial.print("   while values:");
        Serial.println(channel->ChildElementCount("Values"));
      }

      XMLElement * values = channel->FirstChildElement("Values");

      while(values != NULL)
      {
        if(bSOFTSERDEBUG)Serial.print("    while VT:");
        if(bSOFTSERDEBUG)Serial.println(values->ChildElementCount("VT"));

        XMLElement * vt = values->LastChildElement("VT");

        while(vt != NULL)
        {
          if(bSOFTSERDEBUG)
          {
            Serial.print("    VAL:");
            Serial.print(vt->Attribute("t"));
            Serial.print(" ");
          }

          if(strTELE_DATETIME.length() < 1)
            strTELE_DATETIME.concat(vt->Attribute("t"));

          float val;

          vt->QueryFloatText(&val);

          if(bSOFTSERDEBUG)Serial.println(val);

          char cval[10];
          snprintf(cval, sizeof(cval), "%.1f", val);

          if(strTELE_VALUES.length() > 0)
            strTELE_VALUES.concat(",");
          strTELE_VALUES.concat(cval);

          vt = vt->NextSiblingElement("VT");
        }

        if(bSOFTSERDEBUG)Serial.println("   next Values");

        values = values->NextSiblingElement("Values");
      }


      if(bSOFTSERDEBUG)Serial.println("  next ChannelData");

      channel = channel->NextSiblingElement("ChannelData");
    }

    if(bSOFTSERDEBUG)Serial.println(" next StationData");

    station = station->NextSiblingElement("StationData");
  }

  if(bSOFTSERDEBUG)Serial.println("next StationDataList");

  // fill Telemetry
  snprintf(meshcom_settings.node_parm_1, sizeof(meshcom_settings.node_parm_1), "%s", strTELE_PARM.c_str());
  if(bSOFTSERDEBUG)Serial.println(meshcom_settings.node_parm_1);
  snprintf(meshcom_settings.node_unit, sizeof(meshcom_settings.node_unit), "%s", strTELE_UNIT.c_str());
  if(bSOFTSERDEBUG)Serial.println(meshcom_settings.node_unit);
  snprintf(meshcom_settings.node_values, sizeof(meshcom_settings.node_values), "T:%s", strTELE_VALUES.c_str());
  if(bSOFTSERDEBUG)Serial.println(meshcom_settings.node_values);
  snprintf(meshcom_settings.node_parm_t, sizeof(meshcom_settings.node_parm_t), "%s", strTELE_DATETIME.c_str());
  if(bSOFTSERDEBUG)Serial.println(meshcom_settings.node_parm_t);
  snprintf(meshcom_settings.node_parm_id, sizeof(meshcom_settings.node_parm_id), "%s", strTELE_CH_ID.c_str());
  if(bSOFTSERDEBUG)Serial.println(meshcom_settings.node_parm_id);
  
  /*
  strTELE_UTCOFF.replace(":", ".");
  meshcom_settings.node_utcoff = strTELE_UTCOFF.toDouble();
  */
  meshcom_settings.node_utcoff = 0.0;

  return true;
  
}

#endif