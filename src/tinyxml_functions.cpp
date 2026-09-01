#include <configuration.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>

#include <Arduino.h>

#include <cctype>

#if defined(ENABLE_XML)

#include <tinyxml_functions.h>

#include <tinyxml2.h>

using namespace tinyxml2;

XMLDocument xmlDocument;

/* only for tresting
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
*/

////////////////////////////////////////////////////////
// SOFTSER APP=1 DECODE
extern String strTELE_PARM;
extern String strTELE_UNIT;
extern String strTELE_VALUES;
extern String strTELE_DATETIME;
extern String strTELE_CH_ID;
extern String strTELE_UTCOFF;

extern unsigned long  lTELE_TIMER;

String strChannelId="";

bool decodeTinyXML(String document)
{
  lTELE_TIMER = millis();

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
#ifndef NATIVE_BUILD
    // Native Serial-Stub (test/support/Arduino.h) hat kein print(int) --
    // dieser Debug-Zweig ist hardwareonly, siehe PT-01 native_xml.
    if(bSOFTSERDEBUG)Serial.print(station->ChildElementCount("ChannelData"));
#endif

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
#ifndef NATIVE_BUILD
        // Native Serial-Stub hat kein println(int) -- hardwareonly, siehe PT-01 native_xml.
        Serial.println(channel->ChildElementCount("Values"));
#endif
      }

      XMLElement * values = channel->FirstChildElement("Values");

      while(values != NULL)
      {
        if(bSOFTSERDEBUG)Serial.print("    while VT:");
#ifndef NATIVE_BUILD
        // Native Serial-Stub hat kein println(int) -- hardwareonly, siehe PT-01 native_xml.
        if(bSOFTSERDEBUG)Serial.println(values->ChildElementCount("VT"));
#endif

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

          float val = 0.0f;

          // PT-01 finding 7: tinyxml2 leaves `val` untouched on
          // XML_NO_TEXT_NODE / XML_CAN_NOT_CONVERT_TEXT (e.g. an empty or
          // non-numeric <VT>), so the old unchecked call formatted an
          // uninitialized stack float into strTELE_VALUES / node_values and
          // relayed it as telemetry. strTELE_PARM and strTELE_VALUES are
          // built in lockstep -- exactly one entry per channel -- and
          // sendTelemetry() (loop_functions.cpp) consumes both lists
          // positionally, so this reading cannot simply be skipped without
          // shifting every later channel's value out of alignment with its
          // PARM name. Emit an explicit 0.0 placeholder instead and flag it
          // on the console.
          if(vt->QueryFloatText(&val) != XML_SUCCESS)
          {
            val = 0.0f;
            Serial.printf("[TXML];vt;novalue\n");
          }

#ifndef NATIVE_BUILD
          // Native Serial-Stub hat kein println(float) -- hardwareonly, siehe PT-01 native_xml.
          if(bSOFTSERDEBUG)Serial.println(val);
#endif

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
  
  // PT-01 finding 8: convert the station's "+HH:MM" / "-HH:MM" timezone
  // attribute (already captured correctly in strTELE_UTCOFF above) into
  // decimal hours, e.g. "+01:00" -> 1.0, "-03:30" -> -3.5. The previous
  // `replace(":", ".")` + toFloat() shortcut is wrong for non-zero minutes
  // (-03:30 -> -3.30, not -3.5), so hours and minutes are parsed and
  // combined separately here. Only overwrite node_utcoff when the attribute
  // is present and well-formed; otherwise leave it untouched rather than
  // stomping a previously-good value with a bogus 0.0. This restores the
  // dead timezone path -- on XML-station nodes node_utcoff now follows the
  // station's reported offset instead of always reading 0.0, which changes
  // the displayed local time (MyClock.setCurrentTime, the web UI); that is
  // the intended effect of fixing this defect, not a side effect.
  if(strTELE_UTCOFF.length() >= 4 &&
     (strTELE_UTCOFF.charAt(0) == '+' || strTELE_UTCOFF.charAt(0) == '-'))
  {
    int colonPos = strTELE_UTCOFF.indexOf(':');

    if(colonPos > 1)
    {
      String hourPart = strTELE_UTCOFF.substring(1, colonPos);
      String minPart  = strTELE_UTCOFF.substring(colonPos + 1);

      bool hourNumeric = hourPart.length() > 0;
      for(unsigned int i = 0; i < hourPart.length() && hourNumeric; i++)
        if(!isdigit((unsigned char)hourPart.charAt(i)))
          hourNumeric = false;

      bool minNumeric = minPart.length() > 0;
      for(unsigned int i = 0; i < minPart.length() && minNumeric; i++)
        if(!isdigit((unsigned char)minPart.charAt(i)))
          minNumeric = false;

      if(hourNumeric && minNumeric)
      {
        float offset = hourPart.toFloat() + (minPart.toFloat() / 60.0f);
        if(strTELE_UTCOFF.charAt(0) == '-')
          offset = -offset;

        meshcom_settings.node_utcoff = offset;
      }
    }
  }

  return true;
  
}

#endif