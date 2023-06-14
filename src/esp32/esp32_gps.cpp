#include "configuration.h"
#include "loop_functions_extern.h"

#if defined (ENABLE_GPS)

#include "esp32_gps.h"
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <clock.h>
#include <configuration.h>

#define GPS_BAUDRATE 9600
#define GPS_SERIAL_NUM 2

#if defined(BOARD_TBEAM) || defined(BOARD_SX1268)
    #define GPS_RX_PIN 34
    #define GPS_TX_PIN 12
#elif defined(BOARD_HELTEC)
    //For heltec these are the pins:
    #define GPS_RX_PIN 2
    #define GPS_TX_PIN 17
#elif defined(BOARD_HELTEC_V3)
    //For heltec these are the pins:
    #define GPS_RX_PIN 37
    #define GPS_TX_PIN 36
#elif defined(BOARD_TLORA_OLV216)
    #define GPS_RX_PIN 13
    #define GPS_TX_PIN 15
#elif defined(BOARD_E22)
    //For heltec these are the pins:
    #define GPS_RX_PIN 16
    #define GPS_TX_PIN 17
#else
    #define GPS_RX_PIN 34
    #define GPS_TX_PIN 12
#endif


#include <HardwareSerial.h>
#include <SparkFun_Ublox_Arduino_Library.h>

HardwareSerial GPS(GPS_SERIAL_NUM);

#include "TinyGPSplus.h"

#if defined(BOARD_TBEAM) || defined(BOARD_SX1268)
#include "axp20x.h"
AXP20X_Class axp;
#endif

// TinyGPS
TinyGPSPlus tinyGPSPlus;

SFE_UBLOX_GPS myGPS;

int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

int state = 0; // steps through auto-baud, reset, etc states

void setupGPS(void)
{
    Wire.begin(I2C_SDA, I2C_SCL);

    #if defined(BOARD_TBEAM) || defined(BOARD_SX1268)
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
        Serial.println("AXP192 Begin PASS");
    } else {
        Serial.println("AXP192 Begin FAIL");
    }
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    Serial.println("All AXP192 started");
    #endif

    delay(100);
}

unsigned int readGPS(void)
{
    //Serial.println("readGPS");
    
    String tmp_data = "";

    bool newData = false;

    if(bDEBUG)
        Serial.println("-----------check GPS-----------");
  
    // For one second we parse GPS data and report some key values
    //for (unsigned long start = millis(); millis() - start < 1000;)
    bool bgrun=true;
    unsigned long start = millis();
    while(bgrun)
    {
      while (GPS.available())
      {
        char c = GPS.read();

        if(c == 0x00)
        {
            bgrun=false;
            break;
        }

        if(bDEBUG)
          Serial.print(c);

        tmp_data += c;
        
        if (tinyGPSPlus.encode(c))// Did a new valid sentence come in?
        {
          newData = true;
        }
      }
    
      if((millis() - start) > 1000)
          bgrun=false;
    }

    if(bDEBUG)
        Serial.printf("newData:%i Fix:%d UPD:%d VAL:%d HDOP:%i\n", newData, tinyGPSPlus.sentencesWithFix(), tinyGPSPlus.location.isUpdated(), tinyGPSPlus.location.isValid(), tinyGPSPlus.hdop.value());

    if (newData && tinyGPSPlus.location.isUpdated() && tinyGPSPlus.location.isValid() && tinyGPSPlus.hdop.isValid() && tinyGPSPlus.hdop.value() < 800)
    {
        direction_parse(tmp_data);

        double dlat, dlon;

        dlat = tinyGPSPlus.location.lat();
        dlon = tinyGPSPlus.location.lng();

        meshcom_settings.node_lat = cround4(dlat);
        meshcom_settings.node_lon = cround4(dlon);

        if(direction_S_N == 0)
        {
            meshcom_settings.node_lat_c = 'S';
        }
        else
        {
            meshcom_settings.node_lat_c = 'N';
        }

        if(direction_E_W == 0)
        {
            meshcom_settings.node_lon_c = 'E';
        }
        else
        {
            meshcom_settings.node_lon_c = 'W';
        }

        meshcom_settings.node_alt = ((meshcom_settings.node_alt * 10) + (int)tinyGPSPlus.altitude.meters()) / 11;

        MyClock.setCurrentTime(true, tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
        
        if(bDEBUG)
        {
            Serial.printf("GPS: LAT:%lf LON:%lf %02d-%02d-%02d %02d:%02d:%02d\n", tinyGPSPlus.location.lat(), tinyGPSPlus.location.lng(), tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
            
            //Serial.printf("INT: LAT:%lf LON:%lf %i-%02i-%02i %02i:%02i:%02i\n", meshcom_settings.node_lat, meshcom_settings.node_lon, meshcom_settings.node_date_year, meshcom_settings.node_date_month,  meshcom_settings.node_date_day,
            // meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second );
        }


        posinfo_satcount = tinyGPSPlus.satellites.value();
        posinfo_hdop = tinyGPSPlus.hdop.value();
        posinfo_fix = true;

        return setSMartBeaconing(dlat, dlon);

    }
    else
    {
        posinfo_fix = false;
        posinfo_satcount = 0;
        posinfo_hdop = 0;
    }

    return 0;
}

unsigned int getGPS(void)
{
    if(!bGPSON)
        return POSINFO_INTERVAL;

    bool bMitHardReset = false;
    
    if(bMitHardReset)
    {
        Serial.print("===== STATE ");
        Serial.println(state);
    }

    switch (state)
    {
        case 0: // auto-baud connection, then switch to 38400 and save config
            do
            {
                Serial.println("GPS: trying 38400 baud");
                
                GPS.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

                if (myGPS.begin(GPS))
                {
                    Serial.println("GPS: connected at 38400 baud");
                    break;
                }

                delay(100);
                Serial.println("GPS: trying 9600 baud");
                GPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
                if (myGPS.begin(GPS)) {
                    Serial.println("GPS: connected at 9600 baud");
                    break;
                    //myGPS.setSerialRate(9600);
                    //delay(100);
                } else {
                    delay(2000); //Wait a bit before trying again to limit the Serial output flood
                }
            }
            while(1);

            myGPS.setUART2Output(COM_TYPE_UBX); //Set the UART port to output UBX only
            myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART2);
            myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART2);
            myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART2);
            myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART2);
            myGPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART2);
            myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART2);
            myGPS.saveConfiguration(); //Save the current settings to flash and BBR

            Serial.println("GPS serial connected, saved config");
            
            state++;
            
            break;
            
        case 1: // hardReset, expect to see GPS back at 38400 baud
            if(bMitHardReset)
            {
                Serial.println("Issuing hardReset (cold start)");
                myGPS.hardReset();
                delay(3000);
                GPS.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
                if (myGPS.begin(GPS)) {
                    Serial.println("Success.");
                    state++;
                } else {
                    Serial.println("*** GPS did not respond at 38400 baud, starting over.");
                    state = 0;
                }
            }
        
            state++;

            break;
            
        case 2: // factoryReset, expect to see GPS back at 9600 baud
            if(bMitHardReset)
            {
                Serial.println("Issuing factoryReset");
                myGPS.factoryReset();
                delay(3000); // takes more than one second... a loop to resync would be best
                GPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
                if (myGPS.begin(GPS)) {
                    Serial.println("Success.");
                    state++;
                } else {
                    Serial.println("*** GPS did not come back at 9600 baud, starting over.");
                    state = 0;
                }
            }
        
            state++;

            break;
            
        case 3: // print version info
            if(bMitHardReset)
            {
                /*
                Serial.print("GPS protocol version: ");
                Serial.print(myGPS.getProtocolVersionHigh());
                Serial.print('.');
                Serial.println(myGPS.getProtocolVersionLow());
                Serial.println();
                */
            
                Serial.println("GPS running");
            }
            
            state++;
        
        case 4: // print position info
        
        if(GPS.available())
        {
            return readGPS();
        }

    }

    return POSINFO_INTERVAL;
}


/**@brief Function for analytical direction.
 */
void direction_parse(String tmp)
{
    if (tmp.indexOf(",E,") != -1)
    {
        direction_E_W = 0;
    }
    else
    {
        direction_E_W = 1;
    }
    
    if (tmp.indexOf(",S,") != -1)
    {
        direction_S_N = 0;
    }
    else
    {
        direction_S_N = 1;
    }
}

#endif
