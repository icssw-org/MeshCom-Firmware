#include "configuration.h"

#if defined (ENABLE_GPS)

#include "esp32_gps.h"
#include <loop_functions.h>
#include <loop_functions_extern.h>

#define GPS_BAUDRATE 9600
#define GPS_SERIAL_NUM 2

#if defined(BOARD_TBEAM) || defined(BOARD_SX1268)
    #define GPS_RX_PIN 34
    #define GPS_TX_PIN 12
    #define I2C_SDA    21
    #define I2C_SCL    22
#elif defined(BOARD_HELTEC)
    //For heltec these are the pins:
    #define GPS_RX_PIN 36
    #define GPS_TX_PIN 37
    #define I2C_SDA    21
    #define I2C_SCL    22
#else
    #define GPS_RX_PIN 34
    #define GPS_TX_PIN 12
    #define I2C_SDA    21
    #define I2C_SCL    22
#endif


#include <HardwareSerial.h>
#include <SparkFun_Ublox_Arduino_Library.h>

#include "TinyGPS.h"

#include "axp20x.h"
AXP20X_Class axp;

// TinyGPS
TinyGPS tinyGPS;

HardwareSerial GPS(GPS_SERIAL_NUM);
SFE_UBLOX_GPS myGPS;

int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

int state = 0; // steps through auto-baud, reset, etc states

void setupGPS(void)
{
    Wire.begin(I2C_SDA, I2C_SCL);
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
    Serial.println("All comms started");
    delay(100);
}

void readGPS(void)
{
    //Serial.println("readGPS");
    
    String tmp_data = "";

    bool newData = false;

    //Serial.printf("GPS check: %i\n", Serial1.available());
  
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
        //Serial.print(c);
        tmp_data += c;
        
        if (tinyGPS.encode(c))// Did a new valid sentence come in?
        {
          newData = true;
        }
      }
    
      if((millis() - start) > 1000)
          bgrun=false;
    }

    if (newData)
    {
        direction_parse(tmp_data);

        float flat, flon;
        unsigned long age;

        tinyGPS.f_get_position(&flat, &flon, &age);
        //flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
        //flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;

        meshcom_settings.node_lat = flat;
        meshcom_settings.node_lon = flon;

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

        meshcom_settings.node_alt = ((meshcom_settings.node_alt * 10) + (int)tinyGPS.f_altitude()) / 11;

        unsigned long date, time;
        tinyGPS.get_datetime(&date, &time, &meshcom_settings.node_age);


        time = time + 2000000;  // MESZ
        if(time > 24000000)
        {
            date++;
            time = time - 24000000;
        }

        meshcom_settings.node_date_hour = time / 1000000;
        meshcom_settings.node_date_minute = (time / 10000) % 100;
        meshcom_settings.node_date_second = (time / 100) % 100;
        meshcom_settings.node_date_hundredths = time % 100;

        meshcom_settings.node_date_year = date % 100;
        meshcom_settings.node_date_year += meshcom_settings.node_date_year > 80 ? 1900 : 2000;
        meshcom_settings.node_date_month = (date / 100) % 100;        
        meshcom_settings.node_date_day = date / 10000;

        //Serial.printf("Time: %ld\n", time);
    }
}

void getGPS(void)
{

    //Serial.print("===== STATE ");
    //Serial.println(state);

    switch (state)
    {
        case 0: // auto-baud connection, then switch to 38400 and save config
            do
            {
                Serial.println("GPS: trying 38400 baud");
                GPS.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
                if (myGPS.begin(GPS)) break;

                delay(100);
                Serial.println("GPS: trying 9600 baud");
                GPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
                if (myGPS.begin(GPS)) {
                    Serial.println("GPS: connected at 9600 baud, switching to 38400");
                    myGPS.setSerialRate(38400);
                    delay(100);
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
        /*
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
        */
            state++;

            break;
            
        case 2: // factoryReset, expect to see GPS back at 9600 baud
        /*
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
        */
            state++;

            break;
            
        case 3: // print version info
        /*
            Serial.print("GPS protocol version: ");
            Serial.print(myGPS.getProtocolVersionHigh());
            Serial.print('.');
            Serial.println(myGPS.getProtocolVersionLow());
            Serial.println();
        */
            Serial.println("GPS running");
            
            state++;
        
        case 4: // print position info
        
        if(GPS.available())
        {
            readGPS();
            /*
            long latitude = myGPS.getLatitude();
            Serial.print(F("Lat: "));
            Serial.print(latitude);

            long longitude = myGPS.getLongitude();
            Serial.print(F(" Long: "));
            Serial.print(longitude);
            Serial.print(F(" (degrees * 10^-7)"));

            long altitude = myGPS.getAltitude();
            Serial.print(F(" Alt: "));
            Serial.print(altitude);
            Serial.print(F(" (mm)"));

            byte SIV = myGPS.getSIV();
            Serial.print(F(" SIV: "));
            Serial.print(SIV);

            Serial.println();
            Serial.println();
            */
        }

    }
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
