#include "configuration.h"

#if defined (ENABLE_GPS)

#include "esp32_gps.h"
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <clock.h>

#define GPS_BAUDRATE 9600
#define GPS_SERIAL_NUM 2

#if defined(MODUL_FW_TBEAM)
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

#if defined(XPOWERS_CHIP_AXP192)
// Defined using AXP192
#define XPOWERS_CHIP_AXP192

//#include <axp20x.h>
//extern AXP20X_Class axp;
#include "XPowersLib.h"

XPowersPMU PMU;

#endif

#if defined(XPOWERS_CHIP_AXP2101)
// Defined using AXP192
#define XPOWERS_CHIP_AXP12101

//#include <axp20x.h>
//extern AXP20X_Class axp;
#include "XPowersLib.h"

XPowersPMU PMU;

#endif

// TinyGPS
TinyGPSPlus tinyGPSPlus;

SFE_UBLOX_GPS myGPS;

int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

int state = 0; // steps through auto-baud, reset, etc states

bool bMitHardReset = false;
    
int maxStateCount=1;

void setupGPS(bool bGPSON)
{
    #if defined(XPOWERS_CHIP_AXP192)
    /* OLD
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
    */

    bool result = PMU.begin(Wire, AXP192_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);

    if (result == false)
    {
        Serial.println("PMU is not online...\nPLEASE check Firmware");
    }

    Serial.printf("AXP-Chip ID:0x%x\n", PMU.getChipID());

    // Set the minimum system operating voltage inside the PMU,
    // below this value will shut down the PMU
    // Range: 2600~3300mV
    PMU.setSysPowerDownVoltage(2700);

    // Set the minimum common working voltage of the PMU VBUS input,
    // below this value will turn off the PMU
    PMU.setVbusVoltageLimit(XPOWERS_AXP192_VBUS_VOL_LIM_4V5);

    // Turn off USB input current limit
    PMU.setVbusCurrentLimit(XPOWERS_AXP192_VBUS_CUR_LIM_OFF);

    // DC1 700~3500mV, IMAX=1.2A
    PMU.setDC1Voltage(3300);    // OLED
    Serial.printf("DC1  :%s   Voltage:%u mV \n",  PMU.isEnableDC1()  ? "+" : "-", PMU.getDC1Voltage());

    // DC2 700~2750 mV, IMAX=1.6A;
    //PMU.setDC2Voltage(700);
    //Serial.printf("DC2  :%s   Voltage:%u mV \n",  PMU.isEnableDC2()  ? "+" : "-", PMU.getDC2Voltage());

    // DC3 700~3500 mV,IMAX=0.7A;
    PMU.setDC3Voltage(3300);    // ESP32
    Serial.printf("DC3  :%s   Voltage:%u mV \n",  PMU.isEnableDC3()  ? "+" : "-", PMU.getDC3Voltage());


    //LDO2 1800~3300 mV, 100mV/step, IMAX=200mA
    PMU.setLDO2Voltage(3300);   // LORA

    //LDO3 1800~3300 mV, 100mV/step, IMAX=200mA
    PMU.setLDO3Voltage(3300);   // GPS

    //LDOio 1800~3300 mV, 100mV/step, IMAX=50mA
    PMU.setLDOioVoltage(3300);


    // Enable power output channel
    PMU.disableDC2();   // not used

    PMU.enableDC1();    // OLED
    PMU.enableDC3();    // ESP32
    PMU.enableLDO2();   // LORA RADIO 3.3V
    
    if(bGPSON)
        PMU.enableLDO3();   // GPS 3.3V
    else
        PMU.disableLDO3();   // GPS 3.3V
    
    PMU.enableExternalPin();
    
    PMU.enableLDOio();

    // set battery temperature sensing pin off to save power
    PMU.disableTSPinMeasure();

    // Enable internal ADC detection
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();

    //disable all axp chip interrupt
    PMU.disableIRQ(XPOWERS_AXP192_ALL_IRQ);

    Serial.println("All AXP192 started");
    #endif

    #if defined(XPOWERS_CHIP_AXP2101)

    bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);

    if (result == false)
    {
        Serial.println("PMU is not online...\nPLEASE check Firmware");
    }

    Serial.printf("AXP-Chip ID:0x%x\n", PMU.getChipID());

    // Set the minimum common working voltage of the PMU VBUS input,
    // below this value will turn off the PMU
    PMU.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);

    // Set the maximum current of the PMU VBUS input,
    // higher than this value will turn off the PMU
    PMU.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);


    // Get the VSYS shutdown voltage
    uint16_t vol = PMU.getSysPowerDownVoltage();
    Serial.printf("->  getSysPowerDownVoltage:%u\n", vol);

    // Set VSY off voltage as 2600mV , Adjustment range 2600mV ~ 3300mV
    PMU.setSysPowerDownVoltage(2600);

    vol = PMU.getSysPowerDownVoltage();
    Serial.printf("->  getSysPowerDownVoltage:%u\n", vol);


    // DC1 IMAX=2A
    // 1500~3400mV,100mV/step,20steps
    PMU.setDC1Voltage(3300);
    Serial.printf("DC1  : %s   Voltage:%u mV \n",  PMU.isEnableDC1()  ? "+" : "-", PMU.getDC1Voltage());

    // DC2 IMAX=2A
    // 500~1200mV  10mV/step,71steps
    // 1220~1540mV 20mV/step,17steps
    PMU.setDC2Voltage(1000);
    Serial.printf("DC2  : %s   Voltage:%u mV \n",  PMU.isEnableDC2()  ? "+" : "-", PMU.getDC2Voltage());

    // DC3 IMAX = 2A
    // 500~1200mV,10mV/step,71steps
    // 1220~1540mV,20mV/step,17steps
    // 1600~3400mV,100mV/step,19steps
    PMU.setDC3Voltage(3300);
    Serial.printf("DC3  : %s   Voltage:%u mV \n",  PMU.isEnableDC3()  ? "+" : "-", PMU.getDC3Voltage());

    // DCDC4 IMAX=1.5A
    // 500~1200mV,10mV/step,71steps
    // 1220~1840mV,20mV/step,32steps
    PMU.setDC4Voltage(1000);
    Serial.printf("DC4  : %s   Voltage:%u mV \n",  PMU.isEnableDC4()  ? "+" : "-", PMU.getDC4Voltage());

    // DC5 IMAX=2A
    // 1200mV
    // 1400~3700mV,100mV/step,24steps
    PMU.setDC5Voltage(3300);
    Serial.printf("DC5  : %s   Voltage:%u mV \n",  PMU.isEnableDC5()  ? "+" : "-", PMU.getDC5Voltage());

    //ALDO1 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO1Voltage(3300);

    //ALDO2 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO2Voltage(3300);

    //ALDO3 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO3Voltage(3300);

    //ALDO4 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO4Voltage(3300);

    //BLDO1 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setBLDO1Voltage(3300);

    //BLDO2 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setBLDO2Voltage(3300);

    //CPUSLDO IMAX=30mA
    //500~1400mV,50mV/step,19steps
    PMU.setCPUSLDOVoltage(1000);

    //DLDO1 IMAX=300mA
    //500~3400mV, 100mV/step,29steps
    PMU.setDLDO1Voltage(3300);

    //DLDO2 IMAX=300mA
    //500~1400mV, 50mV/step,2steps
    PMU.setDLDO2Voltage(3300);


    // PMU.enableDC1();
    PMU.enableDC2();
    PMU.enableDC3();
    PMU.enableDC4();
    PMU.enableDC5();
    PMU.enableALDO1();
    PMU.enableALDO2();  // LORA

    if(bGPSON)
        PMU.enableALDO3();  // GPS
    else
        PMU.disableALDO3();  // NO GPS

    PMU.enableALDO4();
    PMU.enableBLDO1();
    PMU.enableBLDO2();
    PMU.enableCPUSLDO();
    PMU.enableDLDO1();
    PMU.enableDLDO2();

    PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

    PMU.enableBattDetection();
    PMU.enableBattVoltageMeasure();

    //disable all axp chip interrupt
    PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);

    Serial.println("All AXP2101 started");
    #endif

    delay(100);
}

unsigned int readGPS(void)
{
    //Serial.println("readGPS");
    
    String tmp_data = "";

    bool newData = false;

    if(bGPSDEBUG)
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

        if(bGPSDEBUG)
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

    if(bGPSDEBUG)
        Serial.printf("newData:%i SAT:%d Fix:%d UPD:%d VAL:%d HDOP:%i\n", newData, tinyGPSPlus.satellites.value(), tinyGPSPlus.sentencesWithFix(), tinyGPSPlus.location.isUpdated(), tinyGPSPlus.location.isValid(), tinyGPSPlus.hdop.value());

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
        
        if(bGPSDEBUG)
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
                Serial.printf("GPS: trying 38400 baud <%i>\n", maxStateCount);
                
                GPS.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

                if (myGPS.begin(GPS))
                {
                    Serial.println("GPS: connected at 38400 baud");
                    maxStateCount=1;
                    break;
                }

                delay(100);

                Serial.printf("GPS: trying 9600 baud <%i>\n", maxStateCount);
                GPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
                if (myGPS.begin(GPS)) {
                    Serial.println("GPS: connected at 9600 baud");
                    maxStateCount=1;
                    break;
                }
                else
                {
                    delay(500); //Wait a bit before trying again to limit the Serial output flood
                    maxStateCount++;

                    if(maxStateCount > 5)
                    {
                        maxStateCount = 1;
                        state = 0;

                        bGPSON=false;
                        meshcom_settings.node_sset = meshcom_settings.node_sset & 0x7FBF;
                        save_settings();

                        Serial.println("GPS serial not connected (set GPS to off)");
        
                        break;
                    }
                }
            }
            while(1);

            if(bGPSON)
            {
                myGPS.setUART2Output(COM_TYPE_UBX); //Set the UART port to output UBX only
                myGPS.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART2);
                myGPS.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART2);
                myGPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART2);
                myGPS.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART2);
                myGPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART2);
                myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART2);
                myGPS.saveConfiguration(); //Save the current settings to flash and BBR

                Serial.println("GPS serial connected, saved config");
                
                state++;
            }
            
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
                }
                else
                {
                    Serial.println("*** GPS did not respond at 38400 baud, starting over.");
                    state = 0;
                    bMitHardReset=false;
                    break;
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
                } else {
                    Serial.println("*** GPS did not come back at 9600 baud, starting over.");
                    state = 0;
                    bMitHardReset=false;
                    break;
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
            bMitHardReset=false;
        
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
