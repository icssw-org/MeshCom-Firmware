#include "configuration.h"
#include "loop_functions_extern.h"

#if defined (ENABLE_BMX680)

#include <Adafruit_BME680.h> 
#include <Adafruit_Sensor.h>
#ifdef NRF52_SERIES
#include <WisBlock-API.h>
#else
#include "esp32/esp32_flash.h"
#endif

#include "bme680.h"

Adafruit_BME680 bme;

#define SEALEVELPRESSURE_HPA (1013.25)

#ifndef I2C_ADDRESS_BME
#define I2C_ADDRESS_BME680_1 0x76
#define I2C_ADDRESS_BME680_2 0x77
#endif

const float STANDARD_ALTITUDE = 180.0;
float fBaseAltidude680 = 0;
float fBasePress680 = 0;
float fTemp680 = 0.0;
float fPress680 = 0.0;

//coompensate approx. altitude from pressure
const float COMPENSATE_ALTITUDE = -9.0; // mBar to zero ASL

float getPressASL(int current_alt);

void setupBME680()
{

	if(bWXDEBUG)	
		Serial.printf("bBME680ON:%i\n", bBME680ON);

  // Don't mix BME28x and BME680 they share same addresses
  if(bBMEON || bBMPON)
  {
    Serial.println("BME680 and BMx280 can't be used together!");
    return; 
  }

  if(!bBME680ON)
    return;

  uint8_t foundAddr = 0; // 1 = 0x76, 2 = 0x77, 3 = both

  // check if the sensor is found on the I2C bus
  Wire.beginTransmission(I2C_ADDRESS_BME680_1);
  int error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("BME680 sensor found at 0x76");
    foundAddr = 1;
  } else {
    Serial.println("BME680 sensor not found at 0x76");
  }
  Wire.beginTransmission(I2C_ADDRESS_BME680_2);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("BME680 sensor found at 0x77");
    foundAddr += 2;
  } else {
    Serial.println("BME680 sensor not found at 0x77");
  }

  // if none found or there is another sensor on the other address return for now!!! TODO: fix this
  if (foundAddr == 0 || foundAddr == 3) {
    Serial.println("Could not find BME680 sensor or Address conflict!");
    return;
  } else {
    bme680_found = true;
  }

  // Initialize the BME680
  switch (foundAddr) {
    case 1:
      bme.begin(I2C_ADDRESS_BME680_1);
      break;
    case 2:
      bme.begin(I2C_ADDRESS_BME680_2);
      break;
  }
  
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

}


// get the time when the sensor reading will be completed
int bme680_get_endTime()
{
  if(!bBME680ON)
    return 0;

  return bme.beginReading();
}


// get the sensor reading
void getBME680()
{
  
  if(!bBME680ON)
    return;

  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }

  meshcom_settings.node_temp = bme.temperature - 1.1; // heater biases temp reading - TODO: implement better compensation
  meshcom_settings.node_hum = bme.humidity;
  meshcom_settings.node_press = bme.pressure / 100.0;
  int bme_alt = bme.readAltitude(SEALEVELPRESSURE_HPA + COMPENSATE_ALTITUDE);
  meshcom_settings.node_press_asl = getPressASL680(bme_alt);
  //Serial.printf("asl: %f\n", meshcom_settings.node_press_asl);
  meshcom_settings.node_press_alt = bme_alt;
  meshcom_settings.node_gas_res = bme.gas_resistance / 1000.0;


  if (Serial && bWXDEBUG)
  {
    Serial.print("BME680: ");
    Serial.print(F("Temperature = "));
    Serial.print(meshcom_settings.node_temp);
    Serial.println(F(" *C"));

    Serial.print(F("Pressure = "));
    Serial.print(meshcom_settings.node_press);
    Serial.println(F(" hPa"));

    Serial.print(F("Humidity = "));
    Serial.print(meshcom_settings.node_hum);
    Serial.println(F(" %"));

    Serial.print(F("Gas = "));
    Serial.print(meshcom_settings.node_gas_res);
    Serial.println(F(" KOhms"));

    Serial.print(F("Approx. Altitude = "));
    Serial.print(bme_alt);
    Serial.println(F(" m"));
    
    Serial.println();
  }
}

float getPressASL680(int current_alt)
{
	//willbe set withj --setpress
	//fBaseAltidude = (float)meshcom_settings.node_alt;
	//fBasePress = meshcom_settings.node_press;
	//
	if(fBaseAltidude680 == 0)
		fBaseAltidude680 = (float)current_alt;

	return meshcom_settings.node_press / powf(1 - ((0.0065 * fBaseAltidude680) /
        (meshcom_settings.node_temp + (0.0065 * STANDARD_ALTITUDE) + 273.15)), 5.257); // in hPa
}

#endif
