#include <Adafruit_BME680.h> 
#include "esp32/esp32_flash.h"

Adafruit_BME680 bme;

#define SEALEVELPRESSURE_HPA (1013.2)

#ifndef I2C_ADDRESS_BME
#define I2C_ADDRESS_BME680_1 0x76
#define I2C_ADDRESS_BME680_2 0x77
#endif

extern bool bme680_found;

const float MIN_TEMP = 0.0;
const float MAX_TEMP = 100.0;
const float MIN_HUMIDITY = 0.0;
const float MAX_HUMIDITY = 100.0;
const float MIN_PRESSURE = 800.0;
const float MAX_PRESSURE = 1200.0;
const float MIN_GAS = 10000.0;
const float MAX_GAS = 50000.0;

// prototypes
float calculate_IAQ(float temp, float humidity, float pressure, float gas_resistance);



void bme680_init()
{
  // TODO: avoid conflicts with BME/BMP280

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



unsigned long bme680_get_endTime()
{
  return bme.beginReading();
}



void bme680_get()
{
  
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  meshcom_settings.node_temp = bme.temperature;
  meshcom_settings.node_hum = bme.humidity;
  meshcom_settings.node_press = bme.pressure / 100.0;
  meshcom_settings.node_press_alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  meshcom_settings.node_gas_res = bme.gas_resistance / 1000.0;

  if (Serial)
  {
    Serial.print(F("Reading completed at "));
    Serial.println(millis());

    Serial.print(F("Temperature = "));
    Serial.print(bme.temperature);
    Serial.println(F(" *C"));

    Serial.print(F("Pressure = "));
    Serial.print(bme.pressure / 100.0);
    Serial.println(F(" hPa"));

    Serial.print(F("Humidity = "));
    Serial.print(bme.humidity);
    Serial.println(F(" %"));

    Serial.print(F("Gas = "));
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(F(" KOhms"));

    Serial.print(F("Approx. Altitude = "));
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(F(" m"));

    calculate_IAQ(bme.temperature, bme.humidity, bme.pressure, bme.gas_resistance);
    
    Serial.println();
  }
}



float calculate_IAQ(float temp, float humidity, float pressure, float gas_resistance) {
    // Normalisiere die Werte
    float normalized_temp = (temp - MIN_TEMP) / (MAX_TEMP - MIN_TEMP);
    float normalized_humidity = (humidity - MIN_HUMIDITY) / (MAX_HUMIDITY - MIN_HUMIDITY);
    float normalized_pressure = (pressure - MIN_PRESSURE) / (MAX_PRESSURE - MIN_PRESSURE);
    float normalized_gas_resistance = (gas_resistance - MIN_GAS) / (MAX_GAS - MIN_GAS);
    
    // Gewichtungen der Parameter anpassen (empirisch oder aufgrund von Forschung)
    float weight_temp = 0.3;
    float weight_humidity = 0.2;
    float weight_pressure = 0.3;
    float weight_gas_resistance = 0.2;
    
    // Berechne den IAQ basierend auf gewichteten und normalisierten Werten
    float iaq = (normalized_temp * weight_temp) + (normalized_humidity * weight_humidity) + (normalized_pressure * weight_pressure) + (normalized_gas_resistance * weight_gas_resistance);
    
    Serial.printf("IAQ: %f\n", iaq);
    return iaq;
}
