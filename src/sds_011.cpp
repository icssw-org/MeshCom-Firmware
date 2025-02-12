#include "configuration.h"
#include "sds_011.h"

#if defined (ENABLE_SDS011)


SDS011 sds011;
HardwareSerial sdsSerial(2);

void onSensorData(float pm25Value, float pm10Value) {
  Serial.printf("[SDS011] pm2.5: %.1f\n", pm25Value);
  Serial.printf("[SDS011] pm10:  %.1f\n", pm10Value);
};

void onSensorResponse(uint8_t command, uint8_t set, uint8_t result){
  Serial.printf("[SDS011] command %d - set %d: %d\n", command, set, result);
};

void onSensorError(int8_t error){
  Serial.printf("[SDS011] error: %d\n", error);
};

void sds_setup() {
  sds011.setup(&sdsSerial,SDS011_RX, SDS011_TX);
  sds011.onData(onSensorData);
  sds011.onResponse(onSensorResponse);
  sds011.onError(onSensorError);
  sds011.setReportMode(false);  // false: active, true: query
  sds011.setWorkingMode(true);  // false: sleep, true, work
  sds011.setWorkingPeriod(1);
}

void sds_process()
{
    //Serial.println(F("[SDS011] loop()"));
    sds011.loop();
}


#endif