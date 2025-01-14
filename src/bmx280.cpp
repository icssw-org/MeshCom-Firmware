#include "configuration.h"
#include "loop_functions_extern.h"

#if defined (ENABLE_BMX280)

#include <Arduino.h>

#include <Wire.h>

#include <BMx280MI.h>

#define I2C_ADDRESS_BMP 0x77
#define I2C_ADDRESS_BME 0x76

uint8_t bmx_i2c_address = 0;

#include "bmx280.h"

unsigned long BMXTimeWait = 0;

float fTemp = 0.0;
float fPress = 0.0;
float fHum = 0.0;

float fBaseAltidude = 0;
float fBasePress = 0;

int maxValideCount=0;

const float STANDARD_SEA_LEVEL_PRESSURE = 1013.25;
const float STANDARD_ALTITUDE = 180.0; // in meters, see note

//class derived from BMx280MI that implements communication via an interface other than native I2C or SPI. 
class BMx280Wire : public BMx280MI
{
	public:	
		//constructor of the derived class. 
		//@param address i2c address of the sensor.
		BMx280Wire(uint8_t bmx_i2c_address):
		address_(bmx_i2c_address)	//initialize the BMx280Wire classes private member address_ to the i2c address provided
		{
			//nothing else to do here...
		}
	
	private:
		//this function must be implemented by derived classes. it is used to initialize the interface or check the sensor for example. 
		//@return true on success, false otherwise. 
		bool beginInterface()
		{
			return true;
		}

		//this function must be implemented by derived classes. this function is responsible for reading data from the sensor. 
		//@param reg register to read. 
		//@return read data (1 byte).
		uint8_t readRegister(uint8_t reg)
		{
		#if defined(ARDUINO_SAM_DUE)
			//workaround for Arduino Due. The Due seems not to send a repeated start with the code above, so this 
			//undocumented feature of Wire::requestFrom() is used. can be used on other Arduinos too (tested on Mega2560)
			//see this thread for more info: https://forum.arduino.cc/index.php?topic=385377.0
			Wire.requestFrom(address_, 1, reg, 1, true);
		#else
			Wire.beginTransmission(address_);
			Wire.write(reg);
			Wire.endTransmission(false);
			Wire.requestFrom(address_, static_cast<uint8_t>(1));
		#endif
			
			return Wire.read();
		}
		
		//this function can be implemented by derived classes. implementing this function is optional, but readings may be incorrect if 
		//it is not (see BMP280 / BME280 datasheet). 
		//@param reg register to read. 
		//@param length number of registers to read (max: 4)
		//@return read data. LSB = last register read. 
		uint32_t readRegisterBurst(uint8_t reg, uint8_t length)
		{
			if (length > 4)
				return 0L;

			uint32_t data = 0L;

#if defined(ARDUINO_SAM_DUE)
			//workaround for Arduino Due. The Due seems not to send a repeated start with the code below, so this 
			//undocumented feature of Wire::requestFrom() is used. can be used on other Arduinos too (tested on Mega2560)
			//see this thread for more info: https://forum.arduino.cc/index.php?topic=385377.0
			Wire.requestFrom(address_, length, data, length, true);
#else
			Wire.beginTransmission(address_);
			Wire.write(reg);
			Wire.endTransmission(false);
			Wire.requestFrom(address_, static_cast<uint8_t>(length));

			for (uint8_t i = 0; i < length; i++)
			{
				data <<= 8;
				data |= Wire.read();
			}
#endif

			return data;
		}

		//this function must be implemented by derived classes. this function is responsible for sending data to the sensor. 
		//@param reg register to write to.
		//@param data data to write to register.
		void writeRegister(uint8_t reg, uint8_t data)
		{
			Wire.beginTransmission(address_);
			Wire.write(reg);
			Wire.write(data);
			Wire.endTransmission();
		}
	
  public:
		uint8_t address_;		//i2c address of sensor
};

//create an BMx280 object using the I2C interface, I2C address 0x01 and IRQ pin number 2
BMx280Wire bmx280(0x00);

void setupBMX280(bool bNewStart)
{
	if(bWXDEBUG)	
		Serial.printf("bBMPON:%i bBMEON:%i\n", bBMPON, bBMEON);

  	if(bBMPON)
    	bmx_i2c_address = I2C_ADDRESS_BMP;
  	else
    	if(bBMEON)
      		bmx_i2c_address = I2C_ADDRESS_BME;
    	else
      		return;

	if(!bNewStart)
		Wire.endTransmission(true);


	bmx280.address_ = bmx_i2c_address;	//initialize the BMx280Wire classes private member address_ to the i2c address provided

    Serial.printf("[INIT]...setupBMX280 - I2C:%02X\n", bmx_i2c_address);

	//begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
	//and reads compensation parameters.
	if (!bmx280.begin())
	{
		Serial.println("[INIT]...begin() failed. check your BMx280 Interface and I2C Address.");
	}

	//reset sensor to default parameters.
	bmx280.resetToDefaults();

	//by default sensing is disabled and must be enabled by setting a non-zero
	//oversampling setting.
	//set an oversampling setting for pressure and temperature measurements. 
	bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
	bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

	//if sensor is a BME280, set an oversampling setting for humidity measurements.
	if (bmx280.isBME280())
		bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);

  if(bBMEON)
    Serial.printf("[INIT]...BME280 startet\n");

  if(bBMPON)
    Serial.printf("[INIT]...BMP280 startet\n");
}

bool loopBMX280()
{
	if(!bBMEON && !bBMPON)
		return false;

	Wire.endTransmission(true);

	//start a measurement
	if (!bmx280.measure())
	{
    	if(bWXDEBUG)
		  Serial.println("could not start measurement, is a measurement already running?");

		return false;
	}

	//wait for the measurement to finish
	int maxLoop=20;
	do
	{
		delay(100);
		maxLoop--;
		
	} while (!bmx280.hasValue() && maxLoop > 0);

	if(maxLoop <= 0)
	{
		Serial.println("BMX280 vales not valid");
		return false;
	}

	//important: measurement data is read from the sensor in function hasValue() only. 
	//make sure to call get*() functions only after hasValue() has returned true. 
  	if(bmx280.getTemperature() == (float)NAN)
		fTemp = 0.0;
	else
  		fTemp = bmx280.getTemperature();

  	if(bmx280.getPressure() == (float)NAN)
		fPress = 0.0;
	else
		fPress = bmx280.getPressure() / 100.0;

	if(bWXDEBUG)
	{
		Serial.print("Pressure: "); Serial.println(fPress);
		Serial.print("Pressure (64 bit): "); Serial.println(bmx280.getPressure64());
		Serial.print("Temperature: "); Serial.println(fTemp);
	}

	if (bmx280.isBME280())
	{
	  	if(bmx280.getHumidity() == (float)NAN)
			fHum = 0.0;
		else
			fHum = bmx280.getHumidity();

		if(bWXDEBUG)
		{
			Serial.print("Humidity: "); 
			Serial.println(fHum);
		}
	}
	else
		fHum = 0.0;

	return true;
}

float getTemp()
{
	return fTemp;
}

float getPress()
{
	return fPress;
}

float getHum()
{
	return fHum;
}

int getPressALT()
{
	if(fPress == 0.0 || fBasePress == 0.0)
		return 0;
		
	double x=(double)fPress/(double)fBasePress;
	x=(double)-7990*log(x);
	x = x + fBaseAltidude;

	return (int)lround(x);
}

float getPressASL(int current_alt)
{
	//willbe set withj --setpress
	//fBaseAltidude = (float)meshcom_settings.node_alt;
	//fBasePress = meshcom_settings.node_press;
	
	// 
	if(fBaseAltidude == 0)
		fBaseAltidude = (float)current_alt;

	return fPress / powf(1 - ((0.0065 * fBaseAltidude) /
        (fTemp + (0.0065 * STANDARD_ALTITUDE) + 273.15)), 5.257); // in hPa
}

#endif
