/*
  RadioLib SX126x Spectrum Scan Example

  This example shows how to perform a spectrum power scan using SX126x.
  The output is in the form of scan lines, each line has 33 power bins.
  First power bin corresponds to -11 dBm, the second to -15 dBm and so on.
  Higher number of samples in a bin corresponds to more power received
  at that level. The example performs frequency sweep over a given range.

  To show the results in a plot, run the Python script
  RadioLib/extras/SX126x_Spectrum_Scan/SpectrumScan.py

  WARNING: This functionality is experimental and requires a binary patch
  to be uploaded to the SX126x device. There may be some undocumented
  side effects!

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/
#include "spectral_scan.h"
#include "lora_setchip.h"
#include "esp32/esp32_flash.h"

#if defined(SX1262X) || defined(SX126X) || defined(SX1268_V3) || defined(SX1262_V3) || defined(SX1262_E290)

#include <RadioLib.h>

using namespace std;


// include the library


// this file contains binary patch for the SX1262
#include <modules/SX126x/patches/SX126x_patch_scan.h>



/**
 * ###########################################################################################################################
 * Initializes the scanning
 * @param freq the initial frequency - this does not need to be the first frequency we start scanning with
 * @return a result code - see RadioLib's TypeDef.h
 */
int sx126x_spectral_init_scan(float freq)
{
  // initialize SX1262 FSK modem at the initial frequency
  int state = radio.beginFSK(freq);
  if (state != RADIOLIB_ERR_NONE)
  {
    return state;
  }

  // upload a patch to the SX1262 to enable spectral scan
  // NOTE: this patch is uploaded into volatile memory,
  //       and must be re-uploaded on every power up
  state = radio.uploadPatch(sx126x_patch_scan, sizeof(sx126x_patch_scan));
  if (state != RADIOLIB_ERR_NONE)
  {
    return state;
  }

  // configure scan bandwidth to 234.4 kHz
  // and disable the data shaping
  state = radio.setRxBandwidth(234.3);  //any other frequwency does not work - maybe those values are somehow fixed in the binary blob of the scan patch
  //ToDo: check if that patch can be recompiled allowing a narrower bandwidth, see: https://github.com/Lora-net/sx1302_hal/blob/master/util_spectral_scan/src/spectral_scan.c
  state |= radio.setDataShaping(RADIOLIB_SHAPING_NONE);
  if (state != RADIOLIB_ERR_NONE)
  {
    return state;
  }

  // if we reach this point, everythung should be okay
  return RADIOLIB_ERR_NONE;
}




/**
 * ###########################################################################################################################
 * Once the scan has finished, we wish ro return to a normal working state.
 * We need to set back our default LoRa Modem configuration
 * 
 */
void sx126x_spectral_finish_scan()
{
  radio.begin(); // Set modem back to LoRa mode
  lora_setchip_meshcom();
}



/**
 * ###########################################################################################################################
 * Starts the scan of a small frequency span beginning with the frequency defined as parameter "freq" and the scan width of 200kHz
 * @param freq the starting frequency of the 200kHz span to scan
 * @param samples the amount of samples taken (fewer samples = better temporal resolution)
 * @return an array containing the results
 */
uint16_t *sx126x_spectral_scan_freq(float freq, uint16_t samples)
{
  uint16_t *results = new uint16_t[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE]; // initialize array with zeros, we will return the array containing zeros if anything goes wrong
  if (radio.setFrequency(freq) == RADIOLIB_ERR_NONE)
  {
    // start spectral scan
    if (radio.spectralScanStart(samples) == RADIOLIB_ERR_NONE)
    { // if scan started without an error, wait for spectral scan to finish
      while (radio.spectralScanGetStatus() != RADIOLIB_ERR_NONE){
        delay(10);
      }

      // read the results
      if (radio.spectralScanGetResult(results) == RADIOLIB_ERR_NONE){
        return results;
      }
    }
  }

  // if we reach this point, something went wrong
  return results; // return an array of zeros
}



/**
 * ###########################################################################################################################
 * Performs a spectrum scan over the defined fequency range (freqStart to freqEnd)
 * Formats the output to be compatible wuth RadioLib's Example.
 *
 * This function might be removed later.
 */
void sx126x_spectral_scan()
{
  
  float freqStart = 430.0;          // scan start frequency in MHz 
  const float freqEnd = 440.2;      // scan end frequency in MHz 
  const float samples = 2048;       // amount of samples to be taken

  sx126x_spectral_init_scan(freqStart);

  while (freqStart <= freqEnd)
  {
    Serial.print("FREQ ");
    Serial.println(freqStart, 2);
    Serial.print(F("[SX1262] Starting spectral scan ... \n"));
    // get the results
    uint16_t *res;
    res = sx126x_spectral_scan_freq(freqStart, samples);

    // we have some results, print it
    Serial.print("SCAN ");
    for (uint8_t i = 0; i < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; i++)
    {
      Serial.print(res[i]);
      Serial.print(',');
    }
    Serial.println(" END");

    // wait a little bit before the next scan
    delay(100);

    // set the next frequency
    // the frequency step should be slightly smaller
    // or the same as the Rx bandwidth set in setup
    freqStart += 0.2;

    // yield();    //pet the watchdog so it stays calm
  }
  Serial.println("SCAN END");

  sx126x_spectral_finish_scan();
}

uint16_t *scan_freq(float freq) {
  uint16_t *results = new uint16_t[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE]{0};
  return results; // return an array with zeros
};

int init_scan(float freq)
{
return RADIOLIB_ERR_WRONG_MODEM;
}

#else
// Serial.print(F("Spectral scan not implemented for this plattform ... "));
// DUMMY FOR NON-SX1262
void sx126x_spectral_scan()
{
  Serial.print(F("Spectral scan not implemented for this plattform ... "));
  return;
}


#endif
