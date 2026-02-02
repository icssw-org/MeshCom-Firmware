/*********************************************************************************
 *                                  INCLUDES
 * *******************************************************************************/
#include "peripheral.h"
#include <XPowersLib.h>
#include "bq27220.h"
#include "Audio.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"
#include "SPIFFS.h"

/*********************************************************************************
 *                                   DEFINES
 * *******************************************************************************/

#define DISP_REFR_MODE_FULL 0
#define DISP_REFR_MODE_PART 1

//KBC #define TINY_GSM_MODEM_SIM7672
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define MODEM_GPS_ENABLE_GPIO               (-1)

#include <TinyGsmClient.h>

extern TinyGsm modem;
extern TaskHandle_t a7682_handle;
/*********************************************************************************
 *                                   MACROS
 * *******************************************************************************/
extern bool peri_init_st[E_PERI_NUM_MAX];
extern XPowersPPM PPM;
extern BQ27220 bq27220;
extern Audio audio;

/*********************************************************************************
 *                                  TYPEDEFS
 * *******************************************************************************/
union flush_buf_pixel
{
    struct _byte {
        uint8_t b1 : 1;
        uint8_t b2 : 1;
        uint8_t b3 : 1;
        uint8_t b4 : 1;
        uint8_t b5 : 1;
        uint8_t b6 : 1;
        uint8_t b7 : 1;
        uint8_t b8 : 1;
    }bit;
    uint8_t full;
};

/*********************************************************************************
 *                              GLOBAL PROTOTYPES
 * *******************************************************************************/
void flush_timer_cb_one();

void disp_full_refr(void); // Next global refresh

void initTDeck_pro();
void loopTDeck_pro();

// MeshCom calls
void TDeck_pro_lora_disp(String strhead, String strtext);
void TDeck_pro_mheard_disp();

void tdeck_refresh_track_view();
void tdeck_send_track_view();

void tdeck_set_gps(bool bGPS);
unsigned int tdeck_get_gps();