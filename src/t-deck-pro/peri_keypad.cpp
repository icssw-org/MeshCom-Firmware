
#include <Adafruit_TCA8418.h>
#include "utilities.h"
#include "peripheral.h"

#include "loop_functions.h"
#include "loop_functions_extern.h"

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 10
#define KEYPAD_PRESS_VAL_MIN   129
#define KEYPAD_PRESS_VAL_MAX   163
#define KEYPAD_RELEASE_VAL_MIN 1
#define KEYPAD_RELEASE_VAL_MAX 35

const char keymap0[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p'},
    {'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', 0x7f},
    {0xfe, 'z', 'x', 'c', 'v', 'b', 'n', 'm', '$', 0x0d},
    {' ', ' ', ' ', ' ', ' ', 0xfc, 0xfd, ' ', 0xff, 0xfc},
};

    // CAP
const char keymap1[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'L', 'O', 'P'},
    {'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 0x7f},
    {0xfe, 'Z', 'X', 'C', 'V', 'B', 'N', 'M', 0x00, 0x0d},
    {' ', ' ', ' ', ' ', ' ', 0xfc, 0xfd, ' ', 0xff, 0xfc},
};
    // SYM
const char keymap2[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'#', '1', '2', '3', '(', ')', '_', '-', '+', '@'},
    {'*', '4', '5', '6', '/', ':', ';', '\'', '"', 0x7f},
    {0xfe, '7', '8', '9', '?', '!', ',', '.', 0x00, 0x0d},
    {' ', ' ', ' ', ' ', ' ', 0xfc, '0', ' ', 0xff, 0xfc},
};

    // ALT
const char keymap3[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'L', 'O', 'P'},
    {'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 0x7f},
    {0xfe, 'Z', 'X', 'C', 'V', 'B', 'N', 'M', 0x00, 0x0d},
    {' ', ' ', ' ', ' ', ' ', 0xfc, 0xfd, ' ', 0xff, 0xfc},
};

Adafruit_TCA8418 keypad; 
keypad_cb keypad_listener = NULL;
char keypad_curr_val = ' ';
int keypad_state = KEYPAD_RELEASE;
bool keypad_update = false;

// 0...small characters
// 1...large characters
// 2...sym characters
// 3...alt characters
int ikeypad_layer = 0;
int ikeypad_layer_save = 0;


bool keypad_init(int address)
{
    if(!i2cIsInit(0)){
        Wire.begin(BOARD_KEYBOARD_SDA, BOARD_KEYBOARD_SCL);
        Wire.beginTransmission(address);
        Wire.endTransmission(true);
    }

    if (!keypad.begin(address, &Wire)) {
        // Serial.println("keypad not found, check wiring & pullups!");
        log_e("keypad not found, check wiring & pullups!");
        return false;
    }

    // configure the size of the keypad matrix.
    // all other pins will be inputs
    keypad.matrix(KEYPAD_ROWS, KEYPAD_COLS);

    // flush the internal buffer
    keypad.flush();

    return true;
}

int keypad_get_val(char *c)
{
    if(keypad_state == 0 && keypad_curr_val > 0x00)
    {
        if(c)
        {
            *c = keypad_curr_val;
        }

        return keypad_update;
    }
    else
        return -1;
} 

void keypad_set_flag(void)
{
    keypad_update = false;
}

void keypad_set_layer(int ilayer)
{
    ikeypad_layer = ilayer;
    ikeypad_layer_save = ilayer;
}

void keypad_loop(void)
{
    char c = -1;
    int state = -1;
    int row, col;
    int k = keypad.getEvent();
    int v = keypad.available();

    if(k >=KEYPAD_RELEASE_VAL_MIN && k <= KEYPAD_RELEASE_VAL_MAX)
    { // release event
        k = k - KEYPAD_RELEASE_VAL_MIN;
        state = KEYPAD_RELEASE;
    }   

    if(k >=KEYPAD_PRESS_VAL_MIN && k <= KEYPAD_PRESS_VAL_MAX)
    { // press event
        k = k - KEYPAD_PRESS_VAL_MIN;
        state = KEYPAD_PRESS;
    }

    if(state != -1)
    {
        row = k / KEYPAD_COLS;
        col = (KEYPAD_COLS-1) - k % KEYPAD_COLS;

        switch (ikeypad_layer)
        {
            case 1:
                c = keymap1[row][col];
                break;

            case 2:
                c = keymap2[row][col];
                break;
            
            case 3:
                c = keymap3[row][col];
                break;
            
            default:
                c = keymap0[row][col];
                break;
        }

        if(bTDECKDEBUG)
            Serial.printf("state=%d k=%d, v=%d, layer:%i/%i press:%d, %d, %c\n", state, k, v, ikeypad_layer, ikeypad_layer_save, row, col, c);

        keypad_curr_val = 0x00;

        if(state == 1)
        {
            // CAP char
            if(c == 0xFC)
            {
                if(ikeypad_layer == 1)
                {
                    ikeypad_layer = 0;
                    c=0xF0;
                }
                else
                {
                    ikeypad_layer = 1;
                    c=0xF1;
                }

                ikeypad_layer_save = ikeypad_layer;
            }
            else
            // SYM
            if(c == 0xFF)
            {
                if(ikeypad_layer == 2)
                {
                    ikeypad_layer = 0;
                    c=0xF0;
                }
                else
                {
                    ikeypad_layer = 2;
                    c=0xF2;
                }

                ikeypad_layer_save = ikeypad_layer;
            }
            else
            // ALT
            if(c == 0xFE)
            {
                ikeypad_layer = 3;
                c=0xF3;
            }

            if(keypad_listener && !meshcom_settings.node_keyboardlock && (c==0XF0 || c==0XF1 || c==0XF2))
                keypad_listener(state, c);
        }
        else
        {
            // state = 0
            keypad_curr_val = c;
            keypad_state = state;
            keypad_update = true;
        

            bool bNORM = true;

            // spezial Keys
            if(ikeypad_layer == 3)
            {
                if(c == 'T' || c == 't') // ALT + T
                {
                    c = 0xF9;

                    bNORM = true;
                }
                else
                if(c == 'R' || c == 'r') // ALT + R
                {
                    c = 0xFA;

                    bNORM = true;
                }
                else
                if(c == 'G' || c == 'g') // ALT + G
                {
                    bGPSON = ! bGPSON;

                    bNORM = true;
                }
                else
                if(c == 'D' || c == 'd') // ALT + D
                {
                    bTDECKDEBUG = ! bTDECKDEBUG;

                    bNORM = false;
                }
                else
                if(c == 'L' || c == 'l') // ALT + L
                {
                    meshcom_settings.node_backlightlock = !meshcom_settings.node_backlightlock;
                    
                    if(meshcom_settings.node_backlightlock)
                        digitalWrite(BOARD_KEYBOARD_LED, HIGH);
                    else
                        digitalWrite(BOARD_KEYBOARD_LED, LOW);

                    //if(!meshcom_settings.node_backlightlock)
                    //    tft_off();

                    save_settings();

                    bNORM = false;
                }
                else
                if(c == 'K' || c == 'k') // ALT + K
                {
                    meshcom_settings.node_keyboardlock = !meshcom_settings.node_keyboardlock;

                    save_settings();

                    bNORM = false;
                }
                else
                if(c == 'O' || c == 'o') // ALT + O
                {
                    if (meshcom_settings.node_map > 0)
                        meshcom_settings.node_map--;
                    
                    //set_map(meshcom_settings.node_map);

                    save_settings();

                    bNORM = false;
                }
                else
                if(c == 'I' || c == 'i') // ALT + I
                {
                    if (meshcom_settings.node_map < MAX_MAP-1)
                        meshcom_settings.node_map++;

                    //set_map(meshcom_settings.node_map);

                    save_settings();

                    bNORM = false;
                }
                else
                if((c == 'B'  || c == 'b') && (!meshcom_settings.node_keyboardlock)) // ALT + B
                {
                    //cycleBrightness();    // TODO
                    bNORM = false;
                }
                else
                if((c == 'M'  || c == 'm') && (!meshcom_settings.node_keyboardlock)) // ALT + M
                {
                    meshcom_settings.node_mute = !meshcom_settings.node_mute;
                    save_settings();

                    bNORM = false;

                }
                
                ikeypad_layer = ikeypad_layer_save;
            }


            if(keypad_listener && bNORM && !meshcom_settings.node_keyboardlock)
                keypad_listener(state, c);
        }

    }
}

void keypad_register_cb(keypad_cb cb)
{
    keypad_listener = cb;
}
