#include "loop_functions.h"
#include "loop_functions_extern.h"
#include "serial_command.h"
#include "printfdeb_functions.h"
#include "command_functions.h"

// C3 carve-out of checkSerialCommand() out of nrf52_main.cpp; see
// serial_command.h for why the two copies are not merged here. Moved
// unchanged apart from the four parser globals below, which were file-scope
// there and are file-static here: nothing outside this function ever used
// them, and no header declared them extern.

// CheckSerialConsole
static String strTextWork;
static char strText[600] = {0};
static int iTxtPos = 0;
static int iTxtLen = 0;

#ifdef UNIT_TEST
// Test-only accessors: strText/iTxtPos are otherwise file-static, with no
// header declaring them extern (see the comment above). UNIT_TEST is defined
// only by the native test environments, never by a firmware build, so these
// never ship. Added for test_serial_command_twin.cpp's 600-byte overflow
// case, which needs to see whether the terminator actually survived rather
// than infer it from behavior alone.
const char *test_get_strText(void) { return strText; }
int test_get_iTxtPos(void) { return iTxtPos; }
#endif

void checkSerialCommand(void)
{
    // Serial available
    if(Serial)
    {
        // Check USB Serial input (Serial == MSerial after telnet_functions.h include)
        if(Serial.available() > 0)
        {
            char rd = (char)Serial.read();
            // Drop NUL bytes: UART RX noise (e.g. unpowered USB-UART bridge on battery
            // supply) delivers 0x00 which strlen() cannot see and wedges the parser
            // (DRY-22 — ported from the ESP32 copy of this function).
            if(rd != 0x00)
            {
                printdeb(rd);   // echo to USB + net console via MSerial
                // Check capacity before writing, not after: strText[599] must
                // stay the terminator once iTxtPos saturates there, or a
                // 600-byte line with no NUL/CR/LF overwrites it and strlen()
                // runs into adjacent BSS.
                if(iTxtPos < (int)sizeof(strText) - 1)
                {
                    strText[iTxtPos] = rd;
                    iTxtPos++;
                }
            }
        }
    }

    iTxtLen = strlen(strText);

    // Self-healing: normally every stored byte is non-NUL, so strlen == iTxtPos.
    // A stray NUL in the buffer breaks that invariant and would block command
    // processing forever (early return below never reaches the memset). Discard.
    // (DRY-22 — ported from the ESP32 copy of this function.)
    if(iTxtLen != iTxtPos)
    {
        memset(strText, 0x00, sizeof(strText));
        iTxtPos = 0;
        return;
    }

    if(iTxtLen == 0)
        return;

    if(strText[0] == ':' || strText[0] == '-' || strText[0] == '{')
    {
        if(strText[iTxtLen-1] == '\n' || strText[iTxtLen-1] == '\r')
        {
            strTextWork = strText;
            strTextWork.trim();
            snprintf(strText, sizeof(strText), "%s", strTextWork.c_str());

            strncpy(msg_text, strText, sizeof(msg_text) - 1);
            msg_text[sizeof(msg_text) - 1] = '\0';

            int inext=0;
            // N-22: 600 B vom knappen 4-KB-Loop-Task-Stack in BSS verlagert —
            // checkSerialCommand() laeuft nur im Loop-Task, und der Pfad
            // ueber sendMessage() -> sendExtern() lief mit Watermark 0
            // (Details: STATUS-Box N-22 im Defektkatalog).
            static char msg_buffer[600];
            iTxtLen = strlen(strText);
            for(int itx=0; itx<iTxtLen; itx++)
            {
                if(msg_text[itx] == 0x08 || msg_text[itx] == 0x7F)
                {
                    inext--;
                    if(inext < 0)
                        inext=0;
                        
                    msg_buffer[inext+1]=0x00;
                }
                else
                {
                    msg_buffer[inext]=msg_text[itx];
                    msg_buffer[inext+1]=0x00;
                    inext++;

                    // buffer size reached
                    if(inext > (int)sizeof(msg_buffer)-2)
                        break;
                }
            }

            if(strText[0] == ':' && strText[1] == ':')
            {
                // BP-01: origin serial -- the notice comes back on the console.
                setMsgOrigin(ORIGIN_SERIAL);
                (void)sendMessage(msg_buffer, inext);
                setMsgOrigin(ORIGIN_NONE);
            }
            else
                if(strText[0] == '-' && strText[1] == '-')
                    commandAction(msg_buffer, isPhoneReady, false);
                else
                    printfdeb("\n...wrong command %s\n", strText);

            memset(strText, 0x00, sizeof(strText));
            iTxtPos = 0;
        }
    }
    else
    {
        if(bDEBUG)
        {
            if(strText[0] != '\n' && strText[0] != '\r')
            {
                printfdeb("MSG:%02X..not sent\n", (unsigned char)strText[0]);
            }
        }

        memset(strText, 0x00, sizeof(strText));
        iTxtPos = 0;
    }
}
