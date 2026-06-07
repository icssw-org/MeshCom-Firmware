// Functions: printfdeb() replaces the printf()
// This function, allowing for different formatting options.
//
// printf() is converted to printfdeb() to maintain functionality
// and passing data to the NETCONSOLE.
//
// Author: Kurt Baumann (OE1KBC), 06/2026
//
// Syntax:
// int printfdeb(const char *format [,argument] ...)    --> Serial.printf(format, argument)
// int printdeb(const char *text)                       --> Serial.print(text)
// int printlndeb(const char *text)                     --> Serial.println(text)
//
// Commands:
// --debug csv/man
// --debug de/en
//
// damit ein printfdeb() auch die Commands --debug de/en umsetzt
// muss vor jedem printfdeb() die Variable
// bDEBUGLNG = true
// gesetzt werden

#include <Arduino.h>

#include "printfdeb_functions.h"

#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <stdio.h>
#include <stdarg.h>

#ifdef ESP32
#include "net_console.h"
#endif

int printfdeb(const char *uformat, ...)
{
    char nformat[300];
    memset(nformat,0x00, sizeof(nformat));

    int inn=0;

    for(int in=0; in<(int)strlen(uformat); in++)
    {
        if(!bDEBUGCSV)
        {
            if(uformat[in] == ';')
            {
                if(uformat[in-1] == ' ' || uformat[in+1] == ' ')
                {
                    continue;
                }
                else
                {
                    if(inn < (int)sizeof(nformat)-2)
                    {
                        nformat[inn] = ' ';
                        inn++;
                        continue;
                    }
                }
            }
        }
        

        if(inn < (int)sizeof(nformat)-2)
        {
            nformat[inn] = uformat[in];
            inn++;
        }
    }

    char loc_buf[64];
    char * temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, uformat);
    va_copy(copy, arg);
    int len = vsnprintf(temp, sizeof(loc_buf), nformat, copy);
    va_end(copy);
    if(len < 0) {
        va_end(arg);
        return 0;
    }
    if(len >= (int)sizeof(loc_buf)){  // comparation of same sign type for the compiler
        temp = (char*) malloc(len+1);
        if(temp == NULL) {
            va_end(arg);
            return 0;
        }
        len = vsnprintf(temp, len+1, nformat, arg);
    }
    va_end(arg);

    // durchlaufe Array, bis Ende-Zeichen '\0' und '.' => ',' für deutsche CSV-Kompatibilität
    if(!bDEBUGEN && bDEBUGCSV && bDEBUGLNG)
        for (int i = 0; temp[i] != '\0'; i++) { if (temp[i] == '.') { temp[i] = ','; } }

    bDEBUGLNG = false; // wieder deaktivieren
    
    Serial.printf(temp);

    if(temp != loc_buf){
        free(temp);
    }
    return len;
}

int printlndeb(const char *buff)
{
    int len = Serial.println(buff);
    return len;
}

int printdeb(const char *buff)
{
    int len = Serial.print(buff);
    return len;
}

int printlndeb(int iVar)
{
    int len = Serial.println(iVar);
    return len;
}

int printdeb(int iVar)
{
    int len = Serial.print(iVar);
    return len;
}

int printfdeb(unsigned int iVar)
{
    int len = Serial.print(iVar);
    return len;
}

int printfdeb(short iVar)
{
    int len = Serial.print(iVar);
    return len;
}

int printdeb(float fVar)
{
    int len = Serial.print(fVar);
    return len;
}

int printdeb(char c)
{
    int len = Serial.print(c);
    return len;
}

int printdeb(unsigned char c)
{
    int len = Serial.print(c);
    return len;
}

int printlndeb(String str)
{
    int len = Serial.println(str);
    return len;
}

int printdeb(String str)
{
    int len = Serial.print(str);
    return len;
}
