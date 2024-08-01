#ifndef _COMPRESS_FUNCTIONS_H_
#define _COMPRESS_FUNCTIONS_H_

#include <Arduino.h>
#include <debugconf.h>

String compress_encode(String text, String compressChar );
String compress_decode(String text, String compressChar);
int getOccurences(String text, String pattern);

void text_compress(String text);

#endif // _COMPRESS_FUNCTIONS_H_