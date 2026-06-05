#pragma once

#include <Arduino.h>

int printlndeb(const char *buff);
int printdeb(const char *buff);

int printlndeb(int iVar);

int printdeb(int iVar);
int printdeb(unsigned int iVar);
int printdeb(short iVar);
int printdeb(float fVar);
int printdeb(char c);
int printdeb(unsigned char c);

int printlndeb(String str);
int printdeb(String str);

int printfdeb(const char *format, ...);
