#ifndef _VIA_FUNCTIONS_H_
#define _VIA_FUNCTIONS_H_

#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>

bool checkMesh(struct aprsMessage &aprsmsg);
void checkVia(struct aprsMessage &aprsmsg);


#endif
