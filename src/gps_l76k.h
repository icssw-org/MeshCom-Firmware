#ifndef _GPS_L76K_H_
#define _GPS_L76K_H_

#ifdef GPS_L76K
    void switchL76KGPS();
    bool setupL76KGPS();
    void stopL76KGPS();
    unsigned int loopL76KGPS();
 
    unsigned int displayInfo();
    bool GPS_Recovery();

    bool beginGPS();

#endif

#endif