#ifndef __UBLOXM8_H__
#define __UBLOXM8_H__           1
    
    
    #define ADDR_UBLOX_M8       0x42
    
    #include "nrf_delay.h"
    #include "nrf_drv_twi.h"
    #include <stdlib.h>
    
    struct gps_pkt_t{
        /*
         * Status Bit definitions:
                    0 = 1 if valid.
                    1 = 1 if North, 0 if South.
                    2 = 1 if East, 0 if West. 
         */
        uint8_t status;
        uint8_t hrs, min, sec;
        uint8_t dd, mm, yy;
        uint8_t latDeg, lonDeg;
        float latMins, lonMins;
    }__attribute__((packed));
    typedef struct gps_pkt_t gps_pkt_t;
    
    void SetGNRMCFilter(void);
    void ReadGPSRaw(uint8_t *buff);
    void ReadGPS(gps_pkt_t *pkt);
    void ParseGPSPkt(gps_pkt_t *pkt, uint8_t *buff); 

#endif