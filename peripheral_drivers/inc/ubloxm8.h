#ifndef __UBLOXM8_H__
#define __UBLOXM8_H__           1
    
    
    #define ADDR_UBLOX_M8       0x42
    
    #include "nrf_delay.h"
    #include "nrf_drv_twi.h"
    
    typedef struct gps_pkt_t{
        uint8_t valid;
    }gps_pkt_t;
    
    void SetGNRMCFilter(void);
    void ReadGPSRaw(uint8_t *buff);
    void ReadGPS(gps_pkt_t *pkt);
    void ParseGPSPkt(gps_pkt_t *pkt, uint8_t *buff); 

#endif