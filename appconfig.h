#ifndef __APPCONFIG_H__
#define __APPCONFIG_H__           1
    
    #include "ubloxm8.h"

    #define  APP_QOS                    15
    #define  SL_BUFFER_LEN              250

    #define  TX_10SEC_TEST              1

    //#define  SL_EVBOARD_RST             1
    /* To Select External UFL Antenna */
    #define  SL_ANT_UFL                 1
    /* To Select on board Trace Antenna */
    #define  SL_ANT_TRACE               2
    /* Select antenna type here */
    #define  SL_ANT_SELECTED            SL_ANT_TRACE
    
    /** Symphony link comm states
    */
    enum sl_states_t
    {
        SL_RESET,
        SL_UNINITIALIZED,
        SL_SCANNING,
        SL_READY,
        SL_TRANSMITTING,
        SL_RECEIVING,
        SL_WAITING,
        SL_ERROR
    };

    enum app_states_t
    {
        APP_READY,
        APP_TRANSMITTING,
        APP_RECEIVING
    };
    
    #define APP_PKT_HDR   0xC0
    #define APP_PKT_FTR   0xDE
    struct app_pkt_t{
        uint8_t header;
        uint8_t batV;
        gps_pkt_t gpsPkt;
        uint8_t footer;
    }__attribute__((packed));
    typedef struct app_pkt_t app_pkt_t;

#endif