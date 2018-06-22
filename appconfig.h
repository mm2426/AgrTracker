#ifndef __APPCONFIG_H__
#define __APPCONFIG_H__           1
    
    #include "ubloxm8.h"

    #define  APP_QOS                    15
    #define  SL_BUFFER_LEN              250

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

    /* Switch states */
    enum swStates
    {
       SW_RELEASED, SW_DETECTING, SW_DEBOUNCING, SW_PRESSED
    };
    
    /* Rec upload states */
    enum uploadStates
    {
       UPLOAD_INIT, UPLOAD_PKTIZE, UPLOAD_WAITRDY, UPLOAD_TXRESP, UPLOAD_TIMEOUT
    };

    
    #define APP_PKT_HDR             0xC0
    #define APP_PKT_FTR             0xDE
    struct app_pkt_t{
        uint8_t header;
        uint8_t batV;
        gps_pkt_t gpsPkt;
        uint8_t footer;
    }__attribute__((packed));
    typedef struct app_pkt_t app_pkt_t;
    
    struct memhdr_t{
        /* "AGRILINX" */
        char id[8];
        uint32_t  memBaseAddr, rPtr, wPtr, nRecs;
        uint8_t startHrs, startMin,startSec;
        uint8_t startDD, startMM,startYY;
        uint8_t stopHrs, stopMin, stopSec;
        uint8_t stopDD, stopMM, stopYY;
        uint8_t updateFreq, retryCount, accelRange;
        uint16_t accelTh;
    }__attribute__((packed));
    typedef struct memhdr_t memhdr_t;

    #define DEV_MODE_OFFLINE        0
    #define DEV_MODE_ONLINE         1
    

#endif