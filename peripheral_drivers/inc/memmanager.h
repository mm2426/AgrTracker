#ifndef __MEMMANAGER_H__
#define __MEMMANAGER__                 1
    
    
    /* Defines size of 1 record */
    #define  RECORD_SIZE          sizeof(app_pkt_t)
    #define  RECS_PER_PAGE        (uint8_t)(256/RECORD_SIZE)    
    
    #include "nrf_delay.h"
    #include "nrf_drv_spi.h"
    #include "nrf_gpio.h"
    #include "boards.h"
    #include "w25q32.h"
    #include "appconfig.h"

    /** 
    * @brief Read MBR from flash. Acquire pointers to the first location in memory.
    * If writing first time, write default configuration to MBR.
    * @param[in] nRecs, Returns number of records present in the memory.
    */
    void MemManInit(void);
    uint32_t MemManGetRecCount(void);
    void MemManUpdateMBR(void);
    void MemManWriteRecs(void *pkt, uint8_t noOfRecs);
    void MemManReadPage(void *pkt, uint8_t *len, uint8_t delRec);

#endif