#ifndef __MEMMANAGER_H__
#define __MEMMANAGER__                 1
    
    
    /* Defines size of 1 record */
    #define  RECORD_SIZE          sizeof(app_pkt_t)
    #define  RECS_PER_PAGE        (uint8_t)(256/RECORD_SIZE)    
    
    /* Defines for error codes */
    #define MEMMAN_PEEK_NO_RECS   1

    #include "nrf_delay.h"
    #include "nrf_drv_spi.h"
    #include "nrf_gpio.h"
    #include "boards.h"
    #include "w25q32.h"
    #include "appconfig.h"

    /** 
    * @brief Read MBR from flash. Acquire pointers to the first location in memory.
    * If writing first time, write default configuration to MBR.
    */
    void MemManInit(void);
    void MemManUpdateMBR(void);
    /** 
    * @brief Write buffer contents to the next free location in the memory.
    * It performs a sector erase before writing if the page address is 0.
    * @param[in] pkt, Buffer to be written in the memory.
    * @param[in] len, Length of the buffer to be written in the memory.
    */
    void MemManWriteRecs(void *pkt, uint8_t noOfRecs);
    void MemManReadPage(uint8_t *pkt, uint8_t *len, uint8_t delRec);
    void MemManRstPeekPtr(void);
    uint8_t MemManPeekPage(uint8_t *pkt, uint8_t *len);

#endif