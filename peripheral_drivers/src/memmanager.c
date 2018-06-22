#include "memmanager.h"

//#define FORCE_PROG_FS_HEADER

/***************************************************************************  
    Memory Manager module for Winbond 32Mbit (4 Mega Byte) Flash.
 ***************************************************************************
**/

/* Declaring an instance of SPIM0 Peripheral. */
static const nrf_drv_spi_t spim = NRF_DRV_SPI_INSTANCE(BOARD_SPI);

//static uint32_t  memBaseAddr, rPtr, wPtr, nRecs;
memhdr_t memHeader;
static uint32_t peekRptr;

/** 
* @brief Read MBR from flash. Acquire pointers to the first location in memory.
* If writing first time, write default configuration to MBR.
*/
void MemManInit(void)
{
    W25Q32CmdReadData(0x00000000, (uint8_t *)&memHeader, sizeof(memHeader));
    
    /* Check if memory is formatted or not */
    #ifndef FORCE_PROG_FS_HEADER
        /* If header is not present */
        if(memcmp(&memHeader.id[0],"AGRILINX",8))
    #endif
    {
        memcpy(&memHeader.id[0], "AGRILINX", 8);
        /* Set sector 1 Address */
        memHeader.memBaseAddr = 0x00001000;
        memHeader.rPtr = memHeader.wPtr = memHeader.memBaseAddr;
        memHeader.nRecs = 0;
        /* Set default update frequency to 1 min */
        memHeader.updateFreq = 6;

        /* Erase Sector 0 */
        W25Q32CmdEraseSector(0x00000000,1);
        /* Write MBR to Page 0 */
        W25Q32CmdProgPage(0x00000000, (uint8_t *)&memHeader, sizeof(memHeader), 1);
    }
    peekRptr = memHeader.rPtr;
}

void MemManUpdateMBR(void)
{
    /* Erase Sector 0 */
    W25Q32CmdEraseSector(0x00000000,1);

    /* Write MBR to Sector 0 */
    W25Q32CmdProgPage(0x00000000, (uint8_t *)&memHeader, sizeof(memHeader), 1);
}

/** 
* @brief Write buffer contents to the next free location in the memory.
* It performs a sector erase before writing if the page address is 0.
* @param[in] pkt, Buffer to be written in the memory.
* @param[in] len, Length of the buffer to be written in the memory.
*/
void MemManWriteRecs(void *pkt, uint8_t noOfRecs)
{
    //uint32_t memPtr = memBaseAddr + (nPages * 256);
    uint8_t buff[256];

    /* if address page no is 0, erase the sector before writing */
    if(!(memHeader.wPtr&0x00000F00))
    {
        W25Q32CmdEraseSector(memHeader.wPtr, 0);
    }

    /* Copy application contents to the buffer to avoid memory faults 
     * as app buffer (pkt) can be of size less than 256.
     */
    memcpy(buff, pkt, (noOfRecs*RECORD_SIZE));
    
    /* Wait for Erase operation to complete */
    while(W25Q32CmdIsBusy());
    
    /* Perform write operation */
    W25Q32CmdProgPage(memHeader.wPtr, buff, 256, 1);

    /* Update wPtr */
    memHeader.wPtr += 256;
    /* Update nRecs */
    memHeader.nRecs += noOfRecs;
    /* wPtr address overflow check */

    /* Update MBR */
    MemManUpdateMBR();
}

void MemManReadPage(uint8_t *pkt, uint8_t *len, uint8_t delRec)
{
    uint8_t rBuff[256], index = 0;
    *len = 0;

    /* if any records present in memory */
    if(memHeader.nRecs)
    {
        W25Q32CmdReadData(memHeader.rPtr, rBuff, 256);
        while(*len != RECS_PER_PAGE)
        {
            if((rBuff[index] == APP_PKT_HDR) && (rBuff[index+sizeof(app_pkt_t)-1] == APP_PKT_FTR))
            {
                memcpy(&pkt[index], &rBuff[index], sizeof(app_pkt_t));
                index += sizeof(app_pkt_t);
                (*len)++;
                
                /* Delete rec from memory if requested */
                if(delRec)
                {
                    /* Update nRecs */
                    memHeader.nRecs--;
                }
            }
            else
            {
                break;
            }
        }
        
        /* Delete rec from memory if requested */
        if(delRec)
        {
            /* Update rPtr */
            memHeader.rPtr  += 256;
            /* rPtr address overflow check */
            /* Check whether memory is empty, reset R/W pointers */
            if(!memHeader.nRecs)
            {
                /* Randomly update memBase address (for wear levelling) */
                /* Reset rPtr, wPtr to base address */
                memHeader.rPtr = memHeader.wPtr = memHeader.memBaseAddr;
            }
            MemManUpdateMBR();
            peekRptr = memHeader.rPtr;
        }
    }
}

void MemManRstPeekPtr(void)
{
    peekRptr = memHeader.rPtr;
}

uint8_t MemManPeekPage(uint8_t *pkt, uint8_t *len)
{
    uint8_t rBuff[256], index = 0;
    *len = 0;

    /* if any records present in memory */
    if(peekRptr!=memHeader.wPtr)
    {
        W25Q32CmdReadData(peekRptr, rBuff, 256);
        while(*len != RECS_PER_PAGE)
        {
            if((rBuff[index] == APP_PKT_HDR) && (rBuff[index+sizeof(app_pkt_t)-1] == APP_PKT_FTR))
            {
                memcpy(&pkt[index], &rBuff[index], sizeof(app_pkt_t));
                index += sizeof(app_pkt_t);
                (*len)++;
            }
            else
            {
                break;
            }
        }
        
        /* Update rPtr */
        peekRptr += 256;
        /* rPtr address overflow check */
        return 0;
    }
    else
      return MEMMAN_PEEK_NO_RECS;
}