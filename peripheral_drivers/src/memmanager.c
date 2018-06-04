#include "memmanager.h"

/***************************************************************************  
    Memory Manager module for Winbond 32Mbit (4 Mega Byte) Flash.
 ***************************************************************************
**/

/* Declaring an instance of SPIM0 Peripheral. */
static const nrf_drv_spi_t spim = NRF_DRV_SPI_INSTANCE(BOARD_SPI);

static uint32_t  memBaseAddr, rPtr, wPtr, nRecs;

/** 
* @brief Read MBR from flash. Acquire pointers to the first location in memory.
* If writing first time, write default configuration to MBR.
* @param[in] nRecs, Returns number of records present in the memory.
*/
void MemManInit(void)
{
    uint8_t buff[24]={};
    //W25Q32CmdReadData(0x00000000, buff, sizeof(buff));
    W25Q32CmdReadData(0x00000000, buff, 8);
    
    /* Check if memory is formatted or not */
    if(!memcmp(buff,"AGRILINX",8))
    {
        memBaseAddr = (((uint32_t)buff[9])<<8) | buff[8];
        memBaseAddr |= (((uint32_t)buff[11])<<24) | (((uint32_t)buff[10])<<16);
          
        rPtr = (((uint32_t)buff[13])<<8) | buff[12];
        rPtr |= (((uint32_t)buff[15])<<24) | (((uint32_t)buff[14])<<16);

        wPtr = (((uint32_t)buff[17])<<8) | buff[16];
        wPtr |= (((uint32_t)buff[19])<<24) | (((uint32_t)buff[18])<<16);

        nRecs = (((uint32_t)buff[21])<<8) | buff[20];
        nRecs |= (((uint32_t)buff[23])<<24) | (((uint32_t)buff[22])<<16);
    }
    else
    {
        memcpy(buff, "AGRILINX", 8);
        /* Set sector 1 Address */
        memBaseAddr = 0x00001000;
        rPtr = wPtr = memBaseAddr;
        nRecs = 0;

        memcpy(&buff[8], &memBaseAddr, 4);
        memcpy(&buff[12], &rPtr, 4);
        memcpy(&buff[16], &wPtr, 4);
        memcpy(&buff[20], &nRecs, 4);
        
        /* Erase Sector 0 */
        W25Q32CmdEraseSector(0x00000000,1);
        /* Write MBR to Page 0 */
        //W25Q32CmdProgPage(0x00000000, buff, sizeof(buff), 1);
        W25Q32CmdProgPage(0x00000000, buff, 8, 1);
    }
}

uint32_t MemManGetRecCount(void)
{
    return nRecs;  
}

void MemManUpdateMBR(void)
{
    uint8_t buff[24];

    /* Erase Sector 0 */
    W25Q32CmdEraseSector(0x00000000,0);

    memcpy(buff, "AGRILINX", 8);
    memcpy(&buff[8], &memBaseAddr, 4);
    memcpy(&buff[12], &rPtr, 4);
    memcpy(&buff[16], &wPtr, 4);
    memcpy(&buff[20], &nRecs, 4);
        

    /* Wait for Erase operation to complete */
    while(W25Q32CmdIsBusy());

    /* Write MBR to Sector 0 */
    W25Q32CmdProgPage(0x00000000, buff, sizeof(buff), 1);
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
    if(!(wPtr&0x00000F00))
    {
        W25Q32CmdEraseSector(wPtr, 0);
    }

    /* Copy application contents to the buffer to avoid memory faults 
     * as app buffer (pkt) can be of size less than 256.
     */
    memcpy(buff, pkt, (noOfRecs*RECORD_SIZE));
    
    /* Wait for Erase operation to complete */
    while(W25Q32CmdIsBusy());
    
    /* Perform write operation */
    W25Q32CmdProgPage(wPtr, buff, 256, 1);

    /* Update wPtr */
    wPtr += 256;
    /* Update nRecs */
    nRecs += noOfRecs;
    /* wPtr address overflow check */

    /* Update MBR */
    MemManUpdateMBR();
}

void MemManReadPage(void *pkt, uint8_t *len, uint8_t delRec)
{
    uint8_t rBuff[256], index = 0;
    *len = 0;

    /* if any records present in memory */
    if(nRecs)
    {
        W25Q32CmdReadData(rPtr, rBuff, 256);
        while(*len != RECS_PER_PAGE)
        {
            if((rBuff[index] == APP_PKT_HDR) && (rBuff[index+sizeof(app_pkt_t)-1] == APP_PKT_FTR))
            {
                memcpy(&pkt[index], &rBuff[index], sizeof(app_pkt_t));
                index += sizeof(app_pkt_t);
                *len++;
                
                /* Delete rec from memory if requested */
                if(delRec)
                {
                    /* Update nRecs */
                    nRecs--;
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
            rPtr  += 256;
            /* rPtr address overflow check */
            /* Check whether memory is empty, reset R/W pointers */
            if(!nRecs)
            {
                /* Randomly update memBase address (for wear levelling) */
                /* Reset rPtr, wPtr to base address */
                rPtr = wPtr = memBaseAddr;
            }
            MemManUpdateMBR();
        }
    }
}





