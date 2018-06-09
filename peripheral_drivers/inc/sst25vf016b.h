#ifndef __SST25VF016B_H__
#define __SST25VF016B_H__               1
    
    #include "nrf_delay.h"
    #include "nrf_drv_spi.h"
    #include "nrf_gpio.h"
    #include "boards.h"

    /*
    * 4K Sector Erase
    * 64K Block Erase
    * Entire Chip Erase
    * Address Format: 0x00XXYZBB
    * Where:
    *   XX = Block No.
    *   Y  = Sector No.
    *   Z  = Page No.
    *   BB = Byte No.
    */
    
    #define SPIM0_MAX_BUFF_LEN          256

    #define W25Q32_PAGE_SIZE            256
    #define W25Q32_MAX_PAGES            16384
    
    /* Standard SPI Commands */
    /* Write Enable Cmd */
    #define SST25_CMD_WRITE_EN          0x06
    /* Write Disable Cmd */
    #define SST25_CMD_WRITE_DIS         0x04
    /* Manufacturer / Device ID Cmd */
    #define SST25_CMD_READ_ID           0x90
    /* JEDEC ID Cmd */
    #define SST25_CMD_JEDEC_ID          0x9F
    /* Read Data Cmd */
    #define SST25_CMD_READ_DATA         0x03
    /* Fast Read Cmd */
    #define SST25_CMD_FAST_READ         0x0B
    /* Byte Program Cmd */
    #define SST25_CMD_BYTE_PROG         0x02
    /* Auto Address Increment Program Cmd */
    #define SST25_CMD_AAI_PROG          0xAD
    /* Sector Erase (4KB) Cmd */
    #define SST25_CMD_SEC_ERASE         0x20
    /* Block Erase (32KB) Cmd */
    #define SST25_CMD_BL32_ERASE        0x52
    /* Block Erase (64KB) Cmd */
    #define SST25_CMD_BL64_ERASE        0xD8
    /* Chip ERASE Cmd */
    #define SST25_CMD_CHIP_ERASE        0xC7
    /* Read Status Reg Cmd */
    #define SST25_CMD_READ_STAT_REG     0x05
    /* Write En Status Reg Cmd */
    #define SST25_CMD_ENWRITE_STAT_REG  0x50
    /* Write Status Reg Cmd */
    #define SST25_CMD_WRITE_STAT_REG    0x01
    /* Enable SO to output RY/BY# status during AAI prog. */
    #define SST25_CMD_EBSY              0x70
    /* Disable SO to output RY/BY# status during AAI prog. */
    #define SST25_CMD_DBSY              0x80

    /* Wait Time intervals */
    /* Time Block Program 10us */
    #define SST25_TIME_TBP              10
    /* Time Sector Erase 25mS */
    #define SST25_TIME_TSE              20
    /* Time Block Erase 25mS */
    #define SST25_TIME_TBE              20
    /* Time Chip Erase 50mS */
    #define SST25_TIME_CE               45

    /* Data Structures */
    typedef struct spi_packet_t
    {
        uint8_t txBuff[SPIM0_MAX_BUFF_LEN];
        uint16_t txBuffLen;
        uint8_t rxBuff[SPIM0_MAX_BUFF_LEN];
        uint16_t rxBuffLen;
    }spi_packet_t;

    /* Functions to Execute Cmds */
    /** 
    * @brief Send Write En / Dis Cmd
    * @param[in] en 1 = Send Write En Cmd, 0 = Send Write disable Cmd.
    */
    void SST25CmdWriteEn(uint8_t en);
    /** 
    * @brief Read Status Reg Cmd
    * @retval Value of the status register
    */
    uint8_t SST25CmdReadStatReg(void);
    
    /** 
    * @brief Executes Read Status Reg Cmd and returns the chip busy status.
    * @retval 1 if chip is busy, 0 otherwise.
    */
    uint8_t SST25CmdIsBusy(void);

    /** 
    * @brief Polls chip busy status and loops until memory chip is busy.
    */
    void SST25CmdBusyWait(void);

    /** 
    * @brief Write Status Reg Cmd
    * @param[in] regVal Value to be written to the status register
    */
    void SST25CmdWriteStatReg(uint8_t regVal);
    /** 
    * @brief Read Data Cmd 
    * @param[in] addr Memory address.
    * @param[in] dBuff Pointer to the data buffer.
    * @param[in] dLen Length of the data buffer.
    */
    void SST25CmdReadData(uint32_t addr, uint8_t *dBuff, uint16_t dLen);
    
    /** 
    * @brief Program Byte Cmd. 
    * An Erase cmd needs to be issued explicitly before executing this cmd.
    * @param[in] addr Memory address.
    * @param[in] dByte 1 Byte data to be written to memory.
    */
    void SST25CmdProgByte(uint32_t addr, uint8_t dByte); 

    /** 
    * @brief Auto Address Increment Program Cmd. 
    * An Erase cmd needs to be issued explicitly before executing this cmd.
    * @param[in] addr Memory address.
    * @param[in] dBuff Pointer to the data buffer.
    * @param[in] dLen Length of the data buffer. (1 - 65535 Bytes)
    */
    void SST25CmdAAIProg(uint32_t addr, uint8_t *dBuff, uint16_t dLen);
    /** 
    * @brief Erase Sector Cmd
    * @param[in] addr Address of the 4K Sector to be erased.
    * @param[in] busyWait if 1 poll status and wait for the command to complete.
    */
    void SST25CmdEraseSector(uint32_t addr, uint8_t busyWait);
    /** 
    * @brief Erase Block 32K Cmd
    * @param[in] addr Address of the 32K Block to be erased.
    * @param[in] busyWait if 1 poll status and wait for the command to complete.
    */
    void SST25CmdEraseBlock32(uint32_t addr, uint8_t busyWait);
    /** 
    * @brief Erase Block 64K Cmd
    * @param[in] addr Address of the 64K Block to be erased.
    * @param[in] busyWait if 1 poll status and wait for the command to complete.
    */
    void SST25CmdEraseBlock64(uint32_t addr, uint8_t busyWait);
    /** 
    * @brief Chip Erase Cmd.
    * @param[in] busyWait if 1 poll status and wait for the command to complete.
    */
    void SST25CmdChipErase(uint8_t busyWait);
    /** 
    * @brief Read Device ID
    * @retval Returns 16-bit manufacturer ID + Device ID(0xBF41).
    */
    uint16_t SST25CmdReadDevId(void);
    /** 
    * @brief Read 24-bit JEDEC ID
    * @param[in] idBuff Buffer where 3 Bytes of id will be stored.
    */
    void SST25CmdReadJEDECId(uint8_t *idBuff);
    
#endif