#ifndef __W25Q32_H__
#define __W25Q32_H__                    1
    
    #include "nrf_delay.h"
    #include "nrf_drv_spi.h"
    #include "nrf_gpio.h"
    #include "boards.h"

    /*
    * 4K Sector Erase
    * 64K Block Erase
    * Entire Chip Erase
    * Address Format: XXYYZZ
    * Where:
    *   XX = Block No.
    *   YY = Sector No.
    *   ZZ = Page No.
    */
    
    #define SPIM0_MAX_BUFF_LEN          256

    #define W25Q32_PAGE_SIZE            256
    #define W25Q32_MAX_PAGES            16384
    
    /* Standard SPI Commands */
    /* Write Enable Cmd */
    #define W25Q32_CMD_WRITE_EN         0x06
    /* Volatile SR Write Enable Cmd */
    #define W25Q32_CMD_VOL_SR_WREN      0x50
    /* Write Disable Cmd */
    #define W25Q32_CMD_WRITE_DIS        0x04
    /* Release Power Down / ID Cmd */
    #define W25Q32_CMD_REL_PWR_DNID     0xAB
    /* Manufacturer / Device ID Cmd */
    #define W25Q32_CMD_MAN_DEV_ID       0x90
    /* JEDEC ID Cmd */
    #define W25Q32_CMD_JEDEC_ID         0x9F
    /* Read Unique ID Cmd */
    #define W25Q32_CMD_READ_UNIQUE_ID   0x4B
    /* Read Data Cmd */
    #define W25Q32_CMD_READ_DATA        0x03
    /* Fast Read Cmd */
    #define W25Q32_CMD_FAST_READ        0x0B
    /* Page Program Cmd */
    #define W25Q32_CMD_PAGE_PROG        0x02
    /* Sector Erase (4KB) Cmd */
    #define W25Q32_CMD_SEC_ERASE        0x20
    /* Block Erase (32KB) Cmd */
    #define W25Q32_CMD_BL32_ERASE       0x52
    /* Block Erase (64KB) Cmd */
    #define W25Q32_CMD_BL64_ERASE       0xD8
    /* Chip ERASE Cmd */
    #define W25Q32_CMD_CHIP_ERASE       0xC7
    /* Read Status Reg1 Cmd */
    #define W25Q32_CMD_READ_STAT_REG1   0x05
    /* Write Status Reg1 Cmd */
    #define W25Q32_CMD_WRITE_STAT_REG1  0x01
    /* Read Status Reg2 Cmd */
    #define W25Q32_CMD_READ_STAT_REG2   0x35
    /* Write Status Reg2 Cmd */
    #define W25Q32_CMD_WRITE_STAT_REG2  0x31
    /* Read Status Reg3 Cmd */
    #define W25Q32_CMD_READ_STAT_REG3   0x15
    /* Write Status Reg3 Cmd */
    #define W25Q32_CMD_WRITE_STAT_REG3  0x11
    /* Read SFDP Reg Cmd */
    #define W25Q32_CMD_READ_SFDP_REG    0x5A
    /* Erase Security Register Cmd */
    #define W25Q32_CMD_ERASE_SEC_REG    0x44
    /* Program Security Register Cmd */
    #define W25Q32_CMD_PROG_SEC_REG     0x42
    /* Read Security Register Cmd */
    #define W25Q32_CMD_READ_SEC_REG     0x48
    /* Globlal Block Lock Cmd */
    #define W25Q32_CMD_GLBL_BL_LOCK     0x7E
    /* Globlal Block Unlock Cmd */
    #define W25Q32_CMD_GLBL_BL_ULOCK    0x98
    /* Read Block Lock Cmd */
    #define W25Q32_CMD_READ_BL_LOCK     0x3D
    /* Individual Block Lock Cmd */
    #define W25Q32_CMD_INDVL_BL_LOCK    0x36
    /* Individual Block Unlock Cmd */
    #define W25Q32_CMD_INDVL_BL_ULOCK   0x39
    /* Erase / Program Suspend Cmd */
    #define W25Q32_CMD_ERASE_SUSPEND    0x75
    /* Erase / Program Resume Cmd */
    #define W25Q32_CMD_ERASE_RESUME     0x7A
    /* Power Down Cmd */
    #define W25Q32_CMD_POWER_DOWN       0xB9
    /* Enable Reset Cmd */
    #define W25Q32_CMD_RST_EN           0x66
    /* Reset Device Cmd */
    #define W25Q32_CMD_RST_DEV          0x99
    
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
    * @param[in] en 1 = Enable Writes, 0 = Disable Writes.
    */
    void W25Q32CmdWriteEn(uint8_t en);
    /** 
    * @brief Read Status Reg Cmd
    * @param[in] regId Id of the status register to be read (1/2/3).
    * @retval Value of the status register
    */
    uint8_t W25Q32CmdReadStatReg(uint8_t regId);
    
    /** 
    * @brief Executes Read Status Reg Cmd and returns the chip busy status.
    * @retval 1 if chip is busy, 0 otherwise.
    */
    uint8_t W25Q32CmdIsBusy(void);

    /** 
    * @brief Write Status Reg Cmd
    * @param[in] regId Id of the status register to be written (1/2/3).
    * @param[in] regVal Value to be written to the status register
    */
    void W25Q32CmdWriteStatReg(uint8_t regId, uint8_t regVal);
    /** 
    * @brief Read Data Cmd 
    * @param[in] addr Memory address.
    * @param[in] dBuff Pointer to the data buffer.
    * @param[in] dLen Length of the data buffer.
    */
    void W25Q32CmdReadData(uint32_t addr, uint8_t *dBuff, uint16_t dLen);
    /** 
    * @brief Program Page Cmd. 
    * An Erase cmd needs to be issued explicitly before executing this cmd.
    * @param[in] addr Memory address.
    * @param[in] dBuff Pointer to the data buffer.
    * @param[in] dLen Length of the data buffer. (1 - 256 Bytes)
    * @param[in] busyWait if 1 poll status and wait for the command to complete.
    */
    void W25Q32CmdProgPage(uint32_t addr, uint8_t *dBuff, uint16_t dLen, uint8_t busyWait); 
    /** 
    * @brief Erase Sector Cmd
    * @param[in] addr Address of the 4K Sector to be erased.
    */
    void W25Q32CmdEraseSector(uint32_t addr, uint8_t busyWait);
    /** 
    * @brief Erase Block 32K Cmd
    * @param[in] addr Address of the 32K Block to be erased.
    * @param[in] busyWait if 1 poll status and wait for the command to complete.
    */
    void W25Q32CmdEraseBlock32(uint32_t addr, uint8_t busyWait);
    /** 
    * @brief Erase Block 64K Cmd
    * @param[in] addr Address of the 64K Block to be erased.
    * @param[in] busyWait if 1 poll status and wait for the command to complete.
    */
    void W25Q32CmdEraseBlock64(uint32_t addr, uint8_t busyWait);
    /** 
    * @brief Chip Erase Cmd.
    * @param[in] busyWait if 1 poll status and wait for the command to complete.
    */
    void W25Q32CmdChipErase(uint8_t busyWait);
    /** 
    * @brief Power Down Cmd
    */
    void W25Q32CmdPowerDn(void);
    /** 
    * @brief Release Power Down Cmd
    */
    void W25Q32CmdRelPowerDn(void);
    /** 
    * @brief Read Device ID
    * @retval Returns 16-bit manufacturer ID + Device ID.
    */
    uint16_t W25Q32CmdReadDevId(void);
    /** 
    * @brief Read 64-bit Unique ID
    * @param[in] idBuff Buffer where 8 Bytes of id will be stored.
    */
    void W25Q32CmdReadUniqueId(uint8_t *idBuff);
    /** 
    * @brief Read 24-bit JEDEC ID
    * @param[in] idBuff Buffer where 3 Bytes of id will be stored.
    */
    void W25Q32CmdReadJEDECId(uint8_t *idBuff);
    /** 
    * @brief Read 256 Byte Serial Flash Discoverable Parameter. (JEDEC Standard).
    * @param[in] addr Starting address from where the parameter is to be read.
    * @param[in] paramBuff Buffer where the parameter will be stored.
    * @param[in] len Length of the parameter bytes to be read. 
    */
    void W25Q32CmdReadSFDPReg(uint32_t addr, uint8_t *paramBuff, uint16_t len);
    /** 
    * @brief Individual Block / Sector Lock / Unlock Cmd.
    * @param[in] addr Address of the Block / Sector to be locked / unlocked
    * @param[in] isBlk Perform operation on a block or sector. 1 = block, 0 = sector. 
    * @param[in] setLock 1 = Lock, 0 = Unlock Block / Sector. 
    */
    void W25Q32CmdBlkSecLock(uint32_t addr, uint8_t isBlk, uint8_t setLock);
    /** 
    * @brief Read Block / Sector Lock Cmd.
    * @param[in] addr Address of the Block / Sector to be locked / unlocked
    * @param[in] isBlk Perform operation on a block or sector. 1 = block, 0 = sector. 
    * @retval 1 if Locked, 0 if unlocked. 
    */
    uint8_t W25Q32CmdReadBlkSecLock(uint32_t addr, uint8_t isBlk);
    
    /** 
    * @brief Global Block / Sector Lock Cmd.
    * @param[in] setLock 1 = Lock, 0 = Unlock Block / Sector.
    */
    void W25Q32CmdGlobalBlkSecLock(uint8_t setLock);
    
    /** 
    * @brief Reset Device Cmd.
    */
    void W25Q32CmdReset(void);
    
#endif