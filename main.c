/**
 * Copyright (c) 2018, Linear Circuits Pvt. Ltd.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Linear Circuits
 *    integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Linear Circuits nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Linear Circuits integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY Linear Circuits "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Linear Circuits OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup agrilinx_tracker_main main.c
 * @{
 * @ingroup agrilinx_tracker
 * @brief Agrilinx Asset and Manpower Tracker Application main file.
 *
 * This file contains the source code for asset and manpower tracking application.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_drv_uart.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_gpiote.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "boards.h"
#include "ifc_struct_defs.h"
#include "ll_ifc_consts.h"
#include "ll_ifc_symphony.h"
#include "ll_ifc_transport_mcu.h"
#include "ubloxm8.h"
#include "lis3dshtr.h"
#include "mcp7940m.h"
#include "lc709203f.h"
#include "memmanager.h"
#include "appconfig.h"
#include "testsuite.h"

/** @brief Function to initialize all the onboard peripherals
 */
void InitPeripherals(void);

/** @brief Function starting the internal LFCLK oscillator.
 */
void LfclkConfig(void);

/**
 * @brief Function for initializing real time counter 1 & 2.
 * RTC1 is used as a base timer to generate 10 sec delay.
 * RTC2 is used as a tick timer to generate delays. 
 */
void RTCInit(void);

/** @brief Function to initialize User switch and charge indication IRQs. 
 */
void SWIrqInit(void);
void ChgIndIrqInit(void);

/** @brief Function to initialize TWI Master
 */
void TWIM1Init(void);

/** @brief Function to initialize SPI Master
 */
void SPIM0Init(void);

/** @brief Lora Rx interrupt handler
 */
static void SWIrqHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

/** @brief Function to process the symphony link state machine.
 */
void SlProcessComm(void);

/** @brief Function to calculate checksum-8
 */
uint8_t CalcChkSum(uint8_t *buff, uint8_t len);

/** @brief Function to detect switch press in a non-blocking manner. 
 */
void CheckSwPress(void);

/** @brief Puts MCU to sleep and enters Low Power Mode. 
 */
void EnterLowPower(void);

/** @brief Parses received packet on LORA and generates resp.
 */
void ParseLoraRxPkt(uint8_t *rxBuff, uint8_t rxDataLen, uint8_t *respBuff, uint8_t *respLen);

/** @brief Auto-syncs time from GPS to RTC.
 */
void AutoSyncRTC(void);

/** 
 * @brief Writes a new application packet to the memory. 
 * @param[in] nvmPkt, pointer to the application packet to be written to the memory.
 * @param[in] forceUpd, boolean value indicating whether to force a write to the memory.
 */
void NVMAddRec(app_pkt_t *newRec, uint8_t forceUpd);

/** @brief Displays charge indication on leds.
 */
void DispChgState(void);

/** @brief Variables tracking LORA comm state.
 */
enum sl_states_t slCommState = SL_RESET;
enum sl_states_t slCommNextState = SL_RESET;

/**< Declaring an instance of RTC1 Peripheral. */
const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1);
/**< Declaring an instance of RTC2 Peripheral. */
const nrf_drv_rtc_t rtc2 = NRF_DRV_RTC_INSTANCE(2);

/* Declaring an instance of TWIM1 Peripheral. */
static const nrf_drv_twi_t twim1 = NRF_DRV_TWI_INSTANCE(BOARD_TWI);

/* Declaring an instance of SPIM0 Peripheral. */
static const nrf_drv_spi_t spim0 = NRF_DRV_SPI_INSTANCE(BOARD_SPI);

uint8_t rtc1Expired = 0, rtc2Expired = 0;
uint8_t slBuffer[SL_BUFFER_LEN];
uint8_t slRxBuffer[SL_BUFFER_LEN];
uint8_t slRespBuff[SL_BUFFER_LEN];
uint8_t uplBuffer[SL_BUFFER_LEN];
uint8_t slRxDataLen = 0, slRespLen = 0;
uint8_t slTxComp = 0, slRxComp = 0;
enum swStates swUsrState = SW_RELEASED;
uint8_t deviceMode = DEV_MODE_OFFLINE;
/* Set to be around 10 secs */
uint8_t wakeUpTime = 80;
extern memhdr_t memHeader;
app_pkt_t appData;
uint8_t uploadRecs = 0, stopUpload = 0;
uint32_t uploadCount = 0;

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t irqFlags;
    uint8_t txCounter = 0, slTxLen = 0, minCounter = 0;
    app_pkt_t timeData={};
    app_pkt_t uploadPkt[RECS_PER_PAGE];
    uint8_t uplPktLen = 0, uplPktIndex = 0, i = 0;
    uint8_t failCount = 0;
    uint8_t sendTime = 0;
    uint8_t memFlag = 0, memOffUpdate = 0, sleepCondition = 0, gpsValid = 0;
    uint8_t tBuff[4];
      
    enum ll_state loraState;
    enum ll_tx_state txState;
    enum ll_rx_state rxState;
    
    enum uploadStates recUplState = UPLOAD_INIT;
    
    /* Configure board. */
    InitPeripherals();

    /* Start 10 sec RTC */
    nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_START);

    while (true)
    {
        /* Poll Online / Offline Switch 
         * Run switch state machine 
         */
        CheckSwPress();
        /* If switch pressed, toggle b/w online and offline modes */
        if(swUsrState == SW_PRESSED)
        {  
            if(deviceMode == DEV_MODE_OFFLINE)
            {
                /* Clear time var */
                memset(&timeData, 0, sizeof(timeData));
                /* Record start time in mem, set flags to send */
                timeData.header = APP_PKT_HDR;
                timeData.footer = APP_PKT_FTR;
                timeData.batV = LC709GetRSOC();
                timeData.gpsPkt.status = (1<<3);
                MCP79GetTime(tBuff);
                timeData.gpsPkt.hrs = tBuff[2];
                timeData.gpsPkt.min = tBuff[1];
                timeData.gpsPkt.sec = tBuff[0];
                MCP79GetFullDate(tBuff);
                timeData.gpsPkt.dd = tBuff[1];
                timeData.gpsPkt.mm = tBuff[2];
                timeData.gpsPkt.yy = tBuff[3];
                /* Record Start time in memory */
                NVMAddRec(&timeData, 0);
                sendTime = 1;
                deviceMode = DEV_MODE_ONLINE;
            }
            else
            {
                /* Clear time var */
                memset(&timeData, 0, sizeof(timeData));
                /* Record start time in mem, set flags to send */
                timeData.header = APP_PKT_HDR;
                timeData.footer = APP_PKT_FTR;
                timeData.batV = LC709GetRSOC();
                timeData.gpsPkt.status = (1<<4);
                MCP79GetTime(tBuff);
                timeData.gpsPkt.hrs = tBuff[2];
                timeData.gpsPkt.min = tBuff[1];
                timeData.gpsPkt.sec = tBuff[0];
                MCP79GetFullDate(tBuff);
                timeData.gpsPkt.dd = tBuff[1];
                timeData.gpsPkt.mm = tBuff[2];
                timeData.gpsPkt.yy = tBuff[3];
                /* Add stop time rec and force update to memory */
                NVMAddRec(&timeData, 1);
                sendTime = 2;   
                deviceMode = DEV_MODE_OFFLINE;
            }
            swUsrState = SW_RELEASED;
        }
        
        SlProcessComm();
        
        if(deviceMode == DEV_MODE_ONLINE)
        {
            /* Turn on green LED indicating online mode */
            nrf_gpio_pin_clear(LEDG_PIN);

            /* Make use of RTC1 interrupt to generate 10s */
            if(rtc1Expired)
            {
                ReadGPS(&appData.gpsPkt);
                appData.batV = LC709GetRSOC();
                if(appData.gpsPkt.status&0x01)
                {
                    gpsValid = 1;
                    /* Copy last valid gps record to be stored in memory */
                    memcpy(&memHeader.memGps, &appData.gpsPkt, sizeof(gps_pkt_t));
                }
                else
                {
                    gpsValid = 0;
                }
                
                /* This check will avoid race conditions */
                if(minCounter<memHeader.updateFreq)
                {
                    minCounter++;
                }
                /*If GPS data valid */
                if((gpsValid) && (minCounter!=memHeader.updateFreq))
                {
                    appData.header = APP_PKT_HDR;
                    appData.footer = APP_PKT_FTR;
                    NVMAddRec(&appData, 0);
                }
                
                rtc1Expired = 0;
            }

            /* Transmit a record in 1 min on LORA */
            if(minCounter == memHeader.updateFreq)
            {
                /* Send data on LORA if ready */
                if(slCommState == SL_READY)
                {
                  /* 
                   * Implement Tx state machine to test if Tx successful.
                   * If Tx successful, do not save current pkt,
                   * else save current pkt in memory.
                   */

                    slBuffer[0] = '$';
                    slBuffer[1] = 0x02;

                    /* If GPS data valid */
                    if(gpsValid)
                    {
                        /* Form Tx Pkt with GPS data */
                        /* Payload length (appData - 2(hdr, ftr) + 1(noOfRecs)) */
                        slBuffer[2] = (sizeof(appData) - 2) + 1;
                        /* Number of Records */
                        slBuffer[3] = 1;
                        /* Indicate that its a live point */
                        appData.gpsPkt.status |= (1<<5);
                        /* Exclude header and footer from app data while copying */
                        memcpy(&slBuffer[4], &appData.batV, sizeof(appData) - 2);
                        slBuffer[22] = CalcChkSum(slBuffer, 22);
                        slTxLen = 23;
                        /* Set save in mem flag */
                        memFlag = 1;
                    }
                    else
                    {
                        /* Form Tx Pkt indicating no GPS data */
                        /* Payload Length */
                        slBuffer[2] = 1;
                        /* Hard-coded byte to indicate no GPS */
                        slBuffer[3] = 0x23;
                        slBuffer[4] = CalcChkSum(slBuffer, 4);
                        slTxLen = 5;
                    }
                    
                    ll_message_send_ack(slBuffer, slTxLen);
                    slTxLen = 0;
                    slCommState = SL_TRANSMITTING;
                }
                else
                {
                    /* If GPS data valid */
                    if(gpsValid)
                    {
                        appData.header = APP_PKT_HDR;
                        appData.footer = APP_PKT_FTR;
                        NVMAddRec(&appData, 0);
                    }
                }
                minCounter = 0;
            }

            nrf_gpio_pin_set(LEDG_PIN);
        }
        
        /* Check for any incoming msgs on LORA */
        if(slCommState == SL_READY)
        {
            ll_get_state(&loraState, &txState, &rxState);
            /* Read and clear IRQ Flags */
            ll_irq_flags(0xFFFFFFFF, &irqFlags);
            if(rxState == LL_RX_STATE_RECEIVED_MSG)
            {
                slRxComp = 0;
                slCommState = SL_RECEIVING;
            }
            nrf_gpio_pin_set(LEDR_PIN);
            nrf_gpio_pin_set(LEDB_PIN);
        }
        else
        {
            nrf_gpio_pin_clear(LEDR_PIN);
            nrf_gpio_pin_clear(LEDB_PIN);
        }
        
        if(sendTime&&(slCommState == SL_READY))
        {
            slBuffer[0] = '$';
            slBuffer[1] = 0x02;

            /* Form Tx Pkt with start / end time data */
            /* Payload length (appData - 2(hdr, ftr) + 1(noOfRecs)) */
            slBuffer[2] = (sizeof(timeData) - 2) + 1;
            /* Number of Records */
            slBuffer[3] = 1;
            /* Exclude header and footer from app data while copying */
            memcpy(&slBuffer[4], &timeData.batV, sizeof(timeData) - 2);
            slBuffer[22] = CalcChkSum(slBuffer, 22);
            slTxLen = 23;
            
            ll_message_send_ack(slBuffer, slTxLen);
            slTxLen = 0;
            slCommState = SL_TRANSMITTING;
            
            sendTime = 0;
        }
        
        /* If response to be sent for any incoming msgs */
        if(slRespLen&&(slCommState == SL_READY))
        {
            ll_message_send_ack(slRespBuff, slRespLen);
            slRespLen = 0;
            slCommState = SL_TRANSMITTING;
        }

        if(slRxComp)
        {
            /* Take Rx Completed action here. */
            /* Parse LORA Rx Packet here */
            ParseLoraRxPkt(slRxBuffer, slRxDataLen, slRespBuff, &slRespLen);
            slRxComp = 0;
        }
        
        if(recUplState!=UPLOAD_TXRESP)
        {
            if(slTxComp)
            {
                /* Take Tx completed action here  */
                /* If memFlag is set and Tx failed, save to memory */
                if(slTxComp == 2 && memFlag)
                {
                    appData.header = APP_PKT_HDR;
                    appData.footer = APP_PKT_FTR;
                    /* Clear live point indication flag */
                    appData.gpsPkt.status = (appData.gpsPkt.status&0xDF);
                    NVMAddRec(&appData, 0);
                }
                memFlag = 0;
                slTxComp = 0;
            }
        }

        /* Upload records state machine */
        switch(recUplState)
        {
            case UPLOAD_INIT:
                if(uploadRecs)
                {
                    if((uploadCount == 0xFF) || (uploadCount > memHeader.nRecs))
                    {
                        /* Transfer all records */
                        uploadCount = memHeader.nRecs;
                    }
                    recUplState = UPLOAD_PKTIZE;
                }
                break;
            case UPLOAD_PKTIZE:
                if(uploadCount > 0)
                {
                    MemManReadPage((uint8_t*)&uploadPkt, &uplPktLen, 0);
                    if(uplPktLen)
                    {
                        uplBuffer[0] = '$';
                        uplBuffer[1] = 0x02;
                        /* Number of Records */
                        uplBuffer[3] = uplPktLen;
                        uplPktIndex = 4;
                        for(i = 0; i < uplPktLen; i++)
                        {
                            /* Exclude header and footer from app data while copying */
                            memcpy(&uplBuffer[uplPktIndex], &uploadPkt[i].batV, sizeof(uploadPkt[i]) - 2);
                            uplPktIndex += sizeof(app_pkt_t) - 2;
                        }
                        /* Payload length */
                        uplBuffer[2] = uplPktIndex - 3;
                        uplBuffer[uplPktIndex] = CalcChkSum(uplBuffer, uplPktIndex);
                        uplPktIndex++;
                        uplPktLen = uplPktIndex;
                        recUplState = UPLOAD_WAITRDY;
                        failCount = 0;
                    }
                    else
                    {
                        uploadRecs = 0;
                        recUplState = UPLOAD_INIT;
                        stopUpload = 0;
                    }
                }
                else
                {
                    uploadRecs = 0;
                    recUplState = UPLOAD_INIT;
                    stopUpload = 0;
                }
                break;
            case UPLOAD_WAITRDY:
                if(!stopUpload)
                {
                    if(slCommState == SL_READY)
                    {
                        ll_message_send_ack(uplBuffer, uplPktLen);
                        slCommState = SL_TRANSMITTING;
                        recUplState = UPLOAD_TXRESP;
                    }
                }
                else
                {
                    uploadRecs = 0;
                    recUplState = UPLOAD_INIT;
                    stopUpload = 0;
                }
                break;
            case UPLOAD_TXRESP:
                if(slTxComp)
                {
                    /* Take Tx completed successfully  */
                    if(slTxComp == 1)
                    {
                        /* Delete page from memory */
                        MemManReadPage((uint8_t*)&uploadPkt, &uplPktLen, 1);
                        uplPktLen = 0;
                        uploadCount--;
                        recUplState = UPLOAD_PKTIZE;
                        if(stopUpload)
                        {
                            uploadRecs = 0;
                            stopUpload = 0;
                            recUplState = UPLOAD_INIT;
                        }
                    }
                    else if(slTxComp == 2)
                    {
                        /* If Tx failed, retransmit */
                        failCount++;
                        if(failCount == 2)
                        {
                            uploadRecs = 0;
                            stopUpload = 0;
                            recUplState = UPLOAD_INIT;
                        }
                        else
                        {
                            recUplState = UPLOAD_WAITRDY;
                        }
                    }
                    slTxComp = 0;
                }  
                break;
            case UPLOAD_TIMEOUT:
                break;
        }
        /* Auto Sync RTC */
        AutoSyncRTC();
        
        /* Sleep condition, add && ((!slTxComp) && (!slRxComp)) if required */
        sleepCondition = (swUsrState==SW_RELEASED) && (slCommState == SL_READY);
        
        /* Display bat chg indication if device is in offline mode. */
        if(deviceMode == DEV_MODE_OFFLINE)
            DispChgState();

        /* Enter Low Power Mode */
        if(sleepCondition)
        {
            EnterLowPower();
        }
    }
}

/** @brief: Function for handling the RTC1 interrupts.
 * Triggered on COMPARE0 match.
 */
static void rtc1_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        /* Clear the RTC counter reg */
        nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_CLEAR);
        nrf_drv_rtc_cc_set(&rtc1, 0, wakeUpTime,true);
        rtc1Expired = 1;
    }
}

static void rtc2_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        /* Clear the RTC counter reg */
        nrf_rtc_task_trigger(rtc2.p_reg, NRF_RTC_TASK_CLEAR);
        /* Stop the RTC */
        nrf_rtc_task_trigger(rtc2.p_reg, NRF_RTC_TASK_STOP);
        rtc2Expired = 1;
    }
}

static void SWIrqHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    /* Implement switch press function if any */
}

void InitPeripherals(void)
{
    LfclkConfig();
    RTCInit();
    ll_uart_init();
    SWIrqInit();
    ChgIndIrqInit();
    TWIM1Init();
    SPIM0Init();
    /* Initialize Battery Gauge */
    LC709Init();
    
    nrf_delay_ms(1000);

    /* Init GPS, Set GPS Packet Filters */
    SetGNRMCFilter();
    /* Init Accelerometer */
    //LIS3DInit(ADDR_LIS3DSHTR);

    /* Init All LEDs & GPIOS */
    /* RST pin for LORA / CATM1 */
    nrf_gpio_cfg_output(COMM_NRST_PIN);
    
    /* Configure LED Pins as Outputs */
    nrf_gpio_pin_set(LEDR_PIN);
    nrf_gpio_cfg_output(LEDR_PIN);
    
    nrf_gpio_pin_set(LEDG_PIN);
    nrf_gpio_cfg_output(LEDG_PIN);
    
    nrf_gpio_pin_set(LEDB_PIN);
    nrf_gpio_cfg_output(LEDB_PIN);

    nrf_gpio_pin_set(LEDG_PIN);
    nrf_gpio_cfg_output(LEDG_PIN);
    
    /* Console Mux Select Pin */
    nrf_gpio_pin_clear(SEL_MUX_PIN);
    nrf_gpio_cfg_output(SEL_MUX_PIN);
    
    /* Input Pins */
    /* PGOOD Pin 1 when power is good, 0 otherwise. */
    nrf_gpio_cfg_input(PGOOD_PIN, NRF_GPIO_PIN_NOPULL);
    /* NCHG Pin 1 when charging battery, 0 otherwise */
    nrf_gpio_cfg_input(NCHG_PIN, NRF_GPIO_PIN_NOPULL);
    /* User switch pin*/
    nrf_gpio_cfg_input(SW_USR_PIN, NRF_GPIO_PIN_NOPULL);

    /* Select LORA Port for UART */
    nrf_gpio_pin_set(SEL_MUX_PIN);
    
    /* Initialize memory manager module */
    MemManInit();
}

void LfclkConfig(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

void RTCInit(void)
{
    uint32_t err_code;

    /* Initialize RTC1 instance for 125ms (8Hz) Tick */
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc1, &config, rtc1_handler);
    APP_ERROR_CHECK(err_code);

    /* Power on RTC instance */
    nrf_drv_rtc_enable(&rtc1);
    
    nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_STOP);
    nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_CLEAR);

    /* Set compare channel 0 */
    nrf_drv_rtc_cc_set(&rtc1, 0, wakeUpTime,true);

    config.prescaler = 32;
    err_code = nrf_drv_rtc_init(&rtc2, &config, rtc2_handler);
    APP_ERROR_CHECK(err_code);

    /* Power on RTC instance */
    nrf_drv_rtc_enable(&rtc2);
    
    nrf_rtc_task_trigger(rtc2.p_reg, NRF_RTC_TASK_STOP);
    nrf_rtc_task_trigger(rtc2.p_reg, NRF_RTC_TASK_CLEAR);
}

void SWIrqInit(void)
{
    uint32_t err_code;

    nrf_drv_gpiote_in_config_t config2 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    
    err_code = nrf_drv_gpiote_in_init(SW_USR_PIN, &config2, SWIrqHandler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(SW_USR_PIN, true);
}

void ChgIndIrqInit(void)
{
    uint32_t err_code;

    nrf_drv_gpiote_in_config_t config =  GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    
    /* GPIOTE Module already enabled in SWIRQInit() function */
    err_code = nrf_drv_gpiote_in_init(PGOOD_PIN, &config, SWIrqHandler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(PGOOD_PIN, true);

    err_code = nrf_drv_gpiote_in_init(NCHG_PIN, &config, SWIrqHandler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(NCHG_PIN, true);
}

void TWIM1Init(void)
{
    ret_code_t err_code;

    nrf_drv_twi_config_t config = NRF_DRV_TWI_DEFAULT_CONFIG;
    config.scl = SCL1_PIN;
    config.sda = SDA1_PIN;
    
    err_code = nrf_drv_twi_init(&twim1, &config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&twim1);
}

void SPIM0Init(void)
{
    ret_code_t err_code;
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.miso_pin = SPIM0_MISO_PIN;
    spi_config.mosi_pin = SPIM0_MOSI_PIN;
    spi_config.sck_pin  = SPIM0_SCK_PIN;
    //spi_config.frequency = SPI_DEFAULT_FREQUENCY;
    spi_config.frequency = NRF_DRV_SPI_FREQ_250K;
    err_code = nrf_drv_spi_init(&spim0, &spi_config, NULL, NULL);
    
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    nrf_gpio_cfg_output(SPIM0_SS_PIN);

    APP_ERROR_CHECK(err_code);
}

void SlProcessComm(void)
{
    //Configure the following to match your network and application tokens
    //Set desired network token 
    static uint32_t netToken = 0x0204B8FA;
  
    //Set desired application token
    //Generate an application token in your Conductor account use it here.
    //The application token identifies this dataflow in Conductor.
    static uint8_t appToken[10] = {0x8b,0xda,0x15,0xfe,0x09,0xc1,0x8d,0x6c,0xba,0xfe};

    static uint8_t txErrCnt, rxErrCnt;
    
    uint8_t status;
    enum ll_state moduleStatus;
    enum ll_tx_state txStatus; 
    enum ll_rx_state rxStatus;

    //llabs_connect_status_t netInfo;
    llabs_network_info_t netInfo;
    
    switch(slCommState)
    {
        case SL_RESET:
            /* Active Low Reset on our board, its active high on EV-Board */
            #ifndef SL_EVBOARD_RST
                nrf_gpio_pin_clear(COMM_NRST_PIN);
                nrf_delay_ms(100);
                nrf_gpio_pin_set(COMM_NRST_PIN);
            #else
                nrf_gpio_pin_set(COMM_NRST_PIN);
                nrf_delay_ms(100);
                nrf_gpio_pin_clear(COMM_NRST_PIN);
            #endif
            
            slCommState = SL_WAITING;
            slCommNextState = SL_UNINITIALIZED;
            //nrf_gpio_pin_clear(LEDR_PIN);
            //nrf_gpio_pin_clear(LEDB_PIN);
            rxErrCnt = 0;
            txErrCnt = 0;
            rtc2Expired = 0;
            /* Set compare channel 0 to overflow in 2 secs */
            nrf_drv_rtc_cc_set(&rtc2, 0, 2000, true);
            nrf_rtc_task_trigger(rtc2.p_reg, NRF_RTC_TASK_START);
            break;
        case SL_UNINITIALIZED:
            if(ll_antenna_set(SL_ANT_SELECTED)>=0)
            {
                
            }
            else
            {
                slCommState = SL_ERROR;
            }

            //Set Symphony Link MAC mode
            if(ll_mac_mode_set(SYMPHONY_LINK)>=0)
            {
                //Write network config
                if(ll_config_set(netToken, appToken, LL_DL_ALWAYS_ON, APP_QOS)>=0)
                {
                    slCommState = SL_SCANNING;
                }
                else
                {
                    slCommState = SL_ERROR;
                }
            }
            else
            {
                slCommState = SL_ERROR;
            }
            break;
        case SL_SCANNING:
            if(ll_net_info_get(&netInfo)>=0)
            {
                switch (netInfo.connection_status)
                {
                    case LLABS_CONNECT_CONNECTED:
                        slCommState = SL_READY;
                        break;
                    case LLABS_CONNECT_DISCONNECTED:
                        if(ll_get_state(&moduleStatus, NULL, NULL)>=0)
                        {
                            if (moduleStatus == LL_STATE_IDLE_DISCONNECTED)
                            {
                                ll_app_reg_get(&status);
                                //Check if the module is registered on the network
                                if (status)
                                {
                                    slCommState = SL_READY;
                                }
                                else
                                {
                                    // One of the following has failed:
                                    // (1) Registration of the module's application token with Conductor, or
                                    // (2) registration of the module's downlink mode with the gateway, or
                                    // (3) crypto key exchange between the module and the gateway.
                                    slCommState = SL_ERROR;
                                }
                            }
                        }
                        else
                        {
                            slCommState = SL_ERROR;
                        }
                        break;
                    default:
                        break;
                }
            }
            else
            {
                slCommState = SL_ERROR;
            }
            break;
        case SL_READY:
            break;
        case SL_TRANSMITTING:
            if(ll_get_state(&moduleStatus, &txStatus, NULL)>=0)
            {
                if (moduleStatus == LL_STATE_IDLE_CONNECTED)
                {
                    switch (txStatus)
                    {
                        case LL_TX_STATE_SUCCESS:
                            txErrCnt = 0;
                            /* Notify tx completed */
                            slTxComp = 1;
                            slCommState = SL_READY;
                            break;
                        case LL_TX_STATE_ERROR:
                            txErrCnt++;
                            /* Notify the application that tx failed. */
                            slTxComp = 2;
                            slCommState = SL_READY;
                            break;
                        case LL_TX_STATE_TRANSMITTING:
                            break;
                    }
                }
                else if (moduleStatus == LL_STATE_IDLE_DISCONNECTED)
                {
                    txErrCnt++;
                    /* Notify the application that tx failed. */
                    slTxComp = 2;
                    slCommState = SL_READY;
                }
                else if (moduleStatus == LL_STATE_INITIALIZING)
                {
                    //Do not do anything, wait for the status to change. 
                }
                else 
                {
                    txErrCnt++;
                    //state == LL_STATE_ERROR or NaN. Reset the module.
                    /* Notify the application that tx failed. */
                    slTxComp = 2;
                    slCommState = SL_ERROR;
                }
            }
            else
            {
                txErrCnt++;
                /* Notify the application that tx failed. */
                slTxComp = 2;
                slCommState = SL_ERROR;
            }
            break;
        case SL_RECEIVING:
            if(ll_get_state(&moduleStatus, NULL, &rxStatus)>=0)
            {
                if (rxStatus == LL_RX_STATE_NO_MSG)
                {
                    slCommState = SL_READY;
                }
                else if (rxStatus == LL_RX_STATE_RECEIVED_MSG)
                {
                    if(ll_retrieve_message(slRxBuffer, &slRxDataLen, NULL, NULL)>=0)
                    {
                        rxErrCnt = 0;
                        //Indicate Rx Success.
                        slRxComp = 1;
                    }
                    else
                    {
                        /* Indicate Rx Failure */
                        slRxComp = 2;
                        rxErrCnt++;
                    }
                    slCommState = SL_READY;
                }
                else //state error
                {
                    rxErrCnt++;
                    /* Indicate Rx Failure */
                    slRxComp = 2;
                    slCommState = SL_ERROR;
                }
            }
            else
            {
                rxErrCnt++;
                /* Indicate Rx Failure */
                slRxComp = 2;
                slCommState = SL_ERROR;
            }
            break;
        case SL_WAITING:
            if(rtc2Expired)
            {
                rtc2Expired = 0;
                slCommState = slCommNextState;
            }
            break;
        case SL_ERROR:
            /* Reset module and try to re-initialize */
            slCommState = SL_WAITING;
            slCommNextState = SL_RESET;
            rtc2Expired = 0;
            /* Set compare channel 0 */
            nrf_drv_rtc_cc_set(&rtc2, 0, 5000,true);
            nrf_rtc_task_trigger(rtc2.p_reg, NRF_RTC_TASK_START);
            break;
    }
    if((txErrCnt > 5) || (rxErrCnt >5))
    {
        slCommState = SL_ERROR;
        txErrCnt = 0;
        rxErrCnt = 0;
    }
}

uint8_t CalcChkSum(uint8_t *buff, uint8_t len)
{
    uint8_t i = 0, chkSum = 0;
    for( i = 0; i < len; i++)
    {
        chkSum += buff[i];
    }
    return chkSum;
}

void CheckSwPress(void)
{
    static uint16_t count;
    switch(swUsrState)
    {
        case SW_RELEASED:
            if(!nrf_gpio_pin_read(SW_USR_PIN))
            {
                swUsrState = SW_DETECTING;
                count = 0;
            }
            break;
        case SW_DETECTING:
            nrf_delay_ms(5);
            if(!nrf_gpio_pin_read(SW_USR_PIN))
            {
                count++;
                if(count==5)
                {
                    nrf_gpio_pin_clear(LEDG_PIN);
                    swUsrState = SW_DEBOUNCING;
                }
            }
            else
            {
                swUsrState = SW_RELEASED;
            }
            break;
        case SW_DEBOUNCING:
            nrf_delay_ms(5);
            if(nrf_gpio_pin_read(SW_USR_PIN))
            {
                swUsrState = SW_PRESSED;
                nrf_gpio_pin_set(LEDG_PIN);
            }
            break;
        default:
            break;
    }
}

void EnterLowPower(void)
{
    // Wait for an event.
    __WFE();
    // Clear the internal event register.
    __SEV();
    __WFE();
}

void ParseLoraRxPkt(uint8_t *rxBuff, uint8_t rxDataLen, uint8_t *respBuff, uint8_t *respLen)
{
    uint8_t chkSum = 0, rxChkSum = 0;
    uint8_t dLen = 0, buffIndex = 0;
    *respLen = 0;
    gps_pkt_t tempGps;
    
    if (rxBuff[0] == '$')
    {
        /* Calc Chksum */
        chkSum = CalcChkSum(rxBuff, rxDataLen-1);
        /* If Chksum Matched */
        if (chkSum == rxBuff[rxDataLen - 1])
        {
            dLen = rxBuff[2];

            /* Check Pkt ID */
            switch (rxBuff[1])
            {
                case 0x01:
                    /* Get status packet request */
                    if ((dLen==1)&&(rxBuff[3] == 0xAB))
                    {
                        /* Generate Status response */
                        respBuff[0] = '$';
                        respBuff[1] = 0x01;
                        
                        /* Copy start time */
                        buffIndex = 3;
                        /* Indicate Device mode (online / offline) */
                        respBuff[buffIndex++] = deviceMode;
                        
                        /* Read Bat Level */
                        respBuff[buffIndex++] = LC709GetRSOC();
                        
                        /* Bat charge indication (1 if charging 0 otherwise) */
                        /* If USB source not connected */
                        if(nrf_gpio_pin_read(PGOOD_PIN))
                        {
                            respBuff[buffIndex++] = 0;
                        }
                        else
                        {
                            respBuff[buffIndex++] = !nrf_gpio_pin_read(NCHG_PIN);
                        }

                        /* If Memory header has last GPS data valid */
                        if(memHeader.memGps.status&0x01)
                        {
                            memcpy(&respBuff[buffIndex], &memHeader.memGps, sizeof(gps_pkt_t));
                            buffIndex += sizeof(gps_pkt_t);
                        }
                        else
                        {
                            /* Indicates GPS data invalid */
                            respBuff[buffIndex++] = 0x23;
                        }
                        
                        /* Copy nRecs from mem buffer */
                        memcpy(&respBuff[buffIndex], &memHeader.nRecs, sizeof(memHeader.nRecs));
                        buffIndex += sizeof(memHeader.nRecs);
                        //memcpy otherParams
                        
                        /* set appropriate data len here */
                        respBuff[2] = buffIndex - 3;

                        /* Calculate checksum */
                        respBuff[buffIndex] = CalcChkSum(respBuff, buffIndex);
                        buffIndex++;
                        
                        /* Set respLen */
                        *respLen = buffIndex;
                    }
                    break;
                case 0x02:
                    /* Get x records from DB request */
                    if (dLen == 1)
                    {
                        /* Get all records / Get x page of records */
                        /* Check if previous request is undergoing */
                        if(uploadRecs)
                        {
                            /* If received stop uploading cmd */
                            if(rxBuff[3] == 0xFE)
                            {
                                stopUpload = 1;
                            }
                            else
                            {
                                /* Send busy response */
                                respBuff[0] = '$';
                                respBuff[1] = 0x02;
                                respBuff[2] = 1;
                                respBuff[3] = 0xBB;
                                respBuff[4] = CalcChkSum(respBuff,4);
                                *respLen = 5;
                            }
                        }
                        else
                        {
                            uploadRecs = 1;
                            uploadCount = rxBuff[3]; 
                        }
                    }
                    break;
                case 0x03:
                    /* Set parameter request */
                    /* Generate Status response */
                    respBuff[0] = '$';
                    respBuff[1] = 0x03;
                    switch(rxBuff[3])
                    {
                        case PARAM_UPDFREQ_ID:
                            /* If update frequency is between 30 secs to 40 mins */
                            if((rxBuff[4] >=3)&&(rxBuff[4] <=240))
                            {
                                /* Update update freq and write it to memory */
                                memHeader.updateFreq = rxBuff[4];
                                MemManUpdateMBR();  

                                buffIndex = 3;
                                /* Set parameter ID */
                                respBuff[buffIndex++] = PARAM_UPDFREQ_ID;
                                respBuff[buffIndex++] = 'O';
                                respBuff[buffIndex++] = 'K';
                                /* set appropriate data len here */
                                respBuff[2] = buffIndex - 3;

                                /* Calculate checksum */
                                respBuff[buffIndex] = CalcChkSum(respBuff, buffIndex);
                                buffIndex++;

                                *respLen = buffIndex;
                            }
                            //else invalid parameter 
                            break;
                        default:
                            break;

                    }
                    break;
                case 0x04:
                    /* Get parameter request */
                    if (dLen==1)  
                    {
                        /* Generate Status response */
                        respBuff[0] = '$';
                        respBuff[1] = 0x04;
                        switch(rxBuff[3])
                        {
                            case PARAM_GPS_ID:
                                buffIndex = 3;
                                /* Set parameter ID */
                                respBuff[buffIndex++] = PARAM_GPS_ID;
                                
                                /* Read current GPS */
                                ReadGPS(&tempGps);
                                /* If GPS data valid */
                                if(tempGps.status&0x01)
                                {
                                    memcpy(&respBuff[buffIndex], &tempGps, sizeof(gps_pkt_t));
                                    buffIndex += sizeof(gps_pkt_t);
                                }
                                else
                                {
                                    /* Indicates GPS data invalid */
                                    respBuff[buffIndex++] = 0x23;
                                }
                                /* set appropriate data len here */
                                respBuff[2] = buffIndex - 3;

                                /* Calculate checksum */
                                respBuff[buffIndex] = CalcChkSum(respBuff, buffIndex);
                                buffIndex++;
                                /* Set respLen */
                                *respLen = buffIndex;
                                break;
                            case PARAM_UPDFREQ_ID:
                                buffIndex = 3;
                                /* Set parameter ID */
                                respBuff[buffIndex++] = PARAM_UPDFREQ_ID;

                                /* Set update frequency */
                                respBuff[buffIndex++] = memHeader.updateFreq;
                        
                                /* set appropriate data len here */
                                respBuff[2] = buffIndex - 3;

                                /* Calculate checksum */
                                respBuff[buffIndex] = CalcChkSum(respBuff, buffIndex);
                                buffIndex++;

                                *respLen = buffIndex;
                                break;
                        }
                    }
                    break;
                case 0x05:
                    /* BLE related requests */
                    break;
                default:
                    break;
            }
        }
    }
}

void AutoSyncRTC(void)
{
    uint8_t tBuff[4];

    /* If GPS pkt valid */
    if(appData.gpsPkt.status&0x01)
    {
        MCP79GetTime(tBuff);
        
        /* If minutes mismatch */
        if(tBuff[1] != appData.gpsPkt.min)
        {
            tBuff[0] = appData.gpsPkt.sec;
            tBuff[1] = appData.gpsPkt.min;
            tBuff[2] = appData.gpsPkt.hrs;
            MCP79SetTime(tBuff);
            
            /* Set Date */
            tBuff[0] = 1; //Weekday
            tBuff[1] = appData.gpsPkt.dd;
            tBuff[2] = appData.gpsPkt.mm;
            tBuff[3] = appData.gpsPkt.yy;
            MCP79SetFullDate(tBuff);
        }
    }
}

void NVMAddRec(app_pkt_t *newRec, uint8_t forceUpd)
{
    static app_pkt_t nvmPkt[RECS_PER_PAGE];
    static uint8_t nvmBIndex;

    if(nvmPkt!=NULL)
    {
        memcpy(&nvmPkt[nvmBIndex++], newRec,sizeof(app_pkt_t));
    }

    /* Check if sufficient records are present and write to nv memory */
    if((nvmBIndex == RECS_PER_PAGE) || (forceUpd))
    {
        MemManWriteRecs(&nvmPkt,nvmBIndex);
        nvmBIndex = 0;
    }
}

void DispChgState(void)
{
    /* If USB source not connected */
    if(nrf_gpio_pin_read(PGOOD_PIN))
    {
        /* Turn off Green and Red LEDs */
        nrf_gpio_pin_set(LEDG_PIN);
        nrf_gpio_pin_set(LEDR_PIN);
    }
    else
    {
        /* If not charging, indicates battery is full / NC */
        if(nrf_gpio_pin_read(NCHG_PIN))
        {
            /* Turn on green LED indicating bat full */
            nrf_gpio_pin_set(LEDR_PIN);
            nrf_gpio_pin_clear(LEDG_PIN);
        }
        else
        {
            /* Turn on red LED indicating charging */
            nrf_gpio_pin_set(LEDG_PIN);
            nrf_gpio_pin_clear(LEDR_PIN);
        }
    }
}

/**
 *@}
 **/
