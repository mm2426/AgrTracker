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

#define  APP_QOS                    15
#define  SL_BUFFER_LEN              100

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_drv_uart.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "boards.h"
#include "ifc_struct_defs.h"
#include "ll_ifc_consts.h"
#include "ll_ifc_symphony.h"
#include "ll_ifc_transport_mcu.h"
#include "ubloxm8.h"

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

/** @brief Function to initialize all the onboard peripherals
 */
void InitPeripherals(void);

/** @brief Function starting the internal LFCLK oscillator.
 */
void LfclkConfig(void);

/**
 * @brief Function for initializing real time counter 1.
 * RTC1 is used as a tick timer to generate delays. 
 */
void RTC1Init(void);

/** @brief Function to initialize Lora RX interrupt
 */
void SlIrqInit(void);

/** @brief Function to initialize TWI Master
 */
void TWIM1Init(void);

/** @brief Lora Rx interrupt handler
 */
void SlIrqHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

/** @brief Function to process the symphony link state machine.
 */
void SlProcessComm(void);

/** @brief Variables tracking LORA comm state.
 */
enum sl_states_t slCommState = SL_RESET;
enum sl_states_t slCommNextState = SL_RESET;

/**< Declaring an instance of RTC1 Peripheral. */
const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1);

/* Declaring an instance of TWIM1 Peripheral. */
static const nrf_drv_twi_t twim1 = NRF_DRV_TWI_INSTANCE(1);


uint8_t rtcExpired = 0;
uint8_t slBuffer[SL_BUFFER_LEN], rxIrq = 0;
uint8_t rxDataLen = 0;

uint8_t slTxComp = 0, slRxComp = 0;

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    enum app_states_t appState = APP_READY;
    uint32_t irqFlags;
    uint8_t gpsData[100];
    
    enum ll_state loraState;
    enum ll_tx_state txState;
    enum ll_rx_state rxState;
     
    /* Configure board. */
    InitPeripherals();
    
    while(1)
    {
        ReadGPSRaw(gpsData);
        if(gpsData[0] == '$')
        {
            nrf_delay_ms(1);
        }
        nrf_delay_ms(1000);
    }

    while (true)
    {
        SlProcessComm();
        //__SEV();
        //__WFE();
        
        if(slCommState == SL_READY)
        {
            ll_get_state(&loraState, &txState, &rxState);
            //Read and clear IRQ Flags
            ll_irq_flags(0xFFFFFFFF, &irqFlags);
            //if(irqFlags & IRQ_FLAGS_RX_DONE) //Interrupt received from LL module
            if(rxState == LL_RX_STATE_RECEIVED_MSG)
            {
                slRxComp = 0;
                slCommState = SL_RECEIVING;
                //rxIrq = 0;
            }
            else if(!slTxComp && !slRxComp)
            {
                //To transmit data on LORA
                /*if(ll_message_send_ack(slBuffer, 20)>=0)
                {
    
                }*/
//                slBuffer[0] = 'A';
//                slBuffer[1] = 'B';
//                slBuffer[2] = 'C';
//                slBuffer[3] = 'D';
//                slBuffer[4] = 'E';
//                slBuffer[5] = 'F';
//                slBuffer[6] = 'G';
//                slBuffer[7] = 'H';
//
//                ll_message_send_ack(slBuffer, 8);
                
                //ll_mailbox_request();
                slCommState = SL_TRANSMITTING;
            }
        }

        if(slRxComp)
        {
            //Take Rx Completed action here.
            slRxComp = 0;
        }

        if(slTxComp)
        {
            //Take Tx completed action here. 
            slTxComp = 0;
        }

        nrf_delay_ms(200);
    }
}

/** @brief: Function for handling the RTC1 interrupts.
 * Triggered on COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        //Clear the RTC counter reg
        nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_CLEAR);
        nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_STOP);
        rtcExpired = 1;
    }
}

void SlIrqHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    rxIrq = 1;
}

void InitPeripherals(void)
{
    bsp_board_leds_init();
    LfclkConfig();
    RTC1Init();
    ll_uart_init();
    SlIrqInit();
    TWIM1Init();
    
    //Set GPS Packet Filters
    SetGNRMCFilter();

    //SPI Init

    //Init Accelerometer
    //Init GPS

    //Init All LEDs & GPIOS
    
    //RST pin for LORA / CATM1
    nrf_gpio_cfg_output(COMM_NRST_PIN);
}

void LfclkConfig(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

void RTC1Init(void)
{
    uint32_t err_code;

    //Initialize RTC instance for 1ms Tick
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 32;
    err_code = nrf_drv_rtc_init(&rtc1, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc1);
    
    nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_STOP);
    nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_CLEAR);
}

void SlIrqInit(void)
{
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    uint32_t err_code = NRF_SUCCESS;
    
    err_code = nrf_drv_gpiote_in_init(WAKE_STS_PIN, &config, SlIrqHandler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(WAKE_STS_PIN, true);
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
            nrf_gpio_pin_set(COMM_NRST_PIN);
            nrf_delay_ms(100);
            nrf_gpio_pin_clear(COMM_NRST_PIN);
            slCommState = SL_WAITING;
            slCommNextState = SL_UNINITIALIZED;
            nrf_gpio_pin_set(LED_1);
            rxErrCnt = 0;
            txErrCnt = 0;
            //Set compare channel 0
            nrf_drv_rtc_cc_set(&rtc1, 0, 2000,true);
            nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_START);
            break;
        case SL_UNINITIALIZED:
            //Select UFL antenna
            if(ll_antenna_set(1)>=0)
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
                            //Notify tx completed
                            slTxComp = 1;
                            slCommState = SL_READY;
                            break;
                        case LL_TX_STATE_ERROR:
                            txErrCnt++;
                            //Notify the application that tx failed. 
                            slCommState = SL_READY;
                            break;
                        case LL_TX_STATE_TRANSMITTING:
                            break;
                    }
                }
                else if (moduleStatus == LL_STATE_IDLE_DISCONNECTED)
                {
                        txErrCnt++;
                        //Notify the application that tx failed. 
                        slCommState = SL_READY;
                }
                else if (moduleStatus == LL_STATE_INITIALIZING)
                {
                    //Do not do anything, wait for the status to change. 
                }
                else 
                {
                    //state == LL_STATE_ERROR or NaN. Reset the module.
                    slCommState = SL_ERROR;
                }
            }
            else
            {
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
                    if(ll_retrieve_message(slBuffer, &rxDataLen, NULL, NULL)>=0)
                    {
                        rxErrCnt = 0;
                        //Indicate Rx Success.
                        slRxComp = 1;
                    }
                    else
                    {
                        rxErrCnt++;
                    }
                    slCommState = SL_READY;
                }
                else //state error
                {
                    slCommState = SL_ERROR;
                }
            }
            else
            {
                slCommState = SL_ERROR;
            }
            break;
        case SL_WAITING:
            if(rtcExpired)
            {
                rtcExpired = 0;
                slCommState = slCommNextState;
                nrf_gpio_pin_clear(LED_1);
            }
            break;
        case SL_ERROR:
            //Reset module and try to re-initialize
            slCommState = SL_WAITING;
            slCommNextState = SL_RESET;
            //Set compare channel 0
            nrf_drv_rtc_cc_set(&rtc1, 0, 5000,true);
            nrf_rtc_task_trigger(rtc1.p_reg, NRF_RTC_TASK_START);
            break;
    }
}

/**
 *@}
 **/
