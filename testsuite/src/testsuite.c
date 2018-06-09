#include "testsuite.h"

void AccelerometerTest(void)
{
    int16_t accVals[3] = {};
    float oriVals[3] = {};
    
    LIS3DInit(ADDR_LIS3DSHTR);
    
    while(true)
    {
        nrf_gpio_pin_toggle(LEDR_PIN);
        LIS3DReadAccDataAll(ADDR_LIS3DSHTR, (uint16_t*)accVals);
        GetOrientation(accVals, oriVals);
        nrf_delay_ms(1000);
    }
}

void GPSTest(void)
{
    uint8_t gpsData[100];
    gps_pkt_t pkt;
    
    SetGNRMCFilter();
    
    //ReadGPS(&pkt);

    while(1)
    {
//        memset(gpsData, 0, sizeof(gpsData));
//        ReadGPSRaw(gpsData);
        memset(&pkt, 0, sizeof(pkt));
        ReadGPS(&pkt);
//        if(gpsData[0] == '$')
//        {
//            //nrf_delay_ms(1);
//            nrf_gpio_pin_toggle(LEDB_PIN);
//        }
        if(pkt.status)
        {
            nrf_gpio_pin_toggle(LEDB_PIN);
        }
        nrf_delay_ms(1000);
    }
}

void LEDTest(void)
{
    while(true)
    {
        nrf_gpio_pin_set(LEDR_PIN);
        nrf_gpio_pin_set(LEDG_PIN);
        nrf_gpio_pin_set(LEDB_PIN);
        nrf_delay_ms(500);
        nrf_gpio_pin_clear(LEDR_PIN);
        nrf_gpio_pin_clear(LEDG_PIN);
        nrf_gpio_pin_clear(LEDB_PIN);
        nrf_delay_ms(500);
    }
}

void DigitalInpTest(void)
{
    while(true)
    {
        if(!nrf_gpio_pin_read(SW_USR_PIN))
        {
            while(!nrf_gpio_pin_read(SW_USR_PIN));
            nrf_gpio_pin_toggle(LEDR_PIN);
        }

        if(nrf_gpio_pin_read(PGOOD_PIN))
        {
            nrf_delay_ms(1);
        }
        
        if(nrf_gpio_pin_read(NCHG_PIN))
        {
            nrf_delay_ms(1);
        }

        nrf_delay_ms(100);
    }
}

void MemoryTest(void)
{
    uint8_t buff[10], rBuff[10];
    uint16_t manId;
    
    W25Q32CmdReset();
    
    W25Q32CmdEraseSector(0x00000000,1);

    buff[0] = 0x12;
    buff[1] = 0x34;
    buff[2] = 0x56;
    buff[3] = 0x78;
    buff[4] = 0x9A;
    buff[5] = 0xBC;
    buff[6] = 0xCD;
    buff[7] = 0xEF;
    buff[8] = 'G';
    buff[9] = 'H';

    W25Q32CmdProgPage(0x00000000,buff,10,1);

    while(true)
    {
        memset(rBuff, 0, 10);
        W25Q32CmdReadData(0x00000000, rBuff, 10);
        if(memcmp(buff,rBuff,10))
        {
            nrf_gpio_pin_set(LEDR_PIN);
        }
        else
        {
            nrf_gpio_pin_toggle(LEDR_PIN);   
        }
        nrf_delay_ms(1000);
    }
}

void MemManTest(void)
{
    app_pkt_t dPkt[2], rPkt[12];
    uint8_t rLen;
    
    dPkt[0].header = APP_PKT_HDR;
    dPkt[0].footer = APP_PKT_FTR;
    dPkt[0].batV = 56;
    dPkt[0].gpsPkt.dd = 0x01;
    dPkt[0].gpsPkt.mm = 0x06;
    dPkt[0].gpsPkt.yy = 0x18;
    dPkt[0].gpsPkt.hrs = 0x17;
    dPkt[0].gpsPkt.min = 0x26;
    dPkt[0].gpsPkt.sec = 0x43;
    dPkt[0].gpsPkt.latDeg = 56;
    dPkt[0].gpsPkt.lonDeg = 143;
    dPkt[0].gpsPkt.latMins = 33.43f;
    dPkt[0].gpsPkt.lonMins = 25.87f;
    dPkt[0].gpsPkt.status = 1;

    dPkt[1].header = APP_PKT_HDR;
    dPkt[1].footer = APP_PKT_FTR;
    dPkt[1].batV = 78;
    dPkt[1].gpsPkt.dd = 0x05;
    dPkt[1].gpsPkt.mm = 0x06;
    dPkt[1].gpsPkt.yy = 0x18;
    dPkt[1].gpsPkt.hrs = 0x18;
    dPkt[1].gpsPkt.min = 0x43;
    dPkt[1].gpsPkt.sec = 0x27;
    dPkt[1].gpsPkt.latDeg = 42;
    dPkt[1].gpsPkt.lonDeg = 52;
    dPkt[1].gpsPkt.latMins = 42.91f;
    dPkt[1].gpsPkt.lonMins = 27.84f;
    dPkt[1].gpsPkt.status = 1;

    MemManInit();
    //MemManWriteRecs(&dPkt,2);
    
    while(1)
    {
        memset(&rPkt, 0, sizeof(rPkt));
        if(!MemManPeekPage((uint8_t *)&rPkt, &rLen))
        {
            nrf_gpio_pin_set(LEDG_PIN);
        }
        else
        {
            nrf_gpio_pin_clear(LEDG_PIN);
            MemManRstPeekPtr();
        }
        
        if(memcmp(&dPkt,&rPkt,sizeof(dPkt)))
        {
            nrf_gpio_pin_set(LEDR_PIN);   
        }
        else
        {
            nrf_gpio_pin_toggle(LEDR_PIN);
        }
        nrf_delay_ms(1000);
    }
}

void RTCTest(void)
{
    uint8_t buff[10]; 
    
    /* Set time compulsorily while you transact with the RTC for 
    the first time to start the oscillator. */

    while(true)
    {
        nrf_gpio_pin_toggle(LEDG_PIN);
        MCP79GetTime(buff);
        buff[0] = buff[0] & 0x7F;
        nrf_delay_ms(1000);
    }
}

void GaugeTest(void)
{
    uint16_t voltage, rsoc;
    
    LC709Init();
    while(true)
    {
        nrf_gpio_pin_toggle(LEDB_PIN);
        voltage = LC709GetVoltage();
        rsoc = LC709GetRSOC();
        nrf_delay_ms(1000);
    }
}

void ConsoleTest(void)
{
    uint8_t buff[12];  
    memcpy(buff, "ABCDEFGH12\r\n", 12);

    /* Select Console Port */
    nrf_gpio_pin_clear(SEL_MUX_PIN);
    
    while(true)
    {
        nrf_gpio_pin_toggle(LEDG_PIN);
        transport_write(buff, 12);
        nrf_delay_ms(1000);
    }
}

void LoraTest(void)
{
    /* Select LORA Port */
    nrf_gpio_pin_set(SEL_MUX_PIN);
}

static const nrf_drv_rtc_t rtc1Inst = NRF_DRV_RTC_INSTANCE(1);
uint8_t ctrTime;
/** @brief: Function for handling the RTC1 interrupts.
 * Triggered on COMPARE0 match.
 */
static void rtc1_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        //Clear the RTC counter reg
        nrf_rtc_task_trigger(rtc1Inst.p_reg, NRF_RTC_TASK_CLEAR);
        nrf_drv_rtc_cc_set(&rtc1Inst, 0, ctrTime,true);
        //nrf_rtc_task_trigger(rtc1Inst.p_reg, NRF_RTC_TASK_STOP);
    }
}

uint8_t swPressed = 0;
static void SWIrqHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    swPressed = 1;
}

void LowPowerTest(void)
{
    uint32_t err_code;
    
    /* Uninitialize RTC component as it is already initialized in app code. */
    nrf_drv_rtc_uninit(&rtc1Inst);

    /* Initialize RTC instance for 125ms (8Hz) Tick */
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc1Inst, &config, rtc1_handler);
    APP_ERROR_CHECK(err_code);

    /* Power on RTC instance */
    nrf_drv_rtc_enable(&rtc1Inst);
    
    nrf_rtc_task_trigger(rtc1Inst.p_reg, NRF_RTC_TASK_STOP);
    nrf_rtc_task_trigger(rtc1Inst.p_reg, NRF_RTC_TASK_CLEAR);
    
    ctrTime = 80;
    /* Set compare channel 0 */
    nrf_drv_rtc_cc_set(&rtc1Inst, 0, ctrTime,true);
    nrf_rtc_task_trigger(rtc1Inst.p_reg, NRF_RTC_TASK_START);
    
    /* Initialize User Switch Interrupt */
    nrf_drv_gpiote_in_config_t config2 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    
    err_code = nrf_drv_gpiote_in_init(SW_USR_PIN, &config2, SWIrqHandler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(SW_USR_PIN, true);

    /* Should wake up on events from RTC1 & USER_SW, else sleep */
    while(1)
    {
        
        nrf_gpio_pin_toggle(LEDR_PIN);
        if(swPressed)
        {
            swPressed = 0;
        }
        // Wait for an event.
        __WFE();
        // Clear the internal event register.
        __SEV();
        __WFE();
    }
}

