#ifndef __TESTSUITE_H__
#define __TESTSUITE_H__
  

  #include "nrf_delay.h"
  #include "boards.h"
  #include "nrf_drv_uart.h"
  #include "nrf_drv_twi.h"
  #include "nrf_drv_spi.h"
  #include "nrf_drv_clock.h"
  #include "nrf_drv_rtc.h"
  #include "nrf_drv_gpiote.h"
  #include "ifc_struct_defs.h"
  #include "ll_ifc_consts.h"
  #include "ll_ifc_symphony.h"
  #include "ll_ifc_transport_mcu.h"
  #include "ubloxm8.h"
  #include "lis3dshtr.h"
  #include "mcp7940m.h"
  #include "w25q32.h"
  #include "lc709203f.h"
  
  void AccelerometerTest(void);
  void GPSTest(void);
  void LEDTest(void);
  void DigitalInpTest(void);
  void MemoryTest(void);
  void RTCTest(void);
  void GaugeTest(void);
  void ConsoleTest(void);
  void LoraTest(void);
#endif