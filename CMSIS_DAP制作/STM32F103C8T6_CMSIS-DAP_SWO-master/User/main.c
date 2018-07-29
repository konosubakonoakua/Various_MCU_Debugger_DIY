//=====================================
//CMSIS-DAP v2.0 for Bluepill board
//--------based on x893 source code
//--------2018-07-24 by RadioOperator
//=====================================

#include <stdio.h>

#include <RTL.h>
#include <rl_usb.h>
#include <stm32f10x.h>

#define  __NO_USB_LIB_C
#include "usb_config.c"

#include "DAP_config.h"
#include "DAP.h"

#if defined ( BLUEPILL )
#if defined ( SWD_REMAP )
#warning "BLUEPILL board: using Remapped SWD/SWC port, SWO/TDO-PB7, nRESET-PB6, TDI-PB5"
#else
#warning "BLUEPILL board: using SWD/TMS-PB9, SWC/TCK-PB8, SWO/TDO-PB7, nRESET-PB6, TDI-PB5"
#endif
#endif

void BoardInit(void);
uint8_t usbd_hid_process(void);
void Delayms(uint32_t delay);

extern void PIN_nRESET_OUT(uint8_t bit);

#if (USBD_CDC_ACM_ENABLE == 1)
extern int32_t USBD_CDC_ACM_PortInitialize(void);
extern void CDC_ACM_UART_to_USB(void);
#endif

uint8_t u8SysTick_Counter = 3;
uint8_t u8LedMode = 0;

#define LED_FLASH_ON()    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk  //turn-on SysTick, LED in flashing mode.
#define LED_FLASH_OFF()   SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk //turn-off SysTick

#if defined ( BLUEPILL ) //Bluepill Board

void LedConnectedOn(void)     { LED_CONNECTED_PORT->BRR  = LED_CONNECTED_MASK;  } //Low active
void LedConnectedOff(void)    { LED_CONNECTED_PORT->BSRR = LED_CONNECTED_MASK;  }
void LedConnectedToggle(void) { LED_CONNECTED_PORT->ODR ^= LED_CONNECTED_MASK;  }

void LedRunningOn(void)       { LED_RUNNING_PORT->BRR    = LED_RUNNING_MASK;    } //Low active
void LedRunningOff(void)      { LED_RUNNING_PORT->BSRR   = LED_RUNNING_MASK;    }
void LedRunningToggle(void)   { LED_RUNNING_PORT->ODR   ^= LED_RUNNING_MASK;    }

#else //all other Board

void LedConnectedOn(void)     { LED_CONNECTED_PORT->BSRR = LED_CONNECTED_MASK;  } //High active
void LedConnectedOff(void)    { LED_CONNECTED_PORT->BRR  = LED_CONNECTED_MASK;  }
void LedConnectedToggle(void) { LED_CONNECTED_PORT->ODR ^= LED_CONNECTED_MASK;  }

void LedRunningOn(void)       { LED_RUNNING_PORT->BSRR   = LED_RUNNING_MASK;    } //High active
void LedRunningOff(void)      { LED_RUNNING_PORT->BRR    = LED_RUNNING_MASK;    }
void LedRunningToggle(void)   { LED_RUNNING_PORT->ODR   ^= LED_RUNNING_MASK;    }

#endif //#if defined ( BLUEPILL )

const GPIO_InitTypeDef INIT_LED_CONNECTED = {
  LED_CONNECTED_MASK,
  GPIO_Speed_50MHz,
  GPIO_Mode_Out_PP
};

const GPIO_InitTypeDef INIT_LED_RUNNING = {
  LED_RUNNING_MASK,
  GPIO_Speed_50MHz,
  GPIO_Mode_Out_PP
};

void LEDS_SETUP (void)
{
#if defined LED_CONNECTED_RCC
  RCC->APB2ENR |= LED_CONNECTED_RCC;
  LED_CONNECTED_PORT->BRR = LED_CONNECTED_MASK;
  GPIO_INIT(LED_CONNECTED_PORT, INIT_LED_CONNECTED);
#endif
  
#if defined LED_RUNNING_RCC
  RCC->APB2ENR |= LED_RUNNING_RCC;
  LED_RUNNING_PORT->BRR = LED_RUNNING_MASK;
  GPIO_INIT(LED_RUNNING_PORT, INIT_LED_RUNNING);  
#endif
  LedRunningOff();
}

void LedConnectedOut(uint16_t bit)
{
  LedRunningOff();
  u8LedMode &= ~0x01;
  
  LED_FLASH_ON();
}

void LedRunningOut(uint16_t bit)
{
  LedConnectedOn();
  u8LedMode |= 0x01;
  
  if (bit & 1)
  {
    LED_FLASH_OFF();
    LedRunningOn();
  }
  else
  {
    LED_FLASH_ON();
    LedRunningOff();
  }
}

void SysTick_Init(void)
{
  SysTick_Config(1800000); // =200ms, 1800000 ticks
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //Freq = 72/8 = 9MHz
  
  LED_FLASH_OFF(); //turn-off SysTick
}

void SysTick_Handler(void)
{
  u8SysTick_Counter--;

  //Connected LED
  if (u8LedMode & 0x02)       //Connected LED: 200ms on/off for CDC, fast
  {
    u8LedMode &= ~0x02;
    
    if (u8SysTick_Counter & 0x01) {
      LedConnectedOn();
    }
    else
    LedConnectedOff();
  }
  else
  {     
    if ((u8SysTick_Counter & 0x07) == 0) //Connected LED: 200ms on, 1400ms off, slower
    LedConnectedOn();
    else
    LedConnectedOff();
  }
  
  //Running LED
  if (u8LedMode & 0x01)           //Running LED: 200ms on, 600ms off
  {
    if ((u8SysTick_Counter & 0x03) == 0)
    LedRunningOn();
    else
    LedRunningOff();
  }
}

CoreDescriptor_t * pCoreDescriptor;
const CoreDescriptor_t CoreDescriptor = {
  &LedConnectedOut,
  &LedRunningOut,
};

void UserAppInit(CoreDescriptor_t *core)
{
  pCoreDescriptor = core;
  DAP_Setup();
}

void UserAppAbort(void)
{
  DAP_TransferAbort = 1;
}

UserAppDescriptor_t * pUserAppDescriptor = NULL;
UserAppDescriptor_t UserAppDescriptor = {
  &UserAppInit,
  &DAP_ProcessCommand,
  &UserAppAbort
};

//=============================================================================
//==main=======================================================================
//=============================================================================
int main(void)
{
  SystemCoreClockUpdate();
  BoardInit();  
  SysTick_Init(); //for LED flash
  
  LedConnectedOn();
  if (UserAppDescriptor.UserInit != NULL)
  {
    pUserAppDescriptor = &UserAppDescriptor;
    pUserAppDescriptor->UserInit((CoreDescriptor_t *)&CoreDescriptor);
  }
  Delayms(10);

  // USB Device Initialization and connect
  usbd_init();
  usbd_connect(__TRUE);
  
  while (!usbd_configured())  // Wait for USB Device to configure
  {
    LedConnectedOff();
    Delayms(10);
  }
  LED_FLASH_ON();
  
  Delayms(100);       // Wait for 100ms
  
#if (USBD_CDC_ACM_ENABLE == 1)
  USBD_CDC_ACM_PortInitialize(); //initial CDC UART port
#endif
  
  //-------------------------------------
  while (1) //Main loop
  {
    usbd_hid_process(); //DAP process

#if (USBD_CDC_ACM_ENABLE == 1)
    CDC_ACM_UART_to_USB(); //CDC, UART to USB
#endif
  }
}
//=======End of main===========================================================

extern uint32_t __Vectors;

void HardFault_Handler(void);
void NMI_Handler(void)      __attribute((alias("HardFault_Handler")));
void MemManage_Handler(void)  __attribute((alias("HardFault_Handler")));
void BusFault_Handler(void)   __attribute((alias("HardFault_Handler")));
void UsageFault_Handler(void) __attribute((alias("HardFault_Handler")));
void SVC_Handler(void)      __attribute((alias("HardFault_Handler")));
void DebugMon_Handler(void)   __attribute((alias("HardFault_Handler")));
void PendSV_Handler(void)   __attribute((alias("HardFault_Handler")));

void HardFault_Handler(void)
{
  __disable_irq();
  __set_MSP(__Vectors);
  LEDS_SETUP();
  {
    register int count;
    for (count = 0; count < 5; count++)
    {
      LedRunningOn();
      Delayms(100);
      LedRunningOff();

      LedConnectedOn();
      Delayms(100);
      LedConnectedOff();

      Delayms(1000);
    }
  }
  NVIC_SystemReset();
}

/* Control USB connecting via SW  */
#ifdef PIN_USB_CONNECT_PORT
const GPIO_InitTypeDef INIT_PIN_USB_CONNECT = {
  PIN_USB_CONNECT_MASK,
  GPIO_Speed_50MHz,
  PIN_USB_MODE
};
#endif

void PORT_USB_CONNECT_SETUP(void)
{
#ifdef PIN_USB_CONNECT_PORT
  RCC->APB2ENR |= PIN_USB_CONNECT_RCC;
  PIN_USB_CONNECT_OFF();
  GPIO_INIT(PIN_USB_CONNECT_PORT, INIT_PIN_USB_CONNECT);
#endif
}

#if ( DAP_SWD != 0 )

#if ( DAP_JTAG != 0 )
const GPIO_InitTypeDef INIT_SWD_TDI = {
  PIN_TDI_MASK,
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_IN_FLOATING
};
#endif //#if ( DAP_JTAG != 0 )

const GPIO_InitTypeDef INIT_SWD_TDO = {
  PIN_TDO_MASK,
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_IN_FLOATING
};

const GPIO_InitTypeDef INIT_SWD_SWCLK_SWDIO = {
  PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK,
  GPIO_Speed_50MHz,
  GPIO_Mode_Out_PP
};

/** Setup SWD I/O pins: SWCLK, SWDIO, and nRESET.
  Configures the DAP Hardware I/O pins for Serial Wire Debug (SWD) mode:
  - SWCLK, SWDIO, nRESET to output mode and set to default high level.
  - TDI, TDO, nTRST to HighZ mode (pins are unused in SWD mode).
*/ 
void PORT_SWD_SETUP()
{
  PIN_SWCLK_TCK_PORT->BSRR = (PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK);

#if ( DAP_JTAG != 0 )
  GPIO_INIT(PIN_TDI_PORT,       INIT_SWD_TDI);
#endif  
  GPIO_INIT(PIN_TDO_PORT,       INIT_SWD_TDO);
  
#if defined ( BLUEPILL ) && defined ( SWD_REMAP )
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
#endif
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  GPIO_INIT(PIN_SWCLK_TCK_PORT, INIT_SWD_SWCLK_SWDIO);
  
  PIN_nRESET_OUT(0U);
  Delayms(100);
  PIN_nRESET_OUT(1U);
}

#endif //#if ( DAP_SWD != 0 )

#if ( DAP_JTAG != 0 )

const GPIO_InitTypeDef INIT_JTAG_TDO = {
  PIN_TDO_MASK,
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_IN_FLOATING
};

const GPIO_InitTypeDef INIT_JTAG_TCK_TMS = {
  PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK,
  GPIO_Speed_50MHz,
  GPIO_Mode_Out_PP
};

const GPIO_InitTypeDef INIT_JTAG_TDI = {
  PIN_TDI_MASK,
  GPIO_Speed_50MHz,
  GPIO_Mode_Out_PP
};

/** Setup JTAG I/O pins: TCK, TMS, TDI, TDO, nTRST, and nRESET.
  Configures the DAP Hardware I/O pins for JTAG mode:
  - TCK, TMS, TDI, nTRST, nRESET to output mode and set to high level.
  - TDO to input mode.
*/ 
void PORT_JTAG_SETUP()
{
  PIN_SWCLK_TCK_PORT->BSRR = PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK;
  PIN_TDI_PORT      ->BSRR = PIN_TDI_MASK;
  
  GPIO_INIT(PIN_TDO_PORT,       INIT_JTAG_TDO);
  GPIO_INIT(PIN_TDI_PORT,       INIT_JTAG_TDI);
  
#if defined ( BLUEPILL ) && defined ( SWD_REMAP )
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
#endif
  
  GPIO_INIT(PIN_SWCLK_TCK_PORT, INIT_JTAG_TCK_TMS);
  
  PIN_nRESET_OUT(0U);
  Delayms(100);
  PIN_nRESET_OUT(1U);
}

const GPIO_InitTypeDef INIT_OFF_TDI = {
  PIN_TDI_MASK,
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_IN_FLOATING
};

#endif //#if ( DAP_JTAG != 0 )

const GPIO_InitTypeDef INIT_OFF_TCK_TMS = {
  PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK,
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_IN_FLOATING
};

const GPIO_InitTypeDef INIT_OFF_TDO = {
  PIN_TDO_MASK,
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_IN_FLOATING
};

const GPIO_InitTypeDef INIT_OFF_nRESET = {
  (PIN_nRESET_MASK),
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_IPU
};

/** Disable JTAG/SWD I/O Pins.
Disables the DAP Hardware I/O pins which configures:
- TCK/SWCLK, TMS/SWDIO, TDI, TDO, nTRST, nRESET to High-Z mode.
*/
void PORT_OFF()
{
#if defined ( BLUEPILL ) && defined ( SWD_REMAP )
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
#endif
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  GPIO_INIT(PIN_SWCLK_TCK_PORT, INIT_OFF_TCK_TMS);
  
#if ( DAP_JTAG != 0 )
  GPIO_INIT(PIN_TDI_PORT,       INIT_OFF_TDI);
  GPIO_INIT(PIN_TDO_PORT,       INIT_OFF_TDO);
#endif
  
  GPIO_INIT(PIN_nRESET_PORT,    INIT_OFF_nRESET);
}

const GPIO_InitTypeDef INIT_PINS_A = {
  ( GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 |
  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
  //    GPIO_Pin_11 | GPIO_Pin_12 |   // USB pins
  //    GPIO_Pin_13 | GPIO_Pin_14 |   // SWD pins
  GPIO_Pin_15
  ),
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_AIN
};

const GPIO_InitTypeDef INIT_PINS_B = {
  GPIO_Pin_All,
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_AIN
};

const GPIO_InitTypeDef INIT_PINS_C = {
  (GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15),
  (GPIOSpeed_TypeDef)0,
  GPIO_Mode_AIN
};

void BoardInit(void)
{
#if defined ( BLUEPILL ) && defined ( SWD_REMAP )
  //release JTAG-SWD Pins for GPIO
  RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;
  AFIO->MAPR   |=  AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
#endif
  RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;
  AFIO->MAPR   |=  AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  // Enable GPIO clock
  RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);
  // Reset all GPIO pins, except USB/SWD port
  GPIO_INIT(GPIOA, INIT_PINS_A);
  GPIO_INIT(GPIOB, INIT_PINS_B);
//  GPIO_INIT(GPIOC, INIT_PINS_C);
  
  LEDS_SETUP();
}

void USBD_Error_Event(void)
{
  LedConnectedOn();
  LedRunningOn();

  usbd_connect(__FALSE);
  usbd_reset_core();

  HardFault_Handler();
}

//-----Soft reset + Hard reset-------------------------------------------------
#define PIN_SWCLK_SET PIN_SWCLK_TCK_SET
#define PIN_SWCLK_CLR PIN_SWCLK_TCK_CLR

#define RST_CLOCK_CYCLE()                \
  PIN_SWCLK_CLR();                       \
  PIN_DELAY();                           \
  PIN_SWCLK_SET();                       \
  PIN_DELAY()

#define RST_WRITE_BIT(bit)               \
  PIN_SWDIO_OUT(bit);                    \
  PIN_SWCLK_CLR();                       \
  PIN_DELAY();                           \
  PIN_SWCLK_SET();                       \
  PIN_DELAY()

#define RST_READ_BIT(bit)                \
  PIN_SWCLK_CLR();                       \
  PIN_DELAY();                           \
  bit = PIN_SWDIO_IN();                  \
  PIN_SWCLK_SET();                       \
  PIN_DELAY()

#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)

uint8_t RST_Transfer(uint32_t request, uint32_t data)
{
  uint32_t ack;                                                                 \
  uint32_t bit;                                                                 \
  uint32_t val;                                                                 \
  uint32_t parity;                                                              \
  uint32_t n;                                                                   \
  \
  /* Packet Request */                                                          \
  parity = 0U;                                                                  \
  RST_WRITE_BIT(1U);                     /* Start Bit */                        \
  bit = request >> 0;                                                           \
  RST_WRITE_BIT(bit);                    /* APnDP Bit */                        \
  parity += bit;                                                                \
  bit = request >> 1;                                                           \
  RST_WRITE_BIT(bit);                    /* RnW Bit */                          \
  parity += bit;                                                                \
  bit = request >> 2;                                                           \
  RST_WRITE_BIT(bit);                    /* A2 Bit */                           \
  parity += bit;                                                                \
  bit = request >> 3;                                                           \
  RST_WRITE_BIT(bit);                    /* A3 Bit */                           \
  parity += bit;                                                                \
  RST_WRITE_BIT(parity);                 /* Parity Bit */                       \
  RST_WRITE_BIT(0U);                     /* Stop Bit */                         \
  RST_WRITE_BIT(1U);                     /* Park Bit */                         \
  \
  /* Turnaround */                                                              \
  PIN_SWDIO_OUT_DISABLE();                                                      \
  for (n = DAP_Data.swd_conf.turnaround; n; n--) {                              \
    RST_CLOCK_CYCLE();                                                          \
  }                                                                             \
  \
  /* Acknowledge response */                                                    \
  RST_READ_BIT(bit);                                                            \
  ack  = bit << 0;                                                              \
  RST_READ_BIT(bit);                                                            \
  ack |= bit << 1;                                                              \
  RST_READ_BIT(bit);                                                            \
  ack |= bit << 2;                                                              \
  \
  /* Data transfer */                                                           \
  /* Turnaround */                                                              \
  for (n = DAP_Data.swd_conf.turnaround; n; n--) {                              \
    RST_CLOCK_CYCLE();                                                          \
  }                                                                             \
  PIN_SWDIO_OUT_ENABLE();                                                       \
  /* Write data */                                                              \
  val = data;                                                                   \
  parity = 0U;                                                                  \
  for (n = 32U; n; n--) {                                                       \
    RST_WRITE_BIT(val);              /* Write WDATA[0:31] */                    \
    parity += val;                                                              \
    val >>= 1;                                                                  \
  }                                                                             \
  RST_WRITE_BIT(parity);             /* Write Parity Bit */                     \
  PIN_SWDIO_OUT_ENABLE();                                                       \
  PIN_SWDIO_OUT(1U);                                                            \
  return ((uint8_t)ack);                                                        \
}

void vResetTarget(uint8_t bit)
{
  uint32_t i;
  //soft-reset for Cortex-M
  RST_Transfer(0x00000CC5, 0xE000ED0C); //set AIRCR address
  for (i=0; i<100; i++);
  RST_Transfer(0x00000CDD, 0x05FA0007); //set RESET data
  for (i=0; i<100; i++);
  RST_Transfer(0x00000CC5, 0xE000ED0C); //repeat
  for (i=0; i<100; i++);
  RST_Transfer(0x00000CDD, 0x05FA0007);
  
  if (bit & 1)  PIN_nRESET_HIGH();
  else          PIN_nRESET_LOW();
}


//=============END=====================
