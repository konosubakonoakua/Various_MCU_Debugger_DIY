/**************************************************************************//**
 * @file     DAP_config.h
 * @brief    CMSIS-DAP Configuration File for STM32F103C6/8/B
 * @version  V1.00
 * @date     02. Oct 2012
 *
 * @note
 * Copyright (C) 2012 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>

typedef const struct
{
	void	(* LedConnected)	(uint16_t);
	void	(* LedRunning)		(uint16_t);
} CoreDescriptor_t;

extern const CoreDescriptor_t * pCoreDescriptor;

typedef const struct
{
	void		(* UserInit)	(CoreDescriptor_t * core);
	uint32_t	(* UserProcess)	(uint8_t *, uint8_t *);
	void		(* UserAbort)	(void);
} UserAppDescriptor_t;

#if !defined ( BOARD_V1      )	\
 && !defined ( BOARD_V2      )	\
 && !defined ( STLINK_V20    )	\
 && !defined ( STLINK_V21    )	\
 && !defined ( BOARD_STM32RF )
	#error "Board undefined"
#endif

#if   defined( __GNUC__ )
	/*	With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
		set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#elif defined( __CC_ARM )
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#else
	#error "Unknown compiler"
#endif

#if   defined( USE_DEBUG )
	#define DEBUG(...)	printf(__VA_ARGS__)
	#define INFO(...)	printf(__VA_ARGS__)
	#define ERROR(...)	printf(__VA_ARGS__)
#elif defined( USE_INFO )
	#define DEBUG(...)
	#define INFO(...)	printf(__VA_ARGS__)
	#define ERROR(...)	printf(__VA_ARGS__)
#elif defined( USE_ERROR )
	#define DEBUG(...)
	#define INFO(...)
	#define ERROR(...)	printf(__VA_ARGS__)
#else
	#define DEBUG(...)
	#define INFO(...)
	#define ERROR(...)
#endif

//**************************************************************************************************
/** 
\defgroup DAP_Config_Debug_gr CMSIS-DAP Debug Unit Information
\ingroup DAP_ConfigIO_gr 
@{
Provides definitions about:
 - Definition of Cortex-M processor parameters used in CMSIS-DAP Debug Unit.
 - Debug Unit communication packet size.
 - Debug Access Port communication mode (JTAG or SWD).
 - Optional information about a connected Target Device (for Evaluation Boards).
*/

#include <stm32f10x.h>

/// Processor Clock of the Cortex-M MCU used in the Debug Unit.
/// This value is used to calculate the SWD/JTAG clock speed.
#define CPU_CLOCK				SystemCoreClock		///< Specifies the CPU Clock in Hz

/// Number of processor cycles for I/O Port write operations.
/// This value is used to calculate the SWD/JTAG clock speed that is generated with I/O
/// Port write operations in the Debug Unit by a Cortex-M MCU. Most Cortex-M processors
/// requrie 2 processor cycles for a I/O Port Write operation.  If the Debug Unit uses
/// a Cortex-M0+ processor with high-speed peripheral I/O only 1 processor cycle might be 
/// requrired.
#define IO_PORT_WRITE_CYCLES	2               ///< I/O Cycles: 2=default, 1=Cortex-M0+ fast I/0

/// Indicate that Serial Wire Debug (SWD) communication mode is available at the Debug Access Port.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#define DAP_SWD                 1               ///< SWD Mode:  1 = available, 0 = not available

/// Indicate that JTAG communication mode is available at the Debug Port.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#if defined ( BOARD_STM32RF )	\
 || defined ( STLINK_V20 )		\
 || defined ( STLINK_V21 )
	#define DAP_JTAG			0				///< JTAG Mode: 1 = available, 0 = not available.
#else
	#define DAP_JTAG			1				///< JTAG Mode: 1 = available, 0 = not available.
#endif

/// Configure maximum number of JTAG devices on the scan chain connected to the Debug Access Port.
/// This setting impacts the RAM requirements of the Debug Unit. Valid range is 1 .. 255.
#define DAP_JTAG_DEV_CNT        8               ///< Maximum number of JTAG devices on scan chain

/// Default communication mode on the Debug Access Port.
/// Used for the command \ref DAP_Connect when Port Default mode is selected.
#define DAP_DEFAULT_PORT        1               ///< Default JTAG/SWJ Port Mode: 1 = SWD, 2 = JTAG.

/// Default communication speed on the Debug Access Port for SWD and JTAG mode.
/// Used to initialize the default SWD/JTAG clock frequency.
/// The command \ref DAP_SWJ_Clock can be used to overwrite this default setting.
#define DAP_DEFAULT_SWJ_CLOCK   1000000         ///< Default SWD/JTAG clock frequency in Hz.

/// Maximum Package Size for Command and Response data.
/// This configuration settings is used to optimized the communication performance with the
/// debugger and depends on the USB peripheral. Change setting to 1024 for High-Speed USB.
#define DAP_PACKET_SIZE			64				///< USB: 64 = Full-Speed, 1024 = High-Speed.

/// Maximum Package Buffers for Command and Response data.
/// This configuration settings is used to optimized the communication performance with the
/// debugger and depends on the USB peripheral. For devices with limited RAM or USB buffer the
/// setting can be reduced (valid range is 1 .. 255). Change setting to 4 for High-Speed USB.
#define DAP_PACKET_COUNT		64				///< Buffers: 64 = Full-Speed, 4 = High-Speed.


/// Debug Unit is connected to fixed Target Device.
/// The Debug Unit may be part of an evaluation board and always connected to a fixed
/// known device.  In this case a Device Vendor and Device Name string is stored which
/// may be used by the debugger or IDE to configure device parameters.
#define TARGET_DEVICE_FIXED     0               ///< Target Device: 1 = known, 0 = unknown;

#if TARGET_DEVICE_FIXED
	#define TARGET_DEVICE_VENDOR    ""		///< String indicating the Silicon Vendor
	#define TARGET_DEVICE_NAME      ""		///< String indicating the Target Device
#endif

///@}

#define GPIO_INIT(port, data)	GPIO_Init(port, (GPIO_InitTypeDef *)&data)
#define PIN_MODE_MASK(pin)		(((uint32_t)0x0F) << ((pin) << 2))
#define PIN_MODE(mode,pin)		(((uint32_t)mode) << ((pin) << 2))
#define PIN_MASK(pin)			(((uint16_t)0x01) << (pin))

typedef enum Pin_e {
	PA = 0x00, PA0 = 0x00, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
	PB = 0x10, PB0 = 0x10, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
	PC = 0x20, PC0 = 0x20, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
	PD = 0x30, PD0 = 0x30, PD1, PD2,
} Pin_t;

// USART Port and I/O Pins

#if   defined ( BOARD_V1 )	\
 ||   defined ( BOARD_V2 )	\
 ||   defined ( BOARD_STM32RF )

	#define USART_CLOCK(state)		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, state)
	#define USART_GPIO_CLOCK(state)	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, state)
	#define USART_REMAP()			/* GPIO_PinRemapConfig(..., ENABLE) */

	#define USART_PORT			USART1
	#define USART_GPIO			GPIOA
	#define USART_TX_PIN		GPIO_Pin_9
	#define USART_RX_PIN		GPIO_Pin_10
	#define USART_IRQn			USART1_IRQn
	#define USART_IRQHandler	USART1_IRQHandler
	#define USART_BUFFER_SIZE	(256)	/*	Size of Receive and Transmit buffers MUST BE 2^n */

#elif defined ( STLINK_V20 )	\
  ||  defined ( STLINK_V21 )

	#define USART_CLOCK(state)		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, state)
	#define USART_REMAP()			/* GPIO_PinRemapConfig(..., ENABLE) */

	#define USART_PORT			USART2
	#define USART_GPIO			GPIOA
	#define USART_TX_PIN		GPIO_Pin_2
	#define USART_RX_PIN		GPIO_Pin_3
	#define USART_IRQn			USART2_IRQn
	#define USART_IRQHandler	USART2_IRQHandler
	#define USART_BUFFER_SIZE	(256)	/*	Size of Receive and Transmit buffers MUST BE 2^n */

#else

	#warning "USART Port not define"

#endif

// USB Connect Pull-Up

#if   defined ( BOARD_V1 )	\
 ||   defined ( BOARD_V2 )

	#define PIN_USB_CONNECT_RCC		RCC_APB2ENR_IOPAEN
	#define PIN_USB_CONNECT_PORT    GPIOA
	#define PIN_USB_CONNECT_PIN		8
	#define PIN_USB_CONNECT         PIN_MASK(PIN_USB_CONNECT_PIN)
	#define PIN_USB_MODE			GPIO_Mode_Out_PP
	#define PIN_USB_CONNECT_ON()	PIN_USB_CONNECT_PORT->BSRR = PIN_USB_CONNECT
	#define PIN_USB_CONNECT_OFF()	PIN_USB_CONNECT_PORT->BRR  = PIN_USB_CONNECT

#elif defined ( STLINK_V20 )

	#define PIN_USB_CONNECT_ON()
	#define PIN_USB_CONNECT_OFF()

#elif defined ( STLINK_V21 )

	#define PIN_USB_CONNECT_RCC		RCC_APB2ENR_IOPAEN
	#define PIN_USB_CONNECT_PORT    GPIOA
	#define PIN_USB_CONNECT_PIN		15
	#define PIN_USB_CONNECT         PIN_MASK(PIN_USB_CONNECT_PIN)
	#define PIN_USB_MODE			GPIO_Mode_Out_OD
	#define PIN_USB_CONNECT_ON()	PIN_USB_CONNECT_PORT->BSRR  = PIN_USB_CONNECT
	#define PIN_USB_CONNECT_OFF()	PIN_USB_CONNECT_PORT->BRR = PIN_USB_CONNECT

#elif defined ( BOARD_STM32RF )

	// USB Connect Pull-Up
	#define PIN_USB_CONNECT_RCC		RCC_APB2ENR_IOPBEN
	#define PIN_USB_CONNECT_PORT    GPIOB
	#define PIN_USB_CONNECT_PIN		5
	#define PIN_USB_CONNECT         PIN_MASK(PIN_USB_CONNECT_PIN)
	#define PIN_USB_MODE			GPIO_Mode_Out_PP
	#define PIN_USB_CONNECT_ON()	PIN_USB_CONNECT_PORT->BSRR = PIN_USB_CONNECT
	#define PIN_USB_CONNECT_OFF()	PIN_USB_CONNECT_PORT->BRR  = PIN_USB_CONNECT

#endif

#if   defined ( BOARD_V1 )

	// SWDIO/TMS Pin
	#define PIN_SWDIO_TMS_PORT      GPIOA
	#define PIN_SWDIO_TMS_PIN		2

	// SWCLK/TCK Pin
	#define PIN_SWCLK_TCK_PORT		GPIOA
	#define PIN_SWCLK_TCK_PIN		4

	// TDO/SWO Pin (input)
	#define PIN_TDO_PORT            GPIOA
	#define PIN_TDO					5

	// TDI Pin (output)
	#define PIN_TDI_PORT			GPIOA
	#define PIN_TDI					7

	// nRESET Pin
	#define PIN_nRESET_PORT         GPIOA
	#define PIN_nRESET_PIN			6

#elif defined ( BOARD_V2 )

	// SWDIO/TMS Pin
	#define PIN_SWDIO_TMS_PORT		GPIOA
	#define PIN_SWDIO_TMS_PIN		4

	// SWCLK/TCK Pin
	#define PIN_SWCLK_TCK_PORT		GPIOA
	#define PIN_SWCLK_TCK_PIN		5

	// TDO/SWO Pin (input)
	#define PIN_TDO_PORT            GPIOA
	#define PIN_TDO					6

	// TDI Pin (output)
	#define PIN_TDI_PORT			GPIOA
	#define PIN_TDI					7

	// nRESET Pin
	#define PIN_nRESET_PORT         GPIOB
	#define PIN_nRESET_PIN			9

#elif defined ( STLINK_V20 )	\
  ||  defined ( STLINK_V21 )

	// SWDIO/TMS Pin
	#define PIN_SWDIO_TMS_PORT		GPIOB
	#define PIN_SWDIO_TMS_PIN		4

	// SWCLK/TCK Pin
	#define PIN_SWCLK_TCK_PORT		GPIOB
	#define PIN_SWCLK_TCK_PIN		5

	// TDO/SWO Pin (input)
	#define PIN_TDO_PORT            GPIOA
	#define PIN_TDO					6

	// nRESET Pin
	#define PIN_nRESET_PORT         GPIOB
	#define PIN_nRESET_PIN			0

#elif defined ( BOARD_STM32RF )

	// SWDIO/TMS Pin
	#define PIN_SWDIO_TMS_PORT		GPIOA
	#define PIN_SWDIO_TMS_PIN		6

	// SWCLK/TCK Pin
	#define PIN_SWCLK_TCK_PORT		GPIOA
	#define PIN_SWCLK_TCK_PIN		7

	// TDI Pin (output)
	#define PIN_TDI_PORT			GPIOA
	#define PIN_TDI					8

	// nRESET Pin
	#define PIN_nRESET_PORT			GPIOB
	#define PIN_nRESET_PIN			9

#endif

// Debug Unit LEDs

#if defined ( BOARD_V1 ) || defined ( BOARD_V2 )

	#define LED_CONNECTED_RCC		RCC_APB2ENR_IOPBEN

	// Connected LED (GREEN)
	#define LED_CONNECTED_PORT      GPIOB
	#define LED_CONNECTED_PIN		13

	// Target Running LED (RED)
	#define LED_RUNNING_PORT		GPIOB
	#define LED_RUNNING_PIN			12

#elif defined ( STLINK_V20 )

	#define LED_CONNECTED_RCC		RCC_APB2ENR_IOPAEN

	// Connected LED (GREEN)	0
	// Target Running LED (RED)	1
	// Off - float
	#define LED_CONNECTED_PORT      GPIOA
	#define LED_CONNECTED_PIN		9

	#define LED_RUNNING_PORT		GPIOA
	#define LED_RUNNING_PIN			9

#elif defined ( STLINK_V21 )

	#define LED_CONNECTED_RCC		RCC_APB2ENR_IOPAEN

	// Connected LED (GREEN)	0
	// Target Running LED (RED)	1
	// Off - float
	#define LED_CONNECTED_PORT      GPIOA
	#define LED_CONNECTED_PIN		9

	#define LED_RUNNING_PORT		GPIOA
	#define LED_RUNNING_PIN			8

	#define LED_CONNECTED			PIN_MASK(LED_CONNECTED_PIN)
	#define LED_RUNNING				PIN_MASK(LED_RUNNING_PIN)

#elif defined ( BOARD_STM32RF )

	#define LED_CONNECTED_RCC		RCC_APB2ENR_IOPBEN

	// Connected LED (GREEN)
	#define LED_CONNECTED_PORT      GPIOB
	#define LED_CONNECTED_PIN		11

	// Target Running LED (RED)
	#define LED_RUNNING_PORT		GPIOB
	#define LED_RUNNING_PIN			12

#endif

#define LED_CONNECTED			PIN_MASK(LED_CONNECTED_PIN)
#define LED_RUNNING				PIN_MASK(LED_RUNNING_PIN)

#define PIN_nRESET				PIN_MASK(PIN_nRESET_PIN)
#define PIN_SWDIO_TMS			PIN_MASK(PIN_SWDIO_TMS_PIN)
#define PIN_SWCLK_TCK			PIN_MASK(PIN_SWCLK_TCK_PIN)

#if (PIN_nRESET_PIN >= 8)
	#define PIN_nRESET_LOW()							\
		do {											\
			/* GPIO_Mode_Out_OD | GPIO_Speed_50MHz */	\
			PIN_nRESET_PORT->CRH = (PIN_nRESET_PORT->CRH & ~PIN_MODE_MASK(PIN_nRESET_PIN - 8))	\
									| PIN_MODE(((GPIO_Mode_Out_OD | GPIO_Speed_50MHz) & 0x0F), PIN_nRESET_PIN - 8);	\
		} while (0)

	#define PIN_nRESET_HIGH()							\
		do {											\
			PIN_nRESET_PORT->CRH = (PIN_nRESET_PORT->CRH & ~PIN_MODE_MASK(PIN_nRESET_PIN - 8))	\
									| PIN_MODE(GPIO_Mode_IN_FLOATING, PIN_nRESET_PIN - 8);		\
		} while (0)
#else
	#define PIN_nRESET_LOW()							\
		do {											\
			/* GPIO_Mode_Out_OD | GPIO_Speed_50MHz */	\
			PIN_nRESET_PORT->CRL = (PIN_nRESET_PORT->CRL & ~PIN_MODE_MASK(PIN_nRESET_PIN))	\
									| PIN_MODE(((GPIO_Mode_Out_OD | GPIO_Speed_50MHz) & 0x0F), PIN_nRESET_PIN);	\
		} while (0)

	#define PIN_nRESET_HIGH()							\
		do {											\
			PIN_nRESET_PORT->CRL = (PIN_nRESET_PORT->CRL & ~PIN_MODE_MASK(PIN_nRESET_PIN))	\
									| PIN_MODE(GPIO_Mode_IN_FLOATING, PIN_nRESET_PIN);		\
		} while (0)
#endif

//	For fast switch between input and output mode
//	without GPIO_Init call
#if (PIN_SWDIO_TMS_PIN >= 8)
	#define PIN_SWDIO_TMS_OUT_DISABLE()					\
		do {											\
			PIN_SWDIO_TMS_PORT->CRH = (PIN_SWDIO_TMS_PORT->CRH & ~PIN_MODE_MASK(PIN_SWDIO_TMS_PIN - 8)) | PIN_MODE(0x8, PIN_SWDIO_TMS_PIN - 8);	\
			PIN_SWDIO_TMS_PORT->BSRR = PIN_SWDIO_TMS;	\
		} while (0)

	#define PIN_SWDIO_TMS_OUT_ENABLE()					\
		do {											\
			PIN_SWDIO_TMS_PORT->CRH = (PIN_SWDIO_TMS_PORT->CRH & ~PIN_MODE_MASK(PIN_SWDIO_TMS_PIN - 8)) | PIN_MODE(0x3, PIN_SWDIO_TMS_PIN - 8);	\
			PIN_SWDIO_TMS_PORT->BRR  = PIN_SWDIO_TMS;	\
		} while (0)
#else
	#define PIN_SWDIO_TMS_OUT_DISABLE()					\
		do {											\
			PIN_SWDIO_TMS_PORT->CRL = (PIN_SWDIO_TMS_PORT->CRL & ~PIN_MODE_MASK(PIN_SWDIO_TMS_PIN)) | PIN_MODE(0x8, PIN_SWDIO_TMS_PIN);	\
			PIN_SWDIO_TMS_PORT->BSRR = PIN_SWDIO_TMS;	\
		} while (0)

	#define PIN_SWDIO_TMS_OUT_ENABLE()					\
		do {											\
			PIN_SWDIO_TMS_PORT->CRL = (PIN_SWDIO_TMS_PORT->CRL & ~PIN_MODE_MASK(PIN_SWDIO_TMS_PIN)) | PIN_MODE(0x3, PIN_SWDIO_TMS_PIN);	\
			PIN_SWDIO_TMS_PORT->BRR  = PIN_SWDIO_TMS;	\
		} while (0)

#endif

void PORT_USB_CONNECT_SETUP(void);
void LEDS_SETUP (void);

//**************************************************************************************************
/** 
\defgroup DAP_Config_PortIO_gr CMSIS-DAP Hardware I/O Pin Access
\ingroup DAP_ConfigIO_gr 
@{

Standard I/O Pins of the CMSIS-DAP Hardware Debug Port support standard JTAG mode
and Serial Wire Debug (SWD) mode. In SWD mode only 2 pins are required to implement the debug 
interface of a device. The following I/O Pins are provided:

JTAG I/O Pin                 | SWD I/O Pin          | CMSIS-DAP Hardware pin mode
---------------------------- | -------------------- | ---------------------------------------------
TCK: Test Clock              | SWCLK: Clock         | Output Push/Pull
TMS: Test Mode Select        | SWDIO: Data I/O      | Output Push/Pull; Input (for receiving data)
TDI: Test Data Input         |                      | Output Push/Pull
TDO: Test Data Output        |                      | Input             
nTRST: Test Reset (optional) |                      | Output Open Drain with pull-up resistor
nRESET: Device Reset         | nRESET: Device Reset | Output Open Drain with pull-up resistor


DAP Hardware I/O Pin Access Functions
-------------------------------------
The various I/O Pins are accessed by functions that implement the Read, Write, Set, or Clear to 
these I/O Pins. 

For the SWDIO I/O Pin there are additional functions that are called in SWD I/O mode only.
This functions are provided to achieve faster I/O that is possible with some advanced GPIO 
peripherals that can independently write/read a single I/O pin without affecting any other pins 
of the same I/O port. The following SWDIO I/O Pin functions are provided:
 - \ref PIN_SWDIO_OUT_ENABLE to enable the output mode from the DAP hardware.
 - \ref PIN_SWDIO_OUT_DISABLE to enable the input mode to the DAP hardware.
 - \ref PIN_SWDIO_IN to read from the SWDIO I/O pin with utmost possible speed.
 - \ref PIN_SWDIO_OUT to write to the SWDIO I/O pin with utmost possible speed.
*/


// Configure DAP I/O pins ------------------------------

void PORT_JTAG_SETUP(void);
void PORT_SWD_SETUP(void);
void PORT_OFF(void);

/** Setup of the Debug Unit I/O pins and LEDs (called when Debug Unit is initialized).
This function performs the initialization of the CMSIS-DAP Hardware I/O Pins and the 
Status LEDs. In detail the operation of Hardware I/O and LED pins are enabled and set:
 - I/O clock system enabled.
 - all I/O pins: input buffer enabled, output pins are set to HighZ mode.
 - for nTRST, nRESET a weak pull-up (if available) is enabled.
 - LED output pins are enabled and LEDs are turned off.
*/
#define DAP_SETUP()	PORT_OFF()

// SWCLK/TCK I/O pin -------------------------------------

/** SWCLK/TCK I/O pin: Get Input.
\return Current status of the SWCLK/TCK DAP hardware I/O pin.
*/
__STATIC_INLINE uint8_t PIN_SWCLK_TCK_IN(void)
{
	return (PIN_SWCLK_TCK_PORT->ODR & PIN_SWCLK_TCK) ? 1 : 0;
}

/** SWCLK/TCK I/O pin: Set Output to High.
Set the SWCLK/TCK DAP hardware I/O pin to high level.
*/
__STATIC_INLINE void PIN_SWCLK_TCK_SET(void)
{
	PIN_SWCLK_TCK_PORT->BSRR = PIN_SWCLK_TCK;
}

/** SWCLK/TCK I/O pin: Set Output to Low.
Set the SWCLK/TCK DAP hardware I/O pin to low level.
*/
__STATIC_INLINE void PIN_SWCLK_TCK_CLR (void)
{
	PIN_SWCLK_TCK_PORT->BRR = PIN_SWCLK_TCK;
}

// SWDIO/TMS Pin I/O --------------------------------------

/** SWDIO/TMS I/O pin: Get Input.
\return Current status of the SWDIO/TMS DAP hardware I/O pin.
*/
__STATIC_INLINE uint8_t PIN_SWDIO_TMS_IN(void)
{
	return (PIN_SWDIO_TMS_PORT->IDR & PIN_SWDIO_TMS) ? 1 : 0;
}

/** SWDIO/TMS I/O pin: Set Output to High.
Set the SWDIO/TMS DAP hardware I/O pin to high level.
*/
__STATIC_INLINE void PIN_SWDIO_TMS_SET(void)
{
	PIN_SWDIO_TMS_PORT->BSRR = PIN_SWDIO_TMS;
}

/** SWDIO/TMS I/O pin: Set Output to Low.
Set the SWDIO/TMS DAP hardware I/O pin to low level.
*/
__STATIC_INLINE void PIN_SWDIO_TMS_CLR(void)
{
	PIN_SWDIO_TMS_PORT->BRR  = PIN_SWDIO_TMS;
}

/** SWDIO I/O pin: Get Input (used in SWD mode only).
\return Current status of the SWDIO DAP hardware I/O pin.
*/
__STATIC_INLINE uint8_t PIN_SWDIO_IN (void)
{
	if (PIN_SWDIO_TMS_PORT->IDR & PIN_SWDIO_TMS)
		return 1;
	return 0;
}

/** SWDIO I/O pin: Set Output (used in SWD mode only).
\param bit Output value for the SWDIO DAP hardware I/O pin.
*/
__STATIC_INLINE void PIN_SWDIO_OUT(uint8_t bit)
{
	if (bit & 1)
		PIN_SWDIO_TMS_PORT->BSRR = PIN_SWDIO_TMS;
	else
		PIN_SWDIO_TMS_PORT->BRR  = PIN_SWDIO_TMS;
}

/** SWDIO I/O pin: Switch to Output mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to output mode. This function is
called prior \ref PIN_SWDIO_OUT function calls.
*/
__STATIC_INLINE void PIN_SWDIO_OUT_ENABLE(void)
{
	PIN_SWDIO_TMS_OUT_ENABLE();
}

/** SWDIO I/O pin: Switch to Input mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to input mode. This function is
called prior \ref PIN_SWDIO_IN function calls.
*/
__STATIC_INLINE void PIN_SWDIO_OUT_DISABLE(void)
{
	PIN_SWDIO_TMS_OUT_DISABLE();
}


// TDI Pin I/O ---------------------------------------------
#if ( DAP_JTAG != 0 )
/** TDI I/O pin: Get Input.
\return Current status of the TDI DAP hardware I/O pin.
*/
__STATIC_INLINE uint8_t PIN_TDI_IN(void)
{
	return (PIN_TDI_PORT->ODR & PIN_TDI) ? 1 : 0;
}

/** TDI I/O pin: Set Output.
\param bit Output value for the TDI DAP hardware I/O pin.
*/
__STATIC_INLINE void PIN_TDI_OUT(uint8_t bit)
{
	if (bit & 1)
		PIN_TDI_PORT->BSRR = PIN_TDI;
	else
		PIN_TDI_PORT->BRR  = PIN_TDI;
}


// TDO Pin I/O ---------------------------------------------

/** TDO I/O pin: Get Input.
\return Current status of the TDO DAP hardware I/O pin.
*/
__STATIC_INLINE uint8_t PIN_TDO_IN(void)
{
	return (PIN_TDO_PORT->IDR & PIN_TDO) ? 1 : 0;
}
#endif

// nTRST Pin I/O -------------------------------------------

/** nTRST I/O pin: Get Input.
\return Current status of the nTRST DAP hardware I/O pin.
*/
__STATIC_INLINE uint8_t PIN_nTRST_IN(void)
{
	return (0);   // Not available
}

/** nTRST I/O pin: Set Output.
\param bit JTAG TRST Test Reset pin status:
           - 0: issue a JTAG TRST Test Reset.
           - 1: release JTAG TRST Test Reset.
*/
__STATIC_INLINE void PIN_nTRST_OUT(uint8_t bit)
{

}

// nRESET Pin I/O------------------------------------------

/** nRESET I/O pin: Get Input.
\return Current status of the nRESET DAP hardware I/O pin.
*/
__STATIC_INLINE uint8_t PIN_nRESET_IN(void)
{
	if (PIN_nRESET_PORT->IDR & PIN_nRESET)
		return 1;
	return 0;
}

/** nRESET I/O pin: Set Output.
\param bit target device hardware reset pin status:
           - 0: issue a device hardware reset.
           - 1: release device hardware reset.
*/
void PIN_nRESET_OUT(uint8_t bit);

///@}

//**************************************************************************************************
/** 
\defgroup DAP_Config_LEDs_gr CMSIS-DAP Hardware Status LEDs
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware may provide LEDs that indicate the status of the CMSIS-DAP Debug Unit.

It is recommended to provide the following LEDs for status indication:
 - Connect LED: is active when the DAP hardware is connected to a debugger.
 - Running LED: is active when the debugger has put the target device into running state.
*/

/** Debug Unit: Set status of Connected LED.
\param bit status of the Connect LED.
           - 1: Connect LED ON: debugger is connected to CMSIS-DAP Debug Unit.
           - 0: Connect LED OFF: debugger is not connected to CMSIS-DAP Debug Unit.
*/
#define LED_CONNECTED_OUT(b)	pCoreDescriptor->LedConnected(b)

/** Debug Unit: Set status Target Running LED.
\param bit status of the Target Running LED.
           - 1: Target Running LED ON: program execution in target started.
           - 0: Target Running LED OFF: program execution in target stopped.
*/
#define LED_RUNNING_OUT(b)		pCoreDescriptor->LedRunning(b)

///@}


//**************************************************************************************************
/** 
\defgroup DAP_Config_Initialization_gr CMSIS-DAP Initialization
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware I/O and LED Pins are initialized with the function \ref DAP_SETUP.
*/

/** Reset Target Device with custom specific I/O pin or command sequence.
This function allows the optional implementation of a device specific reset sequence.
It is called when the command \ref DAP_ResetTarget and is for example required 
when a device needs a time-critical unlock sequence that enables the debug port.
\return 0 = no device specific reset sequence is implemented.\n
        1 = a device specific reset sequence is implemented.
*/
__STATIC_INLINE uint8_t RESET_TARGET(void)
{
	return (0); // change to '1' when a device reset sequence is implemented
}

///@}


#endif /* __DAP_CONFIG_H__ */
