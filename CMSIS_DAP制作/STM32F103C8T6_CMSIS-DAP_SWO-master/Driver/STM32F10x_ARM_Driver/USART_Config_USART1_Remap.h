/* -----------------------------------------------------------------------------
 * Copyright (C) 2016 ARM Limited. All rights reserved.
 *
 * $Date:        29. August 2016
 * $Revision:    V1.1.2
 *
 * Project:      RTE Device Configuration for STMicroelectronics STM32F1xx
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __USART1_CONFIG_H
#define __USART1_CONFIG_H


#define GPIO_PORT(num) \
 ((num == 0) ? GPIOA : \
  (num == 1) ? GPIOB : \
  (num == 2) ? GPIOC : \
  (num == 3) ? GPIOD : \
  (num == 4) ? GPIOE : \
  (num == 5) ? GPIOF : \
  (num == 6) ? GPIOG : \
  NULL)

#define RTE_PCLK2                       72000000 //USART1 Clock, max 72MHz
#define RTE_PCLK1                       36000000 //USART2 Clock, max 36MHz

// <e> USART1 (Universal synchronous asynchronous receiver transmitter)
// <i> Configuration settings for Driver_USART1 in component ::CMSIS Driver:USART
#define RTE_USART1                       1

//   <o> USART1_TX Pin <0=>Not Used <1=>PA9
#define RTE_USART1_TX_PORT_ID_DEF       0
#if    (RTE_USART1_TX_PORT_ID_DEF == 0)
#define RTE_USART1_TX_DEF               0
#elif  (RTE_USART1_TX_PORT_ID_DEF == 1)
#define RTE_USART1_TX_DEF               1
#define RTE_USART1_TX_PORT_DEF          GPIOA
#define RTE_USART1_TX_BIT_DEF           9
#else
#error "Invalid USART1_TX Pin Configuration!"
#endif

//   <o> USART1_RX Pin <0=>Not Used <1=>PA10
#define RTE_USART1_RX_PORT_ID_DEF       0
#if    (RTE_USART1_RX_PORT_ID_DEF == 0)
#define RTE_USART1_RX_DEF               0
#elif  (RTE_USART1_RX_PORT_ID_DEF == 1)
#define RTE_USART1_RX_DEF               1
#define RTE_USART1_RX_PORT_DEF          GPIOA
#define RTE_USART1_RX_BIT_DEF           10
#else
#error "Invalid USART1_RX Pin Configuration!"
#endif

//   <o> USART1_CK Pin <0=>Not Used <1=>PA8
#define RTE_USART1_CK_PORT_ID_DEF       0
#if    (RTE_USART1_CK_PORT_ID_DEF == 0)
#define RTE_USART1_CK                   0
#elif  (RTE_USART1_CK_PORT_ID_DEF == 1)
#define RTE_USART1_CK                   1
#define RTE_USART1_CK_PORT_DEF          GPIOA
#define RTE_USART1_CK_BIT_DEF           8
#else
#error "Invalid USART1_CK Pin Configuration!"
#endif

//   <o> USART1_CTS Pin <0=>Not Used <1=>PA11
#define RTE_USART1_CTS_PORT_ID_DEF      0
#if    (RTE_USART1_CTS_PORT_ID_DEF == 0)
#define RTE_USART1_CTS                  0
#elif  (RTE_USART1_CTS_PORT_ID_DEF == 1)
#define RTE_USART1_CTS                  1
#define RTE_USART1_CTS_PORT_DEF         GPIOA
#define RTE_USART1_CTS_BIT_DEF          11
#else
#error "Invalid USART1_CTS Pin Configuration!"
#endif

//   <o> USART1_RTS Pin <0=>Not Used <1=>PA12
#define RTE_USART1_RTS_PORT_ID_DEF      0
#if    (RTE_USART1_RTS_PORT_ID_DEF == 0)
#define RTE_USART1_RTS                  0
#elif  (RTE_USART1_RTS_PORT_ID_DEF == 1)
#define RTE_USART1_RTS                  1
#define RTE_USART1_RTS_PORT_DEF         GPIOA
#define RTE_USART1_RTS_BIT_DEF          12
#else
#error "Invalid USART1_RTS Pin Configuration!"
#endif

//   <e> USART1 Pin Remap
//   <i> Enable USART1 Pin Remapping
#define RTE_USART1_REMAP_FULL           1

//     <o> USART1_TX Pin <0=>Not Used <1=>PB6
#define RTE_USART1_TX_PORT_ID_FULL      0
#if    (RTE_USART1_TX_PORT_ID_FULL == 0)
#define RTE_USART1_TX_FULL              0
#elif  (RTE_USART1_TX_PORT_ID_FULL == 1)
#define RTE_USART1_TX_FULL              1
#define RTE_USART1_TX_PORT_FULL         GPIOB
#define RTE_USART1_TX_BIT_FULL          6
#else
#error "Invalid USART1_TX Pin Configuration!"
#endif

//     <o> USART1_RX Pin <0=>Not Used <1=>PB7
#define RTE_USART1_RX_PORT_ID_FULL      1
#if    (RTE_USART1_RX_PORT_ID_FULL == 0)
#define RTE_USART1_RX_FULL              0
#elif  (RTE_USART1_RX_PORT_ID_FULL == 1)
#define RTE_USART1_RX_FULL              1
#define RTE_USART1_RX_PORT_FULL         GPIOB
#define RTE_USART1_RX_BIT_FULL          7
#else
#error "Invalid USART1_RX Pin Configuration!"
#endif
//   </e>

#if    (RTE_USART1_REMAP_FULL)
#define RTE_USART1_AF_REMAP              AFIO_USART1_REMAP
#define RTE_USART1_TX                    RTE_USART1_TX_FULL
#define RTE_USART1_TX_PORT               RTE_USART1_TX_PORT_FULL
#define RTE_USART1_TX_BIT                RTE_USART1_TX_BIT_FULL
#define RTE_USART1_RX                    RTE_USART1_RX_FULL
#define RTE_USART1_RX_PORT               RTE_USART1_RX_PORT_FULL
#define RTE_USART1_RX_BIT                RTE_USART1_RX_BIT_FULL
#define RTE_USART1_CK_PORT               RTE_USART1_CK_PORT_DEF
#define RTE_USART1_CK_BIT                RTE_USART1_CK_BIT_DEF
#define RTE_USART1_CTS_PORT              RTE_USART1_CTS_PORT_DEF
#define RTE_USART1_CTS_BIT               RTE_USART1_CTS_BIT_DEF
#define RTE_USART1_RTS_PORT              RTE_USART1_RTS_PORT_DEF
#define RTE_USART1_RTS_BIT               RTE_USART1_RTS_BIT_DEF
#else 
#define RTE_USART1_AF_REMAP              AFIO_USART1_NO_REMAP
#define RTE_USART1_TX                    RTE_USART1_TX_DEF
#define RTE_USART1_TX_PORT               RTE_USART1_TX_PORT_DEF
#define RTE_USART1_TX_BIT                RTE_USART1_TX_BIT_DEF
#define RTE_USART1_RX                    RTE_USART1_RX_DEF
#define RTE_USART1_RX_PORT               RTE_USART1_RX_PORT_DEF
#define RTE_USART1_RX_BIT                RTE_USART1_RX_BIT_DEF
#define RTE_USART1_CK_PORT               RTE_USART1_CK_PORT_DEF
#define RTE_USART1_CK_BIT                RTE_USART1_CK_BIT_DEF
#define RTE_USART1_CTS_PORT              RTE_USART1_CTS_PORT_DEF
#define RTE_USART1_CTS_BIT               RTE_USART1_CTS_BIT_DEF
#define RTE_USART1_RTS_PORT              RTE_USART1_RTS_PORT_DEF
#define RTE_USART1_RTS_BIT               RTE_USART1_RTS_BIT_DEF
#endif

//   <e> DMA Rx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Channel <5=>5
//     <i>  Selects DMA Channel (only Channel 5 can be used)
//     <o3> Priority <0=>Low <1=>Medium <2=>High <3=>Very high
//     <i>  Set DMA Channel priority
//   </e>
#define RTE_USART1_RX_DMA               1
#define RTE_USART1_RX_DMA_NUMBER        1
#define RTE_USART1_RX_DMA_CHANNEL       5
#define RTE_USART1_RX_DMA_PRIORITY      0
//   <e> DMA Tx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o3> Priority <0=>Low <1=>Medium <2=>High <3=>Very high
//     <i>  Set DMA Channel priority
//   </e>
#define RTE_USART1_TX_DMA               1
#define RTE_USART1_TX_DMA_NUMBER        1
#define RTE_USART1_TX_DMA_CHANNEL       4
#define RTE_USART1_TX_DMA_PRIORITY      0
// </e>


// <e> USART2 (Universal synchronous asynchronous receiver transmitter)
// <i> Configuration settings for Driver_USART2 in component ::CMSIS Driver:USART
#define RTE_USART2                      1

//   <o> USART2_TX Pin <0=>Not Used <1=>PA2
#define RTE_USART2_TX_PORT_ID_DEF       1
#if    (RTE_USART2_TX_PORT_ID_DEF == 0)
#define RTE_USART2_TX_DEF               0
#elif  (RTE_USART2_TX_PORT_ID_DEF == 1)
#define RTE_USART2_TX_DEF               1
#define RTE_USART2_TX_PORT_DEF          GPIOA
#define RTE_USART2_TX_BIT_DEF           2
#else
#error "Invalid USART2_TX Pin Configuration!"
#endif

//   <o> USART2_RX Pin <0=>Not Used <1=>PA3
#define RTE_USART2_RX_PORT_ID_DEF       1
#if    (RTE_USART2_RX_PORT_ID_DEF == 0)
#define RTE_USART2_RX_DEF               0
#elif  (RTE_USART2_RX_PORT_ID_DEF == 1)
#define RTE_USART2_RX_DEF               1
#define RTE_USART2_RX_PORT_DEF          GPIOA
#define RTE_USART2_RX_BIT_DEF           3
#else
#error "Invalid USART2_RX Pin Configuration!"
#endif

//   <o> USART2_CK Pin <0=>Not Used <1=>PA4
#define RTE_USART2_CK_PORT_ID_DEF       0
#if    (RTE_USART2_CK_PORT_ID_DEF == 0)
#define RTE_USART2_CK_DEF               0
#elif  (RTE_USART2_CK_PORT_ID_DEF == 1)
#define RTE_USART2_CK_DEF               1
#define RTE_USART2_CK_PORT_DEF          GPIOA
#define RTE_USART2_CK_BIT_DEF           4
#else
#error "Invalid USART2_CK Pin Configuration!"
#endif

//   <o> USART2_CTS Pin <0=>Not Used <1=>PA0
#define RTE_USART2_CTS_PORT_ID_DEF      0
#if    (RTE_USART2_CTS_PORT_ID_DEF == 0)
#define RTE_USART2_CTS_DEF              0
#elif  (RTE_USART2_CTS_PORT_ID_DEF == 1)
#define RTE_USART2_CTS_DEF              1
#define RTE_USART2_CTS_PORT_DEF         GPIOA
#define RTE_USART2_CTS_BIT_DEF          0
#else
#error "Invalid USART2_CTS Pin Configuration!"
#endif

//   <o> USART2_RTS Pin <0=>Not Used <1=>PA1
#define RTE_USART2_RTS_PORT_ID_DEF      0
#if    (RTE_USART2_RTS_PORT_ID_DEF == 0)
#define RTE_USART2_RTS_DEF              0
#elif  (RTE_USART2_RTS_PORT_ID_DEF == 1)
#define RTE_USART2_RTS_DEF              1
#define RTE_USART2_RTS_PORT_DEF         GPIOA
#define RTE_USART2_RTS_BIT_DEF          1
#else
#error "Invalid USART2_RTS Pin Configuration!"
#endif

//   <e> USART2 Pin Remap
//   <i> Enable USART2 Pin Remapping
#define RTE_USART2_REMAP_FULL           0

//     <o> USART2_TX Pin <0=>Not Used <1=>PD5
#define RTE_USART2_TX_PORT_ID_FULL      0
#if    (RTE_USART2_TX_PORT_ID_FULL == 0)
#define RTE_USART2_TX_FULL              0
#elif  (RTE_USART2_TX_PORT_ID_FULL == 1)
#define RTE_USART2_TX_FULL              1
#define RTE_USART2_TX_PORT_FULL         GPIOD
#define RTE_USART2_TX_BIT_FULL          5
#else
#error "Invalid USART2_TX Pin Configuration!"
#endif

//     <o> USART2_RX Pin <0=>Not Used <1=>PD6
#define RTE_USART2_RX_PORT_ID_FULL      0
#if    (RTE_USART2_RX_PORT_ID_FULL == 0)
#define RTE_USART2_RX_FULL              0
#elif  (RTE_USART2_RX_PORT_ID_FULL == 1)
#define RTE_USART2_RX_FULL              1
#define RTE_USART2_RX_PORT_FULL         GPIOD
#define RTE_USART2_RX_BIT_FULL          6
#else
#error "Invalid USART2_RX Pin Configuration!"
#endif

//     <o> USART2_CK Pin <0=>Not Used <1=>PD7
#define RTE_USART2_CK_PORT_ID_FULL      0
#if    (RTE_USART2_CK_PORT_ID_FULL == 0)
#define RTE_USART2_CK_FULL              0
#elif  (RTE_USART2_CK_PORT_ID_FULL == 1)
#define RTE_USART2_CK_FULL              1
#define RTE_USART2_CK_PORT_FULL         GPIOD
#define RTE_USART2_CK_BIT_FULL          7
#else
#error "Invalid USART2_CK Pin Configuration!"
#endif

//     <o> USART2_CTS Pin <0=>Not Used <1=>PD3
#define RTE_USART2_CTS_PORT_ID_FULL     0
#if    (RTE_USART2_CTS_PORT_ID_FULL == 0)
#define RTE_USART2_CTS_FULL             0
#elif  (RTE_USART2_CTS_PORT_ID_FULL == 1)
#define RTE_USART2_CTS_FULL             1
#define RTE_USART2_CTS_PORT_FULL        GPIOD
#define RTE_USART2_CTS_BIT_FULL         3
#else
#error "Invalid USART2_CTS Pin Configuration!"
#endif

//     <o> USART2_RTS Pin <0=>Not Used <1=>PD4
#define RTE_USART2_RTS_PORT_ID_FULL     0
#if    (RTE_USART2_RTS_PORT_ID_FULL == 0)
#define RTE_USART2_RTS_FULL             0
#elif  (RTE_USART2_RTS_PORT_ID_FULL == 1)
#define RTE_USART2_RTS_FULL             1
#define RTE_USART2_RTS_PORT_FULL        GPIOD
#define RTE_USART2_RTS_BIT_FULL         4
#else
#error "Invalid USART2_RTS Pin Configuration!"
#endif
//   </e>

#if    (RTE_USART2_REMAP_FULL)
#define RTE_USART2_AF_REMAP              AFIO_USART2_REMAP
#define RTE_USART2_TX                    RTE_USART2_TX_FULL
#define RTE_USART2_TX_PORT               RTE_USART2_TX_PORT_FULL
#define RTE_USART2_TX_BIT                RTE_USART2_TX_BIT_FULL
#define RTE_USART2_RX                    RTE_USART2_RX_FULL
#define RTE_USART2_RX_PORT               RTE_USART2_RX_PORT_FULL
#define RTE_USART2_RX_BIT                RTE_USART2_RX_BIT_FULL
#define RTE_USART2_CK                    RTE_USART2_CK_FULL
#define RTE_USART2_CK_PORT               RTE_USART2_CK_PORT_FULL
#define RTE_USART2_CK_BIT                RTE_USART2_CK_BIT_FULL
#define RTE_USART2_CTS                   RTE_USART2_CTS_FULL
#define RTE_USART2_CTS_PORT              RTE_USART2_CTS_PORT_FULL
#define RTE_USART2_CTS_BIT               RTE_USART2_CTS_BIT_FULL
#define RTE_USART2_RTS                   RTE_USART2_RTS_FULL
#define RTE_USART2_RTS_PORT              RTE_USART2_RTS_PORT_FULL
#define RTE_USART2_RTS_BIT               RTE_USART2_RTS_BIT_FULL
#else 
#define RTE_USART2_AF_REMAP              AFIO_USART2_NO_REMAP
#define RTE_USART2_TX                    RTE_USART2_TX_DEF
#define RTE_USART2_TX_PORT               RTE_USART2_TX_PORT_DEF
#define RTE_USART2_TX_BIT                RTE_USART2_TX_BIT_DEF
#define RTE_USART2_RX                    RTE_USART2_RX_DEF
#define RTE_USART2_RX_PORT               RTE_USART2_RX_PORT_DEF
#define RTE_USART2_RX_BIT                RTE_USART2_RX_BIT_DEF
#define RTE_USART2_CK                    RTE_USART2_CK_DEF
#define RTE_USART2_CK_PORT               RTE_USART2_CK_PORT_DEF
#define RTE_USART2_CK_BIT                RTE_USART2_CK_BIT_DEF
#define RTE_USART2_CTS                   RTE_USART2_CTS_DEF
#define RTE_USART2_CTS_PORT              RTE_USART2_CTS_PORT_DEF
#define RTE_USART2_CTS_BIT               RTE_USART2_CTS_BIT_DEF
#define RTE_USART2_RTS                   RTE_USART2_RTS_DEF
#define RTE_USART2_RTS_PORT              RTE_USART2_RTS_PORT_DEF
#define RTE_USART2_RTS_BIT               RTE_USART2_RTS_BIT_DEF
#endif

//   <e> DMA Rx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Channel <6=>6
//     <i>  Selects DMA Channel (only Channel 6 can be used)
//     <o3> Priority <0=>Low <1=>Medium <2=>High <3=>Very high
//     <i>  Set DMA Channel priority
//   </e>
#define RTE_USART2_RX_DMA               1
#define RTE_USART2_RX_DMA_NUMBER        1
#define RTE_USART2_RX_DMA_CHANNEL       6
#define RTE_USART2_RX_DMA_PRIORITY      0

//   <e> DMA Tx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Channel <7=>7
//     <i>  Selects DMA Channel (only Channel 7 can be used)
//     <o3> Priority <0=>Low <1=>Medium <2=>High <3=>Very high
//     <i>  Set DMA Channel priority
//   </e>
#define RTE_USART2_TX_DMA               1
#define RTE_USART2_TX_DMA_NUMBER        1
#define RTE_USART2_TX_DMA_CHANNEL       7
#define RTE_USART2_TX_DMA_PRIORITY      0

// </e>

#endif  /* __USART1_CONFIG_H */
