/******************************************************************************
 * @file     main.c
 * @brief    CMSIS-DAP Main module
 * @version  V1.00
 * @date     31. May 2012
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

#include <RTL.h>
#include <rl_usb.h>
#include "DAP_config.h"
#include "..\DAP.h"

extern void usbd_hid_process (void);

// Main program
int main (void) {

  DAP_Setup();                          // DAP Setup 

  usbd_init();                          // USB Device Initialization
  usbd_connect(__TRUE);                 // USB Device Connect

  while (!usbd_configured());           // Wait for USB Device to configure

  LED_CONNECTED_OUT(1);                 // Turn on  Debugger Connected LED
  LED_RUNNING_OUT(1);                   // Turn on  Target Running LED
  Delayms(500);                         // Wait for 500ms
  LED_RUNNING_OUT(0);                   // Turn off Target Running LED
  LED_CONNECTED_OUT(0);                 // Turn off Debugger Connected LED

  while (1) {                           // Endless Loop
    usbd_hid_process();                 // Process USB HID Data
  }
}
