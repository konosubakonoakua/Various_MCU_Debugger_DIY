/**************************************************************************//**
 * @file     DAP_vendor.c
 * @brief    CMSIS-DAP Vendor Commands
 * @version  V1.00
 * @date     1. June 2012
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
 
#include "DAP_config.h"
#include "..\DAP.h"

//**************************************************************************************************
/** 
\defgroup DAP_Vendor_Adapt_gr Adapt Vendor Commands
\ingroup DAP_Vendor_gr 
@{

The file DAP_vendor.c provides template source code for extension of a Debug Unit with 
Vendor Commands. Copy this file to the project folder of the Debug Unit and add the 
file to the MDK-ARM project under the file group Configuration.
*/

/** Process DAP Vendor Command and prepare Response Data
\param request   pointer to request data
\param response  pointer to response data
\return          number of bytes in Response Data
*/
uint32_t DAP_ProcessVendorCommand(uint8_t *request, uint8_t *response) {
  uint32_t num = 0;
  
  switch (*request++) {          // first byte in request is Command ID
    case ID_DAP_Vendor0:
#if 0                            // example user command
      *response++ = *request++;  // copy Command ID
      num++;                     // increment return count
      if (*request == 1) {       // when first command data byte is 1
        *response++ = 'U';       // send 'U' as response
        num++;                   // increment return count
      }
#endif
      break;

    case ID_DAP_Vendor1:  break;
    case ID_DAP_Vendor2:  break;
    case ID_DAP_Vendor3:  break;
    case ID_DAP_Vendor4:  break;
    case ID_DAP_Vendor5:  break;
    case ID_DAP_Vendor6:  break;
    case ID_DAP_Vendor7:  break;
    case ID_DAP_Vendor8:  break;
    case ID_DAP_Vendor9:  break;
    case ID_DAP_Vendor10: break;
    case ID_DAP_Vendor11: break;
    case ID_DAP_Vendor12: break;
    case ID_DAP_Vendor13: break;
    case ID_DAP_Vendor14: break;
    case ID_DAP_Vendor15: break;
    case ID_DAP_Vendor16: break;
    case ID_DAP_Vendor17: break;
    case ID_DAP_Vendor18: break;
    case ID_DAP_Vendor19: break;
    case ID_DAP_Vendor20: break;
    case ID_DAP_Vendor21: break;
    case ID_DAP_Vendor22: break;
    case ID_DAP_Vendor23: break;
    case ID_DAP_Vendor24: break;
    case ID_DAP_Vendor25: break;
    case ID_DAP_Vendor26: break;
    case ID_DAP_Vendor27: break;
    case ID_DAP_Vendor28: break;
    case ID_DAP_Vendor29: break;
    case ID_DAP_Vendor30: break;
    case ID_DAP_Vendor31: break;
  }

  return (num);
}

///@}
