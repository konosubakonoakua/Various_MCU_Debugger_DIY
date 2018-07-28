#ifndef __UART_H
#define __UART_H

#include <RTL.h>
#include <stdint.h>
#include <stm32f10x.h>

/*-----------------------------------------------------------------------------
 * ENUMERATOR and STRUCTURE DEFINITIONS
 *----------------------------------------------------------------------------*/
 
/* Control Lines */
#define UART_CONTROL_LINE_RTS_Pos   0   /* request to send  control line      */
#define UART_CONTROL_LINE_RTS_Msk  (1 << UART_CONTROL_LINE_RTS_Pos)
#define UART_CONTROL_LINE_DTR_Pos   0   /* request to send  control line      */
#define UART_CONTROL_LINE_DTR_Msk  (1 << UART_CONTROL_LINE_DTR_Pos)

/* Status Lines */
#define UART_STATUS_LINE_CTS_Pos    0   /* clear to send control line         */
#define UART_STATUS_LINE_CTS_Msk   (1 << UART_STATUS_LINE_CTS_Pos)
#define UART_STATUS_LINE_DCD_Pos    1   /* data carrier detect                */
#define UART_STATUS_LINE_DCD_Msk   (1 << UART_STATUS_LINE_DCD_Pos)
#define UART_STATUS_LINE_DSR_Pos    2   /* data set ready                     */
#define UART_STATUS_LINE_DSR_Msk   (1 << UART_STATUS_LINE_DSR_Pos)
#define UART_STATUS_LINE_RI_Pos     3   /* ring indicator                     */
#define UART_STATUS_LINE_RI_Msk    (1 << UART_STATUS_LINE_RI_Pos)

/* Communication Errors */
#define UART_FRAMING_ERROR_Pos      0
#define UART_FRAMING_ERROR_Msk     (1 << UART_FRAMING_ERROR_Pos)
#define UART_PARITY_ERROR_Pos       1
#define UART_PARITY_ERROR_Msk      (1 << UART_PARITY_ERROR_Pos)
#define UART_OVERRUN_ERROR_Pos      2
#define UART_OVERRUN_ERROR_Msk     (1 << UART_OVERRUN_ERROR_Pos)

/* Parity enumerator */
typedef enum {
  UART_PARITY_NONE    = 0,
  UART_PARITY_ODD     = 1,
  UART_PARITY_EVEN    = 2,
  UART_PARITY_MARK    = 3,
  UART_PARITY_SPACE   = 4
} UART_Parity;

/* Stop Bits enumerator */
typedef enum {
  UART_STOP_BITS_1    = 0,
  UART_STOP_BITS_1_5  = 1,
  UART_STOP_BITS_2    = 2
} UART_StopBits;

/* Data Bits enumerator */
typedef enum {
  UART_DATA_BITS_5    = 5,
  UART_DATA_BITS_6    = 6,
  UART_DATA_BITS_7    = 7,
  UART_DATA_BITS_8    = 8,
  UART_DATA_BITS_16   = 16
} UART_DataBits;

/* Flow control enumerator */
typedef enum {
  UART_FLOW_CONTROL_NONE     = 0,
  UART_FLOW_CONTROL_RTS_CTS  = 1,
  UART_FLOW_CONTROL_XON_XOFF = 2
} UART_FlowControl;

/* UART driver function prototypes */

int32_t  UART_GetCommunicationErrorStatus (void);
int32_t  UART_GetStatusLineState          (void);
int32_t  UART_GetBreak                    (void);
int32_t  UART_GetChar                     (void);
int32_t  UART_PutChar                     (uint8_t ch);

#endif /* __UART_H */
