/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_CH32V307

#include "usbdc.h"
#include "device/dcd.h"
#include <portable/ch32v307/ch32v30x_usbhs_device.h>

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

typedef struct {
    uint32_t totXfer;
    uint32_t nXferred;
    uint8_t *pDataXfer;
} tAsyncXfer;

void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* */
static void handleIntDetect(void);
/* */
static void handleIntSetup(
    uint8_t epNum,
    uint32_t pid);
/* */
static void handleIntTransfer(
    uint8_t epNum,
    uint32_t pid);

static uint8_t m_endpt0Dma[USBDC_ENDPT0_MPS_MAX] = { 0 }; // TODO:
static uint8_t m_emdptnTxDma[USBDC_ENDPTN_CNT][USBDC_ENDPTN_MPS_MAX] = { 0 };
static uint8_t m_emdptnRxDma[USBDC_ENDPTN_CNT][USBDC_ENDPTN_MPS_MAX] = { 0 };

static tAsyncXfer m_endptXfer[USBDC_ENDPTN_CNT][2] = { 0 };

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

// Initialize controller to device mode
void dcd_init (uint8_t rhport)
{
    (void) rhport;
    printf("dcd_init\r\n");
    usbdc_Init();
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport)
{
    (void) rhport;
    usbdc_IntEnable();
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport)
{
    (void) rhport;
    usbdc_IntDisable();
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
    (void) rhport;
    printf("dcd_set_address %u\r\n", (unsigned int) dev_addr);
    usbdc_SetAddress(dev_addr);
    dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport)
{
    (void) rhport;
    printf("dcd_remote_wakeup\r\n");
    usbdc_WakeRemote();
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
    (void) rhport;
    printf("dcd_connect\r\n");
    usbdc_Enable();
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
    (void) rhport;
    printf("dcd_disconnect\r\n");
    usbdc_Disable();
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  (void) rhport;

  printf("dcd_edpt_open\r\n");
  uint8_t epAddr = ep_desc->bEndpointAddress;
  //tusb_xfer_type_t xferType = ep_desc->bmAttributes.xfer;
  uint16_t mps = tu_edpt_packet_size(ep_desc);
  uint8_t epn = tu_edpt_number(epAddr);
  tusb_dir_t dir = tu_edpt_dir(epAddr);
  // TODO: check epn cannot be zero
  // TODO: Only for non-control endpoints
  if (dir == TUSB_DIR_IN) {
      usbdc_EndptnTxConfig(epn, mps, m_emdptnTxDma[epn]);
      usbdc_EndptnTxDisable(epn);
  } else {
      usbdc_EndptnRxConfig(epn, mps, m_emdptnRxDma[epn]);
      usbdc_EndptnRxEnable(epn);
  }

  return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  printf("dcd_edpt_close_all\r\n");
  // TODO
  USBHSD->ENDP_CONFIG = 0x00;
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
    (void) rhport;
    //printf("dcd_edpt_xfer\r\n");
    uint8_t epn = tu_edpt_number(ep_addr);
    tusb_dir_t dir = tu_edpt_dir(ep_addr);
    printf("* dcd_edpt_xfer %u %u\r\n", (unsigned int) epn, (unsigned int) dir);
    m_endptXfer[epn][dir].nXferred = 0u;
    m_endptXfer[epn][dir].totXfer = total_bytes;
    m_endptXfer[epn][dir].pDataXfer = buffer;
    if (epn == 0) {
        if (dir == TUSB_DIR_IN) {
            uint32_t nToSend = 0u;
            usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
            usbdc_Endpt0Write(buffer, total_bytes, &nToSend);
            usbdc_Endpt0TxToggleDatax();
            usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_READY);
        } else {
            usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
        }
    } else {
        if (dir == TUSB_DIR_IN) {
            uint32_t nToSend = 0u;
            usbdc_EndptnTxReportStatus(epn, USBDC_ENDPTN_STATUS_BUSY);
            usbdc_EndptnWrite(epn, buffer, total_bytes, &nToSend);
            usbdc_EndptnTxToggleDatax(epn);
            usbdc_EndptnTxReportStatus(epn, USBDC_ENDPTN_STATUS_READY);
        } else {
            usbdc_EndptnRxReportStatus(epn, USBDC_ENDPTN_STATUS_READY);
        }
    }

    return true;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
    printf("dcd_edpt_xfer_fifo\r\n");
  (void) rhport;
  (void) ep_addr;
  (void) ff;
  (void) total_bytes;
  return false;
}

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;
    printf("dcd_edpt_stall\r\n");
    uint8_t epn = tu_edpt_number(ep_addr);
    tusb_dir_t dir = tu_edpt_dir(ep_addr);
    if (epn == 0u) {
        if (dir == TUSB_DIR_IN) {
            usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_ERROR);
        } else {
            usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_ERROR);
        }
    } else {
        if (dir == TUSB_DIR_IN) {
            usbdc_EndptnTxReportStatus(epn, USBDC_ENDPTN_STATUS_ERROR);
        } else {
            usbdc_EndptnRxReportStatus(epn, USBDC_ENDPTN_STATUS_ERROR);
        }
    }
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;
    printf("dcd_edpt_clear_stall\r\n");
    uint8_t epn = tu_edpt_number(ep_addr);
    tusb_dir_t dir = tu_edpt_dir(ep_addr);
    if (epn == 0u) {
        if (dir == TUSB_DIR_IN) {
            usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_READY);
        } else {
            usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
        }
    } else {
        if (dir == TUSB_DIR_IN) {
            usbdc_EndptnTxReportStatus(epn, USBDC_ENDPTN_STATUS_READY);
        } else {
            usbdc_EndptnRxReportStatus(epn, USBDC_ENDPTN_STATUS_READY);
        }
    }
}

void dcd_int_handler(uint8_t rhport)
{
    (void) rhport;
    uint8_t intflag = USBHSD->INT_FG;
    if (intflag & USBHS_DETECT_FLAG) {
        handleIntDetect();
        USBHSD->INT_FG |= USBHS_DETECT_FLAG;
    }

    uint8_t epn = (USBHSD->INT_ST) & MASK_UIS_ENDP;
    uint32_t pid = (((USBHSD->INT_ST) & MASK_UIS_TOKEN) >> 4) & 0x03;
    if (intflag & USBHS_SETUP_FLAG) {
        handleIntSetup(epn, pid);
        USBHSD->INT_FG |= USBHS_DETECT_FLAG;
    } else if (intflag & USBHS_TRANSFER_FLAG) {
        handleIntTransfer(epn, pid);
        USBHSD->INT_FG |= USBHS_TRANSFER_FLAG;
    }
}

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
    (void) rhport;
    (void) request;

    usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
    usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
    usbdc_Endpt0TxSetData0();
    usbdc_Endpt0RxSetData0();
    usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
}

void USBHS_IRQHandler(void)
{
    dcd_int_handler(BOARD_TUD_RHPORT);
}

static void handleIntDetect(void)
{
    usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
    usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
    usbdc_Endpt0TxSetData0();
    usbdc_Endpt0RxSetData0();
    /* Open endpoint0 */
    usbdc_Endpt0Config(USBDC_ENDPT0_MPS_MAX, m_endpt0Dma);
    /* Close all endpoints other than endpoint0 */
    // todo
    /* Set device address to zero */
    usbdc_SetAddress(0u);
    /* Notify bus reset to usbd stack */
    dcd_event_bus_signal(BOARD_TUD_RHPORT, DCD_EVENT_BUS_RESET, true);
    /* Ready to read Setup packet */
    usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
}

static void handleIntSetup(
    uint8_t epNum,
    uint32_t pid)
{
    (void) epNum;
    (void) pid;
    // TODO: checking
    /* Notify setup packet received */
    dcd_event_setup_received(BOARD_TUD_RHPORT, m_endpt0Dma, true);
    usbdc_Endpt0RxSetData0();
    usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_READY);
}

static void handleIntTransfer(
    uint8_t epNum,
    uint32_t pid)
{
    if (pid == PID_IN) {
        if (epNum == 0u) {
            uint32_t nTxBytes = usbdc_Endpt0TxLen();
            if (nTxBytes > 0) {
                tAsyncXfer *pXfer = &m_endptXfer[0][TUSB_DIR_IN];
                pXfer->nXferred += nTxBytes;
                if (pXfer->nXferred < pXfer->totXfer) {
                    uint32_t rem = pXfer->totXfer - pXfer->nXferred;
                    usbdc_Endpt0Write(&pXfer->pDataXfer[pXfer->nXferred], rem, NULL);
                    usbdc_Endpt0TxToggleDatax();
                    usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_READY);
                } else {
                    uint8_t epAddr = tu_edpt_addr(0, TUSB_DIR_IN);
                    dcd_event_xfer_complete(BOARD_TUD_RHPORT, epAddr, pXfer->nXferred, XFER_RESULT_SUCCESS, true);
                }
            }
        } else {
            uint32_t nTxBytes = 0;
            usbdc_EndptnTxLen(epNum, &nTxBytes);
            if (nTxBytes) {
                tAsyncXfer *pXfer = &m_endptXfer[epNum][TUSB_DIR_IN];
                pXfer->nXferred += nTxBytes;
                if (pXfer->nXferred < pXfer->totXfer) {
                    uint32_t rem = pXfer->totXfer - pXfer->nXferred;
                    usbdc_EndptnWrite(epNum, &pXfer->pDataXfer[pXfer->nXferred], rem, NULL);
                    usbdc_EndptnTxToggleDatax(epNum);
                    usbdc_EndptnTxReportStatus(epNum, USBDC_ENDPTN_STATUS_READY);
                } else {
                    uint8_t epAddr = tu_edpt_addr(epNum, TUSB_DIR_IN);
                    dcd_event_xfer_complete(BOARD_TUD_RHPORT, epAddr, pXfer->nXferred, XFER_RESULT_SUCCESS, true);
                }
            }
        }
    } else if (pid == PID_OUT) {
        if (USBHSD->INT_ST & USBHS_DEV_UIS_TOG_OK) {
            if (epNum == 0u) {
                uint32_t nRxBytes = usbdc_Endpt0RxLen();
                if (nRxBytes) {
                    tAsyncXfer *pXfer = &m_endptXfer[0][TUSB_DIR_OUT];
                    usbdc_Endpt0Read(&pXfer->pDataXfer[pXfer->nXferred], nRxBytes, NULL);
                    pXfer->nXferred += nRxBytes;
                    if (pXfer->nXferred < pXfer->totXfer) {
                        usbdc_Endpt0RxToggleDatax();
                        usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
                    } else {
                        uint8_t epAddr = tu_edpt_addr(0, TUSB_DIR_OUT);
                        dcd_event_xfer_complete(BOARD_TUD_RHPORT, epAddr, pXfer->nXferred, XFER_RESULT_SUCCESS, true);
                    }
                } else {
                    usbdc_Endpt0TxSetData0();
                    usbdc_Endpt0RxSetData0();
                    usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
                    usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
                }
            } else {
                uint32_t nRxBytes = 0u;
                usbdc_EndptnRxLen(epNum, &nRxBytes);
                if (nRxBytes) {
                    tAsyncXfer *pXfer = &m_endptXfer[epNum][TUSB_DIR_OUT];
                    usbdc_EndptnRead(epNum, &pXfer->pDataXfer[pXfer->nXferred], nRxBytes, NULL);
                    pXfer->nXferred += nRxBytes;
                    if (pXfer->nXferred < pXfer->totXfer) {
                        usbdc_EndptnRxToggleDatax(epNum);
                        usbdc_EndptnRxReportStatus(epNum, USBDC_ENDPTN_STATUS_READY);
                    } else {
                        uint8_t epAddr = tu_edpt_addr(epNum, TUSB_DIR_OUT);
                        dcd_event_xfer_complete(BOARD_TUD_RHPORT, epAddr, pXfer->nXferred, XFER_RESULT_SUCCESS, true);
                    }
                }
            }
        }
    }
}

#endif
