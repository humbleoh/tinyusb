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

#if CFG_TUD_ENABLED && (CFG_TUSB_MCU == OPT_MCU_CH32V307)
#include "device/dcd.h"
#include "usbdc.h"
#include "usb_ch32_usbhs_reg.h"
#include "core_riscv.h"

#define BOARD_TUD_RHPORT      0

#define XFER_CTX(ep, dir)  ( &m_endptXfer[ep][dir] )

typedef struct {
    volatile uint32_t totXfer;
    volatile uint32_t nXferred;
    uint8_t *pDataXfer;
} tAsyncXfer;

#define EP_TX_LEN(ep) *(volatile uint16_t *)((volatile uint16_t *)&(USBHSD->UEP0_TX_LEN) + (ep)*2)
#define EP_TX_CTRL(ep) *(volatile uint8_t *)((volatile uint8_t *)&(USBHSD->UEP0_TX_CTRL) + (ep)*4)
#define EP_RX_CTRL(ep) *(volatile uint8_t *)((volatile uint8_t *)&(USBHSD->UEP0_RX_CTRL) + (ep)*4)
#define EP_RX_MAX_LEN(ep) *(volatile uint16_t *)((volatile uint16_t *)&(USBHSD->UEP0_MAX_LEN) + (ep)*2)

#define EP_TX_DMA_ADDR(ep) *(volatile uint32_t *)((volatile uint32_t *)&(USBHSD->UEP1_TX_DMA) + (ep - 1))
#define EP_RX_DMA_ADDR(ep) *(volatile uint32_t *)((volatile uint32_t *)&(USBHSD->UEP1_RX_DMA) + (ep - 1))

/* Endpoint Buffer */
TU_ATTR_ALIGNED(4) static uint8_t m_endpt0Dma[USBDC_ENDPT0_MPS_MAX];
TU_ATTR_ALIGNED(4) static uint8_t m_endptnTxDma[USBDC_ENDPTN_CNT][USBDC_ENDPTN_MPS_MAX];
TU_ATTR_ALIGNED(4) static uint8_t m_endptnRxDma[USBDC_ENDPTN_CNT][USBDC_ENDPTN_MPS_MAX];
static tAsyncXfer m_endptXfer[USBDC_ENDPTN_CNT][2] = { 0 };

/** */
//void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
/** */
static void handleIntDetect(void);
/** */
static void handleIntSetup(
    uint8_t epNum,
    uint32_t pid);
/** */
static void handleIntTransfer(
    uint8_t epNum,
    uint32_t pid);

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

// Initialize controller to device mode
void dcd_init(uint8_t rhport)
{
    (void) rhport;

    usbdc_Init();

    usbdc_Endpt0Config(USBDC_ENDPT0_MPS_MAX, m_endpt0Dma);
    usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
    usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);

    for (uint8_t i = 1; i < USBDC_ENDPTN_CNT; ++i) {
        usbdc_EndptnTxReportStatus(i, USBDC_ENDPTN_STATUS_BUSY);
        usbdc_EndptnRxReportStatus(i, USBDC_ENDPTN_STATUS_BUSY);
        usbdc_EndptnTxDisable(i);
        usbdc_EndptnRxDisable(i);
    }

    usbdc_Enable();
}

// Enable device interrupt
void dcd_int_enable(uint8_t rhport)
{
    (void) rhport;
    usbdc_IntEnable();
}

// Disable device interrupt
void dcd_int_disable(uint8_t rhport)
{
    (void) rhport;
    usbdc_IntDisable();
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
    (void) dev_addr;
    // Don't set the device address, as the current transaction hasn't completed yet.
    // Need to keep the current device address.
    dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport)
{
    (void) rhport;
    usbdc_WakeRemote();
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
    (void) rhport;
    usbdc_Enable();
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
    (void) rhport;
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
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *desc_edpt)
{
    (void) rhport;

    const uint8_t epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
    TU_ASSERT(epnum > 0 && epnum < USBDC_ENDPTN_CNT);
    const uint8_t dir = tu_edpt_dir(desc_edpt->bEndpointAddress);
    const uint16_t mps = tu_edpt_packet_size(desc_edpt);
    if (dir == TUSB_DIR_OUT) {
        usbdc_EndptnRxConfig(epnum, mps, m_endptnRxDma[epnum]);
        usbdc_EndptnRxReportStatus(epnum, USBDC_ENDPTN_STATUS_READY);
        usbdc_EndptnRxSetData0(epnum);
        usbdc_EndptnRxEnable(epnum);
    } else {
        usbdc_EndptnTxConfig(epnum, mps, m_endptnTxDma[epnum]);
        usbdc_EndptnTxReportStatus(epnum, USBDC_ENDPTN_STATUS_BUSY);
        usbdc_EndptnTxSetData0(epnum);
        usbdc_EndptnTxEnable(epnum);
    }

    return true;
}

void dcd_edpt_close_all(uint8_t rhport)
{
    (void) rhport;
    for (uint8_t i = 1; i < USBDC_ENDPTN_CNT; ++i) {
        usbdc_EndptnTxDisable(i);
        usbdc_EndptnRxDisable(i);
        usbdc_EndptnTxReportStatus(i, USBDC_ENDPTN_STATUS_BUSY);
        usbdc_EndptnRxReportStatus(i, USBDC_ENDPTN_STATUS_BUSY);
    }
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
    (void) rhport;
    const uint8_t epnum = tu_edpt_number(ep_addr);
    const uint8_t dir = tu_edpt_dir(ep_addr);
    tAsyncXfer *xferCtx = XFER_CTX(epnum, dir);
    xferCtx->nXferred = 0;
    xferCtx->totXfer = total_bytes;
    xferCtx->pDataXfer = buffer;
    if (epnum == 0) {
        if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
            usbdc_Endpt0Write(buffer, xferCtx->totXfer, NULL);
            usbdc_Endpt0TxToggleDatax();
            usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_READY);
        } else {
            usbdc_Endpt0RxToggleDatax();
            usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
        }
    } else {
        if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
            usbdc_EndptnWrite(epnum, buffer, xferCtx->totXfer, NULL);
            usbdc_EndptnTxReportStatus(epnum, USBDC_ENDPTN_STATUS_READY);
        } else {
            usbdc_EndptnRxReportStatus(epnum, USBDC_ENDPTN_STATUS_READY);
        }
    }

    return true;
}

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const *request)
{
    (void) rhport;

    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE
        && request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD
        && request->bRequest == TUSB_REQ_SET_ADDRESS) {
        // Set the device address after the status stage has been completed.
        // The new transfer will use the new device address.
        usbdc_SetAddress((uint8_t) request->wValue);
    }

    usbdc_Endpt0RxSetData0();
    usbdc_Endpt0TxSetData0();
    usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
    usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;

    const uint8_t epnum = tu_edpt_number(ep_addr);
    const uint8_t dir = tu_edpt_dir(ep_addr);
    if (epnum == 0) {
        if (dir == TUSB_DIR_OUT) {
            usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_ERROR);
        } else {
            usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_ERROR);
        }
    } else {
        if (dir == TUSB_DIR_OUT) {
            usbdc_EndptnRxReportStatus(epnum, USBDC_ENDPTN_STATUS_ERROR);
        } else {
            usbdc_EndptnTxReportStatus(epnum, USBDC_ENDPTN_STATUS_ERROR);
        }
    }
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;

    const uint8_t epnum = tu_edpt_number(ep_addr);
    const uint8_t dir = tu_edpt_dir(ep_addr);
    if (epnum == 0) {
        if (dir == TUSB_DIR_OUT) {
            // TODO: Check need to reset DataX or not
            // Based on my testing, will cause intermittent issue where the device undetected 
            //usbdc_Endpt0RxSetData0();
            usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
        } else {
            usbdc_Endpt0TxSetData0();
            usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
        }
    } else {
        if (dir == TUSB_DIR_OUT) {
            usbdc_EndptnRxSetData0(epnum);
            usbdc_EndptnRxReportStatus(epnum, USBDC_ENDPTN_STATUS_READY);
        } else {
            usbdc_EndptnTxSetData0(epnum);
            usbdc_EndptnWrite(epnum, NULL, 0, NULL);
            usbdc_EndptnTxReportStatus(epnum, USBDC_ENDPTN_STATUS_READY);
        }
    }
}

/** */
void dcd_int_handler(uint8_t rhport)
{
    (void) rhport;
    uint8_t intflag = USBHSD->INT_FG;
    if (intflag & USBHS_TRANSFER_FLAG) {
        handleIntTransfer(0, 0);
        USBHSD->INT_FG = USBHS_TRANSFER_FLAG;
    } else if (intflag & USBHS_SETUP_FLAG) {
        handleIntSetup(0, 0);
        USBHSD->INT_FG = USBHS_SETUP_FLAG;
    } else if (intflag & USBHS_DETECT_FLAG) {
        handleIntDetect();
        USBHSD->INT_FG = USBHS_DETECT_FLAG;
    } else if (intflag & USBHS_SUSPEND_FLAG) {
        dcd_event_t event = { .rhport = rhport, .event_id = DCD_EVENT_SUSPEND };
        dcd_event_handler(&event, true);
        USBHSD->INT_FG = USBHS_SUSPEND_FLAG;
    }
}

/** */
static void handleIntDetect(void)
{
    usbdc_SetAddress(0u);

    usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
    usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
    usbdc_Endpt0Config(USBDC_ENDPT0_MPS_MAX, m_endpt0Dma);
    usbdc_Endpt0RxSetData0();
    usbdc_Endpt0TxSetData0();
    usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);

    dcd_event_bus_reset(0, TUSB_SPEED_HIGH, true);
}

/** */
static void handleIntSetup(
    uint8_t epNum,
    uint32_t pid)
{
    (void) epNum;
    (void) pid;
    // TODO: checking parameters
    usbdc_Endpt0RxSetData0();
    usbdc_Endpt0TxSetData0();
    /* Notify setup packet received */
    dcd_event_setup_received(BOARD_TUD_RHPORT, m_endpt0Dma, true);
}

/** */
static void handleIntTransfer(
    uint8_t epNum,
    uint32_t pid)
{
    const uint32_t ep = USBHSD->INT_ST & MASK_UIS_ENDP;
    const uint32_t token = ((USBHSD->INT_ST & MASK_UIS_TOKEN) >> 4) & 0x03;
    const bool dir = (token == PID_IN);
    tAsyncXfer *xferCtx = XFER_CTX(ep, dir);
    if (ep == 0) {
        if (token == PID_IN) {
            uint32_t nTxBytes = usbdc_Endpt0TxLen();
            xferCtx->nXferred += nTxBytes;
            if (xferCtx->nXferred < xferCtx->totXfer) {
                uint32_t rem = xferCtx->totXfer - xferCtx->nXferred;
                usbdc_Endpt0Write(&xferCtx->pDataXfer[xferCtx->nXferred], rem, NULL);
                usbdc_Endpt0TxToggleDatax();
                usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_READY);
            } else {
                uint8_t epAddr = tu_edpt_addr(0, TUSB_DIR_IN);
                dcd_event_xfer_complete(BOARD_TUD_RHPORT, epAddr, xferCtx->nXferred, XFER_RESULT_SUCCESS, true);
                usbdc_Endpt0TxReportStatus(USBDC_ENDPTN_STATUS_BUSY);
            }
        } else if (token == PID_OUT) {
            uint32_t nRxBytes = usbdc_Endpt0RxLen();
            usbdc_Endpt0Read(&xferCtx->pDataXfer[xferCtx->nXferred], nRxBytes, NULL);
            xferCtx->nXferred += nRxBytes;
            if (xferCtx->nXferred < xferCtx->totXfer) {
                usbdc_Endpt0RxToggleDatax();
                usbdc_Endpt0RxReportStatus(USBDC_ENDPTN_STATUS_READY);
            } else {
                uint8_t epAddr = tu_edpt_addr(0, TUSB_DIR_OUT);
               dcd_event_xfer_complete(BOARD_TUD_RHPORT, epAddr, xferCtx->nXferred, XFER_RESULT_SUCCESS, true);
            }
        }
    } else {
        if (token == PID_IN) {
            uint32_t nTxBytes = 0;
            usbdc_EndptnTxLen(ep, &nTxBytes);
            xferCtx->nXferred += nTxBytes;
            if (xferCtx->nXferred < xferCtx->totXfer) {
                uint32_t rem = xferCtx->totXfer - xferCtx->nXferred;
                usbdc_EndptnWrite(ep, &xferCtx->pDataXfer[xferCtx->nXferred], rem, NULL);
                usbdc_EndptnTxReportStatus(ep, USBDC_ENDPTN_STATUS_READY);
            } else {
                uint8_t epAddr = tu_edpt_addr(ep, TUSB_DIR_IN);
                dcd_event_xfer_complete(BOARD_TUD_RHPORT, epAddr, xferCtx->nXferred, XFER_RESULT_SUCCESS, true);
                usbdc_EndptnTxReportStatus(ep, USBDC_ENDPTN_STATUS_BUSY);
            }
        } else if (token == PID_OUT) {
            if (USBHSD->INT_ST & USBHS_DEV_UIS_TOG_OK) {
                uint32_t nRxBytes = 0;
                usbdc_EndptnRxLen(ep, &nRxBytes);
                usbdc_EndptnRead(ep, &xferCtx->pDataXfer[xferCtx->nXferred], nRxBytes, NULL);
                xferCtx->nXferred += nRxBytes;
                if (xferCtx->nXferred < xferCtx->totXfer) {
                    usbdc_EndptnRxReportStatus(ep, USBDC_ENDPTN_STATUS_READY);
                } else {
                    uint8_t epAddr = tu_edpt_addr(ep, TUSB_DIR_OUT);
                    dcd_event_xfer_complete(BOARD_TUD_RHPORT, epAddr, xferCtx->nXferred, XFER_RESULT_SUCCESS, true);
                }
            }
        }
    }
}

#endif
