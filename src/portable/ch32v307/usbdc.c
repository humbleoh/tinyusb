#include "usbdc.h"

#include "ch32v30x_rcc.h"
#include "ch32v30x_usbhs_device.h"

#include <stdbool.h>
#include <stdint.h>

#define USB_SET_RX_DMA(ep_idx, addr) (*(volatile uint32_t *)((uint32_t)(&USBHSD->UEP1_RX_DMA) + 4 * (ep_idx - 1)) = addr)
#define USB_SET_TX_DMA(ep_idx, addr) (*(volatile uint32_t *)((uint32_t)(&USBHSD->UEP1_TX_DMA) + 4 * (ep_idx - 1)) = addr)
#define USB_SET_MAX_LEN(ep_idx, len) (*(volatile uint16_t *)((uint32_t)(&USBHSD->UEP0_MAX_LEN) + 4 * ep_idx) = len)
#define USB_SET_TX_LEN(ep_idx, len)  (*(volatile uint16_t *)((uint32_t)(&USBHSD->UEP0_TX_LEN) + 4 * ep_idx) = len)
#define USB_GET_TX_LEN(ep_idx)       (*(volatile uint16_t *)((uint32_t)(&USBHSD->UEP0_TX_LEN) + 4 * ep_idx))
#define USB_SET_TX_CTRL(ep_idx, val) (*(volatile uint8_t *)((uint32_t)(&USBHSD->UEP0_TX_CTRL) + 4 * ep_idx) = val)
#define USB_GET_TX_CTRL(ep_idx)      (*(volatile uint8_t *)((uint32_t)(&USBHSD->UEP0_TX_CTRL) + 4 * ep_idx))
#define USB_SET_RX_CTRL(ep_idx, val) (*(volatile uint8_t *)((uint32_t)(&USBHSD->UEP0_RX_CTRL) + 4 * ep_idx) = val)
#define USB_GET_RX_CTRL(ep_idx)      (*(volatile uint8_t *)((uint32_t)(&USBHSD->UEP0_RX_CTRL) + 4 * ep_idx))

#define USBHSD_RX_CTRL(epId)  USB_GET_RX_CTRL(epId)
#define USBHSD_TX_CTRL(epId)  USB_GET_TX_CTRL(epId)
#define USBHSD_TX_LEN(epId)  USB_GET_TX_LEN(epId)
#define USBHSD_MAX_LEN(epId) (*(volatile uint16_t *)((uint32_t)(&USBHSD->UEP0_MAX_LEN) + 4 * epId))
#define USBHSD_RX_DMA(epId)  (*(volatile uint32_t *)((uint32_t)(&USBHSD->UEP1_RX_DMA) + 4 * (epId - 1)))
#define USBHSD_TX_DMA(epId)  (*(volatile uint32_t *)((uint32_t)(&USBHSD->UEP1_TX_DMA) + 4 * (epId - 1)))

#define REG_SET_BITFIELD(reg, mask, val) do { reg = (reg & ~mask) | val; } while (0)
#define REG_CLR_BITFIELD(reg, mask) do { reg = (reg & ~mask); } while (0)
#define REG_SET_BITMASK(reg, mask) do { reg |= mask; } while (0)
#define REG_CLR_BITMASK(reg, mask) do { reg &= ~mask; } while (0)
#define REG_SET_VAL(reg, val) do { reg = val; } while (0)
#define REG_GET_BITMASK(reg, mask) (reg & mask)

#define MIN(a, b) (a < b ? a : b)

static void *m_endpt0pData = NULL;
static void *m_endptnpData[USBDC_ENDPTN_CNT][2] = { };
static uint16_t m_endpt0Mps = 0u;
static uint16_t m_endptnTxMps[USBDC_ENDPTN_CNT] = { 0 };
static uint16_t m_endptnRxMps[USBDC_ENDPTN_CNT] = { 0 };

/** */
static uint32_t endptStatus2RxRes(
    usbdc_tEndptnStatus status);
/** */
static uint32_t endptStatus2TxRes(
    usbdc_tEndptnStatus status);

/** */
void usbdc_Init(void)
{
    /* Disable usbd controller */
    usbdc_Disable();
    /* Init clock */
    RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
    RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
    RCC_USBHSConfig(RCC_USBPLL_Div2);
    RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
    RCC_USBHSPHYPLLALIVEcmd(ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
    /* Multiplex GPIO */
    /* Init usbd controller */
    USBHSD->HOST_CTRL = 0u;
    USBHSD->HOST_CTRL = USBHS_SUSPENDM;
    USBHSD->CONTROL = 0u;
    USBHSD->CONTROL = USBHS_DMA_EN | USBHS_INT_BUSY_EN | USBHS_HIGH_SPEED;
    USBHSD->INT_FG = 0xffu;
    USBHSD->INT_EN = 0u;
    USBHSD->INT_EN |= USBHS_SETUP_ACT_EN;
    USBHSD->INT_EN |= USBHS_TRANSFER_EN;
    USBHSD->INT_EN |= USBHS_DETECT_EN;
    USBHSD->INT_EN |= USBHS_SUSPEND_EN;
    /* Disable all endpoints */
    USBHSD->ENDP_CONFIG = 0u;
    USBHSD->ENDP_TYPE = 0u;
    USBHSD->BUF_MODE = 0u;
}

/** */
void usbdc_Enable(void)
{
    USBHSD->CONTROL |= USBHS_DEV_PU_EN;
}

/** */
void usbdc_Disable(void)
{
    USBHSD->CONTROL &= ~USBHS_DEV_PU_EN;
}

/** */
void usbdc_SetAddress(
    uint8_t address)
{
    USBHSD->DEV_AD = address;
}

/** */
void usbdc_IntEnable(void)
{
    NVIC_EnableIRQ(USBHS_IRQn);
}

/** */
void usbdc_IntDisable(void)
{
    NVIC_DisableIRQ(USBHS_IRQn);
}

/** */
void usbdc_WakeRemote(void)
{
    REG_SET_BITMASK(USBHSH->HOST_CTRL, UHOST_REMOTE_WAKE);
    Delay_Us( 200 );
    REG_CLR_BITMASK(USBHSH->HOST_CTRL, UHOST_REMOTE_WAKE);
}

/** */
bool usbdc_EndptnTxConfig(
    uint8_t epNum,
    uint16_t mps,
    void *pDma)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    if (mps > USBDC_ENDPTN_MPS_MAX)
        return false;
    if (!pDma)
        return false;

    if (mps > USBHSD_MAX_LEN(epNum))
        USBHSD_MAX_LEN(epNum) = mps;
    /* Configure to be synchronized transfer */
    uint32_t eptPosBitmask = 1u << epNum;
    USBHSD->ENDP_CONFIG |= eptPosBitmask;
    /* Set DMA address */
    USB_SET_TX_DMA(epNum, (uint32_t) pDma);
    m_endptnpData[epNum][1] = pDma;
    /* Bookkeep mps */
    m_endptnTxMps[epNum] = mps;

    return true;
}

/** */
bool usbdc_EndptnRxConfig(
    uint8_t epNum,
    uint16_t mps,
    void *pDma)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    if (mps > USBDC_ENDPTN_MPS_MAX)
        return false;
    if (!pDma)
        return false;

    if (mps > USBHSD_MAX_LEN(epNum))
        USBHSD_MAX_LEN(epNum) = mps;
    /* Configure to be synchronized transfer */
    uint32_t eptPosBitmask = 1u << (epNum + 16u);
    USBHSD->ENDP_CONFIG |= eptPosBitmask;
    /* Set DMA address */
    USB_SET_RX_DMA(epNum, (uint32_t) pDma);
    m_endptnpData[epNum][0] = pDma;
    /* Bookkeep mps */
    m_endptnRxMps[epNum] = mps;

    return true;
}
/** */
bool usbdc_EndptnTxEnable(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    USBHSD_TX_CTRL(epNum) &= USBHS_EP_T_TOG_MASK;
    USBHSD_TX_CTRL(epNum) |= USBHS_EP_T_AUTOTOG;
    uint32_t eptPosBitmask = 1u << epNum;
    USBHSD->ENDP_CONFIG |= eptPosBitmask;
    return true;
}

/** */
bool usbdc_EndptnRxEnable(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    USBHSD_RX_CTRL(epNum) &= USBHS_EP_R_TOG_MASK;
    USBHSD_RX_CTRL(epNum) |= USBHS_EP_R_AUTOTOG;
    uint32_t eptPosBitmask = 1u << (epNum + 16u);
    USBHSD->ENDP_CONFIG |= eptPosBitmask;
    return true;
}

/** */
bool usbdc_EndptnTxDisable(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    uint32_t eptPosBitmask = 1u << epNum;
    USBHSD->ENDP_CONFIG &= ~eptPosBitmask;
    USBHSD_TX_LEN(epNum) = 0;
    USB_SET_TX_DMA(epNum, (uint32_t) NULL);
    return true;
}

/** */
bool usbdc_EndptnRxDisable(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    uint32_t eptPosBitmask = 1u << (epNum + 16u);
    USBHSD->ENDP_CONFIG &= ~eptPosBitmask;
    USB_SET_RX_DMA(epNum, (uint32_t) NULL);
    return true;
}

/** */
bool usbdc_EndptnRead(
    uint8_t epNum,
    void *pData,
    uint32_t nBytes,
    uint32_t *pReadBytes)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    if (!pData && nBytes > 0)
        return false;
    uint32_t nToRead = MIN(USBHSD->RX_LEN, nBytes);
    if (pData)
        memcpy(pData, m_endptnpData[epNum][0], nToRead);
    if (pReadBytes)
        *pReadBytes = nToRead;
    return true;
}

/** */
bool usbdc_EndptnWrite(
    uint8_t epNum,
    void *pData,
    uint32_t nBytes,
    uint32_t *pWriteBytes)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    if (!pData && nBytes > 0u)
        return false;
    uint32_t nToSend = MIN(m_endptnTxMps[epNum], nBytes);
    if (pData)
        memcpy(m_endptnpData[epNum][1], pData, nToSend);
    USBHSD_TX_LEN(epNum) = nToSend;
    if (pWriteBytes)
        *pWriteBytes = nToSend;
    return true;
}

/** */
bool usbdc_EndptnTxLen(
    uint8_t epNum,
    uint32_t *pTxLen)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    if (!pTxLen)
        return false;
    *pTxLen = USBHSD_TX_LEN(epNum);
    return true;
}

/** */
bool usbdc_EndptnRxLen(
    uint8_t epNum,
    uint32_t *pRxLen)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    if (!pRxLen)
        return false;
    *pRxLen = USBHSD->RX_LEN;
    return true;
}

/** */
bool usbdc_EndptnTxSetData0(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    USBHSD_TX_CTRL(epNum) &= ~USBHS_EP_T_TOG_MASK;
    USBHSD_TX_CTRL(epNum) |= USBHS_EP_T_TOG_0;
    return true;
}

/** */
bool usbdc_EndptnRxSetData0(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    USBHSD_RX_CTRL(epNum) &= ~USBHS_EP_R_TOG_MASK;
    USBHSD_RX_CTRL(epNum) |= USBHS_EP_R_TOG_0;
    return true;
}

/** */
bool usbdc_EndptnTxToggleDatax(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;

    uint32_t reg = USBHSD_TX_CTRL(epNum);
    uint32_t tog = reg & USBHS_EP_T_TOG_MASK;
    tog = (tog == USBHS_EP_T_TOG_0) ? USBHS_EP_T_TOG_1 : USBHS_EP_T_TOG_0;
    REG_SET_BITFIELD(USBHSD_TX_CTRL(epNum), USBHS_EP_T_TOG_MASK, tog);
    return true;
}

/** */
bool usbdc_EndptnRxToggleDatax(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;

    uint32_t reg = USBHSD_RX_CTRL(epNum);
    uint32_t tog = reg & USBHS_EP_R_TOG_MASK;
    tog = (tog == USBHS_EP_R_TOG_0) ? USBHS_EP_R_TOG_1 : USBHS_EP_R_TOG_0;
    REG_SET_BITFIELD(USBHSD_RX_CTRL(epNum), USBHS_EP_R_TOG_MASK, tog);
    return true;
}

bool usbdc_EndptnTxReportStatus(
    uint8_t epNum,
    usbdc_tEndptnStatus status)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    uint32_t res = endptStatus2TxRes(status);
    USBHSD_TX_CTRL(epNum) &= ~USBHS_EP_T_RES_MASK;
    USBHSD_TX_CTRL(epNum) |= res;
    return true;
}

/** */
bool usbdc_EndptnRxReportStatus(
    uint8_t epNum,
    usbdc_tEndptnStatus status)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    uint32_t res = endptStatus2RxRes(status);
    USBHSD_RX_CTRL(epNum) &= ~USBHS_EP_R_RES_MASK;
    USBHSD_RX_CTRL(epNum) |= res;
    return true;
}

/** */
bool usbdc_Endpt0Config(
    uint16_t mps,
    void *pDma)
{
    if (mps > USBDC_ENDPT0_MPS_MAX)
        return false;
    if (!pDma)
        return false;
    m_endpt0pData = pDma;
    USBHSD->UEP0_DMA = (uint32_t) pDma;
    USBHSD->UEP0_MAX_LEN = mps;
    m_endpt0Mps = mps;
    return true;
}

/** */
bool usbdc_Endpt0Read(
    void *pData,
    uint32_t nBytes,
    uint32_t *pReadBytes)
{
    if (!pData && nBytes == 0u)
        return false;
    uint32_t nToRead = MIN(USBHSD->RX_LEN, nBytes);
    if (pData)
        memcpy(pData, m_endpt0pData, nToRead);
    if (pReadBytes)
        *pReadBytes = nToRead;
    return true;
}

/** */
bool usbdc_Endpt0Write(
    void *pData,
    uint32_t nBytes,
    uint32_t *pWriteBytes)
{
    if (!pData && nBytes > 0u)
        return false;
    uint32_t nToSend = MIN(m_endpt0Mps, nBytes);
    if (pData)
        memcpy(m_endpt0pData, pData, nToSend);
    USBHSD_TX_LEN(0u) = nToSend;
    if (pWriteBytes)
        *pWriteBytes = nToSend;
    return true;
}

/** */
uint32_t usbdc_Endpt0TxLen(void)
{
    return USBHSD_TX_LEN(0u);
}

/** */
uint32_t usbdc_Endpt0RxLen(void)
{
    return USBHSD->RX_LEN;
}

/** */
void usbdc_Endpt0TxSetData0(void)
{
    USB_GET_TX_CTRL(0u) &= ~USBHS_EP_T_TOG_MASK;
    USB_GET_TX_CTRL(0u) |= USBHS_EP_T_TOG_0;
}

/** */
void usbdc_Endpt0RxSetData0(void)
{
    USBHSD_RX_CTRL(0u) &= ~USBHS_EP_R_TOG_MASK;
    USBHSD_RX_CTRL(0u) |= USBHS_EP_R_TOG_0;
}

/** */
void usbdc_Endpt0TxToggleDatax(void)
{
    uint32_t reg = USBHSD_TX_CTRL(0u);
    uint32_t tog = reg & USBHS_EP_T_TOG_MASK;
    tog = (tog == USBHS_EP_T_TOG_0) ? USBHS_EP_T_TOG_1 : USBHS_EP_T_TOG_0;
    USBHSD_TX_CTRL(0u) &= ~USBHS_EP_T_TOG_MASK;
    USBHSD_TX_CTRL(0u) |= tog;
}

/** */
void usbdc_Endpt0RxToggleDatax(void)
{
    uint32_t reg = USBHSD_RX_CTRL(0u);
    uint32_t tog = reg & USBHS_EP_R_TOG_MASK;
    tog = (tog == USBHS_EP_R_TOG_0) ? USBHS_EP_R_TOG_1 : USBHS_EP_R_TOG_0;
    USBHSD_RX_CTRL(0u) &= ~USBHS_EP_R_TOG_MASK;
    USBHSD_RX_CTRL(0u) |= tog;
}

/** */
void usbdc_Endpt0TxReportStatus(
    usbdc_tEndptnStatus status)
{
    uint32_t res = endptStatus2TxRes(status);
    USBHSD_TX_CTRL(0u) &= ~USBHS_EP_T_RES_MASK;
    USBHSD_TX_CTRL(0u) |= res;
}
/** */
void usbdc_Endpt0RxReportStatus(
    usbdc_tEndptnStatus status)
{
    uint32_t res = endptStatus2RxRes(status);
    USBHSD_RX_CTRL(0u) &= ~USBHS_EP_R_RES_MASK;
    USBHSD_RX_CTRL(0u) |= res;
}

/** */
static uint32_t endptStatus2RxRes(
    usbdc_tEndptnStatus status)
{
    uint32_t res = USBHS_EP_R_RES_STALL;
    switch (status) {
    case USBDC_ENDPTN_STATUS_READY:
        res = USBHS_EP_R_RES_ACK;
        break;
    case USBDC_ENDPTN_STATUS_BUSY:
        res = USBHS_EP_R_RES_NAK;
        break;
    case USBDC_ENDPTN_STATUS_ERROR:
    default:
        res = USBHS_EP_R_RES_STALL;
        break;
    }

    return res;
}

/** */
static uint32_t endptStatus2TxRes(
    usbdc_tEndptnStatus status)
{
    uint32_t res = USBHS_EP_R_RES_STALL;
    switch (status) {
    case USBDC_ENDPTN_STATUS_READY:
        res = USBHS_EP_T_RES_ACK;
        break;
    case USBDC_ENDPTN_STATUS_BUSY:
        res = USBHS_EP_T_RES_NAK;
        break;
    case USBDC_ENDPTN_STATUS_ERROR:
    default:
        res = USBHS_EP_T_RES_STALL;
        break;
    }

    return res;
}
