#include "usbdc.h"
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
    USBHS_RCC_Init();
    /* Multiplex GPIO */
    /* Init usbd controller */
    REG_SET_VAL(USBHSD->HOST_CTRL, USBHS_SUSPENDM);
    REG_SET_BITMASK(USBHSD->CONTROL, USBHS_DMA_EN);
    REG_SET_BITMASK(USBHSD->CONTROL, USBHS_INT_BUSY_EN);
    REG_SET_BITMASK(USBHSD->CONTROL, USBHS_FULL_SPEED);
    REG_SET_VAL(USBHSD->INT_FG, 0xffu);
    REG_SET_BITMASK(USBHSD->INT_EN, USBHS_SETUP_ACT_EN);
    REG_SET_BITMASK(USBHSD->INT_EN, USBHS_TRANSFER_EN);
    REG_SET_BITMASK(USBHSD->INT_EN, USBHS_DETECT_EN);
    /* Disable all endpoints */
    REG_SET_VAL(USBHSD->ENDP_CONFIG, 0u);
    REG_SET_VAL(USBHSD->ENDP_TYPE, 0u);
    REG_SET_VAL(USBHSD->BUF_MODE, 0u);
}

/** */
void usbdc_Enable(void)
{
    REG_SET_BITMASK(USBHSD->CONTROL, USBHS_DEV_PU_EN);
}

/** */
void usbdc_Disable(void)
{
    REG_CLR_BITMASK(USBHSD->CONTROL, USBHS_DEV_PU_EN);
}

/** */
void usbdc_SetAddress(
    uint8_t address)
{
    REG_SET_VAL(USBHSD->DEV_AD, address);
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

    /* Configure to be synchronized transfer */
    uint32_t eptPosBitmask = 1u << epNum;
    REG_SET_BITMASK(USBHSD->ENDP_CONFIG, eptPosBitmask);
    /* Set DMA address */
    USB_SET_TX_DMA(epNum, (uint32_t) pDma);
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

    /* Configure to be synchronized transfer */
    uint32_t eptPosBitmask = 1u << (epNum + 16u);
    REG_SET_BITMASK(USBHSD->ENDP_CONFIG, eptPosBitmask);
    /* Set DMA address */
    USB_SET_RX_DMA(epNum, (uint32_t) pDma);
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

    uint32_t eptPosBitmask = 1u << epNum;
    REG_SET_BITMASK(USBHSD->ENDP_CONFIG, eptPosBitmask);
    return true;
}

/** */
bool usbdc_EndptnRxEnable(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;

    uint32_t eptPosBitmask = 1u << (epNum + 16u);
    REG_SET_BITMASK(USBHSD->ENDP_CONFIG, eptPosBitmask);
    return true;
}

/** */
bool usbdc_EndptnTxDisable(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;

    uint32_t eptPosBitmask = 1u << epNum;
    REG_CLR_BITMASK(USBHSD->ENDP_CONFIG, eptPosBitmask);
    return true;
}

/** */
bool usbdc_EndptnRxDisable(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;

    uint32_t eptPosBitmask = 1u << (epNum + 16u);
    REG_CLR_BITMASK(USBHSD->ENDP_CONFIG, eptPosBitmask);
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
    if (!pData)
        return false;
    if (!pReadBytes)
        return false;

    uint32_t nToRead = MIN(USBHSD->RX_LEN, nBytes);
    void *pDma = (void *) USBHSD_RX_DMA(epNum);
    memcpy(pDma, pData, nToRead);
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
    if (!pData)
        return false;
    if (!pWriteBytes)
        return false;

    uint32_t nToSend = MIN(m_endptnTxMps[epNum], nBytes);
    void *pDma = (void *) USBHSD_TX_DMA(epNum);
    memcpy(pData, pDma, nToSend);
    USBHSD_TX_LEN(epNum) = nToSend;
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
    *pTxLen = USBHSD_TX_LEN(0u);
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
    REG_SET_BITFIELD(USBHSD_TX_CTRL(epNum), USBHS_EP_T_TOG_MASK, USBHS_EP_T_TOG_0);
    return true;
}

/** */
bool usbdc_EndptnRxSetData0(
    uint8_t epNum)
{
    if (epNum == 0 || epNum > USBDC_ENDPTN_MAX)
        return false;
    REG_SET_BITFIELD(USBHSD_RX_CTRL(epNum), USBHS_EP_R_TOG_MASK, USBHS_EP_R_TOG_0);
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
    REG_SET_BITFIELD(USBHSD_TX_CTRL(epNum), USBHS_EP_T_RES_MASK, res);
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
    REG_SET_BITFIELD(USBHSD_RX_CTRL(epNum), USBHS_EP_R_RES_MASK, res);
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

    REG_SET_VAL(USBHSD->UEP0_DMA, (uint32_t) pDma);
    m_endpt0Mps = mps;
    return true;
}

/** */
bool usbdc_Endpt0Read(
    void *pData,
    uint32_t nBytes,
    uint32_t *pReadBytes)
{
    if (!pData)
        return false;
    uint32_t nToRead = MIN(USBHSD->RX_LEN, nBytes);
    void *pDma = (void *) USBHSD->UEP0_DMA;
    memcpy(pDma, pData, nToRead);
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
    if (!pData)
        return false;
    uint32_t nToSend = MIN(m_endpt0Mps, nBytes);
    void *pDma = (void *) USBHSD->UEP0_DMA;
    memcpy(pDma, pData, nToSend);
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
    REG_SET_BITFIELD(USB_GET_TX_CTRL(0u), USBHS_EP_T_TOG_MASK, USBHS_EP_T_TOG_0);
}

/** */
void usbdc_Endpt0RxSetData0(void)
{
    REG_SET_BITFIELD(USBHSD_RX_CTRL(0u), USBHS_EP_R_TOG_MASK, USBHS_EP_R_TOG_0);
}

/** */
void usbdc_Endpt0TxToggleDatax(void)
{
    uint32_t reg = USBHSD_TX_CTRL(0u);
    uint32_t tog = reg & USBHS_EP_T_TOG_MASK;
    tog = (tog == USBHS_EP_T_TOG_0) ? USBHS_EP_T_TOG_1 : USBHS_EP_T_TOG_0;
    REG_SET_BITFIELD(USB_GET_TX_CTRL(0u), USBHS_EP_T_TOG_MASK, tog);
}

/** */
void usbdc_Endpt0RxToggleDatax(void)
{
    uint32_t reg = USBHSD_RX_CTRL(0u);
    uint32_t tog = reg & USBHS_EP_R_TOG_MASK;
    tog = (tog == USBHS_EP_R_TOG_0) ? USBHS_EP_R_TOG_1 : USBHS_EP_R_TOG_0;
    REG_SET_BITFIELD(USBHSD_RX_CTRL(0u), USBHS_EP_R_TOG_MASK, tog);
}

/** */
void usbdc_Endpt0TxReportStatus(
    usbdc_tEndptnStatus status)
{
    uint32_t res = endptStatus2TxRes(status);
    REG_SET_BITFIELD(USBHSD_TX_CTRL(0u), USBHS_EP_T_RES_MASK, res);
}
/** */
void usbdc_Endpt0RxReportStatus(
    usbdc_tEndptnStatus status)
{
    uint32_t res = endptStatus2RxRes(status);
    REG_SET_BITFIELD(USBHSD_RX_CTRL(0u), USBHS_EP_R_RES_MASK, res);
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
