/*
 * usbdc.h
 *
 *  Created on: 11 Oct 2022
 *      Author: osheng
 */

#ifndef TINYUSB_SRC_PORTABLE_CH32V307_USBDC_H_
#define TINYUSB_SRC_PORTABLE_CH32V307_USBDC_H_

#include <stdbool.h>
#include <stdint.h>

/** NOTES:
1. Don't support isochornous transfer
2. DMA transfer only
*/

//#define USBDC_ENDPTN_MPS_MAX   ( 1024u )
#define USBDC_ENDPTN_MPS_MAX   ( 64u )
#define USBDC_ENDPT0_MPS_MAX   ( 64u )
#define USBDC_ENDPTN_CNT       ( 16u )
#define USBDC_ENDPTN_MAX       ( USBDC_ENDPTN_CNT - 1u )

/** */
typedef enum {
    USBDC_ENDPTN_STATUS_READY = 0u,
    USBDC_ENDPTN_STATUS_BUSY,
    USBDC_ENDPTN_STATUS_ERROR
} usbdc_tEndptnStatus;

/** */
void usbdc_Init(void);
/** */
void usbdc_Enable(void);
/** */
void usbdc_Disable(void);
/** */
void usbdc_SetAddress(
    uint8_t address);
/** */
void usbdc_IntEnable(void);
/** */
void usbdc_IntDisable(void);
/** */
void usbdc_WakeRemote(void);

/** */
bool usbdc_EndptnTxConfig(
    uint8_t epNum,
    uint16_t mps,
    void *pDma);
/** */
bool usbdc_EndptnRxConfig(
    uint8_t epNum,
    uint16_t mps,
    void *pDma);
/** */
bool usbdc_EndptnTxEnable(
    uint8_t epNum);
/** */
bool usbdc_EndptnRxEnable(
    uint8_t epNum);
/** */
bool usbdc_EndptnTxDisable(
    uint8_t epNum);
/** */
bool usbdc_EndptnRxDisable(
    uint8_t epNum);
/** */
bool usbdc_EndptnRead(
    uint8_t epNum,
    void *pData,
    uint32_t nBytes,
    uint32_t *pReadBytes);
/** */
bool usbdc_EndptnWrite(
    uint8_t epNum,
    void *pData,
    uint32_t nBytes,
    uint32_t *pWriteBytes);
/** */
bool usbdc_EndptnTxLen(
    uint8_t epNum,
    uint32_t *pTxLen);
/** */
bool usbdc_EndptnRxLen(
    uint8_t epNum,
    uint32_t *pRxLen);
/** */
bool usbdc_EndptnTxSetData0(
    uint8_t epNum);
/** */
bool usbdc_EndptnRxSetData0(
    uint8_t epNum);
/** */
bool usbdc_EndptnTxToggleDatax(
    uint8_t epNum);
/** */
bool usbdc_EndptnRxToggleDatax(
    uint8_t epNum);
/** */
bool usbdc_EndptnTxReportStatus(
    uint8_t epNum,
    usbdc_tEndptnStatus status);
/** */
bool usbdc_EndptnRxReportStatus(
    uint8_t epNum,
    usbdc_tEndptnStatus status);

/** */
bool usbdc_Endpt0Config(
    uint16_t mps,
    void *pDma);
/** */
bool usbdc_Endpt0Read(
    void *pData,
    uint32_t nBytes,
    uint32_t *pReadBytes);
/** */
bool usbdc_Endpt0Write(
    void *pData,
    uint32_t nBytes,
    uint32_t *pWriteBytes);
/** */
uint32_t usbdc_Endpt0TxLen(void);
/** */
uint32_t usbdc_Endpt0RxLen(void);
/** */
void usbdc_Endpt0TxSetData0(void);
/** */
void usbdc_Endpt0RxSetData0(void);
/** */
void usbdc_Endpt0TxToggleDatax(void);
/** */
void usbdc_Endpt0RxToggleDatax(void);
/** */
void usbdc_Endpt0TxReportStatus(
    usbdc_tEndptnStatus status);
/** */
void usbdc_Endpt0RxReportStatus(
    usbdc_tEndptnStatus status);

#endif /* TINYUSB_SRC_PORTABLE_CH32V307_USBDC_H_ */
