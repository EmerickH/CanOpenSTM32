/*
 * CAN module object for STM32 (FD)CAN peripheral IP.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Hamed Jafarzadeh 	2022
 * 				Tilen Marjerle		2021
 * 				Janez Paternoster	2020
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Implementation Author:               Tilen Majerle <tilen@majerle.eu>
 */
#include "301/CO_driver.h"
#include "CO_app_STM32.h"
#if CO_STM32_PHY_COUNT > 1
#include <string.h>
#endif

/* Local CAN module object */
static CO_CANmodule_t* CANModule_local = NULL; /* Local instance of global CAN module */
volatile uint32_t CO_CANrxLostFrames = 0;       /* See CO_driver_target.h */

/* CAN masks for identifiers */
#define CANID_MASK 0x07FF /*!< CAN standard ID mask */
#define FLAG_RTR   0x8000 /*!< RTR flag, part of identifier */

#ifdef CO_STM32_FDCAN_Driver
#ifndef FDCAN_BUFFER_INDEXES
#if defined(FDCAN_TX_BUFFER31)
#define FDCAN_BUFFER_INDEXES 0xFFFFFFFFU
#elif defined(FDCAN_TX_BUFFER2)
#define FDCAN_BUFFER_INDEXES FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2
#else
#define FDCAN_BUFFER_INDEXES 0xFFFFFFFFU
#warning "FDCAN_BUFFER_INDEXES not defined"
#endif
#endif
#endif /* CO_STM32_FDCAN_Driver */

/******************************************************************************/
void
CO_CANsetConfigurationMode(void* CANptr) {
    /* Put CAN module in configuration mode */
    if (CANptr != NULL) {
#ifdef CO_STM32_FDCAN_Driver
        HAL_FDCAN_Stop(((CANopenNodeSTM32*)CANptr)->CANHandle);
#if CO_STM32_PHY_COUNT > 1
        HAL_FDCAN_Stop(((CANopenNodeSTM32*)CANptr)->CANHandle2);
#endif
#else
        HAL_CAN_Stop(((CANopenNodeSTM32*)CANptr)->CANHandle);
#endif
    }
}

/******************************************************************************/
void
CO_CANsetNormalMode(CO_CANmodule_t* CANmodule) {
    /* Put CAN module in normal mode */
    if (CANmodule->CANptr != NULL) {
#ifdef CO_STM32_FDCAN_Driver
#if CO_STM32_PHY_COUNT > 1
        /* Both phys start "up" for CO_CANPHY_UP_MS, so that the first broadcasts (NMT reset, TIME) go out */
        uint32_t now = HAL_GetTick();
        for (uint8_t p = 0; p < CO_STM32_PHY_COUNT; p++) {
            CANmodule->lastRxMs[p] = now;
            CANmodule->up[p] = true;
        }
        HAL_FDCAN_Start(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle2);
#endif
        if (HAL_FDCAN_Start(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle) == HAL_OK)
#else
        if (HAL_CAN_Start(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle) == HAL_OK)
#endif
        {
            CANmodule->CANnormal = true;
        }
    }
}

/******************************************************************************/
CO_ReturnError_t
CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                  uint16_t txSize, uint16_t CANbitRate) {

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Hold CANModule variable */
    CANmodule->CANptr = CANptr;

    /* Keep a local copy of CANModule */
    CANModule_local = CANmodule;

    /* Configure object variables */
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false; /* Do not use HW filters */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;
#if CO_STM32_PHY_COUNT > 1
    for (uint8_t p = 0; p < CO_STM32_PHY_COUNT; p++) {
        CANmodule->errOldPhy[p] = 0U;
        CANmodule->lastRxMs[p] = 0U;
        CANmodule->up[p] = false;
        CANmodule->lost[p] = 0U;
        CANmodule->dropped[p] = 0U;
        CANmodule->rxFrames[p] = 0U;
        CANmodule->txFrames[p] = 0U;
        CANmodule->rxBits[p] = 0U;
        CANmodule->txBits[p] = 0U;
    }
#endif

    /* Reset all variables */
    for (uint16_t i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (uint16_t i = 0U; i < txSize; i++) {
        txArray[i].bufferFull = false;
#if CO_STM32_PHY_COUNT > 1
        txArray[i].phyPending = 0U;
#endif
    }

    /***************************************/
    /* STM32 related configuration */
    /***************************************/
    if (((CANopenNodeSTM32*)CANptr)->HWInitFunction != NULL) {
        ((CANopenNodeSTM32*)CANptr)->HWInitFunction();
    }
#if CO_STM32_PHY_COUNT > 1
    if (((CANopenNodeSTM32*)CANptr)->HWInitFunction2 != NULL) {
        ((CANopenNodeSTM32*)CANptr)->HWInitFunction2();
    }
#endif

    /*
     * Configure global filter that is used as last check if message did not pass any of other filters:
     *
     * We do not rely on hardware filters in this example
     * and are performing software filters instead
     *
     * Accept non-matching standard ID messages
     * Reject non-matching extended ID messages
     */

#ifdef CO_STM32_FDCAN_Driver
    if (HAL_FDCAN_ConfigGlobalFilter(((CANopenNodeSTM32*)CANptr)->CANHandle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)
        != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#if CO_STM32_PHY_COUNT > 1
    if (HAL_FDCAN_ConfigGlobalFilter(((CANopenNodeSTM32*)CANptr)->CANHandle2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)
        != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#endif
#else
    CAN_FilterTypeDef FilterConfig;
#if defined(CAN)
    FilterConfig.FilterBank = 0;
#else
    if (((CAN_HandleTypeDef*)((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle)->Instance == CAN1) {
        FilterConfig.FilterBank = 0;
    } else {
        FilterConfig.FilterBank = 14;
    }
#endif
    FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    FilterConfig.FilterIdHigh = 0x0;
    FilterConfig.FilterIdLow = 0x0;
    FilterConfig.FilterMaskIdHigh = 0x0;
    FilterConfig.FilterMaskIdLow = 0x0;
    FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    FilterConfig.FilterActivation = ENABLE;
    FilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(((CANopenNodeSTM32*)CANptr)->CANHandle, &FilterConfig) != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#endif
    /* Enable notifications */
    /* Activate the CAN notification interrupts */
#ifdef CO_STM32_FDCAN_Driver
    if (HAL_FDCAN_ActivateNotification(((CANopenNodeSTM32*)CANptr)->CANHandle,
                                       0 | FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                                           | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST
                                           | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_ABORT_COMPLETE
                                           | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                                           | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                           | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                       FDCAN_BUFFER_INDEXES)
        != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#if CO_STM32_PHY_COUNT > 1
    if (HAL_FDCAN_ActivateNotification(((CANopenNodeSTM32*)CANptr)->CANHandle2,
                                       0 | FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                                           | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST
                                           | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_ABORT_COMPLETE
                                           | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                                           | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                           | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                       FDCAN_BUFFER_INDEXES)
        != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#endif
#else
    if (HAL_CAN_ActivateNotification(((CANopenNodeSTM32*)CANptr)->CANHandle, CAN_IT_RX_FIFO0_MSG_PENDING
                                                                                 | CAN_IT_RX_FIFO1_MSG_PENDING
                                                                                 | CAN_IT_TX_MAILBOX_EMPTY)
        != HAL_OK) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
#endif

    return CO_ERROR_NO;
}

/******************************************************************************/
void
CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
    if (CANmodule != NULL && CANmodule->CANptr != NULL) {
#ifdef CO_STM32_FDCAN_Driver
        HAL_FDCAN_Stop(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle);
#if CO_STM32_PHY_COUNT > 1
        HAL_FDCAN_Stop(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle2);
#endif
#else
        HAL_CAN_Stop(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle);
#endif
    }
}

/******************************************************************************/
CO_ReturnError_t
CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object,
                   void (*CANrx_callback)(void* object, void* message)) {
    CO_ReturnError_t ret = CO_ERROR_NO;

    if (CANmodule != NULL && object != NULL && CANrx_callback != NULL && index < CANmodule->rxSize) {
        CO_CANrx_t* buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /*
         * Configure global identifier, including RTR bit
         *
         * This is later used for RX operation match case
         */
        buffer->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0x00);
        buffer->mask = (mask & CANID_MASK) | FLAG_RTR;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {
            __NOP();
        }
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

/******************************************************************************/
CO_CANtx_t*
CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag) {
    CO_CANtx_t* buffer = NULL;

    if (CANmodule != NULL && index < CANmodule->txSize) {
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer */
        buffer->ident = ((uint32_t)ident & CANID_MASK) | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
#if CO_STM32_PHY_COUNT > 1
        buffer->phyPending = 0U;
#endif
        buffer->syncFlag = syncFlag;
    }
    return buffer;
}

#if CO_STM32_PHY_COUNT > 1
/* Two physical buses. phy 0 = CANHandle, phy 1 = CANHandle2. */

static FDCAN_HandleTypeDef*
prv_phy_handle(CO_CANmodule_t* CANmodule, uint8_t phy) {
    CANopenNodeSTM32* app = (CANopenNodeSTM32*)CANmodule->CANptr;
    return phy ? app->CANHandle2 : app->CANHandle;
}

static uint8_t
prv_phy_of(CO_CANmodule_t* CANmodule, FDCAN_HandleTypeDef* hfdcan) {
    return hfdcan == ((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle2 ? 1U : 0U;
}

static const uint32_t prv_dlc_map[9] = {FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
                                        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
                                        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};

/**
 * \brief           Put one frame in the TX FIFO of a phy. Must be called with atomic access.
 * \param[in]       ident: 11 bits identifier, with FLAG_RTR
 * \return          1 if the frame was queued
 */
static uint8_t
prv_send_frame(FDCAN_HandleTypeDef* hfdcan, uint32_t ident, uint8_t dlc, uint8_t* data) {
    static FDCAN_TxHeaderTypeDef tx_hdr; /* shared by both phys: always filled under CO_LOCK_CAN_SEND */

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U) {
        return 0;
    }
    tx_hdr.Identifier = ident & CANID_MASK;
    tx_hdr.TxFrameType = (ident & FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx_hdr.IdType = FDCAN_STANDARD_ID;
    tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
    tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
    tx_hdr.MessageMarker = 0;
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_hdr.DataLength = prv_dlc_map[dlc > 8U ? 8U : dlc];
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_hdr, data) == HAL_OK;
}

/**
 * \brief           Route a buffer to the phys of the mask and send it. Must be called with atomic access.
 *                  The buffer itself is never rewritten: each phy gets a rewritten copy.
 * \return          the phys (bits) that are routed, up, and could not take the frame now
 */
static uint8_t
prv_route_send(CO_CANmodule_t* CANmodule, const CO_CANtx_t* buffer, uint8_t phys, bool* sent) {
    uint8_t pending = 0;

    for (uint8_t p = 0; p < CO_STM32_PHY_COUNT; p++) {
        if (!(phys & (1U << p))) {
            continue;
        }
        uint16_t ident = (uint16_t)(buffer->ident & CANID_MASK);
        uint8_t data[8];
        memcpy(data, buffer->data, sizeof(data));
        if (!CO_CANphyTx(p, &ident, data, buffer->DLC)) {
            continue; /* the frame is not for this bus */
        }
        if (!CANmodule->up[p]) {
            CANmodule->dropped[p]++;
            continue;
        }
        if (prv_send_frame(prv_phy_handle(CANmodule, p), (buffer->ident & FLAG_RTR) | ident, buffer->DLC, data)) {
            *sent = true;
            CANmodule->txFrames[p]++;
            CANmodule->txBits[p] += CO_CANPHY_FRAME_BITS(buffer->DLC);
        } else {
            pending |= (uint8_t)(1U << p);
        }
    }
    return pending;
}

/*
 * Sends the frames of the queue still to be sent on the phys of the mask, in the order of the buffers. A phy whose
 * TX FIFO is full gets nothing more in this call, so that a later buffer doesn't overtake an earlier one on it.
 * CANtxCount is counted again: a lost interrupt can't leave it wrong. Must be called with CO_LOCK_CAN_SEND.
 * (protronic/CanOpenSTM32#1, for two phys)
 */
static void
prv_flush_tx_queue(CO_CANmodule_t* CANmodule, uint8_t phys) {
    uint16_t count = 0U;
    CO_CANtx_t* buffer = &CANmodule->txArray[0];
    for (uint16_t i = CANmodule->txSize; i > 0U; i--, buffer++) {
        if (!buffer->bufferFull) {
            continue;
        }
        uint8_t mask = buffer->phyPending & phys;
        if (mask != 0U) {
            bool sent = false;
            uint8_t again = prv_route_send(CANmodule, buffer, mask, &sent);
            if (sent) {
                CANmodule->bufferInhibitFlag = buffer->syncFlag;
            }
            buffer->phyPending = (uint8_t)((buffer->phyPending & ~mask) | again);
            phys &= (uint8_t)~again; /* its TX FIFO is full */
        }
        if (buffer->phyPending == 0U) {
            buffer->bufferFull = false;
        } else {
            count++;
        }
    }
    CANmodule->CANtxCount = count;
}

/* A phy went down: abort what it has in its TX FIFO and forget the frames still to be sent on it */
static void
prv_phy_down(CO_CANmodule_t* CANmodule, uint8_t phy) {
    CO_LOCK_CAN_SEND(CANmodule);
    /* a frame may have come in since the caller checked */
    if ((int32_t)(HAL_GetTick() - CANmodule->lastRxMs[phy]) >= (int32_t)CO_CANPHY_UP_MS) {
        CANmodule->up[phy] = false;
        HAL_FDCAN_AbortTxRequest(prv_phy_handle(CANmodule, phy), FDCAN_BUFFER_INDEXES);
        CO_CANtx_t* buffer = &CANmodule->txArray[0];
        for (uint16_t i = CANmodule->txSize; i > 0U; i--, buffer++) {
            buffer->phyPending &= (uint8_t)~(1U << phy);
            if (buffer->bufferFull && buffer->phyPending == 0U) {
                buffer->bufferFull = false;
                CANmodule->CANtxCount--;
            }
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
}
#else
/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       CANmodule: CAN module instance
 * \param[in]       buffer: Pointer to buffer to transmit
 */
static uint8_t
prv_send_can_message(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {

    uint8_t success = 0;

    /* Check if TX FIFO is ready to accept more messages */
#ifdef CO_STM32_FDCAN_Driver
    static FDCAN_TxHeaderTypeDef tx_hdr;
    if (HAL_FDCAN_GetTxFifoFreeLevel(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle) > 0) {
        /*
         * RTR flag is part of identifier value
         * hence it needs to be properly decoded
         */
        tx_hdr.Identifier = buffer->ident & CANID_MASK;
        tx_hdr.TxFrameType = (buffer->ident & FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
        tx_hdr.IdType = FDCAN_STANDARD_ID;
        tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
        tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
        tx_hdr.MessageMarker = 0;
        tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

        switch (buffer->DLC) {
            case 0:
                tx_hdr.DataLength = FDCAN_DLC_BYTES_0;
                break;
            case 1:
                tx_hdr.DataLength = FDCAN_DLC_BYTES_1;
                break;
            case 2:
                tx_hdr.DataLength = FDCAN_DLC_BYTES_2;
                break;
            case 3:
                tx_hdr.DataLength = FDCAN_DLC_BYTES_3;
                break;
            case 4:
                tx_hdr.DataLength = FDCAN_DLC_BYTES_4;
                break;
            case 5:
                tx_hdr.DataLength = FDCAN_DLC_BYTES_5;
                break;
            case 6:
                tx_hdr.DataLength = FDCAN_DLC_BYTES_6;
                break;
            case 7:
                tx_hdr.DataLength = FDCAN_DLC_BYTES_7;
                break;
            case 8:
                tx_hdr.DataLength = FDCAN_DLC_BYTES_8;
                break;
            default: /* Hard error... */
                break;
        }

        /* Now add message to FIFO. Should not fail */
        success =
            HAL_FDCAN_AddMessageToTxFifoQ(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle, &tx_hdr, buffer->data)
            == HAL_OK;
    }
#else
    static CAN_TxHeaderTypeDef tx_hdr;
    /* Check if TX FIFO is ready to accept more messages */
    if (HAL_CAN_GetTxMailboxesFreeLevel(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle) > 0) {
        /*
         * RTR flag is part of identifier value
         * hence it needs to be properly decoded
         */
        tx_hdr.ExtId = 0u;
        tx_hdr.IDE = CAN_ID_STD;
        tx_hdr.DLC = buffer->DLC;
        tx_hdr.StdId = buffer->ident & CANID_MASK;
        tx_hdr.RTR = (buffer->ident & FLAG_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;

        uint32_t TxMailboxNum; // Transmission MailBox number

        /* Now add message to FIFO. Should not fail */
        success = HAL_CAN_AddTxMessage(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle, &tx_hdr, buffer->data,
                                       &TxMailboxNum)
                  == HAL_OK;
    }
#endif
    return success;
}

/*
 * Sends the frames of the queue, in the order of the buffers, until the hardware is full. A buffer is free once
 * given to the hardware. CANtxCount is counted again: a lost interrupt can't leave it wrong. Must be called with
 * CO_LOCK_CAN_SEND. (protronic/CanOpenSTM32#1)
 */
static void
prv_flush_tx_queue(CO_CANmodule_t* CANmodule) {
    uint16_t pending = 0U;
    bool_t hwFull = false;
    CO_CANtx_t* buffer = &CANmodule->txArray[0];
    for (uint16_t i = CANmodule->txSize; i > 0U; --i, ++buffer) {
        if (!buffer->bufferFull) {
            continue;
        }
        if (!hwFull) {
            if (prv_send_can_message(CANmodule, buffer)) {
                buffer->bufferFull = false;
                CANmodule->bufferInhibitFlag = buffer->syncFlag;
                continue;
            }
            hwFull = true; /* the next ones wait */
        }
        pending++;
    }
    CANmodule->CANtxCount = pending;
}
#endif

/******************************************************************************/
CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    /*
     * Send message to CAN network
     *
     * Lock interrupts for atomic operation
     */
    CO_LOCK_CAN_SEND(CANmodule);
#if CO_STM32_PHY_COUNT > 1
    {
        /* Queued for every phy and sent through the queue: on a phy with frames waiting, the frame doesn't overtake
         * them. The phys it isn't for, or that are down, drop it at once (prv_route_send). */
        buffer->phyPending = (uint8_t)((1U << CO_STM32_PHY_COUNT) - 1U);
        buffer->bufferFull = true;
        prv_flush_tx_queue(CANmodule, buffer->phyPending);
    }
#else
    /* Straight to the hardware only when nothing waits: it would overtake the frames of the queue
     * (protronic/CanOpenSTM32#1) */
    if (CANmodule->CANtxCount == 0U && prv_send_can_message(CANmodule, buffer)) {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
    } else {
        /* Only increment count if buffer wasn't already full */
        if (!buffer->bufferFull) {
            buffer->bufferFull = true;
            CANmodule->CANtxCount++;
        }
        /* Behind the frames waiting, at once if the hardware has room */
        prv_flush_tx_queue(CANmodule);
    }
#endif
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

#ifdef CO_STM32_FDCAN_Driver
/*
 * Bus off: the FDCAN sets CCCR.INIT and stays off the bus until the software clears it. Started again, it rejoins
 * the bus after 129 times 11 recessive bits; the frames of its TX FIFO are lost, the queue is sent again
 * (prv_flush_tx_queue). Only while INIT is set: a recovery under way (PSR.BO still set) is not started over. A stop
 * of the application (HAL state not busy) is left alone. (protronic/CanOpenSTM32#1)
 */
static void
prv_busoff_recover(FDCAN_HandleTypeDef* hfdcan) {
    if ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U && (hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0U
        && hfdcan->State == HAL_FDCAN_STATE_BUSY) {
        if (HAL_FDCAN_Stop(hfdcan) == HAL_OK) {
            (void)HAL_FDCAN_Start(hfdcan);
        }
    }
}
#endif

/******************************************************************************/
void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag) {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount > 0) {
        uint16_t i;
        CO_CANtx_t* buffer = &CANmodule->txArray[0];
        for (i = CANmodule->txSize; i > 0U; i--) {
            if (buffer->bufferFull) {
                if (buffer->syncFlag) {
#if CO_STM32_PHY_COUNT > 1
                    buffer->phyPending = 0U;
#endif
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    if (tpdoDeleted) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
    * different way to determine errors. */

#if CO_STM32_PHY_COUNT > 1
void
CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
    uint32_t errAll = 0;
    bool changed = false;
    uint32_t now = HAL_GetTick();

    for (uint8_t p = 0; p < CO_STM32_PHY_COUNT; p++) {
        prv_busoff_recover(prv_phy_handle(CANmodule, p));
        /* signed difference: a frame may be stamped by an interrupt after "now" was read */
        if (CANmodule->up[p] && (int32_t)(now - CANmodule->lastRxMs[p]) >= (int32_t)CO_CANPHY_UP_MS) {
            prv_phy_down(CANmodule, p);
        }
        /* The errors of a bus that is down (unused connector) don't count */
        uint32_t err = 0;
        if (CANmodule->up[p]) {
            err = prv_phy_handle(CANmodule, p)->Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EW | FDCAN_PSR_EP);
        }
        if (CANmodule->errOldPhy[p] != err) {
            CANmodule->errOldPhy[p] = err;
            changed = true;
        }
        errAll |= err;
    }

    if (changed) {
        uint16_t status = CANmodule->CANerrorStatus;

        if (errAll & FDCAN_PSR_BO) {
            status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            status &= 0xFFFF
                      ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
                         | CO_CAN_ERRTX_PASSIVE);
            if (errAll & FDCAN_PSR_EW) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
            }
            if (errAll & FDCAN_PSR_EP) {
                status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
            }
        }
        CANmodule->CANerrorStatus = status;
    }

    /* The queue is sent from the transmit complete interrupt, which may not come (an abort, a lost interrupt): it is
     * started again here, else it would wait until a reset (protronic/CanOpenSTM32#1) */
    if (CANmodule->CANnormal && CANmodule->CANtxCount > 0U) {
        CO_LOCK_CAN_SEND(CANmodule);
        prv_flush_tx_queue(CANmodule, (uint8_t)((1U << CO_STM32_PHY_COUNT) - 1U));
        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}

/******************************************************************************/
void
CO_CANphyGetStatus(CO_CANmodule_t* CANmodule, uint8_t phy, CO_CANphyStatus_t* status) {
    memset(status, 0, sizeof(*status));
    if (phy >= CO_STM32_PHY_COUNT) {
        return;
    }
    FDCAN_GlobalTypeDef* can = prv_phy_handle(CANmodule, phy)->Instance;
    uint32_t psr = can->PSR;
    uint32_t ecr = can->ECR;

    status->up = CANmodule->up[phy];
    if (psr & FDCAN_PSR_BO) {
        status->state = CO_CANPHY_BUSOFF;
    } else if (psr & FDCAN_PSR_EP) {
        status->state = CO_CANPHY_PASSIVE;
    } else if (psr & FDCAN_PSR_EW) {
        status->state = CO_CANPHY_WARNING;
    } else {
        status->state = CO_CANPHY_ACTIVE;
    }
    status->tec = (uint8_t)(ecr & 0xFFU);
    status->rec = (uint8_t)((ecr >> 8) & 0x7FU);
    status->lost = CANmodule->lost[phy];
    status->dropped = CANmodule->dropped[phy];
    status->rxFrames = CANmodule->rxFrames[phy];
    status->txFrames = CANmodule->txFrames[phy];
    status->rxBits = CANmodule->rxBits[phy];
    status->txBits = CANmodule->txBits[phy];
}
#else
void
CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
    uint32_t err = 0;

    // CANOpen just care about Bus_off, Warning, Passive and Overflow
    // I didn't find overflow error register in STM32, if you find it please let me know

#ifdef CO_STM32_FDCAN_Driver

    prv_busoff_recover(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle);
    err = ((FDCAN_HandleTypeDef*)((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle)->Instance->PSR
          & (FDCAN_PSR_BO | FDCAN_PSR_EW | FDCAN_PSR_EP);

    if (CANmodule->errOld != err) {

        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (err & FDCAN_PSR_BO) {
            status |= CO_CAN_ERRTX_BUS_OFF;

        } else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF
                      ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
                         | CO_CAN_ERRTX_PASSIVE);

            if (err & FDCAN_PSR_EW) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
            }

            if (err & FDCAN_PSR_EP) {
                status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
            }

            /* If the transmitter is not passive, clear also the (non-latching) TX overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0U) {
                status &= 0xFFFFU ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        CANmodule->CANerrorStatus = status;
    }
#else

    err = ((CAN_HandleTypeDef*)((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle)->Instance->ESR
          & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF);

    //    uint32_t esrVal = ((CAN_HandleTypeDef*)((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle)->Instance->ESR; Debug purpose
    if (CANmodule->errOld != err) {

        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (err & CAN_ESR_BOFF) {
            status |= CO_CAN_ERRTX_BUS_OFF;
            /* With automatic bus-off management the bxCAN recovers by itself, else it is started again
             * (protronic/CanOpenSTM32#1) */
            CAN_HandleTypeDef* hcan = ((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle;
            if (hcan->Init.AutoBusOff == DISABLE && HAL_CAN_Stop(hcan) == HAL_OK) {
                (void)HAL_CAN_Start(hcan);
            }

        } else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF
                      ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
                         | CO_CAN_ERRTX_PASSIVE);

            if (err & CAN_ESR_EWGF) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
            }

            if (err & CAN_ESR_EPVF) {
                status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
            }

            /* If the transmitter is not passive, clear also the (non-latching) TX overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0U) {
                status &= 0xFFFFU ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        CANmodule->CANerrorStatus = status;
    }

#endif

    /* The queue is sent from the transmit complete interrupt, which may not come (an abort, a lost interrupt): it is
     * started again here, else it would wait until a reset (protronic/CanOpenSTM32#1) */
    if (CANmodule->CANnormal && CANmodule->CANtxCount > 0U) {
        CO_LOCK_CAN_SEND(CANmodule);
        prv_flush_tx_queue(CANmodule);
        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}
#endif

/**
 * \brief           Read message from RX FIFO
 * \param           hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       fifo: Fifo number to use for read
 * \param[in]       fifo_isrs: List of interrupts for respected FIFO
 */
#ifdef CO_STM32_FDCAN_Driver
static void
prv_read_can_received_msg(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo, uint32_t fifo_isrs)
#else
static void
prv_read_can_received_msg(CAN_HandleTypeDef* hcan, uint32_t fifo, uint32_t fifo_isrs)
#endif
{

    CO_CANrxMsg_t rcvMsg;
    CO_CANrx_t* buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
    uint16_t index;            /* index of received message */
    uint32_t rcvMsgIdent;      /* identifier of the received message */
    uint8_t messageFound = 0;

#ifdef CO_STM32_FDCAN_Driver
    FDCAN_RxHeaderTypeDef rx_hdr;
    /* Written to a buffer of the size of the largest FD frame, so that a node that does not comply with the
     * classic CAN format and sends a longer frame cannot overflow the 8 bytes of the message */
    uint8_t rx_data[64];
    /* Read received message from FIFO */
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_hdr, rx_data) != HAL_OK) {
        return;
    }
    /* Setup identifier (with RTR) and length */
    rcvMsg.ident = rx_hdr.Identifier | (rx_hdr.RxFrameType == FDCAN_REMOTE_FRAME ? FLAG_RTR : 0x00);
    switch (rx_hdr.DataLength) {
        case FDCAN_DLC_BYTES_0:
            rcvMsg.dlc = 0;
            break;
        case FDCAN_DLC_BYTES_1:
            rcvMsg.dlc = 1;
            break;
        case FDCAN_DLC_BYTES_2:
            rcvMsg.dlc = 2;
            break;
        case FDCAN_DLC_BYTES_3:
            rcvMsg.dlc = 3;
            break;
        case FDCAN_DLC_BYTES_4:
            rcvMsg.dlc = 4;
            break;
        case FDCAN_DLC_BYTES_5:
            rcvMsg.dlc = 5;
            break;
        case FDCAN_DLC_BYTES_6:
            rcvMsg.dlc = 6;
            break;
        case FDCAN_DLC_BYTES_7:
            rcvMsg.dlc = 7;
            break;
        case FDCAN_DLC_BYTES_8:
            rcvMsg.dlc = 8;
            break;
        default:
            rcvMsg.dlc = 0;
            break; /* Invalid length when more than 8 */
    }
    if (rcvMsg.dlc > 0) {
        memcpy(rcvMsg.data, rx_data, rcvMsg.dlc);
    }
    rcvMsgIdent = rcvMsg.ident;
#if CO_STM32_PHY_COUNT > 1
    {
        uint8_t phy = prv_phy_of(CANModule_local, hfdcan);
        /* Any frame, even one that is dropped below, shows the bus is alive */
        CANModule_local->lastRxMs[phy] = HAL_GetTick();
        CANModule_local->up[phy] = true;
        CANModule_local->rxFrames[phy]++;
        CANModule_local->rxBits[phy] += CO_CANPHY_FRAME_BITS(rcvMsg.dlc);

        uint16_t ident = (uint16_t)(rcvMsg.ident & CANID_MASK);
        if (!CO_CANphyRx(phy, &ident, rcvMsg.data, rcvMsg.dlc)) {
            return;
        }
        rcvMsg.ident = (rcvMsg.ident & FLAG_RTR) | ident;
        rcvMsgIdent = rcvMsg.ident;
    }
#endif
#else
    CAN_RxHeaderTypeDef rx_hdr;
    /* Read received message from FIFO */
    if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_hdr, rcvMsg.data) != HAL_OK) {
        return;
    }
    /* Setup identifier (with RTR) and length */
    rcvMsg.ident = rx_hdr.StdId | (rx_hdr.RTR == CAN_RTR_REMOTE ? FLAG_RTR : 0x00);
    rcvMsg.dlc = rx_hdr.DLC;
    rcvMsgIdent = rcvMsg.ident;
#endif

    /*
     * Hardware filters are not used for the moment
     * \todo: Implement hardware filters...
     */
    if (CANModule_local->useCANrxFilters) {
        __BKPT(0);
    } else {
        /*
         * We are not using hardware filters, hence it is necessary
         * to manually match received message ID with all buffers
         */
        buffer = CANModule_local->rxArray;
        for (index = CANModule_local->rxSize; index > 0U; --index, ++buffer) {
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
                messageFound = 1;
                break;
            }
        }
    }

    /* Call specific function, which will process the message */
    if (messageFound && buffer != NULL && buffer->CANrx_callback != NULL) {
        buffer->CANrx_callback(buffer->object, (void*)&rcvMsg);
    }
}

/**
 * \brief           Read every message waiting in an RX FIFO
 *
 * From protronic/CanOpenSTM32#1 and #104: the FDCAN "new message" interrupt is an event per
 * message, not a level that follows the fill level of the FIFO. Read one message per interrupt, a FIFO that
 * once held two messages at an interrupt (the interrupt came late: interrupts masked, a debugger halt) keeps
 * one forever: each new message then reads out an older one. The node answers late and, its free FIFO
 * smaller, loses the frames of a burst (an SDO block) until it is reset. Reading the FIFO empty removes it.
 * On bxCAN the interrupt follows the fill level: the loop only saves entering it again for each message.
 */
#define CO_RX_FIFO_DRAIN_MAX 64U /* the largest FIFO of the FDCAN (H7) */
#ifdef CO_STM32_FDCAN_Driver
static void
prv_drain_rx_fifo(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo, uint32_t fifo_isrs) {
    /* Bounded: a read that fails (it doesn't, the FIFO isn't empty) must not hold the interrupt forever */
    for (uint32_t n = 0; n < CO_RX_FIFO_DRAIN_MAX && HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo) > 0U; n++) {
        prv_read_can_received_msg(hfdcan, fifo, fifo_isrs);
    }
}
#else
static void
prv_drain_rx_fifo(CAN_HandleTypeDef* hcan, uint32_t fifo, uint32_t fifo_isrs) {
    for (uint32_t n = 0; n < CO_RX_FIFO_DRAIN_MAX && HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0U; n++) {
        prv_read_can_received_msg(hcan, fifo, fifo_isrs);
    }
}
#endif

#ifdef CO_STM32_FDCAN_Driver
/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signaled.
 */
void
HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
        CO_CANrxLostFrames++;
#if CO_STM32_PHY_COUNT > 1
        CANModule_local->lost[prv_phy_of(CANModule_local, hfdcan)]++;
#endif
    }
    /* The whole FIFO is read (prv_drain_rx_fifo), also after a loss */
    if (RxFifo0ITs & (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST)) {
        prv_drain_rx_fifo(hfdcan, FDCAN_RX_FIFO0, RxFifo0ITs);
    }
}

/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       RxFifo1ITs: indicates which Rx FIFO 0 interrupts are signaled.
 */
void
HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs) {
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) {
        CO_CANrxLostFrames++;
#if CO_STM32_PHY_COUNT > 1
        CANModule_local->lost[prv_phy_of(CANModule_local, hfdcan)]++;
#endif
    }
    if (RxFifo1ITs & (FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_MESSAGE_LOST)) {
        prv_drain_rx_fifo(hfdcan, FDCAN_RX_FIFO1, RxFifo1ITs);
    }
}

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       BufferIndexes: Bits of successfully sent TX buffers
 */
void
HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t BufferIndexes) {
    CANModule_local->firstCANtxMessage = false;            /* First CAN message (bootup) was sent successfully */
    CANModule_local->bufferInhibitFlag = false;            /* Clear flag from previous message */
    if (CANModule_local->CANtxCount > 0U) {                /* Are there any new messages waiting to be send */

        /*
         * Try to send more buffers, process all empty ones
         *
         * This function is always called from interrupt,
         * however to make sure no preemption can happen, interrupts are anyway locked
         * (unless you can guarantee no higher priority interrupt will try to access to FDCAN instance and send data,
         *  then no need to lock interrupts..)
         */
        CO_LOCK_CAN_SEND(CANModule_local);
#if CO_STM32_PHY_COUNT > 1
        /* This callback is for one phy: only the buffers still pending on it */
        prv_flush_tx_queue(CANModule_local, (uint8_t)(1U << prv_phy_of(CANModule_local, hfdcan)));
#else
        prv_flush_tx_queue(CANModule_local);
#endif
        CO_UNLOCK_CAN_SEND(CANModule_local);
    }
}

/**
 * \brief           A transmission was aborted (bus off, prv_phy_down())
 *
 * The frame is lost, but the queue must go on: no transmit complete interrupt would send it.
 * (protronic/CanOpenSTM32#1)
 */
void
HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t BufferIndexes) {
    (void)BufferIndexes;
    CANModule_local->bufferInhibitFlag = false;
    if (CANModule_local->CANtxCount > 0U) {
        CO_LOCK_CAN_SEND(CANModule_local);
#if CO_STM32_PHY_COUNT > 1
        prv_flush_tx_queue(CANModule_local, (uint8_t)(1U << prv_phy_of(CANModule_local, hfdcan)));
#else
        (void)hfdcan;
        prv_flush_tx_queue(CANModule_local);
#endif
        CO_UNLOCK_CAN_SEND(CANModule_local);
    }
}
#else
/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void
HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    prv_drain_rx_fifo(hcan, CAN_RX_FIFO0, 0);
}

/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void
HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    prv_drain_rx_fifo(hcan, CAN_RX_FIFO1, 0);
}

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 * \param[in]       MailboxNumber: the mailbox number that has been transmitted
 */
void
CO_CANinterrupt_TX(CO_CANmodule_t* CANmodule, uint32_t MailboxNumber) {

    CANmodule->firstCANtxMessage = false;            /* First CAN message (bootup) was sent successfully */
    CANmodule->bufferInhibitFlag = false;            /* Clear flag from previous message */
    if (CANmodule->CANtxCount > 0U) {                /* Are there any new messages waiting to be send */

        /*
		 * Try to send more buffers, process all empty ones
		 *
		 * This function is always called from interrupt,
		 * however to make sure no preemption can happen, interrupts are anyway locked
		 * (unless you can guarantee no higher priority interrupt will try to access to CAN instance and send data,
		 *  then no need to lock interrupts..)
		 */
        CO_LOCK_CAN_SEND(CANmodule);
        prv_flush_tx_queue(CANmodule);
        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}

/* A mailbox was aborted (bus off, or a lost arbitration or an error without automatic retransmission): the frame is
 * lost, the queue goes on (protronic/CanOpenSTM32#1) */
static void
prv_tx_aborted(CO_CANmodule_t* CANmodule) {
    CANmodule->bufferInhibitFlag = false;
    if (CANmodule->CANtxCount > 0U) {
        CO_LOCK_CAN_SEND(CANmodule);
        prv_flush_tx_queue(CANmodule);
        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}

void
HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan) {
    CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
}

void
HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan) {
    CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX1);
}

void
HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan) {
    CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX2);
}

void
HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef* hcan) {
    prv_tx_aborted(CANModule_local);
}

void
HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef* hcan) {
    prv_tx_aborted(CANModule_local);
}

void
HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef* hcan) {
    prv_tx_aborted(CANModule_local);
}
#endif
