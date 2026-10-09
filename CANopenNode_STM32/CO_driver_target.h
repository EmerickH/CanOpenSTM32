/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
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
 */
#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include "main.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Determining the CANOpen Driver

#if defined(FDCAN) || defined(FDCAN1) || defined(FDCAN2) || defined(FDCAN3)
#define CO_STM32_FDCAN_Driver 1
#elif defined(CAN) || defined(CAN1) || defined(CAN2) || defined(CAN3)
#define CO_STM32_CAN_Driver 1
#else
#error This STM32 Do not support CAN or FDCAN
#endif

/* Number of physical CAN buses served by the one stack (FDCAN only). 1 = the original driver.
 * The application sets 2 (see CO_CANphyRx/CO_CANphyTx below). */
#ifndef CO_STM32_PHY_COUNT
#define CO_STM32_PHY_COUNT 1
#endif
#if CO_STM32_PHY_COUNT > 1 && !defined(CO_STM32_FDCAN_Driver)
#error CO_STM32_PHY_COUNT > 1 needs the FDCAN driver
#endif

#undef CO_CONFIG_STORAGE_ENABLE // We don't need Storage option, implement based on your use case and remove this line from here

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x

/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/**
 * \brief           CAN RX message for platform
 *
 * This is platform specific one
 */
typedef struct {
    uint32_t ident;  /*!< Standard identifier */
    uint8_t dlc;     /*!< Data length */
    uint8_t data[8]; /*!< Received data */
} CO_CANrxMsg_t;

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t*)(msg)))->ident)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)(((CO_CANrxMsg_t*)(msg)))->dlc)
#define CO_CANrxMsg_readData(msg)  ((uint8_t*)(((CO_CANrxMsg_t*)(msg)))->data)

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void* object;
    void (*CANrx_callback)(void* object, void* message);
} CO_CANrx_t;

/* Transmit message object */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
#if CO_STM32_PHY_COUNT > 1
    volatile uint8_t phyPending; /* bit per phy where the frame is still to be sent (bufferFull = phyPending != 0) */
#endif
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void* CANptr;
    CO_CANrx_t* rxArray;
    uint16_t rxSize;
    CO_CANtx_t* txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
#if CO_STM32_PHY_COUNT > 1
    uint32_t errOldPhy[CO_STM32_PHY_COUNT];       /* last error flags (PSR) of each phy that is up */
    volatile uint32_t lastRxMs[CO_STM32_PHY_COUNT]; /* HAL tick of the last frame received on each phy */
    volatile bool_t up[CO_STM32_PHY_COUNT];       /* a frame was received in the last CO_CANPHY_UP_MS */
    volatile uint32_t lost[CO_STM32_PHY_COUNT];   /* receive FIFO overruns */
    volatile uint32_t dropped[CO_STM32_PHY_COUNT]; /* frames routed to a phy that was down */
    /* frames received (all, before the id filter) and queued for sending, and their bits (CO_CANPHY_FRAME_BITS):
     * counters that wrap */
    volatile uint32_t rxFrames[CO_STM32_PHY_COUNT], txFrames[CO_STM32_PHY_COUNT];
    volatile uint32_t rxBits[CO_STM32_PHY_COUNT], txBits[CO_STM32_PHY_COUNT];
#endif

    /* STM32 specific features */
    uint32_t primask_send; /* Primask register for interrupts for send operation */
    uint32_t primask_emcy; /* Primask register for interrupts for emergency operation */
    uint32_t primask_od;   /* Primask register for interrupts for send operation */

} CO_CANmodule_t;

#if CO_STM32_PHY_COUNT > 1
/* A phy is "up" while a frame was received on it in the last CO_CANPHY_UP_MS (nodes send a heartbeat per second).
 * A down phy gets no frames, so an empty bus never fills its TX FIFO. */
#define CO_CANPHY_UP_MS 3000U

#define CO_CANPHY_ACTIVE  0
#define CO_CANPHY_WARNING 1
#define CO_CANPHY_PASSIVE 2
#define CO_CANPHY_BUSOFF  3

/* Given by the application . They translate the node ids
 * between the logical ids of the stack and the physical ids of each bus. Called from interrupts and with
 * interrupts masked: no blocking, no logging.
 * Rx: false drops the frame, else *ident is the logical COB-ID (11 bits).
 * Tx: whether the frame goes on bus phy; *ident and data (8 bytes, a copy) are rewritten for that bus. */
bool CO_CANphyRx(uint8_t phy, uint16_t* ident, const uint8_t* data, uint8_t dlc);
bool CO_CANphyTx(uint8_t phy, uint16_t* ident, uint8_t* data, uint8_t dlc);

/* Bits of a standard data frame on the bus with the interframe space, without the stuff bits (up to ~20 % more):
 * a lower bound */
#define CO_CANPHY_FRAME_BITS(dlc) (47U + 8U * (uint32_t)(dlc))

typedef struct {
    bool up;
    uint8_t state; /* CO_CANPHY_* */
    uint8_t tec, rec;
    uint32_t lost, dropped;
    uint32_t rxFrames, txFrames, rxBits, txBits;
} CO_CANphyStatus_t;

void CO_CANphyGetStatus(CO_CANmodule_t* CANmodule, uint8_t phy, CO_CANphyStatus_t* status);
#endif

/* Data storage object for one entry */
typedef struct {
    void* addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    /* Additional variables (target specific) */
    void* addrNV;
} CO_storage_entry_t;

/* (un)lock critical section in CO_CANsend() */
// Why disabling the whole Interrupt
#define CO_LOCK_CAN_SEND(CAN_MODULE)                                                                                   \
    do {                                                                                                               \
        (CAN_MODULE)->primask_send = __get_PRIMASK();                                                                  \
        __disable_irq();                                                                                               \
    } while (0)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) __set_PRIMASK((CAN_MODULE)->primask_send)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)                                                                                       \
    do {                                                                                                               \
        (CAN_MODULE)->primask_emcy = __get_PRIMASK();                                                                  \
        __disable_irq();                                                                                               \
    } while (0)
#define CO_UNLOCK_EMCY(CAN_MODULE) __set_PRIMASK((CAN_MODULE)->primask_emcy)

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)                                                                                         \
    do {                                                                                                               \
        (CAN_MODULE)->primask_od = __get_PRIMASK();                                                                    \
        __disable_irq();                                                                                               \
    } while (0)
#define CO_UNLOCK_OD(CAN_MODULE) __set_PRIMASK((CAN_MODULE)->primask_od)

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                                             \
    do {                                                                                                               \
        CO_MemoryBarrier();                                                                                            \
        rxNew = (void*)1L;                                                                                             \
    } while (0)
#define CO_FLAG_CLEAR(rxNew)                                                                                           \
    do {                                                                                                               \
        CO_MemoryBarrier();                                                                                            \
        rxNew = NULL;                                                                                                  \
    } while (0)

/* Received frames lost because the receive FIFO of the FDCAN was full (its interrupt
 * came too late). A lower bound: frames lost between two interrupts count once. */
extern volatile uint32_t CO_CANrxLostFrames;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
