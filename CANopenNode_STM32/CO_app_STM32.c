/*
 * CANopen main program file.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        main_generic.c
 * @author      Hamed Jafarzadeh 	2022
 * 				Janez Paternoster	2021
 * @copyright   2021 Janez Paternoster
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
#include "CO_app_STM32.h"
#include "CANopen.h"
#include "main.h"
#include <inttypes.h>
#include <stdio.h>

#include "CO_storageBlank.h"
#include "OD.h"

CANopenNodeSTM32*
    canopenNodeSTM32; // It will be set by canopen_app_init and will be used across app to get access to CANOpen objects

/* Printf function of CanOpen app */
#ifndef log_printf
#define log_printf(macropar_message, ...) printf(macropar_message, ##__VA_ARGS__)
#endif

/* default values for CO_CANopenInit() */
#ifndef NMT_CONTROL
#define NMT_CONTROL                                                                                                    \
    CO_NMT_STARTUP_TO_OPERATIONAL                                                                                      \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#endif
#ifndef FIRST_HB_TIME
#define FIRST_HB_TIME 500
#endif
#ifndef SDO_SRV_TIMEOUT_TIME
#define SDO_SRV_TIMEOUT_TIME 1000
#endif
#ifndef SDO_CLI_TIMEOUT_TIME
#define SDO_CLI_TIMEOUT_TIME 500
#endif
#ifndef SDO_CLI_BLOCK
#define SDO_CLI_BLOCK false
#endif
#ifndef OD_STATUS_BITS
#define OD_STATUS_BITS NULL
#endif

/* Global variables and objects */
CO_t* CO = NULL; /* CANopen object */

// Global variables
uint32_t time_old;
uint32_t interrupt_time_old;
CO_ReturnError_t err;

/* This function will basically setup the CANopen node */
int
canopen_app_init(CANopenNodeSTM32* _canopenNodeSTM32) {

    // Keep a copy global reference of canOpenSTM32 Object
    canopenNodeSTM32 = _canopenNodeSTM32;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    static CO_storage_t storage;
    static CO_storage_entry_t storageEntries[] = {{.addr = &OD_PERSIST_COMM,
                                                   .len = sizeof(OD_PERSIST_COMM),
                                                   .subIndexOD = 2,
                                                   .attr = CO_storage_cmd | CO_storage_restore,
                                                   .addrNV = NULL}};
    uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
    uint32_t storageInitError = 0;
#endif

    /* Allocate memory */
    CO_config_t* config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    /* example usage of CO_MULTIPLE_OD (but still single OD here) */
    CO_config_t co_config = {0};
    OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
    co_config.CNT_LEDS = 1;
#if ((CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE) != 0
    co_config.CNT_LSS_SLV = 1;
#endif
    config_ptr = &co_config;
#endif /* CO_MULTIPLE_OD */

    uint32_t heapMemoryUsed;
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf("Error: Can't allocate memory\n");
        return 1;
    } else {
        log_printf("Allocated %" PRIu32 " bytes for CANopen objects\n", heapMemoryUsed);
    }

    canopenNodeSTM32->canOpenStack = CO;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        log_printf("Error: Storage %d\n", storageInitError);
        return 2;
    }
#endif

    canopen_app_resetCommunication();
    return 0;
}

__weak uint32_t
canopen_app_get_time() {
    return HAL_GetTick() * 1000;
}

int
canopen_app_resetCommunication() {
    /* CANopen communication reset - initialize CANopen objects *******************/
    log_printf("CANopenNode - Reset communication...\n");

    /* Wait rt_thread. */
    CO->CANmodule->CANnormal = false;

    /* Enter CAN configuration. */
    CO_CANsetConfigurationMode(canopenNodeSTM32);
    CO_CANmodule_disable(CO->CANmodule);

    /* initialize CANopen */
    err = CO_CANinit(CO, canopenNodeSTM32, 0); // Bitrate for STM32 microcontroller is being set in MXCube Settings
    if (err != CO_ERROR_NO) {
        log_printf("Error: CAN initialization failed: %d\n", err);
        return 1;
    }

#if ((CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE) != 0
    CO_LSS_address_t lssAddress = {.identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                                .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                                .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                                .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};
    err = CO_LSSinit(CO, &lssAddress, &canopenNodeSTM32->desiredNodeID, &canopenNodeSTM32->baudrate);
    if (err != CO_ERROR_NO) {
        log_printf("Error: LSS slave initialization failed: %d\n", err);
        return 2;
    }
#endif

    canopenNodeSTM32->activeNodeID = canopenNodeSTM32->desiredNodeID;
    uint32_t errInfo = 0;

    err = CO_CANopenInit(CO,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         canopenNodeSTM32->activeNodeID, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        if (err == CO_ERROR_OD_PARAMETERS) {
            log_printf("Error: Object Dictionary entry 0x%" PRIx32 "\n", errInfo);
        } else {
            log_printf("Error: CANopen initialization failed: %d\n", err);
        }
        return 3;
    }

    err = CO_CANopenInitPDO(CO, CO->em, OD, canopenNodeSTM32->activeNodeID, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        if (err == CO_ERROR_OD_PARAMETERS) {
            log_printf("Error: Object Dictionary entry 0x%" PRIx32 "\n", errInfo);
        } else {
            log_printf("Error: PDO initialization failed: %d\n", err);
        }
        return 4;
    }

    /* Signal callbacks of the application (see CO_app_STM32.h) */
#if ((CO_CONFIG_NMT) & CO_CONFIG_FLAG_CALLBACK_PRE) != 0
    if (canopenNodeSTM32->signalNMT != NULL) {
        CO_NMT_initCallbackPre(CO->NMT, CO->NMT, canopenNodeSTM32->signalNMT);
    }
#endif
#if ((CO_CONFIG_EM) & CO_CONFIG_FLAG_CALLBACK_PRE) != 0
    if (canopenNodeSTM32->signalEM != NULL) {
        CO_EM_initCallbackPre(CO->em, CO->em, canopenNodeSTM32->signalEM);
    }
#endif
#if ((CO_CONFIG_SDO_SRV) & CO_CONFIG_FLAG_CALLBACK_PRE) != 0
    if (canopenNodeSTM32->signalSDOserver != NULL) {
        for (int i = 0; i < OD_CNT_SDO_SRV; i++) {
            CO_SDOserver_initCallbackPre(&CO->SDOserver[i], &CO->SDOserver[i], canopenNodeSTM32->signalSDOserver);
        }
    }
#endif
#if (((CO_CONFIG_SDO_CLI) & CO_CONFIG_SDO_CLI_ENABLE) != 0)                                                            \
    && (((CO_CONFIG_SDO_CLI) & CO_CONFIG_FLAG_CALLBACK_PRE) != 0) && defined(OD_CNT_SDO_CLI)
    if (canopenNodeSTM32->signalSDOclient != NULL) {
        for (int i = 0; i < OD_CNT_SDO_CLI; i++) {
            CO_SDOclient_initCallbackPre(&CO->SDOclient[i], &CO->SDOclient[i], canopenNodeSTM32->signalSDOclient);
        }
    }
#endif
#if (((CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE) != 0)                                                            \
    && (((CO_CONFIG_HB_CONS) & CO_CONFIG_FLAG_CALLBACK_PRE) != 0)
    if (canopenNodeSTM32->signalHBconsumer != NULL) {
        CO_HBconsumer_initCallbackPre(CO->HBcons, CO->HBcons, canopenNodeSTM32->signalHBconsumer);
    }
#endif
#if (((CO_CONFIG_TIME) & CO_CONFIG_TIME_ENABLE) != 0) && (((CO_CONFIG_TIME) & CO_CONFIG_FLAG_CALLBACK_PRE) != 0)
    if (canopenNodeSTM32->signalTIME != NULL) {
        CO_TIME_initCallbackPre(CO->TIME, CO->TIME, canopenNodeSTM32->signalTIME);
    }
#endif

    /* Configure Timer interrupt function for execution every 1 millisecond */
    if (canopenNodeSTM32->timerHandle != NULL) {
        HAL_TIM_Base_Start_IT(canopenNodeSTM32->timerHandle); // 1ms interrupt
    }

    /* Configure CAN transmit and receive interrupt */

    /* Configure CANopen callbacks, etc */
    if (!CO->nodeIdUnconfigured) {

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
        if (storageInitError != 0) {
            CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
        }
#endif
    } else {
        log_printf("CANopenNode - Node-id not initialized\n");
    }

    /* start CAN */
    CO_CANsetNormalMode(CO->CANmodule);

    log_printf("CANopenNode - Running...\n");
    fflush(stdout);
    time_old = interrupt_time_old = canopen_app_get_time();
    return 0;
}

uint32_t
canopen_app_process(void) {
    /* No "elapsed > 0" guard: the task can wake within the same tick of the clock to process a frame
     * (CO_process with a zero time difference does that). Returns timerNext_us. */
    uint32_t now = canopen_app_get_time();
    uint32_t timeDifference_us = now - time_old;
    time_old = now;

    uint32_t timerNext_us = UINT32_MAX;
    CO_NMT_reset_cmd_t reset_status = CO_process(CO, false, timeDifference_us, &timerNext_us);
#if ((CO_CONFIG_LEDS) & CO_CONFIG_LEDS_ENABLE) != 0
    canopenNodeSTM32->outStatusLEDRed = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
    canopenNodeSTM32->outStatusLEDGreen = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);
#endif

    if (reset_status == CO_RESET_COMM) {
        /* The same objects are initialized again, as in the examples of CANopenNode, instead of
         * deleted and allocated again (CO_delete, canopen_app_init): the application may keep
         * pointers to them (e.g. the SDO clients) */
        if (canopenNodeSTM32->timerHandle != NULL) {
            HAL_TIM_Base_Stop_IT(canopenNodeSTM32->timerHandle);
        }
        log_printf("CANopenNode Reset Communication request\n");
        canopen_app_resetCommunication(); // Reset Communication routine
        return 0;
    } else if (reset_status == CO_RESET_APP) {
        log_printf("CANopenNode Device Reset\n");
        HAL_NVIC_SystemReset(); // Reset the STM32 Microcontroller
    }
    return timerNext_us;
}

/* Thread function executes in constant intervals, this function can be called from FreeRTOS tasks or Timers ********/
void
canopen_app_interrupt(void) {
    CO_LOCK_OD(CO->CANmodule);
    if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
        bool_t syncWas = false;
        /* get time difference since last function call */
        /* Local, the CANopen task has its own time (canopen_app_process) */
        uint32_t now = canopen_app_get_time();
        uint32_t timeDifference_us = now - interrupt_time_old; // 1ms second
        interrupt_time_old = now;
        if (timeDifference_us == 0) {
            timeDifference_us = 1000;
        }

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif

        /* Further I/O or nonblocking application code may go here. */
    }
    CO_UNLOCK_OD(CO->CANmodule);
}
