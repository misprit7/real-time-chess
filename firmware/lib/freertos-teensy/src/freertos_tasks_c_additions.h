/*
 * FreeRTOS Kernel V11.0.1 task additions
 * Copyright (C) 2021-2024 Timo Sandmann.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file    freertos_tasks_c_additions.h
 * @brief   FreeRTOS task additions
 * @author  Timo Sandmann
 * @date    12.03.2021
 */

#pragma once

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>


static TaskHandle_t prvStackWithinList(List_t* pxList, StackType_t* pxStack) {
    UBaseType_t uxQueue = configMAX_PRIORITIES;
    TCB_t* pxNextTCB = NULL;

    do {
        uxQueue--;
        configLIST_VOLATILE TCB_t* pxFirstTCB;

        if (listCURRENT_LIST_LENGTH(pxList) > (UBaseType_t) 0) {
            listGET_OWNER_OF_NEXT_ENTRY(pxFirstTCB, pxList);
            do {
                listGET_OWNER_OF_NEXT_ENTRY(pxNextTCB, pxList);
                if (pxNextTCB && pxStack >= pxNextTCB->pxStack && pxStack <= pxNextTCB->pxEndOfStack) {
                    return pxNextTCB;
                }
            } while (pxNextTCB != pxFirstTCB);
        }
    } while (uxQueue > (UBaseType_t) tskIDLE_PRIORITY);

    return NULL;
}

TaskHandle_t pxGetTaskFromStack(StackType_t* pxStack) {
    extern TCB_t* volatile pxCurrentTCB;
    if (pxCurrentTCB && pxStack >= pxCurrentTCB->pxStack && pxStack <= pxCurrentTCB->pxEndOfStack) {
        return pxCurrentTCB;
    }

    if ((portNVIC_INT_CTRL_REG & 0xff) == 0) {
        portENTER_CRITICAL();
    }

    TaskHandle_t pxTask = NULL;
    UBaseType_t uxQueue = configMAX_PRIORITIES;
    do {
        uxQueue--;
        pxTask = prvStackWithinList(&pxReadyTasksLists[uxQueue], pxStack);
    } while (uxQueue > (UBaseType_t) tskIDLE_PRIORITY && pxTask == NULL);

    if (!pxTask) {
        pxTask = prvStackWithinList(pxDelayedTaskList, pxStack);
    }
    if (!pxTask) {
        pxTask = prvStackWithinList(pxOverflowDelayedTaskList, pxStack);
    }
#if INCLUDE_vTaskDelete == 1
    if (!pxTask) {
        pxTask = prvStackWithinList(&xTasksWaitingTermination, pxStack);
    }
#endif
#if INCLUDE_vTaskSuspend == 1
    if (!pxTask) {
        pxTask = prvStackWithinList(&xSuspendedTaskList, pxStack);
    }
#endif

    if ((portNVIC_INT_CTRL_REG & 0xff) == 0) {
        portEXIT_CRITICAL();
    }

    return pxTask;
}
