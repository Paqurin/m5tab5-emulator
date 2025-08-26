/**
 * @file queue.h
 * @brief FreeRTOS queue compatibility header for M5Stack Tab5 Emulator
 * 
 * This header provides basic FreeRTOS queue APIs for ESP-IDF compatibility
 * in the emulator environment.
 */

#pragma once

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Queue creation and management
 */

/**
 * @brief Create a new queue
 * 
 * @param uxQueueLength Maximum number of items that the queue can contain
 * @param uxItemSize Size in bytes of each item in the queue
 * @return Handle to the created queue, or NULL if creation failed
 */
QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize);

/**
 * @brief Delete a queue
 * 
 * @param xQueue Handle of the queue to delete
 */
void vQueueDelete(QueueHandle_t xQueue);

/**
 * @brief Send an item to the back of a queue
 * 
 * @param xQueue Handle of the queue to send to
 * @param pvItemToQueue Pointer to the item to queue
 * @param xTicksToWait Maximum time to wait for space to become available
 * @return pdTRUE if item was successfully sent, pdFALSE otherwise
 */
BaseType_t xQueueSend(QueueHandle_t xQueue, const void* pvItemToQueue, TickType_t xTicksToWait);

/**
 * @brief Send an item to the back of a queue (same as xQueueSend)
 * 
 * @param xQueue Handle of the queue to send to
 * @param pvItemToQueue Pointer to the item to queue
 * @param xTicksToWait Maximum time to wait for space to become available
 * @return pdTRUE if item was successfully sent, pdFALSE otherwise
 */
BaseType_t xQueueSendToBack(QueueHandle_t xQueue, const void* pvItemToQueue, TickType_t xTicksToWait);

/**
 * @brief Send an item to the front of a queue
 * 
 * @param xQueue Handle of the queue to send to
 * @param pvItemToQueue Pointer to the item to queue
 * @param xTicksToWait Maximum time to wait for space to become available
 * @return pdTRUE if item was successfully sent, pdFALSE otherwise
 */
BaseType_t xQueueSendToFront(QueueHandle_t xQueue, const void* pvItemToQueue, TickType_t xTicksToWait);

/**
 * @brief Receive an item from a queue
 * 
 * @param xQueue Handle of the queue to receive from
 * @param pvBuffer Pointer to buffer where received item will be copied
 * @param xTicksToWait Maximum time to wait for an item to become available
 * @return pdTRUE if item was successfully received, pdFALSE otherwise
 */
BaseType_t xQueueReceive(QueueHandle_t xQueue, void* pvBuffer, TickType_t xTicksToWait);

/**
 * @brief Peek at an item in a queue without removing it
 * 
 * @param xQueue Handle of the queue to peek at
 * @param pvBuffer Pointer to buffer where peeked item will be copied
 * @param xTicksToWait Maximum time to wait for an item to become available
 * @return pdTRUE if item was successfully peeked, pdFALSE otherwise
 */
BaseType_t xQueuePeek(QueueHandle_t xQueue, void* pvBuffer, TickType_t xTicksToWait);

/**
 * @brief Get the number of messages waiting in a queue
 * 
 * @param xQueue Handle of the queue to query
 * @return Number of messages available
 */
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t xQueue);

/**
 * @brief Get the number of free spaces in a queue
 * 
 * @param xQueue Handle of the queue to query
 * @return Number of free spaces available
 */
UBaseType_t uxQueueSpacesAvailable(QueueHandle_t xQueue);

/**
 * @brief Reset a queue to its empty state
 * 
 * @param xQueue Handle of the queue to reset
 * @return pdPASS if queue was successfully reset
 */
BaseType_t xQueueReset(QueueHandle_t xQueue);

// ISR versions (simplified for emulation)
#define xQueueSendFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken) \
    xQueueSend((xQueue), (pvItemToQueue), 0)

#define xQueueReceiveFromISR(xQueue, pvBuffer, pxHigherPriorityTaskWoken) \
    xQueueReceive((xQueue), (pvBuffer), 0)

#ifdef __cplusplus
}
#endif