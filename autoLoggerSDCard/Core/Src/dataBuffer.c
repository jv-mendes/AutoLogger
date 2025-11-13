/*
 * dataBuffer.c
 *
 *  Created on: Nov 12, 2025
 *      Author: jvmen
 */
// SensorBuffer.c
#include "dataBuffer.h"

void dataBufferInit(DataBuffer_t *buf) {
    buf->head = 0;
    buf->tail = 0;
    buf->count = 0;
}

bool dataBufferPush(DataBuffer_t *buf, const SensorData_t *data) {
    if (buf->count >= DATA_BUFFER_SIZE)
        return false; // buffer cheio

    buf->buffer[buf->head] = *data;
    buf->head = (buf->head + 1) % DATA_BUFFER_SIZE;
    buf->count++;
    return true;
}

bool dataBufferPop(DataBuffer_t *buf, SensorData_t *data) {
    if (buf->count == 0)
        return false; // buffer vazio

    *data = buf->buffer[buf->tail];
    buf->tail = (buf->tail + 1) % DATA_BUFFER_SIZE;
    buf->count--;
    return true;
}


