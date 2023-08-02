/**
 ******************************************************************************
 * File Name          : os_resources.c
 * Description        : OS resources source file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 Branimir Mihelčić
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************/
#include "cmsis_os.h"
#include "master.h"
#include "log.h"
#include "pid_control.h"
#include "message_buffer.h"
#include "app_cfg.h"

MessageBufferHandle_t master_rx_message_buffer_handle;
MessageBufferHandle_t command_rx_message_buffer_handle;
MessageBufferHandle_t pid_rx_message_buffer_handle;
MessageBufferHandle_t log_rx_message_buffer_handle;
SemaphoreHandle_t uart_mutex;

static uint8_t master_rx_message_queue_buffer[50];
static uint8_t command_rx_message_queue_buffer[50];
static uint8_t pid_rx_message_queue_buffer[50];
static uint8_t log_rx_message_queue_buffer[50];

static StaticMessageBuffer_t master_rx_message_struct;
static StaticMessageBuffer_t command_rx_message_struct;
static StaticMessageBuffer_t pid_rx_message_struct;
static StaticMessageBuffer_t log_rx_message_struct;

StaticSemaphore_t uart_mutex_buffer;
char uart_tx_buffer[CFG_UART_TX_BUFFER_SIZE];

static void os_resources_error_hook();

int OS_RESOURCES_Init()
{

    master_rx_message_buffer_handle = xMessageBufferCreateStatic(sizeof(master_rx_message_queue_buffer),
                                                                 master_rx_message_queue_buffer,
                                                                 &master_rx_message_struct);
    if (NULL == master_rx_message_buffer_handle) {
        os_resources_error_hook();
    }

    command_rx_message_buffer_handle = xMessageBufferCreateStatic(sizeof(command_rx_message_queue_buffer),
                                                                  command_rx_message_queue_buffer,
                                                                  &command_rx_message_struct);
    if (NULL == command_rx_message_buffer_handle) {
        os_resources_error_hook();
    }

    pid_rx_message_buffer_handle = xMessageBufferCreateStatic(sizeof(pid_rx_message_queue_buffer),
                                                              pid_rx_message_queue_buffer,
                                                              &pid_rx_message_struct);
    if (NULL == pid_rx_message_buffer_handle) {
        os_resources_error_hook();
    }

    log_rx_message_buffer_handle = xMessageBufferCreateStatic(sizeof(log_rx_message_queue_buffer),
                                                              log_rx_message_queue_buffer,
                                                              &log_rx_message_struct);
    if (NULL == log_rx_message_buffer_handle) {
        os_resources_error_hook();
    }

    uart_mutex = xSemaphoreCreateMutexStatic(&uart_mutex_buffer);
    if (NULL == uart_mutex) {
        os_resources_error_hook();
    }

    return 0;
}

static void os_resources_error_hook()
{
    while (1);
}
