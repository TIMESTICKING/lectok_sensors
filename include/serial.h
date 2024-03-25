#ifndef MY_SERIAL_LG_H
#define MY_SERIAL_LG_H

#include <Arduino.h>
#include <Adafruit_AMG88xx.h>


void serial_queue_init();
void uartSendTask(void *pvParameters);

// queue usart
#define QUEUE_LENGTH 10
#define MAX_DATA_SIZE 20

extern QueueHandle_t uartQueue;

extern TaskHandle_t *serial_send_handler;


#endif