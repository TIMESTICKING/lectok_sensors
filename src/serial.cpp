#include <SPI.h>
#include "serial.h"



// 创建一个队列用于存储要发送的数据
QueueHandle_t uartQueue;

TaskHandle_t *serial_send_handler;

// 串口发送任务
void uartSendTask(void *pvParameters) {
    char data[MAX_DATA_SIZE];
    while (1) {
        // 从队列中接收数据
        if (xQueueReceive(uartQueue, &data, portMAX_DELAY) == pdTRUE) {
            // 发送数据到串口
            Serial.write(data, MAX_DATA_SIZE);
        }
      // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
      // Serial.print("serial:");
      // Serial.println(uxHighWaterMark);
    }
}


void send_float(u_char * num){
  // 发送4个字节
  for (int j = 0; j < sizeof(float); j++) {
    Serial.write(*(num + j));
  }
}

void serial_queue_init(){
  // 创建队列
  uartQueue = xQueueCreate(QUEUE_LENGTH, MAX_DATA_SIZE);
}