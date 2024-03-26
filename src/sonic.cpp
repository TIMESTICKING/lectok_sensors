#include <SPI.h>
#include "sonic.h"
#include "serial.h"

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

HCSR04 ultrasonicSensor(trigger_pin, echo_pin, 15, 200);

TaskHandle_t *sonic_read_handler;

u_char sonic_package[8];

void ultrasonic_reader(void * para){
  sonic_package[0] = PKG_HD1;
  sonic_package[1] = PKG_HD2;
  sonic_package[2] = 0x02;
  sonic_package[7] = PKG_TL;
  float filterd_distance = 0;
  while (1)
  {
    // distance = ultrasonicSensor.getDistance();
    // vTaskDelay(pdMS_TO_TICKS(100));


    filterd_distance = ultrasonicSensor.getMedianFilterDistance(); //pass 3 measurements through median filter, better result on moving obstacles

    memcpy(&sonic_package[2], (u_char*)&filterd_distance, 4);

    xQueueSend(uartQueue, &sonic_package, portMAX_DELAY);

    // Serial.print(headers);
    // send_float((u_char *)&filterd_distance);
    // Serial.print(tail);

    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("sonic:");
    // Serial.println(uxHighWaterMark);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
}
