#include <SPI.h>
#include <Arduino.h>

#include "sonic.h"
#include "IR.h"
#include "serial.h"
#include "vars.h"

void rtos_setup();
void send_float(u_char * num);

// mux
// portMUX_TYPE serial_mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t xMutex;


void setup() {
  // 创建互斥量
  xMutex = xSemaphoreCreateMutex();
  // 检查互斥量是否创建成功
  if (xMutex == NULL) {
      printf("Mutex creation failed\n");
      return;
  }

  Serial.begin(115200);

  serial_queue_init();
  IR_setup();
  ultrasonicSensor.begin();

  rtos_setup();
}



void rtos_setup(){
  xTaskCreate(
    IR_reader
    , "IRread"
    , 3000
    , NULL
    , 1
    , IR_read_handler
  );

  xTaskCreate(
    ultrasonic_reader
    , "SonicRead"
    , 4000
    , NULL
    , 1 
    , sonic_read_handler
  );

  xTaskCreate(
    uartSendTask
    , "uartsend"
    , 2000
    , NULL
    , 2
    , serial_send_handler
  );
}


UBaseType_t uxHighWaterMark;
void loop(){
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( sonic_read_handler );
    // Serial.print("sonic:");
    // Serial.println(uxHighWaterMark);
}
