#include <SPI.h>
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "serial.h"
#include "vars.h"

Adafruit_AMG88xx amg;

//INT pin from the sensor board goes to this pin on your microcontroller board
#define INT_PIN 34

//interrupt levels (in degrees C)
//any reading on any pixel above TEMP_INT_HIGH degrees C, or under TEMP_INT_LOW degrees C will trigger an interrupt
//30 degrees should trigger when you wave your hand in front of the sensor
#define TEMP_INT_HIGH 30
#define TEMP_INT_LOW 15
#define PACKAGE_SZ (AMG88xx_PIXEL_ARRAY_SIZE * 4 + 4)

volatile bool intReceived = false;
uint8_t pixelInts[8];
u_char IR_package[PACKAGE_SZ];
/******* 
we can tell which pixels triggered the interrupt by reading the
bits in this array of bytes. Any bit that is a 1 means that pixel triggered

         bit 0  bit 1  bit 2  bit 3  bit 4  bit 5  bit 6  bit 7
byte 0 |  0      1      0      0      0      0      0      1
byte 1 |  0      0      0      0      0      0      0      0
byte 2 |  0      0      0      0      0      0      0      0
byte 3 |  0      0      0      1      0      0      0      0
byte 4 |  0      0      0      0      0      0      0      0
byte 5 |  0      0      0      0      0      0      0      0
byte 6 |  0      0      0      0      0      1      0      0
byte 7 |  0      0      0      0      0      0      0      0

*****/

TaskHandle_t *IR_read_handler;


void IR_setup() {
  bool status;

	// pinMode(INT_PIN, INPUT);

  // Serial.println("AMG88xx interrupt test");
  
  // default settings
  status = amg.begin();
  if (!status) {
      Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
      while (1);
  }
  
  // amg.setInterruptLevels(TEMP_INT_HIGH, TEMP_INT_LOW);

  // //set to absolue value mode
  // amg.setInterruptMode(AMG88xx_ABSOLUTE_VALUE);

  // //enable interrupts
  // amg.enableInterrupt();

  // //attach to our Interrupt Service Routine (ISR)
  // attachInterrupt(digitalPinToInterrupt(INT_PIN), AMG88xx_ISR, FALLING);
}

void AMG88xx_ISR() {
  //keep your ISR short!
  //we don't really want to be reading from or writing to the sensor from inside here.
  intReceived = true;
}

void IR_reader(void *para) { 
	IR_package[0] = PKG_HD1;
	IR_package[1] = PKG_HD2;
	IR_package[2] = 0x01; // tag
	IR_package[3 + AMG88xx_PIXEL_ARRAY_SIZE * 4] = PKG_TL;

  while (1)
  {  
    //read all the pixels
    amg.readPixels(
			(float*)&IR_package[3]
		);

    if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE) {
      for (int i=0; i<PACKAGE_SZ; i+=MAX_DATA_SIZE)
        xQueueSend(uartQueue, &IR_package[i], portMAX_DELAY);
      // 释放互斥量
      xSemaphoreGive(xMutex);
    }
    // Serial.print(headers);
    // for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
    //   send_float((u_char *)&pixels[i]);
    // }
    // Serial.print(tail);

    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("IR:");
    // Serial.println(uxHighWaterMark);
    //delay a second
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}