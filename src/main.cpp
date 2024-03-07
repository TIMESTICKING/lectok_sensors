#include <SPI.h>
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

//INT pin from the sensor board goes to this pin on your microcontroller board
#define INT_PIN 34

//interrupt levels (in degrees C)
//any reading on any pixel above TEMP_INT_HIGH degrees C, or under TEMP_INT_LOW degrees C will trigger an interrupt
//30 degrees should trigger when you wave your hand in front of the sensor
#define TEMP_INT_HIGH 30
#define TEMP_INT_LOW 15

volatile bool intReceived = false;
uint8_t pixelInts[8];
char headers[2] = {0x77, 0x88};
char tail = 0x66;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
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

void AMG88xx_ISR() {
  //keep your ISR short!
  //we don't really want to be reading from or writing to the sensor from inside here.
  intReceived = true;
}

void setup() {

  pinMode(INT_PIN, INPUT);
  
  Serial.begin(115200);
  Serial.println("AMG88xx interrupt test");

  bool status;
  
  // default settings
  status = amg.begin();
  if (!status) {
      Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
      while (1);
  }
  
  amg.setInterruptLevels(TEMP_INT_HIGH, TEMP_INT_LOW);

  //set to absolue value mode
  amg.setInterruptMode(AMG88xx_ABSOLUTE_VALUE);

  //enable interrupts
  amg.enableInterrupt();

  //attach to our Interrupt Service Routine (ISR)
  attachInterrupt(digitalPinToInterrupt(INT_PIN), AMG88xx_ISR, FALLING);

}

// void loop() {
  
//   if(intReceived){
//     //get which pixels triggered
//     amg.getInterrupt(pixelInts);

//     Serial.println("**** interrupt received! ****");
//     for(int i=0; i<8; i++){
//       Serial.println(pixelInts[i], BIN);
//     }
//     Serial.println();

//     //clear the interrupt so we can get the next one!
//     amg.clearInterrupt();
    
//     intReceived = false;
//   }
// }

void loop() { 
    //read all the pixels
    amg.readPixels(pixels);

    // Serial.print("[");
    // for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    //   Serial.print(pixels[i-1]);
    //   Serial.print(", ");
    //   if( i%8 == 0 ) Serial.println();
    // }
    // Serial.println("]");
    // Serial.println();

    Serial.print(headers);
    for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
      unsigned char *byteArray = (unsigned char *)&pixels[i];
    
      // 发送4个字节
      for (int j = 0; j < sizeof(float); j++) {
        Serial.write(byteArray[j]);
      }
    }
    Serial.print(tail);

    //delay a second
    delay(1000);
}
