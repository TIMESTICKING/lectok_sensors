#include <SPI.h>
#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <HCSR04.h>

void IR_reader(void *para);
void ultrasonic_reader(void * para);
void rtos_setup();
void IR_setup();
void send_float(u_char * num);


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

TaskHandle_t *IR_read_handler;
TaskHandle_t *sonic_read_handler;

// ultrasonic settings
int trigger_pin = 32;
int echo_pin = 33;
HCSR04 ultrasonicSensor(trigger_pin, echo_pin, 15, 200);

void AMG88xx_ISR() {
  //keep your ISR short!
  //we don't really want to be reading from or writing to the sensor from inside here.
  intReceived = true;
}

void setup() {

  pinMode(INT_PIN, INPUT);
  
  Serial.begin(115200);

  IR_setup();
  ultrasonicSensor.begin();

  rtos_setup();

}


void IR_setup() {
  bool status;

  Serial.println("AMG88xx interrupt test");
  
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
}



void IR_reader(void *para) { 
  static char headers[2] = {0x77, 0x88};
  static char tail = 0x66;
  while (1)
  {  
    //read all the pixels
    amg.readPixels(pixels);

    Serial.print(headers);
    for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
      send_float((u_char *)&pixels[i]);
    }
    Serial.print(tail);

    //delay a second
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


void ultrasonic_reader(void * para){
  static char headers[2] = {0xAF, 0xFA};
  static char tail = 0xFF;
  float distance = 0;
  float filterd_distance = 0;
  while (1)
  {
    distance = ultrasonicSensor.getDistance();

    

    vTaskDelay(pdMS_TO_TICKS(100));


    filterd_distance = ultrasonicSensor.getMedianFilterDistance(); //pass 3 measurements through median filter, better result on moving obstacles



    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
}


void send_float(u_char * num){
  // 发送4个字节
  for (int j = 0; j < sizeof(float); j++) {
    Serial.write(num[j]);
  }
}


void loop(){
    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( sonic_read_handler );
    // Serial.print("SampleTask Stack High Water Mark:");
    // Serial.println(uxHighWaterMark);
}
