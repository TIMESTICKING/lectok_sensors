#ifndef MY_SONIC_LG_H
#define MY_SONIC_LG_H

#include <Arduino.h>
#include <HCSR04.h>

// ultrasonic settings
#define trigger_pin 32
#define echo_pin 33

void ultrasonic_reader(void * para);

extern HCSR04 ultrasonicSensor;

extern TaskHandle_t *sonic_read_handler;

#endif