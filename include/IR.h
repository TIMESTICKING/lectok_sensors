#ifndef MY_IR_LG_H
#define MY_IR_LG_H



#include <Arduino.h>



extern TaskHandle_t *IR_read_handler;



void IR_setup();
void IR_reader(void *para);

#endif