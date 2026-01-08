#include "main.h"




#ifndef SBUS_H
#define SBUS_H
#define SBUS_RECV_MAX 25
 
 

extern uint8_t sbus_data1[SBUS_RECV_MAX];
extern uint8_t sbus_data2[SBUS_RECV_MAX];
extern int16_t g_sbus_channels[18];
 void SBUS_Parse_Data(uint8_t *sbus_data);
#endif