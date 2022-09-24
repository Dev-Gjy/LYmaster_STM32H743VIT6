#ifndef CCT_H_
#define CCT_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

unsigned char cct(float cct_Brightness, uint16_t cct_Color_Temperature, float *cct_Cold_Out, float *cct_Warm_Out);

#endif  // CCT_H_