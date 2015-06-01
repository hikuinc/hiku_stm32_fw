#ifndef __BARCODE_H
#define __BARCODE_H

#include "stm32f0xx_hal.h"

/* define USE_LINEAR_SENSOR to use linear sensor instead of camera */
#define USE_LINEAR_SENSOR

/* barcode image dimensions in pixels */
/* number of rows in the image to capture with a single scan*/
#define IMAGE_COLUMNS 1016

#define CMOS_SENSOR_STOP 0
#define CMOS_SENSOR_READY 1
#define CMOS_SENSOR_ARM	2
#define CMOS_SENSOR_CAPTURE 3

// CMOS sensor state defined in main.c, shared between
// main thread and interrupt routines
extern volatile uint32_t cmos_sensor_state;

extern uint8_t img_buf[IMAGE_COLUMNS][2];
extern uint32_t img_buf_wr_ptr;

// scans to execute at high and low signal gains
//#define SCANS_HIGH_GAIN 2
//#define SCANS_LOW_GAIN 1

/* number of timing samples to use for computation of scans/s */
//#define SCAN_TIMING_SAMPLES 8

#endif
