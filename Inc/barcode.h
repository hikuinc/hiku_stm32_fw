#ifndef __BARCODE_H
#define __BARCODE_H

#include "stm32f0xx_hal.h"

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

extern uint8_t img_buf[2][IMAGE_COLUMNS];
extern uint32_t img_buf_wr_ptr;

// number of decoded scans to keep in a FIFO for a "best of"
// best of 5: 3 identical scans out of 5 decoded scans
// needed to signal a recognized barcode
#define DECODE_BUFFERS 4
// decode buffer size in bytes
#define DECODE_BUFFER_SIZE 64

#endif
