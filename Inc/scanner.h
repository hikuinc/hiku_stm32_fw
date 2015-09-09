#ifndef __SCANNER_H
#define __SCANNER_H

#include <stdint.h>
#include "zbar.h"

zbar_symbol_type_t zbar_scan_y_new (zbar_scanner_t *scn,
                                    uint8_t *img_buf, uint32_t size, uint8_t scale, uint8_t min_val);
#endif
