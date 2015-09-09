#include <stdint.h>     
#include <stdlib.h>     /* malloc, free, abs */
#include <string.h>     /* memset */

#include "zbar.h"
#include "scanner.h"
#include "div_mem.h"

/* scanner state */
struct zbar_scanner_s {
    zbar_decoder_t *decoder; /* associated bar width decoder */
    unsigned y1_min_thresh; /* minimum threshold */

    unsigned x;             /* relative scan position of next sample */
    int y0[4];              /* short circular buffer of average intensities */

    int y1_sign;            /* slope at last crossing */
    unsigned y1_thresh;     /* current slope threshold */

    unsigned cur_edge;      /* interpolated position of tracking edge */
    unsigned last_edge;     /* interpolated position of last located edge */
    unsigned width;         /* last element width */
};

#ifndef ZBAR_FIXED
# define ZBAR_FIXED 5
#endif
#ifndef ROUND
#define ROUND (1 << (ZBAR_FIXED - 1))
#endif

/* FIXME add runtime config API for these */
#ifndef ZBAR_SCANNER_THRESH_MIN
# define ZBAR_SCANNER_THRESH_MIN  4
#endif

#ifndef ZBAR_SCANNER_THRESH_INIT_WEIGHT
# define ZBAR_SCANNER_THRESH_INIT_WEIGHT .44
#endif
#ifndef THRESH_INIT
#define THRESH_INIT ((unsigned)((ZBAR_SCANNER_THRESH_INIT_WEIGHT       \
                                 * (1 << (ZBAR_FIXED + 1)) + 1) / 2))
#endif

#ifndef ZBAR_SCANNER_THRESH_FADE
# define ZBAR_SCANNER_THRESH_FADE 8
#endif
#define ZBAR_SCANNER_THRESH_FADE_SHIFT 3

#define ZBAR_SCANNER_EWMA_WEIGHT 1.5
#define EWMA_WEIGHT ((unsigned)((ZBAR_SCANNER_EWMA_WEIGHT              \
                                 * (1 << (ZBAR_FIXED + 1)) + 1) / 2))
//#endif

static inline zbar_symbol_type_t process_edge_new (zbar_scanner_t *scn)
{
    if(!scn->y1_sign)
        scn->last_edge = scn->cur_edge = (1 << ZBAR_FIXED) + ROUND;
    else if(!scn->last_edge)
        scn->last_edge = scn->cur_edge;

    scn->width = scn->cur_edge - scn->last_edge;
    scn->last_edge = scn->cur_edge;

    /* pass to decoder */
    if(scn->decoder)
        return(zbar_decode_width(scn->decoder, scn->width));
    return(ZBAR_PARTIAL);
}

static inline unsigned calc_thresh_new (zbar_scanner_t *scn, unsigned x)
{	
    /* threshold 1st to improve noise rejection */
    unsigned thresh = scn->y1_thresh;
    if((thresh <= scn->y1_min_thresh) || !scn->width) {
        return(scn->y1_min_thresh);
    }
    /* slowly return threshold to min */
    unsigned dx = (x << ZBAR_FIXED) - scn->last_edge;
    unsigned t = thresh * dx;
    t /= scn->width;
		t >>= ZBAR_SCANNER_THRESH_FADE_SHIFT;
    if(thresh > t) {
        thresh -= t;
        if(thresh > scn->y1_min_thresh)
            return(thresh);
    }
    scn->y1_thresh = scn->y1_min_thresh;
    return(scn->y1_min_thresh);
}

zbar_symbol_type_t zbar_scan_y_new (zbar_scanner_t *scn,
                                uint8_t *img_buf, uint32_t size)
{
	  register int x;
    zbar_symbol_type_t edge = ZBAR_NONE;
    register int y0_0 = img_buf[0]; 
    register int y0_1 = y0_0;
    register int y0_2 = y0_0;
    register int y1_1 = 0;
    register int y1_2 = 0;
    register int y2_1 = 0;
    register int y2_2 = 0;

	for (x=0; x<size; x++){
    /* 1st differential @ x-1 */
    y1_2 = y1_1;
    y1_1 = y0_1 - y0_2;
    if((abs(y1_1) < abs(y1_2)) &&
       ((y1_1 >= 0) == (y1_2 >= 0)))
        y1_1 = y1_2;

    /* 2nd differentials*/
    y2_2 = y2_1;
    y2_1 = y0_0 - (y0_1 * 2) + y0_2;

    /* 2nd zero-crossing is 1st local min/max - could be edge */
    if((!y2_1 ||
        ((y2_1 > 0) ? y2_2 < 0 : y2_2 > 0)) &&
       (calc_thresh_new(scn, x) <= abs(y1_1)))
    {
        /* check for 1st sign change */
        char y1_rev = (scn->y1_sign > 0) ? y1_1 < 0 : y1_1 > 0;
        if(y1_rev) {
            /* intensity change reversal - finalize previous edge */
            edge = process_edge_new(scn);
					  if (edge > ZBAR_PARTIAL)
							return edge;
				}

        if(y1_rev || (abs(scn->y1_sign) < abs(y1_1))) {
            scn->y1_sign = y1_1;

            /* adaptive thresholding */
            /* start at multiple of new min/max */
            scn->y1_thresh = (abs(y1_1) * THRESH_INIT + ROUND) >> ZBAR_FIXED;
            if(scn->y1_thresh < scn->y1_min_thresh)
                scn->y1_thresh = scn->y1_min_thresh;

            /* update current edge */
            int d = y2_1 - y2_2;
            scn->cur_edge = 1 << ZBAR_FIXED;
            if(!d)
                scn->cur_edge >>= 1;
            else if(y2_1)  
                scn->cur_edge -= ((y2_1 << ZBAR_FIXED) + 1) / d;
            scn->cur_edge += x << ZBAR_FIXED;
        }
    }
		y0_2 = y0_1;
    y0_1 = y0_0;
    y0_0 += ((int)((img_buf[x+1] - y0_1) * EWMA_WEIGHT)) >> ZBAR_FIXED;
	}
		scn->x = x;
    return(edge);
}
