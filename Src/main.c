/**
  ******************************************************************************
  * @file    Src/main.c 
  * @author  Nils Gura
  * @version V1.0
  * @date    26-Apr-2015
  * @brief   Scanner/Microphone
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "zbar.h"
#include "decoder.h"
//#include "stm32f10x.h"
#include "stm32f0xx_hal.h"
#include "barcode.h"
#include "pdm_filter.h"
#include "scanner_hal.h"
#include "main.h"


//#define SCAN_DEBUG
//#define SCAN_DEBUG_SAMPLES 128

/** @addtogroup STM32F0xx_HAL_Demonstrations
  * @{
  */
// AD-converted test data for Zbar that should result in correct bar codes
/*
unsigned short test0[1024] = { 0x03fc, 0x0250, 0x024e, 0x0252, 0x0251, 0x0244, 0x0245, 0x025f, 0x026b, 0x0258, 0x0259, 0x0267, 0x026e, 0x0264, 0x025a, 0x0268, 0x0277, 0x0278, 0x026a, 0x0260, 0x0279, 0x0271, 0x026e, 0x0268, 0x0278, 0x0282, 0x0285, 0x027d, 0x0279, 0x0274, 0x0263, 0x0275, 0x0259, 0x0272, 0x0272, 0x026a, 0x0276, 0x0266, 0x025c, 0x026d, 0x0270, 0x027a, 0x026c, 0x0288, 0x0266, 0x0276, 0x026f, 0x0270, 0x025a, 0x0263, 0x0278, 0x0272, 0x0280, 0x025f, 0x0275, 0x0265, 0x0273, 0x026d, 0x0270, 0x0269, 0x0270, 0x0270, 0x0280, 0x0265, 0x0266, 0x027e, 0x027e, 0x0275, 0x026b, 0x0276, 0x0269, 0x026c, 0x0268, 0x0278, 0x0277, 0x0274, 0x026b, 0x0260, 0x027f, 0x026d, 0x027a, 0x026d, 0x0267, 0x0288, 0x0282, 0x027e, 0x026b, 0x0281, 0x0274, 0x026b, 0x0268, 0x026e, 0x0279, 0x0272, 0x0265, 0x027b, 0x0264, 0x0279, 0x0280, 0x0274, 0x027d, 0x026d, 0x027e, 0x0283, 0x026e, 0x028a, 0x0284, 0x0282, 0x0276, 0x0277, 0x0277, 0x0280, 0x0273, 0x0276, 0x0274, 0x0262, 0x0279, 0x026e, 0x0281, 0x027e, 0x0283, 0x0273, 0x026b, 0x0277, 0x0298, 0x0289, 0x026b, 0x026d, 0x0274, 0x0276, 0x0281, 0x0273, 0x028a, 0x0285, 0x0276, 0x026d, 0x0288, 0x0265, 0x029d, 0x0287, 0x0286, 0x0274, 0x0273, 0x025b, 0x027a, 0x0276, 0x027d, 0x027a, 0x0276, 0x0280, 0x026f, 0x0286, 0x0272, 0x0299, 0x0286, 0x0280, 0x027d, 0x028a, 0x027e, 0x027c, 0x0276, 0x0288, 0x027e, 0x0280, 0x0276, 0x028a, 0x0277, 0x0272, 0x0277, 0x0285, 0x027c, 0x0281, 0x027f, 0x0290, 0x0293, 0x0275, 0x0290, 0x027d, 0x029c, 0x0286, 0x0283, 0x0284, 0x0281, 0x0277, 0x0280, 0x0288, 0x0289, 0x027d, 0x027a, 0x0273, 0x0275, 0x0287, 0x0277, 0x0284, 0x027a, 0x0285, 0x027c, 0x0270, 0x0291, 0x027d, 0x0280, 0x026c, 0x028d, 0x0285, 0x02a7, 0x0286, 0x029a, 0x0284, 0x0294, 0x0291, 0x026c, 0x028a, 0x027c, 0x026f, 0x028c, 0x027d, 0x0286, 0x0290, 0x027c, 0x0270, 0x026d, 0x0273, 0x0271, 0x0269, 0x0271, 0x0281, 0x027a, 0x0276, 0x0266, 0x0283, 0x0274, 0x0272, 0x027e, 0x027b, 0x0272, 0x027a, 0x028a, 0x0286, 0x027b, 0x026f, 0x0294, 0x026d, 0x0284, 0x026f, 0x0277, 0x026e, 0x0279, 0x0275, 0x02a5, 0x027e, 0x028c, 0x0277, 0x027c, 0x028a, 0x0297, 0x0299, 0x02b6, 0x02ca, 0x02f3, 0x031f, 0x032e, 0x0318, 0x02f1, 0x02cd, 0x02ba, 0x029c, 0x0299, 0x028f, 0x028b, 0x0280, 0x0296, 0x026b, 0x0282, 0x028c, 0x0288, 0x0284, 0x028a, 0x028f, 0x0285, 0x027a, 0x0291, 0x029d, 0x0288, 0x0295, 0x0293, 0x02a4, 0x02cb, 0x02cb, 0x02c2, 0x02b2, 0x02a9, 0x028c, 0x02a0, 0x028e, 0x0284, 0x0286, 0x0288, 0x0297, 0x028e, 0x0292, 0x029e, 0x0289, 0x027e, 0x029b, 0x0298, 0x02a1, 0x0292, 0x029c, 0x02a2, 0x029f, 0x02b6, 0x029d, 0x02ab, 0x02b1, 0x02a6, 0x029e, 0x02c3, 0x02be, 0x02e1, 0x0325, 0x038e, 0x03b7, 0x0404, 0x042e, 0x0419, 0x0418, 0x0417, 0x0402, 0x0410, 0x03f3, 0x03ec, 0x03d1, 0x03be, 0x0397, 0x03b5, 0x03a5, 0x03dd, 0x0431, 0x043d, 0x0457, 0x042d, 0x03fa, 0x03ed, 0x03b7, 0x0376, 0x0337, 0x0300, 0x02e0, 0x02c2, 0x02c8, 0x02d8, 0x02ae, 0x02d4, 0x02cf, 0x02c8, 0x02e2, 0x02c7, 0x02d4, 0x02df, 0x02f0, 0x032f, 0x0386, 0x03f6, 0x04cc, 0x057f, 0x0603, 0x0639, 0x0635, 0x0651, 0x065b, 0x0661, 0x0656, 0x0673, 0x068b, 0x0677, 0x0679, 0x0690, 0x0677, 0x0670, 0x067a, 0x066f, 0x0664, 0x0697, 0x0679, 0x068c, 0x0683, 0x0695, 0x067e, 0x0686, 0x0690, 0x0693, 0x067f, 0x0698, 0x0690, 0x0680, 0x0684, 0x067e, 0x0691, 0x0689, 0x068d, 0x068c, 0x0689, 0x0694, 0x0669, 0x066e, 0x066e, 0x0679, 0x067d, 0x0676, 0x065f, 0x0661, 0x063e, 0x061b, 0x0589, 0x0503, 0x044f, 0x03f0, 0x03aa, 0x0434, 0x04ce, 0x055b, 0x05b7, 0x0583, 0x04db, 0x0430, 0x03dd, 0x0409, 0x04be, 0x0565, 0x05f2, 0x0642, 0x0629, 0x0665, 0x0644, 0x05ce, 0x055e, 0x0494, 0x0412, 0x041b, 0x04a4, 0x0555, 0x05fc, 0x0645, 0x0640, 0x0649, 0x0646, 0x064d, 0x0660, 0x064b, 0x0624, 0x05ce, 0x0550, 0x0476, 0x0409, 0x0451, 0x04d9, 0x0583, 0x05f6, 0x0638, 0x064b, 0x0636, 0x062a, 0x05dd, 0x0549, 0x0488, 0x0408, 0x0428, 0x04bc, 0x0570, 0x0604, 0x0627, 0x063b, 0x065e, 0x0665, 0x064d, 0x0646, 0x0642, 0x061d, 0x05c5, 0x0505, 0x0449, 0x03ea, 0x0412, 0x04c9, 0x0572, 0x05e4, 0x0623, 0x0621, 0x0628, 0x05d0, 0x0540, 0x046a, 0x03bb, 0x0346, 0x0320, 0x02ef, 0x0328, 0x0343, 0x03ca, 0x0470, 0x052f, 0x056d, 0x0544, 0x047f, 0x03c2, 0x033d, 0x0301, 0x0304,
 0x0300, 0x033a, 0x0395, 0x0455, 0x0504, 0x0563, 0x056c, 0x04e4, 0x042e, 0x03c6, 0x03d1, 0x0482, 0x0538, 0x05bd, 0x05fb, 0x05ee, 0x0600, 0x05d6, 0x055f, 0x04a9, 0x0401, 0x0355, 0x030c, 0x02e8, 0x02f4, 0x02dc, 0x02dc, 0x02df, 0x02f4, 0x0332, 0x0376, 0x0432, 0x04ff, 0x0596, 0x05c9, 0x05dd, 0x05dc, 0x05b6, 0x0554, 0x048a, 0x03ba, 0x033b, 0x031b, 0x030b, 0x0300, 0x032b, 0x039c, 0x0464, 0x0510, 0x0567, 0x0547, 0x04b4, 0x03e3, 0x0344, 0x0312, 0x0310, 0x0309, 0x0334, 0x037f, 0x0428, 0x04fa, 0x0595, 0x05f2, 0x05f1, 0x05fa, 0x05fa, 0x060c, 0x0609, 0x05f9, 0x05d7, 0x059d, 0x04ef, 0x0451, 0x03ad, 0x03ef, 0x0493, 0x054e, 0x05ac, 0x05ef, 0x060c, 0x05ed, 0x05b8, 0x0561, 0x04e9, 0x0428, 0x03b1, 0x041f, 0x04ad, 0x0539, 0x056a, 0x0525, 0x0493, 0x03d1, 0x03a6, 0x03f0, 0x0495, 0x052d, 0x0567, 0x0527, 0x0474, 0x03da, 0x0392, 0x03f2, 0x048a, 0x054c, 0x0566, 0x0517, 0x0465, 0x03df, 0x037d, 0x03dd, 0x0495, 0x052c, 0x055a, 0x0513, 0x046b, 0x03b6, 0x0332, 0x02fb, 0x0301, 0x0312, 0x033f, 0x03a5, 0x0449, 0x0512, 0x0595, 0x05b2, 0x05bd, 0x05c3, 0x05c1, 0x05c7, 0x05c1, 0x0595, 0x0556, 0x04b9, 0x03fa, 0x0390, 0x03ab, 0x0425, 0x04ef, 0x053b, 0x0517, 0x04af, 0x03ee, 0x0361, 0x031f, 0x02fa, 0x030a, 0x031d, 0x0370, 0x040d, 0x04d4, 0x0552, 0x0585, 0x0589, 0x05af, 0x0596, 0x05ab, 0x05a6, 0x059b, 0x0571, 0x04e0, 0x0439, 0x03a6, 0x0377, 0x03f2, 0x048b, 0x050e, 0x052a, 0x04d1, 0x041f, 0x0375, 0x02fa, 0x0305, 0x0301, 0x02f7, 0x034e, 0x03d4, 0x0479, 0x051e, 0x0560, 0x0578, 0x0576, 0x0586, 0x0573, 0x057b, 0x055b, 0x052a, 0x04c7, 0x0406, 0x0371, 0x02e6, 0x02e7, 0x02e4, 0x02dd, 0x02f5, 0x0377, 0x0409, 0x04a2, 0x04ce, 0x045c, 0x03cf, 0x0332, 0x02f2, 0x02bb, 0x02b1, 0x02b6, 0x02c2, 0x02a6, 0x02bc, 0x02c8, 0x02fa, 0x0381, 0x0421, 0x0489, 0x047e, 0x0426, 0x037f, 0x0344, 0x0365, 0x03d5, 0x044f, 0x048a, 0x045d, 0x03af, 0x030c, 0x02d2, 0x02b0, 0x02aa, 0x02c7, 0x0298, 0x0286, 0x02a6, 0x0298, 0x028d, 0x02b2, 0x02a4, 0x02b3, 0x0316, 0x037c, 0x0412, 0x044b, 0x0434, 0x03ce, 0x035e, 0x031b, 0x0366, 0x03bd, 0x042d, 0x0447, 0x0417, 0x0379, 0x031a, 0x02d2, 0x02c3, 0x02ca, 0x02d1, 0x02fb, 0x036b, 0x03df, 0x0441, 0x0472, 0x048b, 0x047b, 0x0486, 0x0475, 0x0475, 0x045a, 0x0438, 0x03cf, 0x0362, 0x030a, 0x032d, 0x037f, 0x03dc, 0x0417, 0x03fe, 0x03a6, 0x0355, 0x0306, 0x0327, 0x0377, 0x03c9, 0x042b, 0x0430, 0x0444, 0x0454, 0x0466, 0x0457, 0x045f, 0x0464, 0x044b, 0x0464, 0x0469, 0x0456, 0x0444, 0x0457, 0x0455, 0x045b, 0x043f, 0x0450, 0x045a, 0x044a, 0x044c, 0x045f, 0x044a, 0x0449, 0x0445, 0x0450, 0x045f, 0x0449, 0x044b, 0x0442, 0x0442, 0x0437, 0x0445, 0x0440, 0x0443, 0x0435, 0x0430, 0x0423, 0x043e, 0x041b, 0x0438, 0x040e, 0x0424, 0x0408, 0x040d, 0x03ea, 0x03b6, 0x0369, 0x0301, 0x02d1, 0x02b4, 0x029e, 0x02a5, 0x02a1, 0x0282, 0x0287, 0x0291, 0x029b, 0x0280, 0x0291, 0x02a1, 0x0294, 0x0277, 0x027a, 0x0275, 0x0279, 0x0280, 0x0282, 0x0282, 0x028d, 0x0277, 0x0272, 0x0260, 0x0277, 0x0280, 0x026a, 0x0271, 0x027e, 0x027c, 0x0280, 0x026f, 0x027b, 0x027a, 0x0278, 0x026a, 0x0269, 0x026c, 0x0270, 0x0271, 0x026e, 0x024c, 0x028a, 0x026b, 0x0271, 0x0266, 0x0276, 0x0276, 0x0263, 0x0255, 0x0267, 0x0274, 0x0262, 0x0269, 0x028b, 0x0268, 0x0262, 0x026e, 0x0275, 0x0274, 0x026d, 0x0263, 0x0273, 0x0267, 0x027c, 0x0274, 0x0265, 0x0274, 0x026f, 0x026b, 0x025d, 0x026b, 0x0269, 0x026b, 0x0278, 0x026a, 0x0266, 0x0267, 0x0265, 0x026c, 0x025a, 0x027b, 0x0274, 0x0266, 0x0271, 0x0278, 0x0277, 0x0273, 0x026d, 0x026b, 0x026a, 0x0262, 0x026d, 0x0263, 0x025c, 0x0260, 0x0250, 0x0279, 0x025c, 0x0268, 0x027f, 0x0281, 0x024b, 0x025b, 0x024d, 0x026a, 0x0277, 0x0268, 0x0255, 0x026d, 0x0277, 0x0270, 0x0264, 0x026f, 0x0262, 0x025a, 0x0271, 0x0264, 0x0255, 0x0253, 0x026b, 0x0265, 0x0274, 0x0272, 0x0258, 0x0249, 0x0261, 0x025f, 0x0261, 0x0278, 0x0250, 0x0268, 0x026c, 0x025a, 0x026a, 0x027f, 0x0254, 0x0272, 0x027a, 0x0279, 0x025e, 0x0268, 0x0257, 0x025e, 0x0253, 0x025e, 0x0251, 0x0269, 0x026d, 0x025a, 0x0269, 0x0269, 0x0251, 0x0277, 0x0272, 0x0258, 0x026b, 0x0266, 0x0271, 0x024c, 0x026d, 0x026d, 0x0269, 0x0264, 0x026e, 0x025a, 0x0277, 0x0267, 0x0258, 0x0260, 0x026a, 0x0271, 0x0256, 0x0270, 0x025a, 0x0241, 0x0242, 0x0242, 0x0252, 0x024c,
 0x0240, 0x024b, 0x0252, 0x0254};
unsigned short test1[1024] = {0x03fd,0x00b5,0x00af,0x0095,0x0093,0x00cd,0x0090,0x006d,0x00b8,0x00bd,0x00d3,0x00fa,0x0110,0x00e7,0x0106,0x012f,0x0100,0x00f9,0x0112,0x0108,0x0110,0x0107,0x0114,0x00e4,0x0129,0x0110,0x0124,0x0103,0x0100,0x00eb,0x011a,0x012e,0x010e,0x0111,0x00f4,0x0127,0x00fc,0x00fd,0x0108,0x00fd,0x0109,0x012c,0x00dc,0x0100,0x012e,0x011a,0x0113,0x0103,0x0128,0x00d0,0x0100,0x0127,0x0107,0x010a,0x0105,0x0100,0x0102,0x0101,0x00d4,0x0128,0x0115,0x00f6,0x0103,0x0139,0x0128,0x0109,0x0104,0x00fe,0x012a,0x00f4,0x00ed,0x0112,0x0121,0x0119,0x00ec,0x0108,0x0128,0x00ee,0x0115,0x00f1,0x0118,0x0113,0x0116,0x0103,0x0103,0x00f6,0x0109,0x0105,0x0118,0x0108,0x011a,0x0120,0x0102,0x0129,0x013f,0x0108,0x011f,0x011b,0x0111,0x0139,0x0102,0x0122,0x0120,0x011c,0x011c,0x012a,0x0120,0x0138,0x010a,0x0133,0x011c,0x00fa,0x0108,0x0139,0x0125,0x0128,0x0104,0x0123,0x0143,0x0104,0x0134,0x010d,0x0132,0x010c,0x013b,0x0129,0x0120,0x011e,0x0119,0x0140,0x0100,0x0129,0x00f8,0x0111,0x0149,0x010f,0x0124,0x0138,0x0137,0x0154,0x0129,0x0113,0x011d,0x0106,0x0110,0x0129,0x015d,0x0128,0x0109,0x0129,0x0134,0x0136,0x011e,0x0126,0x0145,0x013c,0x0112,0x0113,0x0103,0x0134,0x0136,0x0126,0x0118,0x0134,0x0121,0x0110,0x011d,0x012a,0x0142,0x012e,0x0146,0x0138,0x012c,0x0114,0x011e,0x013d,0x0131,0x0137,0x013e,0x0141,0x012b,0x0106,0x0135,0x012b,0x013d,0x010b,0x015a,0x00f2,0x0123,0x00fc,0x0114,0x0131,0x00eb,0x0121,0x013c,0x0114,0x0127,0x0107,0x010b,0x0102,0x0130,0x011d,0x0120,0x0119,0x0136,0x0134,0x012a,0x00fb,0x0122,0x011a,0x0129,0x010b,0x012e,0x0120,0x012f,0x013f,0x0119,0x00f7,0x0151,0x0114,0x011b,0x0117,0x0134,0x013d,0x0108,0x0128,0x013d,0x0121,0x014a,0x0126,0x0111,0x00f1,0x00f0,0x00f5,0x0110,0x00f3,0x00da,0x00f0,0x0108,0x0101,0x0100,0x0107,0x00e6,0x00ef,0x00e4,0x0108,0x00e7,0x00f2,0x012c,0x0110,0x0103,0x0100,0x0116,0x00ed,0x00fc,0x0112,0x0127,0x00e0,0x0113,0x00ea,0x012e,0x0118,0x0161,0x0125,0x0116,0x0153,0x0154,0x017d,0x015d,0x01ba,0x0273,0x0313,0x0364,0x0384,0x03a1,0x03aa,0x037f,0x03b7,0x03b5,0x03d0,0x03d2,0x03c1,0x03bf,0x03ce,0x03ce,0x03d1,0x03c5,0x03af,0x03c9,0x03b6,0x03ea,0x03fb,0x03d2,0x03e5,0x0400,0x03f9,0x03ee,0x03f2,0x03da,0x03c5,0x0416,0x03ea,0x03cb,0x03d9,0x03f2,0x03ed,0x03ce,0x03d2,0x03e2,0x03ac,0x030a,0x0249,0x01d6,0x0235,0x02d8,0x036c,0x0395,0x031d,0x0281,0x01e0,0x01ed,0x025f,0x0344,0x03d8,0x03dc,0x03ca,0x0400,0x03f9,0x03f8,0x03d6,0x03d4,0x0382,0x02a0,0x01ec,0x019c,0x0194,0x01a9,0x01c6,0x01cc,0x028f,0x032d,0x03a7,0x03b1,0x031e,0x0222,0x01f5,0x024b,0x030e,0x03b3,0x0396,0x0325,0x025f,0x01ee,0x0208,0x02d6,0x03b1,0x041d,0x040b,0x0410,0x041c,0x044b,0x0411,0x0424,0x042d,0x03b2,0x02d5,0x0216,0x01b3,0x01b4,0x0190,0x01bc,0x01bf,0x0234,0x0356,0x03f7,0x042f,0x0439,0x0458,0x0435,0x042b,0x0458,0x041a,0x0420,0x03bd,0x0304,0x023f,0x020c,0x0295,0x0371,0x0404,0x0419,0x038d,0x0266,0x01cd,0x01b8,0x01dd,0x01a2,0x01c6,0x0208,0x02b2,0x03c4,0x0426,0x0479,0x0476,0x0445,0x046d,0x046b,0x043f,0x044c,0x043e,0x0389,0x02a8,0x01d2,0x01b7,0x01b2,0x01b9,0x01c3,0x01eb,0x0260,0x0356,0x03cc,0x043e,0x03d9,0x02fd,0x021d,0x0210,0x02c0,0x03b6,0x0492,0x0483,0x049d,0x047b,0x0496,0x04b4,0x0496,0x0484,0x048a,0x043f,0x0333,0x0231,0x01f3,0x01c9,0x01ad,0x01b4,0x01c3,0x0202,0x02c3,0x03cc,0x0473,0x0452,0x039c,0x029d,0x0236,0x0239,0x0292,0x03d9,0x0485,0x0499,0x04bf,0x04cb,0x04ae,0x04d8,0x04a9,0x0480,0x0492,0x0438,0x03ae,0x0284,0x01c4,0x01d2,0x01ba,0x01e9,0x01e1,0x01f2,0x0276,0x0382,0x0441,0x0460,0x0450,0x0321,0x025c,0x0204,0x027d,0x0396,0x047d,0x046a,0x0451,0x036b,0x025c,0x022c,0x0267,0x0373,0x0443,0x0461,0x0455,0x03a1,0x02a4,0x023e,0x025e,0x0330,0x0446,0x0454,0x044f,0x03cb,0x02ba,0x020a,0x01a3,0x01cc,0x018e,0x01a2,0x0198,0x0199,0x01b9,0x01dc,0x01b3,0x01ee,0x028d,0x03cf,0x0485,0x04bf,0x04de,0x04ca,0x04dc,0x04b8,0x0456,0x036b,0x0291,0x024d,0x02a1,0x03b8,0x0484,0x04a6,0x04a3,0x03c9,0x02b1,0x0210,0x025a,0x0345,0x044c,0x04a7,0x04ad,0x03f9,0x031d,0x0221,0x01d4,0x01eb,0x01db,0x01b8,0x019a,0x01c8,0x01cf,0x01d3,0x01d9,0x01fa,0x029b,0x03aa,0x046e,0x04c3,0x04f1,0x04e1,0x04ed,0x0502,0x04b0,0x043e,0x031d,0x0221,0x01ee,0x020a,0x01f1,0x021b,0x01f2,0x0252,0x02e4,0x0426,0x04c9,0x0485,0x0451,0x0363,0x0241,0x01dd,
	0x01e7,0x020d,0x01f5,0x020c,0x0230,0x0297,0x03c9,0x04a9,0x04cb,0x04f8,0x0519,0x0509,0x052a,0x04d7,0x0404,0x0322,0x0273,0x025c,0x0312,0x0425,0x04ca,0x0521,0x0516,0x0518,0x04f3,0x04e2,0x04cb,0x0443,0x0341,0x0267,0x0237,0x021e,0x020b,0x01d5,0x01f0,0x0205,0x01ee,0x01fd,0x020a,0x0233,0x028f,0x036f,0x044f,0x04cc,0x04e4,0x047f,0x03a0,0x02a5,0x0266,0x02a2,0x0387,0x0488,0x04c7,0x04df,0x0494,0x03a2,0x02e9,0x027d,0x02b9,0x0357,0x045f,0x0500,0x053c,0x054e,0x0587,0x057b,0x0579,0x0557,0x056f,0x0545,0x0576,0x0577,0x0560,0x055a,0x057b,0x053d,0x0534,0x0458,0x032b,0x027c,0x025b,0x0306,0x03c8,0x049f,0x0532,0x04ee,0x0480,0x0347,0x02ce,0x029a,0x02aa,0x031e,0x043b,0x0516,0x0561,0x0546,0x0542,0x054c,0x0555,0x0569,0x0583,0x0562,0x0557,0x0577,0x055e,0x054d,0x058f,0x0511,0x052d,0x0498,0x0397,0x02ea,0x0288,0x02b9,0x0339,0x044e,0x04d2,0x0515,0x04dd,0x0471,0x0333,0x02c2,0x02c7,0x02e1,0x03bb,0x049b,0x0516,0x0538,0x0559,0x053e,0x053d,0x055d,0x058d,0x0580,0x0556,0x0561,0x0566,0x054d,0x0559,0x053f,0x0533,0x056c,0x055e,0x0557,0x053a,0x052d,0x053d,0x054d,0x0532,0x0512,0x0502,0x0577,0x050e,0x0501,0x0537,0x0501,0x054b,0x0531,0x0510,0x052b,0x051b,0x0537,0x0510,0x0511,0x050b,0x0511,0x0536,0x04f6,0x04f6,0x04f8,0x0507,0x0536,0x04ec,0x04de,0x04ce,0x04ce,0x04b4,0x048c,0x0473,0x03f5,0x030d,0x0284,0x01c6,0x0183,0x017d,0x0185,0x0176,0x0163,0x0156,0x0188,0x0122,0x0177,0x0162,0x0144,0x0155,0x0182,0x014c,0x0145,0x014a,0x012b,0x013f,0x013d,0x014f,0x0129,0x0156,0x013b,0x012e,0x014d,0x011c,0x014b,0x012c,0x0134,0x0143,0x0121,0x0133,0x0128,0x011d,0x011c,0x012c,0x0126,0x0145,0x0171,0x013b,0x014b,0x0143,0x0162,0x012b,0x0161,0x0170,0x0157,0x0159,0x0129,0x013d,0x0142,0x012b,0x014d,0x015d,0x015c,0x0169,0x0146,0x0139,0x014d,0x0149,0x0149,0x0146,0x0142,0x0161,0x0164,0x0157,0x017f,0x0166,0x0165,0x016c,0x014a,0x0170,0x018a,0x0144,0x018d,0x0157,0x016f,0x019b,0x0170,0x01aa,0x01a8,0x0190,0x0193,0x01be,0x0178,0x01c3,0x01d8,0x01fb,0x0225,0x0215,0x022a,0x0225,0x022d,0x0240,0x025b,0x029e,0x02aa,0x02ab,0x02fd,0x0335,0x037a,0x0372,0x03a4,0x03dd,0x048b,0x04e6,0x0566,0x05ef,0x064e,0x07f6,0x09c0,0x09c7,0x0a24,0x0c9a,0x0ea2,0x0f81,0x0fed,0x0ff8,0x0ff2,0x0ff7,0x0d17,0x0bf3,0x0c7c,0x0ca5,0x0c00,0x0a7a,0x0920,0x08e7,0x0918,0x08db,0x095d,0x091b,0x085b,0x0768,0x0679,0x0523,0x04c8,0x0468,0x046b,0x04a6,0x045d,0x0465,0x0460,0x03fa,0x0409,0x03ef,0x0456,0x04e5,0x05a1,0x0670,0x077c,0x0830,0x07f6,0x07bf,0x0872,0x091b,0x0a53,0x0bee,0x0cf6,0x0efa,0x0fee,0x0ff8,0x0ff3,0x0ff4,0x0ff5,0x0ff9,0x0ffa,0x0ff9,0x0ffe,0x0ff6,0x0ff8,0x0ffb,0x0ff6,0x0ff3,0x0ff8,0x0ffb,0x0ffe,0x0ff3,0x0ff7,0x0ff7,0x0ff7,0x0ff9,0x0ff6,0x0ffc,0x0ffa,0x0ff7,0x0ff6,0x0ff7,0x0ff2,0x0ff8,0x0ff8,0x0ff4,0x0ffc,0x0fef,0x0ff6,0x0ff8,0x0ff4,0x0ff7,0x0ff4,0x0ff6,0x0e63,0x0d11,0x0c3e,0x0bf2,0x0b37,0x0a15,0x08f6,0x089e,0x07b7,0x06e8,0x0650,0x05a3,0x0539,0x04ee,0x04e6,0x0482,0x04ad,0x0460,0x040e,0x03e1,0x03a0,0x036e,0x0347,0x0368,0x02de,0x031b,0x030a,0x032b,0x0305,0x02d0,0x02fa,0x02e6,0x0306,0x02ea,0x02fe,0x02c6,0x02c5,0x02a3,0x0273,0x0267,0x0220,0x01df,0x01ce,0x01ef,0x01c9,0x01b4,0x01a1,0x019a,0x0191,0x018b,0x01ac,0x018e,0x00ed,0x00bc,0x00ce,0x00a4,0x00d1,0x00aa,0x00e2,0x00d0,0x009e};
*/

/* Private typedef -----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef    TimHandleSp;
	
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;
PDMFilter_InitStruct Filter[DEFAULT_AUDIO_IN_CHANNEL_NBR];
static int16_t RecBuf[PCM_OUT_SIZE*DEFAULT_AUDIO_IN_CHANNEL_NBR];
static uint8_t AudioPacketBuf[PACKET_HDR_LEN + AUDIO_PAYLOAD_LEN];
static uint8_t ScanPacketBuf[PACKET_HDR_LEN + SCAN_PAYLOAD_LEN];

/* ADC handle declaration */
ADC_HandleTypeDef             AdcHandle;

/* Buffer used for reception */
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];
/* Buffer used for processing */
static uint16_t InternalBufferCopy[INTERNAL_BUFF_SIZE];

/* Scanner buffers */
// use double-buffering (ping-pong buffering) to have one buffer to process
// for bar codes while the other is being filled from the AD converter
uint8_t img_buf[IMAGE_COLUMNS][2];
uint32_t img_buf_wr_ptr;

//static volatile uint8_t scan_triggered;
static uint8_t scan_decoded;
unsigned char decode_buffer[DECODE_BUFFER_SIZE][DECODE_BUFFERS];
uint8_t decode_count;
uint8_t scan_wr_ptr;
uint8_t audio_pkt_count;
uint8_t discard_samp;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/
	
void UART_Config(void) {
		/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = UART_BAUD_RATE;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK) Error_Handler();
  if(HAL_UART_Init(&UartHandle) != HAL_OK) Error_Handler();
}

// Send string via UART
void SendString(char* data_string){
	uint32_t char_count=0;	
	while (char_count<255 && data_string[char_count] != 0) char_count++;
	if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*) data_string, char_count)!= HAL_OK) Error_Handler();
}

/**
  * @brief  Initialize the PDM library.
  * @param  AudioFreq: Audio sampling frequency
  * @param  ChnlNbr: Number of audio channels (1: mono; 2: stereo)
  * @retval None
  */
static void PDMDecoder_Init(uint32_t AudioFreq, uint32_t ChnlNbr)
{ 
  uint32_t i = 0;
  
  /* Enable CRC peripheral to unlock the PDM library */
  __CRC_CLK_ENABLE();
  
  for(i = 0; i < ChnlNbr; i++)
  {
    /* Filter LP and HP Init */
    Filter[i].LP_HZ = AudioFreq / 2;
    Filter[i].HP_HZ = 10;
    Filter[i].Fs = AudioFreq;
    Filter[i].Out_MicChannels = 1;
    Filter[i].In_MicChannels = ChnlNbr; 
    PDM_Filter_Init((PDMFilter_InitStruct *)&Filter[i]);
  }  
	//for(i = 0; i < INTERNAL_BUFF_SIZE; i++)
	//  InternalBufferCopy[i] = 0xAAAA;
	//for(i = 0; i < AUDIO_DISCARD_SAMP; i++)
	//  PDM_Filter_64_LSB((uint8_t*)&InternalBufferCopy[0], (uint16_t*)&(RecBuf[0]), DEFAULT_AUDIO_IN_VOLUME , (PDMFilter_InitStruct *)&Filter[0]);
}

int8_t ALaw_Encode(int16_t number)
{
   const uint16_t ALAW_MAX = 0xFFF;
   uint16_t mask = 0x800;
   uint8_t sign = 0;
   uint8_t position = 11;
   uint8_t lsb = 0;
   if (number < 0)
   {
      number = -number;
      sign = 0x80;
   }
   if (number > ALAW_MAX)
   {
      number = ALAW_MAX;
   }
   for (; ((number & mask) != mask && position >= 5); mask >>= 1, position--);
   lsb = (number >> ((position == 4) ? (1) : (position - 4))) & 0x0f;
   return (sign | ((position - 4) << 4) | lsb) ^ 0x55;
}

static void symbol_handler (zbar_decoder_t *dcode)
{
    uint32_t i, j;
	  uint32_t decodes_equal;
    zbar_symbol_type_t type = zbar_decoder_get_type(dcode);

	  if(type <= ZBAR_PARTIAL)
      return;
		
    const char *data = zbar_decoder_get_data(dcode);
    unsigned datalen = zbar_decoder_get_data_length(dcode);
		
		if (datalen > DECODE_BUFFER_SIZE)
			return;

		if (decode_count >= DECODE_BUFFERS/2) {
			decodes_equal = 0;
			for (i=0; i<decode_count; i++) {
				for (j=0; j<datalen; j++) 
			     if (data[j] != decode_buffer[j][i])
							break;
				if (j == datalen)
					decodes_equal++;
				if (decodes_equal == DECODE_BUFFERS/2) {
		      ScanPacketBuf[PACKET_LEN_FIELD] = datalen+3;
					for (j=0; j<datalen; j++)
					  ScanPacketBuf[PACKET_HDR_LEN + j] = data[j];
		      ScanPacketBuf[PACKET_HDR_LEN + j] = '\r';
		      ScanPacketBuf[PACKET_HDR_LEN + j+1] = '\n';
		      ScanPacketBuf[PACKET_HDR_LEN + j+2] = 0;
					scan_decoded = 1;
					//scan result is sent in the main loop
					return;
				}
		  }
		}

    for (i=0; i<datalen; i++) 
      decode_buffer[i][scan_wr_ptr] = data[i];
		scan_wr_ptr = (scan_wr_ptr+1) % DECODE_BUFFERS;
		decode_count = (decode_count >= DECODE_BUFFERS) ? DECODE_BUFFERS : (decode_count + 1);
}

/* turn on scanner LED on/off */
void setScannerLED (uint8_t value) {
	  HAL_GPIO_WritePin(SCAN_ON_LED_PORT, SCAN_ON_LED_PIN, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* set scanner image sensor gain high/low */
void setScannerGAIN (uint8_t value) {
	HAL_GPIO_WritePin(SCAN_GS_PORT, SCAN_GS_PIN, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void toggleDebug() {
	HAL_GPIO_TogglePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN);
}

// send the software version to the Imp via UART
void sendSWVersion(){
	  uint8_t pkt_buf[8];
	
		pkt_buf[0] = (PACKET_HDR >> 24) & 0xFF;
		pkt_buf[1] = (PACKET_HDR >> 16) & 0xFF;
		pkt_buf[2] = (PACKET_HDR >> 8) & 0xFF;
		pkt_buf[3] = PACKET_HDR & 0xFF;
		pkt_buf[4] = PKT_TYPE_SW_VERSION;
		pkt_buf[5] = 0x02;
		pkt_buf[6] = SOFTWARE_VERSION;
		pkt_buf[7] = SOFTWARE_REVISION;
	
	  // use a blocking transmit with a 20ms timeout to transmit the software version
	  if(HAL_UART_Transmit(&UartHandle, pkt_buf, PACKET_HDR_LEN + pkt_buf[PACKET_LEN_FIELD], 20)!= HAL_OK) Error_Handler();
}

void Img_Scanner_Configuration(void)
{ 												
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_HandleTypeDef    TimHandle;
	TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfig;
  ADC_ChannelConfTypeDef        adc_sConfig;
		
  /*
  *  Image sensor chip enable (active low), MT710T SENSOR_PD/pin 8
  */
	SENSOR_PD_CLK_ENABLE();
  GPIO_InitStruct.Pin = SENSOR_PD_PIN;
  GPIO_InitStruct.Mode = SENSOR_PD_MODE;
  GPIO_InitStruct.Pull = SENSOR_PD_PULL;
  GPIO_InitStruct.Speed = SENSOR_PD_SPEED;
  HAL_GPIO_Init(SENSOR_PD_PORT, &GPIO_InitStruct);  
  /* enable image sensor */
  HAL_GPIO_WritePin(SENSOR_PD_PORT, SENSOR_PD_PIN, GPIO_PIN_RESET); 

  /*
  *  Scanner LED enable PA6, MT700 pin 3
  */
	SCAN_ON_LED_CLK_ENABLE();
  GPIO_InitStruct.Pin = SCAN_ON_LED_PIN;
  GPIO_InitStruct.Mode = SCAN_ON_LED_MODE;
  GPIO_InitStruct.Pull = SCAN_ON_LED_PULL;
  GPIO_InitStruct.Speed = SCAN_ON_LED_SPEED;
  HAL_GPIO_Init(SCAN_ON_LED_PORT, &GPIO_InitStruct);  
  /* turn on scanner LED off */
  HAL_GPIO_WritePin(SCAN_ON_LED_PORT, SCAN_ON_LED_PIN, GPIO_PIN_RESET); 
	
  /*
  *  Image gain, MT710T GS/pin 2
  */
	SCAN_GS_CLK_ENABLE();
  GPIO_InitStruct.Pin = SCAN_GS_PIN;
  GPIO_InitStruct.Mode = SCAN_GS_MODE;
  GPIO_InitStruct.Pull = SCAN_GS_PULL;
  GPIO_InitStruct.Speed = SCAN_GS_SPEED;
  HAL_GPIO_Init(SCAN_GS_PORT, &GPIO_InitStruct);  
  /* set high gain */
  HAL_GPIO_WritePin(SCAN_GS_PORT, SCAN_GS_PIN, GPIO_PIN_SET); 

  /*
  *  Scan line start/end, MT710T SYNC/pin 1
  */
	SCAN_SYNC_CLK_ENABLE();
  GPIO_InitStruct.Pin = SCAN_SYNC_PIN;
  GPIO_InitStruct.Mode = SCAN_SYNC_MODE;
  GPIO_InitStruct.Pull = SCAN_SYNC_PULL;
  GPIO_InitStruct.Speed = SCAN_SYNC_SPEED;
  HAL_GPIO_Init(SCAN_SYNC_PORT, &GPIO_InitStruct);  

  /* Enable and set EOS EXTI Interrupt to the second highest priority */
  HAL_NVIC_SetPriority(SCAN_SYNC_EXTI_IRQn, 0x01, 0x00);
  HAL_NVIC_EnableIRQ(SCAN_SYNC_EXTI_IRQn);
  	
  /*
  *  Scanner start pulse signal, MT710T SP/pin 6
  */			 

  TimHandleSp.Instance = SCAN_SP_TIM;
  TimHandleSp.Init.Prescaler         = SCAN_SP_PRESCALER;
  TimHandleSp.Init.Period            = SCAN_SP_PERIOD;
  TimHandleSp.Init.ClockDivision     = 0;
  TimHandleSp.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandleSp.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&TimHandleSp) != HAL_OK) Error_Handler();

  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.Pulse = SCAN_SP_PULSE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandleSp, &sConfig, SCAN_SP_TIM_CHANNEL) != HAL_OK) Error_Handler();
	 
  HAL_NVIC_SetPriority(SCAN_SP_IRQn, 0x01, 0x00);
  HAL_NVIC_EnableIRQ(SCAN_SP_IRQn);

  /*
  *  Scanner clock pulse signal, MT710T CP/pin 7
  */
  TimHandle.Instance = SCAN_CP_TIM;
  TimHandle.Init.Prescaler         = SCAN_CP_PRESCALER;
  TimHandle.Init.Period            = SCAN_CP_PERIOD;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) Error_Handler();

  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.Pulse = SCAN_CP_PULSE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, SCAN_CP_TIM_CHANNEL) != HAL_OK) Error_Handler();
	
	sMasterConfig.MasterOutputTrigger = SCAN_CP_TIM_TRGO;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig)!= HAL_OK) Error_Handler();
	
	if (HAL_TIM_PWM_Start(&TimHandle, SCAN_CP_TIM_CHANNEL)!= HAL_OK) Error_Handler();

 /*
  *  Debug output pin
  */
	DEBUG_OUT_CLK_ENABLE();
  GPIO_InitStruct.Pin = DEBUG_OUT_PIN;
  GPIO_InitStruct.Mode = DEBUG_OUT_MODE;
  GPIO_InitStruct.Pull = DEBUG_OUT_PULL;
  GPIO_InitStruct.Speed = DEBUG_OUT_SPEED;
  HAL_GPIO_Init(DEBUG_OUT_PORT, &GPIO_InitStruct);  
  HAL_GPIO_WritePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN, GPIO_PIN_RESET); 


  /*
  *  Analog image sensor signal, MT710T CP/pin 7
  */
  AdcHandle.Instance = SCAN_VOUT_ADC;	
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;
  AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION8b; //ADC_RESOLUTION12b;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.ContinuousConvMode    = DISABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle.Init.ExternalTrigConv      = SCAN_VOUT_ADC_TRIG;
	AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING; //ADC_EXTERNALTRIGCONVEDGE_FALLING; 
  AdcHandle.Init.EOCSelection          = EOC_SINGLE_CONV; // convert once per trigger
  AdcHandle.Init.DMAContinuousRequests = DISABLE; 
  AdcHandle.Init.Overrun               = OVR_DATA_OVERWRITTEN;
 
  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK) Error_Handler();
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK) Error_Handler();
  
  /* ### - 3 - Channel configuration ######################################## */
  adc_sConfig.Channel      = SCAN_VOUT_ADC_CHANNEL;
  adc_sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
  adc_sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; //ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &adc_sConfig) != HAL_OK) Error_Handler();

	if (HAL_TIM_PWM_Start_IT(&TimHandleSp, SCAN_SP_TIM_CHANNEL)!= HAL_OK) Error_Handler();
}

void Mic_Configuration() {
  TIM_HandleTypeDef    TimHandle;
  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Configure PA6 to turn on SCAN_ON_LED_EN for the scanner */
  /* Enable the GPIO_A clock */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); 

	/*## Configure the TIM peripheral to output 1MHz microphone clock #######################################*/
  TimHandle.Instance = TIMx;
	/* Compute the prescaler value to have TIM3 counter clock equal to 16000000 Hz */
  TimHandle.Init.Prescaler         = (uint32_t)(SystemCoreClock / 16000000) - 1;
  TimHandle.Init.Period            = PERIOD_VALUE;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) Error_Handler();

  /*## Configure PWM channel 4 on timer 3 to output on PB1 #########################################*/
  /* Common configuration for all channels */
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfig.Pulse = PULSE3_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
  /* Output microphone clock signal */
  if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

	/* Configure the PDM library */
  PDMDecoder_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_CHANNEL_NBR);

  /*## Configure the SPI as a slave interface #######################################*/
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES_RXONLY;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
  SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLED;
  SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  SpiHandle.Init.Mode              = SPI_MODE_SLAVE;
  if (HAL_SPI_Init(&SpiHandle) != HAL_OK) Error_Handler();
	
	HAL_NVIC_SetPriority(SPIx_DMA_RX_IRQn, 0x02, 0x00);
	
if(HAL_SPI_Receive_DMA(&SpiHandle, (uint8_t *)InternalBuffer, 2*INTERNAL_BUFF_SIZE) != HAL_OK) Error_Handler();

}

//
// HACK HACK HACK
//
// temporary workaround to configure LP3923 before Imp is connected to WiFi
// avoids LP3923 shutdown

void writeI2CBit (uint8_t i2c_bit) {
	int j;
	int jdelay = 10;
	int ddelay = 1;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, i2c_bit);
  for(j=0;j<ddelay;j++);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET); 
	for(j=0;j<jdelay;j++);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET); 
	for(j=0;j<(jdelay-ddelay);j++);
}

void startLP3923() {
	GPIO_InitTypeDef GPIO_InitStruct;
	int i, j;
	int jdelay = 10;

	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	/* PA0 SCL */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* PB3 SDA */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  // start condition
  for(j=0; j<jdelay; j++);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
  for(j=0;j<jdelay;j++);
  
	// I2C Address 0x7E
  writeI2CBit(1);
  writeI2CBit(1);
  writeI2CBit(1);
  writeI2CBit(1);
  writeI2CBit(1);
  writeI2CBit(1);
  writeI2CBit(0);
	// Write command
  writeI2CBit(0);
	// Ack
  writeI2CBit(1);

	// Register 0x00
	for (i=0; i<8;i++)
    writeI2CBit(0);
	// Ack
  writeI2CBit(1);

	// Data 0x06
  writeI2CBit(0);
  writeI2CBit(0);
  writeI2CBit(0);
  writeI2CBit(0);
  writeI2CBit(0);
  writeI2CBit(1);
  writeI2CBit(1);
  writeI2CBit(0);
	// Ack
  writeI2CBit(1);

	for(j=0;j<jdelay;j++);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{	
  uint32_t scans;
  zbar_decoder_t *decoder;
  zbar_scanner_t *scanner;
  zbar_symbol_type_t edge;

	uint32_t i;
  TIM_HandleTypeDef    TimHandle;
		
  /* STM32F0xx HAL library initialization */
  HAL_Init();

  startLP3923();
	
  /* Configure the system clock to have a system clock = 48 Mhz */
  SystemClock_Config();
		
	img_buf_wr_ptr = 0;
	discard_samp = 0;
	cmos_sensor_state = CMOS_SENSOR_ARM;
	
    Img_Scanner_Configuration();
		setScannerLED(1);

    Mic_Configuration();
		// UART needs to be configured last in order for DMA remapping to work
	  UART_Config();


		sendSWVersion();
		
		AudioPacketBuf[0] = ScanPacketBuf[0] = (PACKET_HDR >> 24) & 0xFF;
		AudioPacketBuf[1] = ScanPacketBuf[1] = (PACKET_HDR >> 16) & 0xFF;
		AudioPacketBuf[2] = ScanPacketBuf[2] = (PACKET_HDR >> 8) & 0xFF;
		AudioPacketBuf[3] = ScanPacketBuf[3] = PACKET_HDR & 0xFF;
		AudioPacketBuf[PACKET_TYPE_FIELD] = PKT_TYPE_AUDIO;
		ScanPacketBuf[PACKET_TYPE_FIELD] = PKT_TYPE_SCAN;
		AudioPacketBuf[PACKET_LEN_FIELD] = AUDIO_PAYLOAD_LEN;

    decoder = zbar_decoder_create();
    scanner = zbar_scanner_create(decoder);
    zbar_decoder_set_handler(decoder, symbol_handler);
	
	  zbar_scanner_new_scan(scanner);
	
	  scan_decoded = 0;
		audio_pkt_count = 0;
		decode_count = 0;
		scan_wr_ptr = 0;
		scans = 0;

    while(1) {
			if (scan_decoded) {
				uint32_t uart_state;
				do {
					uart_state = HAL_UART_GetState(&UartHandle);
				} while ((uart_state == HAL_UART_STATE_BUSY) || (uart_state == HAL_UART_STATE_BUSY_TX) || (uart_state == HAL_UART_STATE_BUSY_TX_RX));
				if(HAL_UART_Transmit_DMA(&UartHandle, ScanPacketBuf, PACKET_HDR_LEN + ScanPacketBuf[PACKET_LEN_FIELD])!= HAL_OK) Error_Handler();
				setScannerLED(0);
				while(1);
			}
				
			// wait for DMA transfer to finish
			while (cmos_sensor_state != CMOS_SENSOR_STOP); 
			// switch between ping-pong buffers
			img_buf_wr_ptr ^= 1;				
      // alternate high gain and low gain scans
			setScannerGAIN(scans%2 == 0);
    	cmos_sensor_state = CMOS_SENSOR_READY;
#ifdef SCAN_DEBUG
			if (scans && (scans % 4 == 0))
		    if(HAL_UART_Transmit_DMA(&UartHandle, img_buf[img_buf_wr_ptr^1], IMAGE_COLUMNS) != HAL_OK) Error_Handler();
#endif
      scans++;
				
    	i=0;
    	do {
    		edge = zbar_scan_y(scanner, img_buf[i][img_buf_wr_ptr^1]);
    		i++;
    	} while ((edge <= ZBAR_PARTIAL) && (i<IMAGE_COLUMNS));

			zbar_scanner_new_scan(scanner);			
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) // HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	static uint32_t i;
	
	//HAL_GPIO_WritePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN, GPIO_PIN_SET);
	
	  if (!scan_decoded) {
			
			// Copy data from DMA buffer and start recording next sample
			// (could be done better with DMA ping-pong buffering using SPI interrupts)
			for (i=0; i<INTERNAL_BUFF_SIZE; i++)
				InternalBufferCopy[i]=InternalBuffer[i];

		  if(HAL_SPI_Receive_DMA(&SpiHandle, (uint8_t *)InternalBuffer, 2*INTERNAL_BUFF_SIZE) != HAL_OK) Error_Handler();
			

				// Convert 1ms of PDM data to 16-bit PCM data
				// For 16kHz/16-bit PCM with a decimation factor of 64, this corresponds to
				// 16*64bits = 128 bytes read from the SPI interface (at 1MHz SCK)
				// (1MHz SCK -> 15.625kHz actual recording frequency; 1.024MHz -> 16kHz)			
				PDM_Filter_64_LSB((uint8_t*)&InternalBufferCopy[0], (uint16_t*)&(RecBuf[0]), DEFAULT_AUDIO_IN_VOLUME , (PDMFilter_InitStruct *)&Filter[0]);

				// Down-sample from 16kHz to 8KHz, convert 16-bit PCM to 8-bit a-law
				for (i=0; i<(PCM_OUT_SIZE*DEFAULT_AUDIO_IN_CHANNEL_NBR)/2; i++)
					AudioPacketBuf[PACKET_HDR_LEN + audio_pkt_count*(PCM_OUT_SIZE*DEFAULT_AUDIO_IN_CHANNEL_NBR)/2 + i] = ALaw_Encode(RecBuf[2*i]);
				
				// Discard the first samples to allow the microphone's power supply to stabilize
				// and the PDM filter to stabilize
				if (discard_samp < AUDIO_DISCARD_SAMP) {
					discard_samp++;
					return;
				}

				audio_pkt_count++;
			  // send the packet header ahead of time to avoid overwriting of data in AudioPacketBuf
				// if the interrupt routine is called again prior to the data having been transmitted
			  if (audio_pkt_count == AUDIO_SAMP_PER_PACKET-1)
				  if(HAL_UART_Transmit_DMA(&UartHandle, AudioPacketBuf, PACKET_HDR_LEN)!= HAL_OK) Error_Handler();
				if (audio_pkt_count >= AUDIO_SAMP_PER_PACKET) {
					audio_pkt_count = 0;
					if(HAL_UART_Transmit_DMA(&UartHandle, &AudioPacketBuf[PACKET_HDR_LEN], AUDIO_PAYLOAD_LEN)!= HAL_OK) Error_Handler();
				}
		}
	//HAL_GPIO_WritePin(DEBUG_OUT_PORT, DEBUG_OUT_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
