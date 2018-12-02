#include "main.h"
const KEY_DEFINE keytable[KEY_COUNT + 16] = {
//Switch definitions
    {HID_NONM,HID_0,	"Rate 0  "},//L0M0
    {HID_NONM,HID_1,	"Rate 1  "},
    {HID_NONM,HID_2,	"Rate 2  "},
    {HID_NONM,HID_3,	"Rate 3  "},//L1M3
    {HID_GUIM,HID_EQ,	"Zoom In "},//L1M0
    {HID_NONM,HID_x,	"Reject  "},//L1M1 lower Rotator SW
    {HID_NONM,HID_u,	"Un-Flags"},//L1M2 Side SW
    {HID_GUIM,HID_z,	"Undo    "},//L1M3
    {HID_GUIM,HID_MI,	"Zoom Out"},//L2M0
    {HID_NONM,HID_NONE,	NULL},		//L2M1 padding
    {HID_NONM,HID_NONE,	NULL},		//L2M2 padding
    {HID_GUIM,HID_F11,	"2nd Mon."},//L2M3
    {HID_GUIM,HID_RB,	"Rotate R"},//L3M0
    {HID_GUIM,HID_LB,	"Rotate L"},
    {HID_ALTM,HID_1,	"Add Kwd."},
    {HID_NONM,HID_b,	"QuikCol."},//L3M3
//Rotator movements
    {HID_NONM,HID_RIGHT,NULL},	//lower CW
    {HID_NONM,HID_LEFT,NULL},	//lower CCW
    {HID_NONM,HID_NONE,NULL},	//padding
    {HID_NONM,HID_NONE,NULL},	//padding
    {HID_NONM,HID_NONE,NULL},	//padding
    {HID_NONM,HID_NONE,NULL},	//padding
    {HID_NONM,HID_NONE,NULL},	//padding
    {HID_NONM,HID_NONE,NULL},	//padding
//etc.
    {HID_NONM,HID_NONE},	//padding
    {HID_NONM,HID_NONE},	//padding
    {HID_NONM,HID_NONE},	//padding
    {HID_NONM,HID_NONE},	//padding
    {HID_NONM,HID_NONE},	//padding
    {HID_NONM,HID_NONE},	//padding
    {HID_NONM,HID_NONE},	//padding
    {HID_NONM,HID_NONE},	//padding
};
