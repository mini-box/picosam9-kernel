#ifndef __TOUCHSCREEN_MSG_PUB_H__
#define __TOUCHSCREEN_MSG_PUB_H__


/* ------------------------ Messages Types -------------------------- */
/**
 * \brief Union of all messages received or sent by Touchscreen
*/

typedef struct
{
    u16     x;
    u16     y;
    u16     pressure;
    u16     mode;

} vm_Touchscreen_data_t;

#define MAX_TOUCH_FINGER 2
typedef struct
{
    u16 Row;
    u16 Col;
} vm_TouchPoint_t;

typedef struct
{
    u8 nFunction; 
    u8 nKeyMode;
    u8 nFingerNum;
    u8 nKeyValue;
    u8 nOldKeyValue;
    vm_TouchPoint_t atPoint[MAX_TOUCH_FINGER];
}MdlTouchScreenInfo_t;


typedef struct{
   u8       KeyMode;
   u16      Row;
   u16      Col;
} Body_t;

#endif  /* __TOUCHSCREEN_MSG_H__ */

