#include "can.h"
#include "usart.h"	

#define encoder_canID1 0x21
#define encoder_canID3 0x22
#define encoder_canID4 0x23
#define encoder_canID5 0x24
#define encoder_canID8 0x25

#define INUSE_CANID 0x26
#define SETTIME_CANID 0x27
#define RESET_CANID 0x28

typedef struct {
	int GetTim1;
	int GetTim3;
	int GetTim4;
	int GetTim5;
	int GetTim8;
} SwitchTIM;

typedef union{
    u8 u8_form[4];
    int s32_form;
}data_convert_readTIM;

void encoder_init(SwitchTIM* W);
void encoder_callback();
void SetUsed(u8 N, u8 S);
void SetTime(u8 T);
