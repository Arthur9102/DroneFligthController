#ifndef __FSIA10B_H
#define __FSIA10B_H
#ifdef __cplusplus
	extern "C"	{
#endif

#include "main.h"
typedef struct FS_IA10B_ibus
{
	unsigned short right_horizontal;	// Cần điều khiển phải theo hướng dọc
	unsigned short right_vertical;		// Cần điều khiển phải theo hướng ngang
	unsigned short left_horizontal;		// Cần điều khiển trái theo hướng dọc
	unsigned short left_vertical;		// Cần điều khiển trái theo hướng ngang
	unsigned short swa;
	unsigned short swb;
	unsigned short swc;
	unsigned short swd;
	unsigned short vra;
	unsigned short vrb;

	unsigned char failsafe;
} FS_IA10B_ibus;

extern FS_IA10B_ibus ibus;

unsigned char ibus_check_CS(unsigned char* data, unsigned char len);
void ibus_message(unsigned char* data, FS_IA10B_ibus* ibus);
void FS_IA10B_USART1_initialization(void);
unsigned char ibus_active_failsafe(FS_IA10B_ibus* ibus);
float map_ibus_to_altitude(unsigned short ibus_value);

#ifdef __cplusplus
}
#endif
#endif