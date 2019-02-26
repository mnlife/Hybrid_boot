#include "main.h"
#include "printf_reloc.h"

#define DEMCR (*(__IO uint32_t*)0xE000EDFC)
#define TPIU_CSPSR (*(__IO uint32_t*) 0xE0040004)
#define TPIU_FFCR (*(__IO uint32_t*) 0xE0040304)
#define TPIU_SPPR (*(__IO uint32_t*) 0xE00400F0)
#define DBGMCU_CR (*(__IO uint32_t*) 0xE0042004)



void ConfigTraceSWO(void)
{
    DEMCR = 0x01000001;
    TPIU_SPPR = 0x00000002;
    DBGMCU_CR = 0x00000020;
}
/* Relocation to SWO */
int fputc(int ch, FILE *f)
{
	ITM_SendChar(ch);
	return (ch);
}
