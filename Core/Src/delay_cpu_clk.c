#include "delay_cpu_clk.h"
#include "stm32f103xe.h"


/* 初始化时间戳 */
void CPUHeartBeatInit(void)
{
    /* 使能DWT外设 */
	DEM_CR |= (uint32_t)DEM_CR_TRCENA;                

	/* DWT CYCCNT寄存器计数清0 */
	DWT_CYCCNT = (uint32_t)0u;
	
	/* 使能Cortex-M3 DWT CYCCNT寄存器 */
    DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
}

void Delay_Clk(uint32_t cpu_tick)
{
	DWT_CYCCNT = 0;
	while (DWT_CYCCNT < cpu_tick);
}
