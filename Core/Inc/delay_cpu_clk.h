#ifndef DELAY_CPU_CLK_H
#define DELAY_CPU_CLK_H

#include <stdint.h>

/*
在Cortex-M3里面有一个外设叫DWT(Data Watchpoint and Trace)，该外设有一个32位的寄存器叫CYCCNT，它是一个向上的
计数器，记录的是内核时钟运行的个数，最长能记录的时间为：60s=2的32次方/72000000(假设内核频率为72M，内核跳一次的时间大概为1/72M=14ns)
当CYCCNT溢出之后，会清0重新开始向上计数。
使能CYCCNT计数的操作步骤：
1、先使能DWT外设，这个由另外内核调试寄存器DEMCR的位24控制，写1使能
2、使能CYCCNT寄存器之前，先清0
3、使能CYCCNT寄存器，这个由DWT_CTRL(代码上宏定义为DWT_CR)的位0控制，写1使能
*/


#define  DWT_CR      (*(__IO uint32_t *)0xE0001000)
#define  DWT_CYCCNT  (*(__IO uint32_t *)0xE0001004)
#define  DEM_CR      (*(__IO uint32_t *)0xE000EDFC)


#define  DEM_CR_TRCENA                   (1 << 24)
#define  DWT_CR_CYCCNTENA                (1 <<  0)

#define CPU_HEART_BEAT ((uint32_t)DWT_CYCCNT)
void CPUHeartBeatInit(void);
void Delay_Clk(uint32_t cpu_tick);
#endif
