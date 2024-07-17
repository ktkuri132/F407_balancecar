/*

兼容AC6，AC5版本的编译器


*/

#include "sys.h"
//采用如下方法实现执行汇编指令WFI  
#ifdef  __CC_ARM
__asm void WFI_SET(void)
{
	WFI;		  
}
//??????(?????fault?NMI??)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//??????
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//??????
//addr:????
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
#elif defined ( __GNUC__ ) || defined (__clang__)
void WFI_SET(void)
{
    __ASM volatile("WFI");
}

//??????(?????fault?NMI??)
void INTX_DISABLE(void)
{
	__ASM volatile("CPSID   I");
	__ASM volatile("BX      LR");  
}
//??????
void INTX_ENABLE(void)
{
	__ASM volatile("CPSIE   I");
	__ASM volatile("BX      LR");
}
//??????
//addr:????
void MSR_MSP(u32 addr) 
{
	__ASM volatile("MSR MSP, r0"); 			//set Main Stack value
	__ASM volatile("BX r14");
}
#endif















