
extern "C"
{
#include "stm32f4xx.h"
#include "delay.h"
}
#include "stdlib.h"
#include "lib.h"
#include "cmath"
GPIO ADC_use;
GPIO DAC_use;
GPIO PWM_use;
GPIO led_use;
GPIO BLE_use;
GPIO Encoder_use;
uint32_t ADC_data;
/*****************************************APB1总线定时器的参数映射****************************************************/

uint32_t APB1Periph_map(TIM_TypeDef* TIMx)
{
	uint32_t RCC_APB1Periph_TIMx;
	if(TIMx==TIM2)
	{
		RCC_APB1Periph_TIMx=RCC_APB1Periph_TIM2;
	}
	else if(TIMx==TIM3)
	{
		RCC_APB1Periph_TIMx=RCC_APB1Periph_TIM3;
	}
	else if(TIMx==TIM4)
	{
		RCC_APB1Periph_TIMx=RCC_APB1Periph_TIM4;
	}
	else if(TIMx==TIM5)
	{
		RCC_APB1Periph_TIMx=RCC_APB1Periph_TIM5;
	}
	else if(TIMx==TIM6)
	{
		RCC_APB1Periph_TIMx=RCC_APB1Periph_TIM6;
	}
	else if(TIMx==TIM7)
	{
		RCC_APB1Periph_TIMx=RCC_APB1Periph_TIM7;
	}
	else if(TIMx==TIM12)
	{
		RCC_APB1Periph_TIMx=RCC_APB1Periph_TIM12;
	}
	else if(TIMx==TIM13)
	{
		RCC_APB1Periph_TIMx=RCC_APB1Periph_TIM13;
	}
	else if(TIMx==TIM14)
	{
		RCC_APB1Periph_TIMx=RCC_APB1Periph_TIM14;
	}
	return RCC_APB1Periph_TIMx;
}

uint32_t APB2Periph_map(ADC_TypeDef* ADCx)
{
	uint32_t RCC_APB2Periph;
	if(ADCx==ADC1)
	{
		RCC_APB2Periph=RCC_APB2Periph_ADC1;
	}
	if(ADCx==ADC2)
	{
		RCC_APB2Periph=RCC_APB2Periph_ADC2;
	}
	return RCC_APB2Periph;
}

/*************************************中断配置****************************************************/
void IT_config(TIM_TypeDef *TIMx,FunctionalState NewState)
{
	
	
}
/*************************************************定时器配置***********************************************/
//统一定时器配置（用于通用定时器TIM2-TIM14）
void TIM_config(TIM_TypeDef* TIMx,uint16_t Prescaler,uint32_t Period,FunctionalState NewState,uint8_t TIMx_IRQn)
{
	RCC_APB1PeriphClockCmd(APB1Periph_map(TIMx),ENABLE);
	TIM_TimeBaseInitTypeDef TIM_StructInit1;
	TIM_StructInit1.TIM_Prescaler=Prescaler;	
	TIM_StructInit1.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_StructInit1.TIM_Period=Period;
	TIM_StructInit1.TIM_ClockDivision=TIM_CKD_DIV1;		
	TIM_TimeBaseInit(TIMx,&TIM_StructInit1);
	TIM_ITConfig(TIMx,TIM_IT_Update,NewState);
	NVIC_InitTypeDef NVIC_InitStruct1;
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStruct1.NVIC_IRQChannel=TIMx_IRQn;
	NVIC_InitStruct1.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStruct1.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStruct1.NVIC_IRQChannelCmd=NewState;
	NVIC_Init(&NVIC_InitStruct1);
	TIM_Cmd(TIMx,ENABLE);
	
}

/*****************************************ADC转化部分*****************************************************************************************************************/
//GPIO模拟输入AD转换
void ADC_config(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_x,ADC_TypeDef* ADCx,uint8_t ADC_Channel_x)
{
	//使能ADCx时钟
	RCC_APB2PeriphClockCmd(APB2Periph_map(ADCx),ENABLE);
	//GPIO初始化配置
    ADC_use.config(GPIOx,GPIO_Pin_x,GPIO_Mode_AN);
	//实例化ADC模式对象
	ADC_CommonInitTypeDef ADC_InitModeStruct;
	ADC_InitModeStruct.ADC_Mode=ADC_Mode_Independent;//独立模式或多重模式Disabled
	ADC_InitModeStruct.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;//DMA 模式禁止或者使能相应 DMA 模式		
	ADC_InitModeStruct.ADC_Prescaler=ADC_Prescaler_Div4;//设置 ADC 预分频器								这个需要调试
	ADC_InitModeStruct.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles;//设置两个采样阶段之间的延迟周期数  这个需要调试
	ADC_CommonInit(&ADC_InitModeStruct);
	//实例化ADC参数对象
	ADC_InitTypeDef ADC_InitParamStruct;
	ADC_InitParamStruct.ADC_Resolution=ADC_Resolution_12b;//设置ADCx采集分辨率
	ADC_InitParamStruct.ADC_ScanConvMode=DISABLE;//设置扫描或非扫描模式
	ADC_InitParamStruct.ADC_ContinuousConvMode=ENABLE;//设置连续或非连续转换
	ADC_InitParamStruct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;//设置触发方式，开启/禁止 外部触发检测，设置为None则禁止触发
	//ADC_InitParamStruct.ADC_ExternalTrigConv=;//选择用于触发开始转换常规组的外部事件
	ADC_InitParamStruct.ADC_DataAlign=ADC_DataAlign_Right;//设置左右对齐
	ADC_InitParamStruct.ADC_NbrOfConversion=1;//在规则序列中设置N个转换   这里只有一个通道
	//参数对象塞进配置函数
	ADC_Init(ADCx,&ADC_InitParamStruct);
	//配置ADC通道
	ADC_RegularChannelConfig(ADCx,ADC_Channel_x,1,ADC_SampleTime_15Cycles);
	//ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE);
	//ADC_DMACmd(ADCx, ENABLE);
	//AD 启动！！
	ADC_Cmd(ADCx,ENABLE);
	
}

/*******************************************DAC转化**********************************************************************************/
//DAC输出配置
void DAC_config(uint32_t DAC_Channel_x,uint32_t GPIO_Pin_x)
{
	//使能DAC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);
	//初始化GPIOA
	DAC_use.config(GPIOA,GPIO_Pin_x,GPIO_Mode_AN);
	//实例化DAC配置对象
	DAC_InitTypeDef DAC_InitStruct;
	DAC_InitStruct.DAC_Trigger=DAC_Trigger_None;//是否使用触发功能（即硬件外设触发）  
	DAC_InitStruct.DAC_WaveGeneration=DAC_WaveGeneration_None;//是否自动使用波形发生
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//在波形发生器下，用来设置屏蔽/幅值选择器，
	DAC_InitStruct.DAC_OutputBuffer=DAC_OutputBuffer_Disable;//设置输出缓存控制位，虽然输出能力强一点，但是输出没法到 0，这里一般不用
	DAC_Init(DAC_Channel_x,&DAC_InitStruct);
	//使能DAC通道
	DAC_Cmd(DAC_Channel_x,ENABLE);
}
/******************************************DMA配置部分*************************************************************************************************************/
//使用前需配置
void DMA_config(DMA_Stream_TypeDef* DMAy_Streamx,uint32_t DMA_Channel_x)
{
	//使能DMA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	//实例化DMA配置对象
	DMA_InitTypeDef DMA_InitStruct;
	//DMA_StructInit(&DMA_InitStruct);
	DMA_InitStruct.DMA_Channel=DMA_Channel_x;//指定数据流对应通道
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)0x4001204C;//指定外设基地址   这里选择ADC数据寄存器地址
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)&ADC_data;//指定存储器基地址		
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;//指定数据运输方向
	DMA_InitStruct.DMA_BufferSize=1;//指定一次传输数据量大小，这个间接指定了终止地址要到哪里  单位（字节）
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;//选择外设传完了一次后地址指针是否递增			这里只是传完一个数组就行了，无需递增
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Disable;//选择存储器传完了一次后地址是否递增
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;//指定外设传输的数据长度	
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;//指定存储器传输的数据长度
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;//选择是否循环传输			
	DMA_InitStruct.DMA_Priority=DMA_Priority_High;//指定通道的优先级
	DMA_InitStruct.DMA_FIFOMode=DMA_FIFOMode_Disable;//是否开启FIFO模式
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_HalfFull;//若开启了FIFO则选择FIFO的阈值
	DMA_InitStruct.DMA_MemoryBurst=DMA_MemoryBurst_Single;//设置存储器突发传输
	DMA_InitStruct.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;//设置外设突发传输
	DMA_DeInit(DMAy_Streamx);
	DMA_Init(DMAy_Streamx,&DMA_InitStruct);
	DMA_Cmd(DMAy_Streamx,ENABLE);
}
/******************************************PWM输出************************************************************************************************************/
//PWM配置函数  Prescaler一般设41，Period一般设99，占空比50%那么CCR设成50
TIM_OCInitTypeDef* PWM_config(TIM_TypeDef* TIMx,uint8_t x,uint32_t CCR,GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_x,uint8_t GPIO_AF_x)
{
	PWM_use.config(GPIOx,GPIO_Pin_x,GPIO_Mode_AF);
	GPIO_PinAFConfig(GPIOx,((uint16_t)(logf((float)GPIO_Pin_x)/logf(2))),GPIO_AF_x);//复用推挽输出
	TIM_OCInitTypeDef* TIM_OCInitStruct=new TIM_OCInitTypeDef;
	TIM_OCInitStruct->TIM_OCMode=TIM_OCMode_PWM1;//模式
	TIM_OCInitStruct->TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct->TIM_Pulse=CCR;			//对应的PWM的占空比参考Period的值，PWM的周期由Prescaler，Period决定
	TIM_OCInitStruct->TIM_OCPolarity=TIM_OCPolarity_High;//极性
	TIM_OCInitStruct->TIM_OCIdleState=TIM_OCIdleState_Set;//空闲时引脚的状态
	switch (x)
	{
		case 1:TIM_OC1Init(TIMx,TIM_OCInitStruct);break;
		case 2:TIM_OC2Init(TIMx,TIM_OCInitStruct);break;
		case 3:TIM_OC3Init(TIMx,TIM_OCInitStruct);break;	
		case 4:TIM_OC4Init(TIMx,TIM_OCInitStruct);break;
		default:break;
	}
	//这里使能时钟就开始输出了
	//TIM_Cmd(TIMx,ENABLE);
	return TIM_OCInitStruct;
}
/************************************************定时器输入比较***********************************************************/
//1，测编码电机速度
TIM_ICInitTypeDef* EncoderTIM_config(TIM_TypeDef* TIMx,uint8_t x,GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_x,uint8_t GPIO_AF_x,
									uint8_t Polarity,uint16_t TIM_Channel_x,uint16_t Filter)
								
{
	Encoder_use.config(GPIOx,GPIO_Pin_x,GPIO_Mode_AF);
	GPIO_PinAFConfig(GPIOx,((uint16_t)(logf((float)GPIO_Pin_x)/logf(2))),GPIO_AF_x);//复用推挽输出
	TIM_ICInitTypeDef* TIM_ICInitStruct=new TIM_ICInitTypeDef;
	uint16_t TIM_Channely;
	TIM_Channel_x==TIM_Channel_1?TIM_Channely=0x0004:TIM_Channely=0x0000;
	TIM_ICStructInit(TIM_ICInitStruct);
	TIM_ICInitStruct->TIM_Channel=TIM_Channel_x;
	TIM_ICInitStruct->TIM_ICFilter=Filter;
	TIM_ICInit(TIMx,TIM_ICInitStruct);
	TIM_ICInitStruct->TIM_Channel=TIM_Channely;
	TIM_ICInitStruct->TIM_ICFilter=Filter;
	TIM_ICInit(TIMx,TIM_ICInitStruct);
	Polarity==1?TIM_EncoderInterfaceConfig(TIMx,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Rising):
	TIM_EncoderInterfaceConfig(TIMx,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Falling);		
	return TIM_ICInitStruct;

}
/*********************************************PID**************************************************************************/
//PID配置函数
int32_t PID_config(PID*pid)
{
	
	pid->error=(pid->target_value)-(pid->current_value);
	
	if((pid->ki*(pid->error+pid->old_error))>pid->integral||(pid->ki*(pid->error+pid->old_error))<-pid->integral)
		pid->result=(pid->kp)*(pid->error)+(pid->kd*(pid->error-pid->old_error))+pid->integral;
	
	else pid->result=(pid->kp)*(pid->error)+(pid->ki*(pid->acc_error))+(pid->kd*(pid->error-pid->old_error));
	if(pid->result<0)
	{
		pid->result=0;
	}
	else if (pid->result>100)
	{
		pid->result=0;
	}
	pid->old_error=pid->error;
	pid->acc_error=pid->acc_error+(pid->error);
	return pid->result;
}


/************************************************蓝牙模块***********************************************************************/
//蓝牙连接串口2，使用A2,A3口
USART_InitTypeDef* BLE_config(FunctionalState NewState)
{
	BLE_use.config(GPIOA,GPIO_Pin_2|GPIO_Pin_3,GPIO_Mode_AF);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2|GPIO_PinSource3,GPIO_AF_USART2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	USART_InitTypeDef* USART_InitStructure =new USART_InitTypeDef;
	USART_InitStructure->USART_BaudRate = 9600;//波特率设置
	USART_InitStructure->USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure->USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure->USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure->USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure->USART_Mode =USART_Mode_Tx|USART_Mode_Rx;	//收发模式
  	USART_Init(USART2, USART_InitStructure); //初始化串口2
	
	USART_ITConfig(USART2, USART_IT_RXNE, NewState);//开启相关中断
	NVIC_InitTypeDef* NVIC_InitStructure = new NVIC_InitTypeDef;
	//Usart1 NVIC 配置
  	NVIC_InitStructure->NVIC_IRQChannel = USART2_IRQn;//串口1中断通道
	NVIC_InitStructure->NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure->NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure->NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	//USART_Cmd(USART2, ENABLE);  //使能串口1 
	return USART_InitStructure;
}


#ifdef __MPU6050_USE
/*******************************************MPU6050****************************************************************/


#endif