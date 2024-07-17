/*

VS Code is the best IDE

*/
extern "C"
{
#include "stm32f4xx.h"
#include "delay.h"
#include "usart.h"
#include "OLED.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "nmswj.h"
#include "math.h"
}
#include "pid.h"
#include "lib.h"

TIM_OCInitTypeDef* OCAdder_left;
TIM_OCInitTypeDef* OCAdder_right;
TIM_ICInitTypeDef* ICAdder_left;
TIM_ICInitTypeDef* ICAdder_right;

float pitch;
float roll;
float yaw;
int16_t left_speed;
int16_t right_speed;
int8_t dir;


PID pide,pids,pidtr,pidtl,pidsr,pidsl;



int main()
{
    delay_init(86);
    uart_init(115200);
    OLED_Init();
    mpu_dmp_init();
	GPIO IN1,IN2,IN3,IN4,LED,key;	
    LED.config(GPIOF,GPIO_Pin_9,GPIO_Mode_OUT);PFout(9)=0;
    //TIM_config(TIM4,84-1,100-1,ENABLE,TIM4_IRQn);
    TIM_config(TIM2,840-1,100-1);
    TIM_config(TIM3,1-1,65536-1);
    TIM_config(TIM5,1-1,65536-1);
    
    OCAdder_left=PWM_config(TIM2,3,0,GPIOB,GPIO_Pin_10,GPIO_AF_TIM2);
    OCAdder_right=PWM_config(TIM2,4,0,GPIOB,GPIO_Pin_11,GPIO_AF_TIM2);
    ICAdder_left=EncoderTIM_config(TIM3,1,GPIOA,GPIO_Pin_0|GPIO_Pin_1,GPIO_AF_TIM3,1);
    ICAdder_right=EncoderTIM_config(TIM5,1,GPIOB,GPIO_Pin_6|GPIO_Pin_7,GPIO_AF_TIM5,1);
    TIM_Cmd(TIM2,ENABLE);
    TIM_Cmd(TIM3,ENABLE);
    //TIM_Cmd(TIM4,ENABLE);
    TIM_Cmd(TIM5,ENABLE);

	
    IN1.config(GPIOD,GPIO_Pin_8,GPIO_Mode_OUT);PDout(8)=0;
    IN2.config(GPIOD,GPIO_Pin_9,GPIO_Mode_OUT);PDout(9)=0;
    IN3.config(GPIOD,GPIO_Pin_10,GPIO_Mode_OUT);PDout(10)=0;
    IN4.config(GPIOD,GPIO_Pin_11,GPIO_Mode_OUT);PDout(11)=0;           
    
    key.config(GPIOA,GPIO_Pin_11,GPIO_Mode_IN);PAin(11)=1;
    
    while (1)
    {
		PBout(8)=0;
        mpu_dmp_get_data(&pitch,&roll,&yaw);
        OLED_Printf(0,0,OLED_8X16,"rol:%5d",(short)(roll*10));
        OLED_Printf(0,16,OLED_8X16,"pit:%5d",(short)(pitch*10));
        OLED_Update();
    }
    OLED_Clear();
    PCout(13)=0;
/*

PAout(5)=0;PAout(4)=1;
*/
    while (PAin(11)==0)//???
    {
        mpu_dmp_get_data(&pitch,&roll,&yaw);
        //OCAdder_left->TIM_Pulse=erect_PID();
        OLED_Printf(0,0,OLED_8X16,"rol:%5d",(short)(roll*10));
        OLED_Printf(0,16,OLED_8X16,"pit:%5d",(short)(pitch*10));
        OLED_Update();

    }
    OLED_Clear();
    while (PAin(11)==1)
    {
        OCAdder_left->TIM_Pulse=0;
        OCAdder_right->TIM_Pulse=0;
        TIM_OC3Init(TIM3,OCAdder_left);
        TIM_OC3Init(TIM3,OCAdder_right);
    }
    
    
}

extern "C"
{
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)
    {
        left_speed=TIM3->CNT;
        right_speed=TIM5->CNT;
        TIM3->CNT=0;
        TIM5->CNT=0;
    }
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
}

}


