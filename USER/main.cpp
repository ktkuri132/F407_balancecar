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
#include "test.h"
#include "lib.h"

extern int16_t RX_sign;
TIM_OCInitTypeDef* OCAdder_lelf;
TIM_OCInitTypeDef* OCAdder_right;
TIM_ICInitTypeDef* ICAdder_lelf;
TIM_ICInitTypeDef* ICAdder_right;
PID* pideAdder;
#ifdef speed
PID* pidsAdder;
#endif
float pitch;
float roll;
float yaw;
#define angle pitch
short gx; short gy; short gz;
int16_t left_speed;
int16_t right_speed;
int16_t result;
float angle_target=-5;
float tunr_target;

GPIO IN1,IN2,IN3,IN4;


/*

逆时针方向旋转->pit++
向前倾斜rol--

*/


int main()
{
    delay_init(168);
    uart_init(9600);
    OLED_Init();

    mpu_dmp_init();
    //BLE_config(ENABLE);
    //              分频 装载  中断开关 中断源
    TIM_config(TIM1,84-1,100-1,ENABLE,TIM1_UP_TIM10_IRQn);//定时器中断        10K
    TIM_config(TIM3,84-1,100-1);//PWM输出
    TIM_config(TIM2,1-1,65536-1);//左电机AB相测速
    TIM_config(TIM4,1-1,65536-1);//右电机AB相测速
    
    OCAdder_lelf=PWM_config(TIM3,3,50,GPIOA,GPIO_Pin_6);//左电机输出
    OCAdder_right=PWM_config(TIM3,4,50,GPIOA,GPIO_Pin_7);//右电机输出
    ICAdder_lelf=EncoderTIM_config(TIM2,GPIOA,GPIO_Pin_2|GPIO_Pin_3,1);//左电机AB相测速
    ICAdder_right=EncoderTIM_config(TIM4,GPIOD,GPIO_Pin_13|GPIO_Pin_14,2);//右电机AB相测速
    TIM_Cmd(TIM1,ENABLE);
    TIM_Cmd(TIM2,ENABLE);
    TIM_Cmd(TIM3,ENABLE);
    TIM_Cmd(TIM4,ENABLE);

    IN1.config(GPIOB,GPIO_Pin_14,GPIO_Mode_OUT);PBout(14)=0; 
    IN2.config(GPIOB,GPIO_Pin_15,GPIO_Mode_OUT);PBout(15)=0;
    IN3.config(GPIOA,GPIO_Pin_4,GPIO_Mode_OUT);PAout(4)=0;//左轮
    IN4.config(GPIOA,GPIO_Pin_5,GPIO_Mode_OUT);PAout(5)=0;                     
    
    PID pide,pids;
    pide.kp=27.5;//  11.3电压  ;36  32.6;
    pide.kd=-0.06;//;-0.12  0.05;
    pide.ki=0;
    pideAdder=&pide;
#ifdef speed
    pids.kp=-0.7;//-0.8    -0.9
    pids.kd=-0.02;//-0.33   -0.86
    pids.ki=-0.069;//0.1     1
    pidsAdder=&pids;
#endif

    while (1)
    {
        
        mpu_dmp_get_data(&pitch,&roll,&yaw);
        //MPU_Get_Gyroscope(&gx,&gy,&gz);
				result=speed_erect_PID(pideAdder,pidsAdder,angle,gy+10,angle_target);
				OCAdder_lelf->TIM_Pulse=result;
				OCAdder_right->TIM_Pulse=result;
				TIM_OC3Init(TIM3,OCAdder_lelf);
				TIM_OC4Init(TIM3,OCAdder_right);
        //pidarg_set(&RX_sign);
        OLED_Printf(0,0,OLED_6X8,"pitch:%4d",(short)(angle-angle_target));
        
        #ifdef up
        OLED_Printf(0,16,OLED_6X8,"p:%.2fd:%.2fi:%.2f",pide.kp,pide.kd,pide.ki);
        #endif
        OLED_Printf(0,8,OLED_6X8,"gy:%6d",gy+10);
        #ifdef speed
        OLED_Printf(0,16,OLED_6X8,"p:%.1fd:%.2fi:%.2f",pids.kp,pids.kd,pids.ki);
        #endif
        OLED_Update();       
        balance_set();
    }
}

extern "C"
{
void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET)
    {
				MPU_Get_Gyroscope(&gx,&gy,&gz);
        //mpu_dmp_get_data(&pitch,&roll,&yaw);
        if(gy>5000||gy< -5000) gy=0;
        left_speed=TIM2->CNT;
        right_speed=TIM4->CNT;
        TIM2->CNT=0;
        TIM4->CNT=0;
    }
    TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
}
}
