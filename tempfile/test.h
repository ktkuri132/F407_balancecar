#include "lib.h"
extern "C"
{
#include "OLED.h"
#include "usart.h"
}
#define speed
//#define up

extern int16_t RX_sign;
extern PID* pideAdder;

#ifdef speed
extern PID* pidsAdder;
#endif
extern TIM_OCInitTypeDef* OCAdder_lelf;
extern TIM_OCInitTypeDef* OCAdder_right;
extern int16_t left_speed;
extern int16_t right_speed;
extern int16_t result;
extern float angle_target;
extern float pitch;
extern float tunr_target;
extern short gx;
extern GPIO IN1;
extern GPIO IN2;
extern GPIO IN3;//left
extern GPIO IN4;


//定义左右轮的正反转~   
#define right_forward() {PBout(14)=1;PBout(15)=0;}
#define right_reverse() {PBout(14)=0;PBout(15)=1;}
#define left_forward()  {PAout(5)=1;PAout(4)=0;}
#define left_reverse()  {PAout(5)=0;PAout(4)=1;}
#define all_forward()   {right_forward();left_forward();}
#define all_reverse()   {right_reverse();left_reverse();}
int32_t PID_updata(PID*pid,short dangle);
int32_t erect_PID(PID* pide,short dangle,short angle,short target);//直立环
#ifdef speed
int32_t speed_erect_PID(PID* pide,PID* pids,short angle,short dangle, short angle_target);//直立环+速度环

int32_t speed_tunr_PID(PID* pide,PID* pids,short angle,short dangle, short angle_target,
                        PID*pidtr,PID* pidtl,PID* pidsr,PID* pidsl,short roll,short tunr_target);//转向环+速度环
#endif
//void pidarg_set(int16_t* RX_sign);
void balance_set();
void tunr_check();
int32_t speed_control(PID* pidc);




