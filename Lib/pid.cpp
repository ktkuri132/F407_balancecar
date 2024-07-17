#include "lib.h"
#include "pid.h"
#include "stdlib.h"

extern int16_t left_speed;
extern int16_t right_speed;
extern GPIO IN1;
extern GPIO IN2;
extern GPIO IN3;
extern GPIO IN4;
//定义左右轮的正反转
#define right_forward() {PDout(8)=1;PDout(9)=0;}
#define right_reverse() {PDout(8)=0;PDout(9)=1;}
#define left_forward()  {PDout(10)=0;PDout(11)=1;}
#define left_reverse()  {PDout(10)=1;PDout(11)=0;}
#define all_forward()   {right_forward();left_forward();}
#define all_reverse()   {right_reverse();left_reverse();}

//单极直立环
int32_t erect_PID(PID* pide,short roll,int32_t target)
{
    pide->current_value=roll;
    pide->target_value=target;

    int32_t result=PID_config(pide);
    if(result>=0) {all_forward();}
    else all_reverse();

    return abs(result);
}

//先根据翻滚角输出速度值，再根据速度值输出电压  直立环+速度环
int32_t speed_erect_PID(PID* pide,PID* pids,short roll,short roll_target)
{
    pide->current_value=roll;
    pide->target_value=roll_target;

    pids->current_value=left_speed;
    pids->target_value = PID_config(pide);

    int32_t result=PID_config(pids);
    if(result>=0) {all_forward();}
    else all_reverse();

    return abs(result);

}

//现在假设小车的偏航角已经稳定   （直立环+速度环）//（速度环+转向环）
int32_t speed_tunr_PID(PID* pide,PID* pids,short roll,short roll_target,
                        PID*pidtr,PID* pidtl,PID* pidsr,PID* pidsl,short pitch,short pitch_target)
{
    pidtl->target_value=(pitch_target+pitch);
    pidtl->current_value=pitch;

    pidtr->target_value=-(pitch_target+pitch);
    pidtr->current_value=pitch;

    pidsl->target_value=PID_config(pidtl);//pidtr和pidtl的kp,kd,ki要一样
    pidsl->current_value=left_speed;

    pidsr->target_value=PID_config(pidtr);
    pidsl->current_value=right_speed;

    int32_t tarsp_value=speed_erect_PID(pide,pids,roll,roll_target);

    int32_t rtunr_value=PID_config(pidsr);
    int32_t resultr=tarsp_value+rtunr_value;

    int32_t ltunr_value=PID_config(pidsl);
    int32_t resultl=tarsp_value+ltunr_value;

    if(resultr>=0) {all_forward();}
    else all_reverse();
    if(resultl>=0) {all_forward();}
    else all_reverse();

    return abs(resultr),abs(resultl);//pidsl,pidsr的kp,kd,ki也要一样

}