#ifndef __pid_h__
#define __pid_h_
#include "lib.h"

int32_t erect_PID(PID* pide,short roll,int32_t target);//直立环
int32_t speed_erect_PID(PID* pide,PID* pids,short roll,short roll_target);//直立环+速度环
int32_t speed_tunr_PID(PID* pide,PID* pids,short roll,short roll_target,
                        PID*pidtr,PID* pidtl,PID* pidsr,PID* pidsl,short pitch,short pitch_target);//转向环+速度环


#endif



