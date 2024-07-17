#include "test.h"
#include "stdlib.h"

//这里用ax角加速度计算PID
int32_t PID_updata(PID*pid,short dangle)
{
	
	pid->error=(pid->target_value)-(pid->current_value);
	
	if((pid->ki*(pid->error+pid->old_error))>pid->integral||(pid->ki*(pid->error+pid->old_error))<-pid->integral)
		pid->result=(pid->kp)*(pid->error)+(pid->kd*dangle)+pid->integral;
	
	else pid->result=(pid->kp)*(pid->error)+(pid->ki*(pid->acc_error))+(pid->kd*dangle);
	if (pid->result>100)
	{
		pid->result=100;
	}
	else if (pid->result< -100)
	{
		pid->result=-100;
	}
	
	pid->old_error=pid->error;
	pid->acc_error=pid->acc_error+(pid->error);
	return pid->result;
}
//单极直立环
int32_t erect_PID(PID* pide,short dangle,short angle,short target)
{
    pide->current_value=angle;
    pide->target_value=target;

    int32_t result=PID_updata(pide,dangle);
	if(result<=0){all_reverse();}
    else all_forward();

    return abs(result);
}
#ifdef speed
//先根据翻滚角输出速度值，再根据速度值输出电压  直立环+速度环
int32_t speed_erect_PID(PID* pide,PID* pids,short angle,short dangle, short angle_target)
{
    
    pide->current_value=angle;
    pide->target_value=angle_target;
    
    pids->current_value=-left_speed;
    pids->target_value = PID_updata(pide,dangle);
    int32_t result=PID_config(pids);
    if(result>=0) {all_reverse();}
    else all_forward();
    return abs(result);

}

int32_t speed_control(PID* pidc,short target)
{
    pidc->target_value=0;
    pidc->current_value=TIM2->CNT-TIM4->CNT;
    
    int32_t result=PID_config(pidc);
    
    return result;

}


//现在假设小车的偏航角已经稳定   （直立环+速度环）//（速度环+转向环）
int32_t speed_tunr_PID(PID* pide,PID* pids,short angle,short dangle, short angle_target,short roll,short tunr_target)
{
  
    pids->target_value=-(tunr_target+roll);
    pids->current_value=roll;

    pids->target_value=PID_config(pids);//pidtr和pidtl的kp,kd,ki要一样
    pids->current_value=left_speed;

    int32_t tarsp_value=speed_erect_PID(pide,pids,angle,dangle,angle_target);

    int32_t rtunr_value=PID_config(pids);
    int32_t result=tarsp_value+rtunr_value;

    if(result>=0) {all_forward();}
    else all_reverse();
    return  result;//pidsl,pidsr的kp,kd,ki也要一样

}

#endif
//检测左右轮子速度是否相等
void tunr_check()
{
    if(left_speed>right_speed)
    {
        OCAdder_lelf->TIM_Pulse=OCAdder_lelf->TIM_Pulse-1;
        OCAdder_right->TIM_Pulse=OCAdder_right->TIM_Pulse+1;
        tunr_check();
    }
    else if(left_speed<right_speed)
    {
        OCAdder_lelf->TIM_Pulse=OCAdder_lelf->TIM_Pulse+1;
        OCAdder_right->TIM_Pulse=OCAdder_right->TIM_Pulse-1;
        tunr_check();
    }
    else 
    {
        TIM_OC3Init(TIM3,OCAdder_lelf);
        TIM_OC4Init(TIM3,OCAdder_right);
    }
        
    
}


void balance_set()
{
    if((pitch-angle_target)==0)
    {
        OCAdder_lelf->TIM_Pulse=0;
        OCAdder_right->TIM_Pulse=0;
        TIM_OC3Init(TIM3,OCAdder_lelf);
        TIM_OC4Init(TIM3,OCAdder_right);
        
        //PAout(3)=1;
    }
}

#ifdef up
void pidarg_set(int16_t* sign)
{
    switch (*sign)
    {
    case 1:
    {
        
        (pideAdder->kp)=(pideAdder->kp)+0.1;
        printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pideAdder->kp,pideAdder->kd,pideAdder->ki);
    } break;

    case 2:
    {
        
        (pideAdder->kp)=(pideAdder->kp)-0.1;
         printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pideAdder->kp,pideAdder->kd,pideAdder->ki);
    }break;

    case 3:
    {
        
        (pideAdder->kd)=(pideAdder->kd)+0.01;
         printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pideAdder->kp,pideAdder->kd,pideAdder->ki);
    } break;

    case 4:
    {
        
        (pideAdder->kd)=(pideAdder->kd)-0.01;
         printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pideAdder->kp,pideAdder->kd,pideAdder->ki);
    } break;

    case 5:
    {
        
        (pideAdder->ki)=(pideAdder->ki)+0.1;
         printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pideAdder->kp,pideAdder->kd,pideAdder->ki);
    } break;

    case 6:
    {
        
        (pideAdder->ki)=(pideAdder->ki)-0.1;
         printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pideAdder->kp,pideAdder->kd,pideAdder->ki);
    } break;

     case 7:
    {
        angle_target=angle_target+0.01;
        OLED_Printf(0,48,OLED_8X16,"mid:%.2f",angle_target);
        printf("当前中值：%.2f\n",angle_target);
    } break;
    
    case 8:
    {
        angle_target=angle_target-0.01;
        OLED_Printf(0,48,OLED_8X16,"mid:%.2f",angle_target);
        printf("当前中值：%.2f\n",angle_target);
    } break;
    
    case 'a':
    {
        angle_target=angle_target-0.1;
        OLED_Printf(0,48,OLED_8X16,"mid:%.2f",angle_target);
        printf("当前中值：%.2f\n",angle_target);
    }break;

    case 's':
    {
        angle_target=angle_target+0.1;
        OLED_Printf(0,48,OLED_8X16,"mid:%.2f",angle_target);
        printf("当前中值：%.2f\n",angle_target);
    }break;

    default:break;
        
    }
    *(&RX_sign)=0;
   
}
#endif
#ifdef speed
void pidarg_set(int16_t* sign)
{
    switch (*sign)
    {
    case 1:
    {
        
        (pidsAdder->kp)=(pidsAdder->kp)+0.1;
        printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pidsAdder->kp,pidsAdder->kd,pidsAdder->ki);
    } break;

    case 2:
    {

        (pidsAdder->kp)=(pidsAdder->kp)-0.1;
        printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pidsAdder->kp,pidsAdder->kd,pidsAdder->ki);
    }break;

    case 3:
    {
        
        (pidsAdder->kd)=(pidsAdder->kd)+0.01;
        printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pidsAdder->kp,pidsAdder->kd,pidsAdder->ki);
    } break;

    case 4:
    {
        
        (pidsAdder->kd)=(pidsAdder->kd)-0.01;
        printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pidsAdder->kp,pidsAdder->kd,pidsAdder->ki);
    } break;

    case 5:
    {
        
        (pidsAdder->ki)=(pidsAdder->ki)+0.01;
        printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pidsAdder->kp,pidsAdder->kd,pidsAdder->ki);
    } break;

    case 6:
    {
        
        (pidsAdder->ki)=(pidsAdder->ki)-0.01;
        printf("kp:%.3f\nkd:%.3f\nki:%.3f\n",pidsAdder->kp,pidsAdder->kd,pidsAdder->ki);
    } break;

    case 7:
    {
        OLED_Printf(0,24,OLED_8X16,"mid:%.2f",angle_target);
        angle_target=angle_target+0.01;
        printf("当前中值：%.2f\n",angle_target);
    } break;
    
    case 8:
    {
        OLED_Printf(0,24,OLED_8X16,"mid:%.2f",angle_target);
        angle_target=angle_target-0.01;
        printf("当前中值：%.2f\n",angle_target);
    } break;

    case 'a':
    {
        angle_target=angle_target-0.1;
        OLED_Printf(0,48,OLED_8X16,"mid:%.2f",angle_target);
        printf("当前中值：%.2f\n",angle_target);
    }break;

    case 's':
    {
        angle_target=angle_target+0.1;
        OLED_Printf(0,48,OLED_8X16,"mid:%.2f",angle_target);
        printf("当前中值：%.2f\n",angle_target);
    }break;

    default:break;
        
    }
    *(&RX_sign)=0;
   
}
#endif

