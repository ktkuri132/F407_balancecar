#include "delay.h"
#include "usart.h"
#include "key16.h"
#include "sys.h"
int a;
Question    My = {
	
	0,        // Q_Start;
	0,         //My.Mode
	0,
};

int key_num = 20;

void ch451_write(u16 command)
{
   u8 i;
   load_0;   
   for(i=0;i<12;i++)           //写12位命令  
   {
      if(command&0x0001) din_1;
      else               din_0;
      dclk_0;                  //给其一个上升沿
      dclk_1;
      command>>=1;
   }
   load_1;                         //加载进去
}



void ch451_init(void )
{

	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_4|GPIO_Pin_8;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;     // 普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);     //初始化   GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	din_0;
	din_1;  
	ch451_write(0x0201);//芯片内部复位
	ch451_write(0x0403);	
}


u16 ch451_read(void)
{  
   u8 i,keycode=0;
   u16 command;
	 u8 t=0;
   command=0x07;            
   load_0;                
   for(i=0;i<4;i++)
   {
      if(command&0x0001) din_1;
      else               din_0;                //读取前四位按键代码命令
      dclk_0;
      dclk_1;
      command>>=1;
   }
   load_1;
   for(i=0;i<7;i++)
   {
      keycode<<=1;
      if(dout) keycode|=0X01;        //串口接收数据
      dclk_1;
      dclk_0; 
   }
   a=keycode;
   switch(keycode)
   {
      case 66:t=1;break;
      case 67:t=2;break;
      case 65:t=3;break;
      case 64:t=4;break;
      case 74:t=5;break;
      case 75:t=6;break;
      case 73:t=7;break;
      case 72:t=8;break;
      case 82:t=9;break;
      case 83:t=10;break;
      case 81:t=11;break;
      case 80:t=12;break;
      case 90:t=13;break;
      case 91:t=14;break;
      case 89:t=15;break;
      case 88:t=16;break;
      default:t=0;break; 
   }
    return t; 	
}

/**********************/
//
//mode 1  连按
//     0  单按
/*******************/

//u16 get_value(u8 mode)
//{
//  static u8 last_key=20,now_key=0, key=20;;
//	   if(key!=20)
//		 {
//	   key=20;           
//			last_key=20;      
//			 }                   
//	     now_key=ch451_read();
//			 if(mode==1)
//{
//				 
//		   if(now_key==10||now_key==9||now_key==8||now_key==7||now_key==6||now_key==5||now_key==4||now_key==3||now_key==2||now_key==1||now_key==11||now_key==12||now_key==13||now_key==14||now_key==15||now_key==16)
//			 {
//			 key=now_key;
//			 }
//			 else
//			 {
//       if(now_key==20)  key=last_key;
//	     else  last_key=now_key;
//			 }
//}
//			 else
//{
//		
//			if(now_key==20)  key=last_key;
//			else  last_key=now_key;
//			 }
//	    return key;
//}





//void CH451_Get_val(void)      //界面和键盘
//{ 
//	if(ch451_read()<=16)
//	{
//		key_num = ch451_read();
//		
//	}
//	printf("%d\r\n",get_value(0));
//	  switch(get_value(0))
//{

//	case 1:    My.Mode =1      ;break;     //向上
//    case 2:    My.Mode =2             ;break;     //向下
//	case 3:    My.Mode =3              ;break;     //向下   确认键
//  
// 	case 4:    My.Mode =4           ;break;     //向上
//    case 5:    My.save_flag   = 1       ;break;     //向下
//	case 6:    My.save_flag   = 2      ;break;     //向下   确认键	case 1: ;break;     //向上
//    case 7:    My.save_flag  = 3   ;break;     //向下
//	case 8:    My.save_flag  = 4   ;          ;break;     //向下   确认键	case 1: ;break;     //向上
//    case 9:    My.save_flag  = 5   ;             ;break;     //向下
//	case 10:   My.save_flag  = 6   ;          ;break;     //向下   确认键	case 1: ;break;     //向上
//    case 11:   My.Mode =11           ;break;     //向下
//	case 12:   fdc.data_CALIBRATE  =1          ;break;     //向下   确认键	case 1: ;break;     //向上
//    case 13:   My.Mode =13          ;break;     //向下
//	case 14:    My.Mode =14   ;break;     //向下   确认键	case 1: ;break;     //向上
//    case 15:       if(My.flash_start ==0) My.flash_start = 1;else My.flash_start  =0;     break;     //向下
//	case 16:       My.Mode =16            ;break;     //返回键				
//             
//    default :break;
//}




