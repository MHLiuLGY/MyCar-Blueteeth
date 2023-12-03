#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "MyCar.h"
#include "IR.h"
#include "HCSR04.h"
#include "Delay.h"
#include "MPU6050.h"

int8_t Speed;//小车速度

uint8_t Address,Command;//红外遥控的地址和指令
uint8_t DataFlag;//判断是否为IR_GetDataFlag()

uint16_t Distance1,Distance2;//超声波检测到的距离
uint8_t Thre_Dist;//避障模式下的距离阈值

struct MPU6050_Data Data;//MPU6050返回的数据

uint8_t ObstAvoModeFlag;//避障模式
uint8_t FollowModeFlag;//跟随模式

uint8_t FollowModeState;//跟随模式状态
uint8_t FollowModeTargetDirection;//跟随模式目标方位
uint8_t FollowDistMax=50;//跟随检测的最大距离
uint8_t FollowDistMin=15;//跟随检测的最小距离

/*
采用有限状态机处理跟随模式的逻辑
状态：
FollowModeState=0 无目标
FollowModeState=1 有目标，近
FollowModeState=2 有目标，远
FollowModeState=3 目标丢失于左前方
FollowModeState=4 目标丢失于前方
FollowModeState=5 目标丢失于右前方

引入中间变量：
FollowModeTargetDirection=0 目标方向未知
FollowModeTargetDirection=1 目标左前方
FollowModeTargetDirection=2 目标前方
FollowModeTargetDirection=3 目标右前方
*/

int main(){
	OLED_Init();
	MyCar_Init();
	IR_Init();
	HCSR04_Init();
	MPU6050_Init();
	
	OLED_ShowString(1,1,"L:000cm R:000cm");
	OLED_ShowString(2,1,"CMD:00");
	OLED_ShowString(3,1,"Speed:+000");
	OLED_ShowString(4,1,"Accel:");
	
	while(1){
		/*IR*/
		if((DataFlag=IR_GetDataFlag())||IR_GetRepeatFlag()){
			Address=IR_GetAddress();
			Command=IR_GetCommand();

			OLED_ShowHexNum(2,5,Command,2);
			
			switch(Command){//以下16进制表示按键编码，可以考虑用宏定义增加可读性
				case 0x45:Speed=100;OLED_ShowString(2,8,"Num1    ");break;//按下数字1
				case 0x46:Speed=75;OLED_ShowString(2,8,"Num2    ");break;//按下数字2
				case 0x47:Speed=50;OLED_ShowString(2,8,"Num3    ");break;//按下数字3
				case 0x44:Speed=25;OLED_ShowString(2,8,"Num4    ");break;//按下数字4
				case 0x40:FollowModeFlag=0;ObstAvoModeFlag=0;Speed=0;MyCar_Stop();OLED_ShowString(2,8,"Num5    ");break;//按下数字5，停车，且速度清零
				case 0x43:Speed=-25;OLED_ShowString(2,8,"Num6    ");break;//按下数字6
				case 0x07:Speed=-50;OLED_ShowString(2,8,"Num7    ");break;//按下数字7
				case 0x15:Speed=-75;OLED_ShowString(2,8,"Num8    ");break;//按下数字8
				case 0x09:Speed=-100;OLED_ShowString(2,8,"Num9    ");break;//按下数字9
				case 0x1C:FollowModeFlag=0;ObstAvoModeFlag=0;MyCar_Stop();OLED_ShowString(2,8,"OK      ");break;//按下OK，停车，但速度不清零
				case 0x16:Speed++;OLED_ShowString(2,8,"Speed++ ");break;//按下*，长按加速
				case 0x0D:Speed--;OLED_ShowString(2,8,"Speed-- ");break;//按下#，长按减速
				case 0x19:if(DataFlag){FollowModeFlag=1-FollowModeFlag;ObstAvoModeFlag=0;MyCar_Stop();OLED_ShowString(2,8,"Num0    ");FollowModeState=0;FollowModeTargetDirection=0;}break;//按下数字0，切换跟随模式
				case 0x18:FollowModeFlag=0;ObstAvoModeFlag=1;MyCar_GoForward(Speed);OLED_ShowString(2,8,"Forward ");break;//按下↑，前进
				case 0x52:FollowModeFlag=0;ObstAvoModeFlag=0;MyCar_GoBackward(Speed);OLED_ShowString(2,8,"Backward");break;//按下↓，后退
				case 0x08:FollowModeFlag=0;ObstAvoModeFlag=0;MyCar_TurnLeft(Speed);OLED_ShowString(2,8,"Left    ");break;//按下←，左转
				case 0x5A:FollowModeFlag=0;ObstAvoModeFlag=0;MyCar_TurnRight(Speed);OLED_ShowString(2,8,"Right   ");break;//按下→，右转
			}
			
			if(Speed==101||Speed==-101)
				Speed=0;
			
			OLED_ShowSignedNum(3,7,Speed,3);
			if(ObstAvoModeFlag)
				OLED_ShowString(3,12,"ObAv");//前进/避障模式
			else if(FollowModeFlag)
				OLED_ShowString(3,12,"Folo");//跟随模式
			else
				OLED_ShowString(3,12,"    ");//左转、右转、后退、停车、关闭跟随后无其他操作 均归为空状态/模式
			if(Speed>80)
				Thre_Dist=50;
			else
				Thre_Dist=15;
		}
		
		/*HCSR04*/
		Distance1=HCSR04_GetDistance1();
		Distance2=HCSR04_GetDistance2();
		OLED_ShowNum(1,3,Distance1/100,3);
		OLED_ShowNum(1,11,Distance2/100,3);
		if(ObstAvoModeFlag){//仅当避障模式时才执行下列逻辑
			if(Distance1/100<Thre_Dist && Distance2/100>Thre_Dist)
				MyCar_TurnRight(Speed);
			else if(Distance1/100>Thre_Dist && Distance2/100<Thre_Dist)
				MyCar_TurnLeft(Speed);
			else if(Distance1/100>Thre_Dist && Distance2/100>Thre_Dist)
				MyCar_GoForward(Speed);
			else if(Distance1/100<Thre_Dist && Distance2/100<Thre_Dist)
				MyCar_TurnRight(Speed);
		}
		else if(FollowModeFlag){//仅当跟随模式时才执行下列逻辑
			switch(FollowModeState){
				case 0:{
					MyCar_Stop();
					if(Distance1/100<FollowDistMin && Distance2/100>FollowDistMin){//输入1
						FollowModeState=1;
						FollowModeTargetDirection=1;
					}else if(Distance1/100<FollowDistMin && Distance2/100<FollowDistMin){//输入2
						FollowModeState=1;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMin && Distance2/100<FollowDistMin){//输入3
						FollowModeState=1;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMax){//输入4
						FollowModeState=2;
						FollowModeTargetDirection=1;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入5
						FollowModeState=2;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入6
						FollowModeState=2;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax){//输入0
						FollowModeState=0;
						FollowModeTargetDirection=0;
					}else{//取等
						FollowModeState=0;
					}
					break;
				}
				case 1:{
					MyCar_Stop();
					if(Distance1/100<FollowDistMin && Distance2/100>FollowDistMin){//输入1
						FollowModeState=1;
						FollowModeTargetDirection=1;
					}else if(Distance1/100<FollowDistMin && Distance2/100<FollowDistMin){//输入2
						FollowModeState=1;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMin && Distance2/100<FollowDistMin){//输入3
						FollowModeState=1;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMax){//输入4
						FollowModeState=2;
						FollowModeTargetDirection=1;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入5
						FollowModeState=2;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入6
						FollowModeState=2;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax && FollowModeTargetDirection==1){//输入7
						FollowModeState=3;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax && FollowModeTargetDirection==2){//输入8
						FollowModeState=4;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax && FollowModeTargetDirection==3){//输入9
						FollowModeState=5;
					}else{//取等
						FollowModeState=1;
					}
					break;
				}
				case 2:{
					MyCar_GoForward(Speed);
					if(Distance1/100<FollowDistMin && Distance2/100>FollowDistMin){//输入1
						FollowModeState=1;
						FollowModeTargetDirection=1;
					}else if(Distance1/100<FollowDistMin && Distance2/100<FollowDistMin){//输入2
						FollowModeState=1;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMin && Distance2/100<FollowDistMin){//输入3
						FollowModeState=1;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMax){//输入4
						FollowModeState=2;
						FollowModeTargetDirection=1;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入5
						FollowModeState=2;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入6
						FollowModeState=2;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax && FollowModeTargetDirection==1){//输入7
						FollowModeState=3;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax && FollowModeTargetDirection==2){//输入8
						FollowModeState=4;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax && FollowModeTargetDirection==3){//输入9
						FollowModeState=5;
					}else{//取等
						FollowModeState=2;
					}
					break;
				}
				case 3:{
					MyCar_TurnLeft(Speed);
					if(Distance1/100<FollowDistMin && Distance2/100>FollowDistMin){//输入1
						FollowModeState=1;
						FollowModeTargetDirection=1;
					}else if(Distance1/100<FollowDistMin && Distance2/100<FollowDistMin){//输入2
						FollowModeState=1;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMin && Distance2/100<FollowDistMin){//输入3
						FollowModeState=1;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMax){//输入4
						FollowModeState=2;
						FollowModeTargetDirection=1;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入5
						FollowModeState=2;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入6
						FollowModeState=2;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax){//输入10
						FollowModeState=3;
					}else{//取等
						FollowModeState=3;
					}
					break;
				}
				case 4:{//极少可能
					MyCar_GoForward(Speed);
					if(Distance1/100<FollowDistMin && Distance2/100>FollowDistMin){//输入1
						FollowModeState=1;
						FollowModeTargetDirection=1;
					}else if(Distance1/100<FollowDistMin && Distance2/100<FollowDistMin){//输入2
						FollowModeState=1;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMin && Distance2/100<FollowDistMin){//输入3
						FollowModeState=1;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMax){//输入4
						FollowModeState=2;
						FollowModeTargetDirection=1;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入5
						FollowModeState=2;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入6
						FollowModeState=2;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax){//输入10
						FollowModeState=4;
					}else{//取等
						FollowModeState=4;
					}
					break;
				}
				case 5:{
					MyCar_TurnRight(Speed);
					if(Distance1/100<FollowDistMin && Distance2/100>FollowDistMin){//输入1
						FollowModeState=1;
						FollowModeTargetDirection=1;
					}else if(Distance1/100<FollowDistMin && Distance2/100<FollowDistMin){//输入2
						FollowModeState=1;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMin && Distance2/100<FollowDistMin){//输入3
						FollowModeState=1;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMax){//输入4
						FollowModeState=2;
						FollowModeTargetDirection=1;
					}else if(Distance1/100>FollowDistMin && Distance1/100<FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入5
						FollowModeState=2;
						FollowModeTargetDirection=2;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMin && Distance2/100<FollowDistMax){//输入6
						FollowModeState=2;
						FollowModeTargetDirection=3;
					}else if(Distance1/100>FollowDistMax && Distance2/100>FollowDistMax){//输入10
						FollowModeState=5;
					}else{//取等
						FollowModeState=5;
					}
					break;
				}
			}
		}
		Delay_ms(40);
		
		/*MPU6050*/
		MPU6050_GetData(&Data);
		
//		OLED_ShowSignedNum(4,1,Data.AccX,5);
		OLED_ShowSignedNum(4,7,Data.AccY,5);//单位：2g/32767
//		OLED_ShowSignedNum(4,1,Data.AccZ,5);
//		OLED_ShowSignedNum(2,8,Data.GyroX,5);
//		OLED_ShowSignedNum(3,8,Data.GyroY,5);
//		OLED_ShowSignedNum(4,8,Data.GyroZ,5);
	}
}
