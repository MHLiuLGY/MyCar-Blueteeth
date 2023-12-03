#include "stm32f10x.h"                  // Device header
#include "MyCar.h"
#include "IR.h"
#include "HCSR04.h"
#include "Delay.h"
#include "MPU6050.h"
#include "HC05.h"
#include <string.h>

uint32_t main_Pow(uint32_t X, uint32_t Y);//求幂

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

char MyCar_Status[6][17];//小车状态的字串变量

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
    HC05_Init();
    MyCar_Init();
    IR_Init();
    HCSR04_Init();
    MPU6050_Init();
    
    //以下考虑到copy的可选字符串均不可能越界，使用更简便的strcpy，其余处使用strncpy防止溢出
    strcpy(MyCar_Status[0],"L:000cm");//左前方超声波传感器探测到的距离
    strcpy(MyCar_Status[1],"R:000cm");//右前方超声波传感器探测到的距离
    strcpy(MyCar_Status[2],"");//当前指令
    strcpy(MyCar_Status[3],"Speed:+000");//当前小车设置的速度
    strcpy(MyCar_Status[4],"None");//避障或跟随的模式指示
    strcpy(MyCar_Status[5],"Accel:");//MPU6050提供的小车当前加速度的AD值
    
    while(1){
        /*HC05*/
        HC05_GetStrPacket();
        if(HC05_RxFlag==1){
            if(!strcmp(HC05_RxStrPacket,"Forward0")){//按下↑，前进
                FollowModeFlag=0;
                ObstAvoModeFlag=1;
                MyCar_GoForward(Speed);
                strcpy(MyCar_Status[2],"Forward0");
            }else if(!strcmp(HC05_RxStrPacket,"Backward0")){//按下↓，后退
                FollowModeFlag=0;
                ObstAvoModeFlag=0;
                MyCar_GoBackward(Speed);
                strcpy(MyCar_Status[2],"Backward0");
            }else if(!strcmp(HC05_RxStrPacket,"Left0")){//按下←，左转
                FollowModeFlag=0;
                ObstAvoModeFlag=0;
                MyCar_TurnLeft(Speed);
                strcpy(MyCar_Status[2],"Left0");
            }else if(!strcmp(HC05_RxStrPacket,"Right0")){//按下→，右转
                FollowModeFlag=0;
                ObstAvoModeFlag=0;
                MyCar_TurnRight(Speed);
                strcpy(MyCar_Status[2],"Right0");
            }else if(!strcmp(HC05_RxStrPacket,"Stop")){//按下STOP，停车，但速度不清零
                FollowModeFlag=0;
                ObstAvoModeFlag=0;
                MyCar_Stop();
                strcpy(MyCar_Status[2],"Stop");
            }else if(!strcmp(HC05_RxStrPacket,"Speed:100")){//按下SPEED:100
                Speed=100;
                strcpy(MyCar_Status[2],"Speed:100");
            }else if(!strcmp(HC05_RxStrPacket,"Speed:75")){//按下SPEED:75
                Speed=75;
                strcpy(MyCar_Status[2],"Speed:75");
            }else if(!strcmp(HC05_RxStrPacket,"Speed:0")){//按下SPEED:0
                FollowModeFlag=0;
                ObstAvoModeFlag=0;
                Speed=0;
                MyCar_Stop();
                strcpy(MyCar_Status[2],"Speed:0");
            }else if(!strcmp(HC05_RxStrPacket,"Speed++1")){//按下SPEED++
                Speed++;
                strcpy(MyCar_Status[2],"Speed++1");
            }else if(!strcmp(HC05_RxStrPacket,"Speed--1")){//按下SPEED--
                Speed--;
                strcpy(MyCar_Status[2],"Speed--1");
            }else if(!strcmp(HC05_RxStrPacket,"Follow")){//按下FOLLOW
                FollowModeFlag=1-FollowModeFlag;
                ObstAvoModeFlag=0;
                MyCar_Stop();
                FollowModeState=0;
                FollowModeTargetDirection=0;
                strcpy(MyCar_Status[2],"Follow");
            }
            
            if(Speed==101||Speed==-101)
                Speed=0;
            
            for (int i = 0; i < 3; i ++){
                MyCar_Status[3][7+i]=(Speed / main_Pow(10, 3 - i - 1) % 10 + '0');
            }
            if(ObstAvoModeFlag)
                strcpy(MyCar_Status[4],"ObAv");//前进/避障模式
            else if(FollowModeFlag)
                strcpy(MyCar_Status[4],"Folo");//跟随模式
            else
                strcpy(MyCar_Status[4],"None");//左转、右转、后退、停车、关闭跟随后无其他操作 均归为空状态/模式
            if(Speed>80)
                Thre_Dist=50;
            else
                Thre_Dist=15;
            
            for(int i=0;i<=5;++i){
                HC05_SendString(MyCar_Status[i]);
                HC05_SendString("\n");
            }
            
            HC05_RxFlag = 0;
        }
        
        /*IR*/
        if((DataFlag=IR_GetDataFlag())||IR_GetRepeatFlag()){
            Address=IR_GetAddress();
            Command=IR_GetCommand();
            
            switch(Command){//以下16进制表示按键编码，可以考虑用宏定义增加可读性
                case 0x45:Speed=100;break;//按下数字1
                case 0x46:Speed=75;break;//按下数字2
                case 0x47:Speed=50;break;//按下数字3
                case 0x44:Speed=25;break;//按下数字4
                case 0x40:FollowModeFlag=0;ObstAvoModeFlag=0;Speed=0;MyCar_Stop();break;//按下数字5，停车，且速度清零
                case 0x43:Speed=-25;break;//按下数字6
                case 0x07:Speed=-50;break;//按下数字7
                case 0x15:Speed=-75;break;//按下数字8
                case 0x09:Speed=-100;break;//按下数字9
                case 0x1C:FollowModeFlag=0;ObstAvoModeFlag=0;MyCar_Stop();break;//按下OK，停车，但速度不清零
                case 0x16:Speed++;break;//按下*，长按加速
                case 0x0D:Speed--;break;//按下#，长按减速
                case 0x19:if(DataFlag){FollowModeFlag=1-FollowModeFlag;ObstAvoModeFlag=0;MyCar_Stop();FollowModeState=0;FollowModeTargetDirection=0;}break;//按下数字0，切换跟随模式
                case 0x18:FollowModeFlag=0;ObstAvoModeFlag=1;MyCar_GoForward(Speed);break;//按下↑，前进
                case 0x52:FollowModeFlag=0;ObstAvoModeFlag=0;MyCar_GoBackward(Speed);break;//按下↓，后退
                case 0x08:FollowModeFlag=0;ObstAvoModeFlag=0;MyCar_TurnLeft(Speed);break;//按下←，左转
                case 0x5A:FollowModeFlag=0;ObstAvoModeFlag=0;MyCar_TurnRight(Speed);break;//按下→，右转
            }
            
            if(Speed==101||Speed==-101)
                Speed=0;
            if(Speed>80)
                Thre_Dist=50;
            else
                Thre_Dist=15;
        }
        
        /*HCSR04*/
        Distance1=HCSR04_GetDistance1();
        Distance2=HCSR04_GetDistance2();
        for (int i = 0; i < 3; i ++){
            MyCar_Status[0][2+i]=(Distance1 / main_Pow(10, 3 - i - 1) % 10 + '0');
            MyCar_Status[1][2+i]=(Distance2 / main_Pow(10, 3 - i - 1) % 10 + '0');
        }
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
        for (int i = 0; i < 5; i ++){
            MyCar_Status[5][6+i]=(Data.AccY / main_Pow(10, 5 - i - 1) % 10 + '0');
        }
    }
}

uint32_t main_Pow(uint32_t X, uint32_t Y){//求幂
    uint32_t Result = 1;
    while (Y --)
    {
        Result *= X;
    }
    return Result;
}
