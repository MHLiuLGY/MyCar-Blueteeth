#include "stm32f10x.h"                  // Device header
#include "KLM.h"

/*卡尔曼滤波数值配置*/
float P=1;
float P_;  //对应公式中的p'
float X=0;
float X_;  //X'
float K=0;
float Q=0.01;//噪声
float R=0.2;  //R如果很大，更相信预测值，那么传感器反应就会迟钝，反之相反
//float R=0.5;

/*卡尔曼函数*/
uint16_t KLM(uint16_t Z){
  X_=X+Q;
  P_=P+Q;
  K=P_/(P_+R);
  X=X_+K*(Z-X_);
  P=P_-K*P_;
  return X;
}