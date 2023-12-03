#include "stm32f10x.h"                  // Device header
#include "OLEDI2C.h"
#include "Delay.h"

//写也可以用宏定义，类似下面
/*引脚配置
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))
*/

void OLEDI2C_W_SCL(uint8_t BitValue){
	GPIO_WriteBit(GPIOA,GPIO_Pin_11,(BitAction)BitValue);
	//该句方便移植（至其他类型MCU/其他引脚）
}

void OLEDI2C_W_SDA(uint8_t BitValue){
	GPIO_WriteBit(GPIOA,GPIO_Pin_12,(BitAction)BitValue);
	//该句方便移植（至其他类型MCU/其他引脚）
}

void OLEDI2C_Init(){
	/*RCC*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	/*GPIO*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_11 | GPIO_Pin_12);
}

void OLEDI2C_Start(){
	OLEDI2C_W_SDA(1);//确保调用start时SDA为1
	
	OLEDI2C_W_SCL(1);
	OLEDI2C_W_SDA(0);
	
	OLEDI2C_W_SCL(0);//保证0收尾
}

void OLEDI2C_Stop(){
	OLEDI2C_W_SDA(0);//确保调用start时SDA为0
	
	OLEDI2C_W_SCL(1);
	OLEDI2C_W_SDA(1);
}

void OLEDI2C_SendByte(uint8_t Byte){
	uint8_t i;
	for(i=0;i<8;i++){
		OLEDI2C_W_SDA(Byte&(0x80>>i));
		OLEDI2C_W_SCL(1);
		//封装的函数中的Delay在此处起作用
		OLEDI2C_W_SCL(0);//保证0收尾
	}
	OLEDI2C_W_SCL(1);	//额外的一个时钟，不处理应答信号
	OLEDI2C_W_SCL(0);
}
