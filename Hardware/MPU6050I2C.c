#include "stm32f10x.h"                  // Device header
#include "MPU6050I2C.h"
#include "Delay.h"

void MPU6050I2C_W_SCL(uint8_t BitValue){
	GPIO_WriteBit(GPIOB,GPIO_Pin_5,(BitAction)BitValue);
	//该句方便移植（至其他类型MCU/其他引脚）
	//Delay_us(10);
	//可以在此处增加延迟，使得外设更上MCU的引脚反转速度
}

void MPU6050I2C_W_SDA(uint8_t BitValue){
	GPIO_WriteBit(GPIOB,GPIO_Pin_14,(BitAction)BitValue);
	//该句方便移植（至其他类型MCU/其他引脚）
	//Delay_us(10);
	//可以在此处增加延迟，使得外设更上MCU的引脚反转速度
}

uint8_t MPU6050I2C_R_SDA(){
	uint8_t BitValue;
	BitValue=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
	//开漏输出模式也可以直接读输入寄存器
	//Delay_us(10);
	//可以在此处增加延迟
	return BitValue;
}

void MPU6050I2C_Init(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_5 | GPIO_Pin_14);
}

void MPU6050I2C_Start(){
	MPU6050I2C_W_SDA(1);//确保调用start时SDA为1
	
	MPU6050I2C_W_SCL(1);
	MPU6050I2C_W_SDA(0);
	
	MPU6050I2C_W_SCL(0);//保证0收尾
}

void MPU6050I2C_Stop(){
	MPU6050I2C_W_SDA(0);//确保调用start时SDA为0
	
	MPU6050I2C_W_SCL(1);
	MPU6050I2C_W_SDA(1);
}

void MPU6050I2C_SendByte(uint8_t Byte){
	uint8_t i;
	for(i=0;i<8;i++){
		MPU6050I2C_W_SDA(Byte&(0x80>>i));
		MPU6050I2C_W_SCL(1);
		//封装的函数中的Delay在此处起作用
		MPU6050I2C_W_SCL(0);//保证0收尾
	}
}

uint8_t MPU6050I2C_ReceiveByte(){
	uint8_t i,Byte=0x00;
	
	MPU6050I2C_W_SDA(1);//释放总线，高阻状态，从机拉低有效
	
	for(i=0;i<8;i++){
		MPU6050I2C_W_SCL(1);
		if(MPU6050I2C_R_SDA())
			Byte|=(0x80>>i);
		MPU6050I2C_W_SCL(0);//保证0收尾
	}
	
	return Byte;
}

void MPU6050I2C_SendACK(uint8_t ACKBit){
	MPU6050I2C_W_SDA(ACKBit);
	MPU6050I2C_W_SCL(1);
	//封装的函数中的Delay在此处起作用
	MPU6050I2C_W_SCL(0);//保证0收尾
}

uint8_t MPU6050I2C_ReceiveACK(){
	uint8_t ACKBit;
	
	MPU6050I2C_W_SDA(1);//释放总线，高阻状态，从机拉低有效
	
	MPU6050I2C_W_SCL(1);
	ACKBit=MPU6050I2C_R_SDA();//可以不用if
	MPU6050I2C_W_SCL(0);//保证0收尾
	
	return ACKBit;
}
