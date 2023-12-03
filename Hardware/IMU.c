#include "stm32f10x.h"                  // Device header
#include "IMU.h"
#include "math.h"
//借用PWM中的定时器产生的中断来触发

/*
在正确读出陀螺仪，加速度计和磁力计原始数据的基础上，使用如下的代码可以实现姿态解算
如果使用的是mpu6050的话，将磁力计的传入参数置为0即可，在姿态解算函数内部会自动忽略，不会加入对磁力计的处理
*/

#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */
 
//数值越大越相信加速度计
float A_Kp = 0.4f;		/*比例增益*/
float A_Ki = 0.001f;	/*积分增益*/
float A_exInt = 0.0f;
float A_eyInt = 0.0f;
float A_ezInt = 0.0f;   /*积分误差累计*/
 
//数值越大越相信磁力计
float M_Kp = 1.0f;		/*比例增益*/
float M_Ki = 0.001f;	/*积分增益*/
float M_exInt = 0.0f;
float M_eyInt = 0.0f;
float M_ezInt = 0.0f;	/*积分误差累计*/
 
 
static float q0 = 1.0f;	/*四元数*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
static float rMat[3][3];/*旋转矩阵*/
 
 
/*快速开平方求倒*/
float invSqrt(float x)	
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
 
/*计算旋转矩阵*/
void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
 
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;
 
    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);
 
    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);
 
    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}
//dt为调用此函数的时间间隔
void imuUpdate(struct MPU6050_Data Data,float *Pitch,float *Roll,float *Yaw,float dt)	/*数据融合 互补滤波*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
//	Axis3f tempacc = acc;
//	Axis3f tempmag = {0};
//  Axis3f tempmag_2 = {0};	
	
	
	Data.GyroX = Data.GyroX * DEG2RAD;	/* 度转弧度 */
	Data.GyroY = Data.GyroY * DEG2RAD;
	Data.GyroZ = Data.GyroZ * DEG2RAD;
 
	/* 加速度计输出有效时,利用加速度计补偿陀螺仪*/
	if((Data.AccX != 0.0f) || (Data.AccY != 0.0f) || (Data.AccZ != 0.0f))
	{
		/*单位化加速计测量值*/
		normalise = invSqrt(Data.AccX * Data.AccX + Data.AccY * Data.AccY + Data.AccZ * Data.AccZ);
		Data.AccX *= normalise;
		Data.AccY *= normalise;
		Data.AccZ *= normalise;
 
		/*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
		ex = (Data.AccY * rMat[2][2] - Data.AccZ * rMat[2][1]);
		ey = (Data.AccZ * rMat[2][0] - Data.AccX * rMat[2][2]);
		ez = (Data.AccX * rMat[2][1] - Data.AccY * rMat[2][0]);
		
		/*误差累计，与积分常数相乘*/
		A_exInt += A_Ki * ex * dt ;  
		A_eyInt += A_Ki * ey * dt ;
		A_ezInt += A_Ki * ez * dt ;
		
		/*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
		Data.GyroX += A_Kp * ex + A_exInt;
		Data.GyroY += A_Kp * ey + A_eyInt;
		Data.GyroZ += A_Kp * ez + A_ezInt;
	}
	
	/* 磁力计输出有效时,利用磁力计补偿陀螺仪*/
//	if((mag.x != 0.0f) || (mag.y != 0.0f) || (mag.z != 0.0f))
//	{
//		/*单位化磁力计测量值*/
//		normalise = invSqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
//		
//		mag.x *= normalise;
//		mag.y *= normalise;
//		mag.z *= normalise;
//        /*磁力计从机体到地球*/
//		tempmag.x = rMat[0][0] * mag.x + rMat[0][1] * mag.y + rMat[0][2] * mag.z;
//		tempmag.y = rMat[1][0] * mag.x + rMat[1][1] * mag.y + rMat[1][2] * mag.z;
//		tempmag.z = rMat[2][0] * mag.x + rMat[2][1] * mag.y + rMat[2][2] * mag.z;
//		/*让导航坐标系中X轴指向正北方*/
//		tempmag.x = sqrt(tempmag.x * tempmag.x + tempmag.y * tempmag.y);
//		tempmag.y = 0;
//		/*磁力计从地球到机体*/
//        tempmag_2.x = rMat[0][0] * tempmag.x + rMat[1][0] * tempmag.y + rMat[2][0] * tempmag.z;
//        tempmag_2.y = rMat[0][1] * tempmag.x + rMat[1][1] * tempmag.y + rMat[2][1] * tempmag.z;
//        tempmag_2.z = rMat[0][2] * tempmag.x + rMat[1][2] * tempmag.y + rMat[2][2] * tempmag.z;
// 
//		
//		/*磁力计转换后的方向与磁力计方向的差值，用向量叉乘计算*/		
//		ex = (mag.y * tempmag_2.z - mag.z * tempmag_2.y);
//		ey = (mag.z * tempmag_2.x - mag.x * tempmag_2.z);
//		ez = (mag.x * tempmag_2.y - mag.y * tempmag_2.x);
//		
//		/*误差累计，与积分常数相乘*/
//		M_exInt += M_Ki * ex * dt ;  
//		M_eyInt += M_Ki * ey * dt ;
//		M_ezInt += M_Ki * ez * dt ;
//		
//		/*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
//		gyro.x += M_Kp * ex + M_exInt;
//		gyro.y += M_Kp * ey + M_eyInt;
//		gyro.z += M_Kp * ez + M_ezInt;
//	}
	
	/* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * Data.GyroX - q2Last * Data.GyroY - q3Last * Data.GyroZ) * halfT;
	q1 += ( q0Last * Data.GyroX + q2Last * Data.GyroZ - q3Last * Data.GyroY) * halfT;
	q2 += ( q0Last * Data.GyroY - q1Last * Data.GyroZ + q3Last * Data.GyroX) * halfT;
	q3 += ( q0Last * Data.GyroZ + q1Last * Data.GyroY - q2Last * Data.GyroX) * halfT;
	
	/*单位化四元数*/
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	
	imuComputeRotationMatrix();	/*计算旋转矩阵*/
	
	/*计算Pitch Roll Yaw 欧拉角*/
	*Pitch = -asinf(rMat[2][0]) * RAD2DEG; 
	*Roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	*Yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;			
}
