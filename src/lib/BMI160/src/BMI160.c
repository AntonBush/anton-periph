
#include "BMI160_SPI.h"
#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include "main.h"

#define PI                  3.1415926535f
#define RAD_TO_DEG          57.2957795f
#define DEG_TO_RAD          0.01745329f
#define ACC_SCALE           0.00059855f
#define GYRO_SCALE          0.0609756f
#define ALPHA 0.98f
#define DT 0.000625f

void BMI_SPI(SPI_HandleTypeDef *SPIhnd, BMI160_Param *Data)
{
	//Copy I2C CubeMX handle to local library
	Data->SPI = SPIhnd;
}

void spiTR_DUS(uint8_t *tData, uint8_t tn, uint8_t rn, uint8_t *rDest, BMI160_Param *Data)
{
//	uint8_t dummy;
	HAL_GPIO_WritePin(DUS_CS_GPIO_Port, DUS_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(Data->SPI, tData, tn, 100);
	if (rn > 0) {
//		HAL_SPI_Receive(&hspi1, &dummy, 1, 100);
		HAL_SPI_Receive(Data->SPI, rDest, rn, 100);
	}
	HAL_GPIO_WritePin(DUS_CS_GPIO_Port, DUS_CS_Pin, GPIO_PIN_SET);
}

void init_BMI(BMI160_Param *Data)
{
	init_quat_angles(&Data->angles);
	HAL_GPIO_WritePin(DUS_CS_GPIO_Port, DUS_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(DUS_CS_GPIO_Port, DUS_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(DUS_CS_GPIO_Port, DUS_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	Data->tData[0] = 0x00 | 0x80;
	spiTR_DUS(Data->tData, 1, 1, &Data->rData[0], Data);
	while(Data->rData[0] != 0xD1){
		HAL_GPIO_WritePin(DUS_CS_GPIO_Port, DUS_CS_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(DUS_CS_GPIO_Port, DUS_CS_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(DUS_CS_GPIO_Port, DUS_CS_Pin, GPIO_PIN_SET);
		spiTR_DUS(Data->tData, 1, 1, &Data->rData[0], Data);
	}
	HAL_Delay(10);
	//  data[0] = 0x7E | (1<<7);
	//  spiTR_DUS(data, 1, 1);  // change to spi mode
	Data->tData[0] = 0x7E; Data->tData[1] = 0x11;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data); // Accel Normal
	HAL_Delay(5);
	Data->tData[0] = 0x7E; Data->tData[1] = 0x15;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data); // Gyro Normal
	HAL_Delay(80);

	// 2. Настраиваем ЧАСТОТУ (ODR) на максимум
	Data->tData[0] = 0x40; Data->tData[1] = 0x2C;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data); // Accel 1600Hz
	Data->tData[0] = 0x42; Data->tData[1] = 0x2C;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data); // Gyro 3200Hz

	// 3. Настраиваем ДИАПАЗОН (Range)
	Data->tData[0] = 0x41; Data->tData[1] = 0x03;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data); // Accel ±2g
	Data->tData[0] = 0x43; Data->tData[1] = 0x03;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data); // Gyro ±2000dps

	BMI_get_raw(Data);
}

void BMI_get_raw(BMI160_Param *Data)
{
	  Data->tData[0] = 0x0C | 0x80;
	  spiTR_DUS(Data->tData, 1, 16, Data->rData, Data);
	  Data->gx = ((Data->rData[1] << 8)  | Data->rData[0]) ;
	  Data->gy = ((Data->rData[3] << 8)  | Data->rData[2]) ;
	  Data->gz = ((Data->rData[5] << 8)  | Data->rData[4]) ;

	  Data->ax = ((Data->rData[7] << 8)  | Data->rData[6]) ;
	  Data->ay = ((Data->rData[9] << 8)  | Data->rData[8]) ;
	  Data->az = ((Data->rData[11] << 8) | Data->rData[10]);
}

void Angle_count(BMI160_Param *Data)
{
	Data->kren_id = -atan2(Data->ay_g,Data->az_g)*57.32484;
	Data->tangaj_id= atan2(Data->ax_g,sqrt(Data->az_g*Data->az_g+Data->ay_g*Data->ay_g))*57.32484;
}

void DataSheet_offset(BMI160_Param *Data)
{

	Data->tData[0] = 0x77; Data->tData[1] = 0xC0;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data);
	Data->tData[0] = 0x69| 0x80;
	spiTR_DUS(Data->tData, 1, 1, &Data->rData[0], Data);
//data_t = 0x7D; датчик чипом вверх
//data_t = 0x7E; датчик чипом вниз
	Data->tData[0] = 0x69; Data->tData[1] = 0x7D;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data);
	Data->tData[0] = 0x69| 0x80;
	spiTR_DUS(Data->tData, 1, 1, &Data->rData[0], Data);
	HAL_Delay(5);
	Data->tData[0] = 0x71| 0x80;
	spiTR_DUS(Data->tData, 1, 1, &Data->rData[0], Data);
	Data->tData[0] = 0x7E; Data->tData[1] = 0x03;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data);
	Data->tData[0] = 0x1B| 0x80;
	spiTR_DUS(Data->tData, 1, 1, &Data->rData[0], Data);
	HAL_Delay(250);
	spiTR_DUS(Data->tData, 1, 1, &Data->rData[0], Data);
	Data->rData[1] = Data->rData[0]&~1<<3;
	spiTR_DUS(Data->tData, 1, 1, &Data->rData[0], Data);
	Data->tData[0] = 0x71| 0x80;
	spiTR_DUS(Data->tData, 1, 8, Data->off, Data);
	BMI_get_raw(Data);

}

void set_EXTI(BMI160_Param *Data)
{
	Data->tData[0] = 0x47; Data->tData[1] = 0xC0;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data);
	HAL_Delay(1);
	Data->tData[0] = 0x46; Data->tData[1] = 0x03;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data);
	HAL_Delay(1);
	Data->tData[0] = 0x51; Data->tData[1] = 0x10;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data);
	HAL_Delay(1);
	Data->tData[0] = 0x53; Data->tData[1] = 0x0A;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data);
	HAL_Delay(1);
	Data->tData[0] = 0x56; Data->tData[1] = 0x80;
	spiTR_DUS(Data->tData, 2, 0, NULL, Data);
}

void A2_count(BMI160_Param *Data)
{
	float gx_deg = (float)Data->gx / 16.4f;
	float gy_deg = (float)Data->gy / 16.4f;

	Data->accel_roll = atan2f((float)Data->ay, (float)Data->az) * RAD_TO_DEG;
	Data->accel_pitch = atan2f(-(float)Data->ax, sqrtf((float)Data->ay * (float)Data->ay + (float)Data->az * (float)Data->az)) * RAD_TO_DEG;

	Data->roll  += ALPHA * (gx_deg * DT) + (1.0f - ALPHA) * Data->accel_roll;
	Data->pitch += ALPHA * (gy_deg * DT) + (1.0f - ALPHA) * Data->accel_pitch;
}

void Quat_Angles_count(BMI160_Param *Data)
{
	Data->wx = Data->gx/ 131.072f;
	Data->wy = Data->gy/ 131.072f;
	Data->wz = Data->gz/ 131.072f;
	Data->angles.omega_1.x = Data->wx * DEG_TO_RAD;
	Data->angles.omega_1.y = Data->wy * DEG_TO_RAD;
	Data->angles.omega_1.z = Data->wz * DEG_TO_RAD;
	Data->angles.a.x = Data->ax_g/9.80665f;
	Data->angles.a.y = Data->ay_g/9.80665f;
	Data->angles.a.z = Data->az_g/9.80665f;

	process_quat_angles(&Data->angles, DT); // сделать расчет времени
	Data->roll = Data->angles.roll * RAD_TO_DEG;
	Data->pitch = Data->angles.pitch * RAD_TO_DEG;
	Data->yaw = Data->angles.yaw * RAD_TO_DEG;
}
