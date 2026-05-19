/*
 * BMP280_SPI.c
 *
 *  Created on: May 9, 2026
 *      Author: Wikto
 */
#include "BMP280_SPI.h"
#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include "main.h"


void BMP_SPI(SPI_HandleTypeDef *SPIhnd, BMP280_Param *Data)
{
	//Copy I2C CubeMX handle to local library
	Data->SPI = SPIhnd;
}

void spiTR_Hbar(uint8_t *tData, uint8_t tn, uint8_t rn, uint8_t *rDest, BMP280_Param *Data)
{
	HAL_GPIO_WritePin(Hbar_CS_GPIO_Port, Hbar_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(Data->SPI, tData, tn, 100);
	if (rn > 0) {
		HAL_SPI_Receive(Data->SPI, rDest, rn, 100);
	}
	HAL_GPIO_WritePin(Hbar_CS_GPIO_Port, Hbar_CS_Pin, GPIO_PIN_SET);
}

void Chec_BMP_answer(BMP280_Param *Data)
{
	Data->tData[0] = 0xD0 | 0x80;
	spiTR_Hbar(Data->tData, 1, 1, &Data->rData[0], Data);
	if (Data->rData[0]== 0x58) {
		Data->connected = 1;
	}else{
		Data->connected = 0;
	}
}

void set_BMP_Init(BMP280_Param *Data)
{
	// 1. Сначала переводим в Sleep, чтобы настройки применились корректно
	Data->tData[0] = 0xF4 & ~0x80; Data->tData[1] = 0x00;
	spiTR_Hbar(Data->tData, 2, 0, NULL, Data);
	// 2. Настраиваем фильтр (0x08 = x4 фильтр, 0.5ms ожидание; 0x00 = нет фильтра)
	Data->tData[0] = 0xF5 & ~0x80; Data->tData[1] = 0x08;
	spiTR_Hbar(Data->tData, 2, 0, NULL, Data);
	// 3. Включаем рабочий режим (100 Гц достижимо при: T x1, P x2, Normal Mode)
	Data->tData[0] = 0xF4 & ~0x80; Data->tData[1] = 0x2B;
	spiTR_Hbar(Data->tData, 2, 0, NULL, Data);
}

void Read_calib_param(BMP280_Param *Data)
{
	//	uint8_t Data->rData[24] = {0};
	//	HAL_I2C_Mem_Read(&Data->i2cHandle, (BMP280_ADR <<1) ,0x88, I2C_MEMADD_SIZE_8BIT, Data->rData, 24,100);
	Data->tData[0] = 0x88 | 0x80;
	spiTR_Hbar(Data->tData, 1, 24, Data->rData, Data);
	Data->dig_T1 =((Data->rData[1] << 8) | Data->rData[0]);
	Data->dig_T2 =((Data->rData[3] << 8) | Data->rData[2]);
	Data->dig_T3 =((Data->rData[5] << 8) | Data->rData[4]);
	Data->dig_P1 =((Data->rData[7] << 8) | Data->rData[6]);
	Data->dig_P2 =((Data->rData[9] << 8) | Data->rData[8]);
	Data->dig_P3 =((Data->rData[11] << 8) | Data->rData[10]);
	Data->dig_P4 =((Data->rData[13] << 8) | Data->rData[12]);
	Data->dig_P5 =((Data->rData[15] << 8) | Data->rData[14]);
	Data->dig_P6 =((Data->rData[17] << 8) | Data->rData[16]);
	Data->dig_P7 =((Data->rData[19] << 8) | Data->rData[18]);
	Data->dig_P8 =((Data->rData[21] << 8) | Data->rData[20]);
	Data->dig_P9 =((Data->rData[23] << 8) | Data->rData[22]);
}

void Read_temp(BMP280_Param *Data)
{
	Data->tData[0] = 0xFA | 0x80;
	spiTR_Hbar(Data->tData, 1, 3, Data->rData, Data);

	Data->adc_T = (Data->rData[0] << 12) | (Data->rData[1] << 4) | (Data->rData[2] >> 4);
	int32_t var1, var2;
	var1 = ((((Data->adc_T >> 3) - (Data->dig_T1 << 1))) * (Data->dig_T2)) >> 11;
	var2 = (((((Data->adc_T >> 4) - (Data->dig_T1)) * ((Data->adc_T >> 4) - (Data->dig_T1))) >> 12) * (Data->dig_T3)) >> 14;
	Data->t_fine = var1 + var2;
	Data->Temp = ((Data->t_fine * 5 + 128) >> 8) / 100.0f;
}

void Read_Press(BMP280_Param *Data)
{
	Data->tData[0] = 0xF7 | 0x80;
	spiTR_Hbar(Data->tData, 1, 3, Data->rData, Data);

	int32_t adc_P = (int32_t)(((uint32_t)Data->rData[0] << 12) | ((uint32_t)Data->rData[1] << 4) | ((uint32_t)Data->rData[2] >> 4));

    int64_t var1, var2, p;

    var1 = ((int64_t)Data->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)Data->dig_P6;
    var2 = var2 + ((var1 * (int64_t)Data->dig_P5) << 17);
    var2 = var2 + (((int64_t)Data->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)Data->dig_P3) >> 8) + ((var1 * (int64_t)Data->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)Data->dig_P1) >> 33;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)Data->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)Data->dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((Data->dig_P7) << 4);

    Data->PressPa = (float)p / 256.0f; // в Паскалях
    Data->PressMMRTST = ((float)p / 256.0f)*0.00750062f; // в мм рт. ст.
}

void Read_H_otnSea(BMP280_Param *Data)
{
	float pressureAtSeaLevel = 101325.0f;
	Data->H_otnSea = 44330.0f * (1.0f - powf(Data->PressPa / pressureAtSeaLevel, 0.1902949f));
}

static void BMP280_PiF_process(struct BMP280_PiF *pif, float dt)
{
	pif->err = pif->in - pif->out;
	pif->p  = pif->K_p * pif->err;
	pif->i += pif->K_i * pif->err * dt;
	pif->vy = pif->p + pif->i;
	pif->out += pif->vy * dt;
}

void Count_Vy(BMP280_Param *Data)
{
	Data->Htek = (Data->H_otnSea - Data->H_strt) * 100;
	Data->pif[0].in = Data->Htek;
	for (int i = 0; i < 3; ++i) {
		struct BMP280_PiF *pif = Data->pif + i;
		pif->K_p = 2;
		pif->K_i = 16;
		BMP280_PiF_process(pif, 1./50.);
		if (i < 2) pif[1].in = pif->out;
	}
}

void Read_all(BMP280_Param *Data)
{
	Chec_BMP_answer(&*Data);
	Read_temp(&*Data);
	Read_Press(&*Data);
	Read_H_otnSea(&*Data);
	Count_Vy(&*Data);
}
