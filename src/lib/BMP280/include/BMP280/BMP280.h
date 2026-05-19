/*
 * BMP280_SPI.h
 *
 *  Created on: May 9, 2026
 *      Author: Wikto
 */

#ifndef CODE_VIKTORS_INC_BMP280_SPI_H_
#define CODE_VIKTORS_INC_BMP280_SPI_H_


#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"

typedef struct {
	SPI_HandleTypeDef *SPI;
	uint8_t rData[26],tData[2];
	bool ready, inited, err, connected;
	uint16_t dig_T1, dig_P1;
	int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	int32_t  t_fine;
	int32_t adc_T;
	float Temp, PressPa, PressMMRTST,H_otnSea, H_strt;
	float Ierr1,H_pi1,Vy1,Htek;
	float Ierr2,H_pi2,Vy2;
	float Ierr3,H_pi3,Vy3;
	struct BMP280_PiF {
		float K_p, K_i;
		float in, err, p, i, vy, out;
	} pif[3];
} BMP280_Param;

void BMP_SPI(SPI_HandleTypeDef *SPIhnd, BMP280_Param *data);

void spiTR_Hbar(uint8_t *tData, uint8_t tn, uint8_t rn, uint8_t *rDest, BMP280_Param *data);

void Chec_BMP_answer(BMP280_Param *data);

void Read_calib_param(BMP280_Param *data);

void set_BMP_Init(BMP280_Param *data);

void Read_temp(BMP280_Param *data);

void Read_Press(BMP280_Param *data);

void Read_H_otnSea(BMP280_Param *data);

void Count_Vy(BMP280_Param *data);

void Read_all(BMP280_Param *data);

#endif /* CODE_VIKTORS_INC_BMP280_SPI_H_ */
