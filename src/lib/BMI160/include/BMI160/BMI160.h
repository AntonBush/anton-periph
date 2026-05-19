
#ifndef INC_BMI160_SPI_H_
#define INC_BMI160_SPI_H_

#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"

#include <quat_angles.h>


/* --- Адреса устройства при подключении SAO --- */
#define BMI160_ADR_GND 0x68
#define BMI160_ADR_VDD 0x69

/* --- Основные регистры идентификации и статуса --- */
#define BMI160_REG_CHIP_ID          0x00  // Должен вернуть 0xD1
#define BMI160_REG_ERR_REG          0x02  // Ошибки системы
#define BMI160_REG_PMU_STATUS       0x03  // Состояние питания (Acc, Gyro, Mag)
#define BMI160_REG_STATUS           0x1B  // Статус готовности данных

/* --- Данные акселерометра (X, Y, Z - 6 байт) --- */
#define BMI160_REG_ACCEL_DATA       0x12  // Начало данных (LSB X)
#define BMI160_REG_ACCEL_X_LSB      0x12
#define BMI160_REG_ACCEL_X_MSB      0x13
#define BMI160_REG_ACCEL_Y_LSB      0x14
#define BMI160_REG_ACCEL_Y_MSB      0x15
#define BMI160_REG_ACCEL_Z_LSB      0x16
#define BMI160_REG_ACCEL_Z_MSB      0x17

/* --- Данные гироскопа (X, Y, Z - 6 байт) --- */
#define BMI160_REG_GYRO_DATA        0x0C  // Начало данных (LSB X)
#define BMI160_REG_GYRO_X_LSB       0x0C
#define BMI160_REG_GYRO_X_MSB       0x0D
#define BMI160_REG_GYRO_Y_LSB       0x0E
#define BMI160_REG_GYRO_Y_MSB       0x0F
#define BMI160_REG_GYRO_Z_LSB       0x10
#define BMI160_REG_GYRO_Z_MSB       0x11

/* --- Регистрация времени и температуры --- */
#define BMI160_REG_SENSORTIME_0     0x18  // Таймштамп (3 байта)
#define BMI160_REG_TEMPERATURE_0    0x20  // Температура (2 байта)

/* --- Настройки датчиков (Конфигурация) --- */
#define BMI160_REG_ACCEL_CONF       0x40  // ODR и фильтрация акселерометра
#define BMI160_REG_ACCEL_RANGE      0x41  // Диапазон: 2g, 4g, 8g, 16g
#define BMI160_REG_GYRO_CONF        0x42  // ODR и фильтрация гироскопа
#define BMI160_REG_GYRO_RANGE       0x43  // Диапазон: 125, 250, 500, 1000, 2000 dps

/* --- Работа с FIFO --- */
#define BMI160_REG_FIFO_CONFIG_0    0x46  // Настройка порогов FIFO
#define BMI160_REG_FIFO_CONFIG_1    0x47  // Что именно писать в FIFO
#define BMI160_REG_FIFO_LENGTH_0    0x22  // Текущий размер данных в FIFO
#define BMI160_REG_FIFO_DATA        0x24  // Порт чтения данных из FIFO

/* --- Прерывания и управление --- */
#define BMI160_REG_INT_ENABLE_0     0x50  // Включение прерываний
#define BMI160_REG_INT_OUT_CTRL     0x53  // Настройка пинов INT1/INT2
#define BMI160_REG_INT_MAP_0        0x55  // Привязка прерываний к пинам
#define BMI160_REG_COMMAND          0x7E  // Командный регистр (Softreset, Power Mode)

/* --- Маски для SPI --- */
#define BMI160_SPI_READ_BIT         0x80
#define BMI160_DUMMY_BYTE           0x7F  // Регистр для "пробуждения" SPI

typedef struct {
	SPI_HandleTypeDef *SPI;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
//	int16_t ax_offset, ay_offset, az_offset;
//	int16_t gx_offset, gy_offset, gz_offset;
//	int16_t mean_ax, mean_ay, mean_az;
//	int16_t mean_gx, mean_gy, mean_gz;
	bool ready, inited,err, dma_spi_done;
	uint8_t acel_deadzone, gyro_deadzone;
	uint8_t status, error;
	uint8_t rData[14],tData[2];
	uint8_t off[8];
	double tangaj_id, kren_id; //по данным линейных ускорений

	float ax_g, ay_g, az_g;
    float ax1_offset, ay1_offset;
    float roll, pitch, yaw,accel_roll, accel_pitch;
    float vx, vy;
    float vx_cm, vy_cm;
    QuatAngles angles;
    float wx,wy,wz;
} BMI160_Param;

void BMI_SPI(SPI_HandleTypeDef *SPIhnd, BMI160_Param *data);

void spiTR_DUS(uint8_t *tData, uint8_t tn, uint8_t rn, uint8_t *rDest, BMI160_Param *Data);

void init_BMI(BMI160_Param *data);

void BMI_get_raw(BMI160_Param *data);

void Angle_count(BMI160_Param *data);

void DataSheet_offset(BMI160_Param *data);

void set_EXTI(BMI160_Param *data);

void A2_count(BMI160_Param *Data);

void Quat_Angles_count(BMI160_Param *Data);

#endif /* INC_BMI160_SPI_H_ */
