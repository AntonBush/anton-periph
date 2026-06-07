
#pragma once

#include <stdint.h>

#include "BMI160/BMI160_datasheet.h"

typedef uint8_t BMI160_Size;

enum BMI160_Status {
	BMI160_STATUS_OK,
	BMI160_STATUS_ERROR,
	BMI160_STATUS_BUSY,
	BMI160_STATUS_TIMEOUT
};

typedef void (*BMI160_WaitFunction)(uint32_t ms);
typedef enum BMI160_Status (*BMI160_TransmitReceiveFunction)(
	const uint8_t *tdata, uint8_t *rdata, BMI160_Size count);

struct BMI160_Handle {
	struct BMI160_HandleInit {
		BMI160_WaitFunction waitf;
		BMI160_TransmitReceiveFunction
			write_readf, async_write_readf;
	} init;
	struct BMI160_Data {
		float acc[3], gyro[3];
	} data;
	uint8_t tx[20], rx[20];
	;//?
};

enum BMI160_Status BMI160_Handle_init(struct BMI160_Handle *h);

//?
void BMI160_process_raw(void);
