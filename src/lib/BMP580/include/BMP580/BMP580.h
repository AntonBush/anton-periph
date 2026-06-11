
#pragma once

#include <stdint.h>

#include "BMP580/BMP580_datasheet.h"

typedef uint8_t BMP580_Size;

enum BMP580_Status {
	BMP580_STATUS_OK,
	BMP580_STATUS_ERROR,
	BMP580_STATUS_BUSY,
	BMP580_STATUS_TIMEOUT
};

typedef void (*BMP580_WaitFunction)(uint32_t ms);
typedef enum BMP580_Status (*BMP580_TransmitReceiveFunction)(
	const uint8_t *tdata, uint8_t *rdata, BMP580_Size count);

struct BMP580_Handle {
	struct BMP580_HandleInit {
		BMP580_WaitFunction waitf;
		BMP580_TransmitReceiveFunction
			write_readf, async_write_readf;
	} init;
	struct BMP580_Data {
		struct BMP580_RawData {
			int32_t temp, press;
		} raw;
		float temp, press;
	} data;
	uint8_t tx[20], rx[20];
	// struct BMP580_Resolution {
	// 	enum BMP580_AccResolution acc;
	// 	enum BMP580_GyrResolution gyr;
	// } resol;
};

enum BMP580_Status BMP580_Handle_init(struct BMP580_Handle *h);
void BMP580_read_data(struct BMP580_Handle *h);

void BMP580_async_read_data(struct BMP580_Handle *h);
void BMP580_process_data(struct BMP580_Handle *h);
