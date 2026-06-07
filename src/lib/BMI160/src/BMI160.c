#include <stddef.h>
#include <math.h>

#include "BMI160/BMI160.h"
#include "BMI160/BMI160_datasheet.h"

#define BMI160_RETURN_IF_NOT_OK(status) \
{enum BMI160_Status s = (status); \
	if (s != BMI160_STATUS_OK) return s;}

static const float BMI160_STD_G_CONSTF = 9.80665;

static const float DEG_TO_RADF = M_PI / 180;
static const float RAD_TO_DEGF = 180 / M_PI;

static enum BMI160_Status BMI160_write_read(
	struct BMI160_Handle *h, BMI160_Size count)
{
	return h->init.write_readf(h->tx, h->rx, count);
}

static void BMI160_wait(struct BMI160_Handle *h, int ms)
{
	h->init.waitf(ms);
}

static enum BMI160_Status BMI160_read(
	struct BMI160_Handle *h, uint8_t reg, uint8_t count)
{
	h->tx[0] = BMI160_read_command(reg);
	return BMI160_write_read(h, 1 + count);
}

static enum BMI160_Status BMI160_write(
	struct BMI160_Handle *h, uint8_t reg, uint8_t count)
{
	h->tx[0] = BMI160_write_command(reg);
	return BMI160_write_read(h, 1 + count);
}

static enum BMI160_Status BMI160_get_reg(
	struct BMI160_Handle *h, uint8_t reg, uint8_t *val_ptr)
{
	BMI160_RETURN_IF_NOT_OK(BMI160_read(h, reg, 1));
	if (val_ptr != NULL) {
		*val_ptr = h->rx[1];
	}
	return BMI160_STATUS_OK;
}

static enum BMI160_Status BMI160_set_reg(
	struct BMI160_Handle *h, uint8_t reg, uint8_t val)
{
	h->tx[1] = val;
	return BMI160_write(h, reg, 1);
}

static enum BMI160_Status BMI160_check_id(
	struct BMI160_Handle *h)
{
	uint8_t id = 0, attempts = 0;
	do {
		if (attempts > 0) {
			BMI160_wait(h, 20 * attempts);
		}
		BMI160_RETURN_IF_NOT_OK(BMI160_get_reg(
			h, BMI160_REG_CHIP_ID, &id));
	} while (id != BMI160_REG_DEFAULT_CHIP_ID && attempts < 3);
	if (id != BMI160_REG_DEFAULT_CHIP_ID) {
		return BMI160_STATUS_ERROR;
	}
	return BMI160_STATUS_OK;
}

enum BMI160_Status BMI160_Handle_init(struct BMI160_Handle *h)
{
	BMI160_RETURN_IF_NOT_OK(BMI160_get_reg(
		h, BMI160_REG_DUMMY, NULL));
	BMI160_wait(h, 20);
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_CMD, BMI160_REG_CMD_SOFTRESET));
	BMI160_wait(h, 200);
	BMI160_RETURN_IF_NOT_OK(BMI160_get_reg(
		h, BMI160_REG_DUMMY, NULL));
	BMI160_wait(h, 20);

	BMI160_RETURN_IF_NOT_OK(BMI160_check_id(h));

	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_CMD, BMI160_REG_CMD_ACC_NORMAL));
	BMI160_wait(h, 20);
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_CMD, BMI160_REG_CMD_GYR_NORMAL));
	BMI160_wait(h, 200);

	const uint8_t acc_conf = BMI160_REG_ACC_CONF_BWP_NORMAL
		| BMI160_REG_ACC_CONF_ODR_1600_HZ;
	const uint8_t gyr_conf = BMI160_REG_GYR_CONF_BWP_NORMAL
		| BMI160_REG_GYR_CONF_ODR_1600_HZ;
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_ACC_CONF, acc_conf));
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_GYR_CONF, gyr_conf));

	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_ACC_RANGE, BMI160_REG_ACC_RANGE_2G));
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_GYR_RANGE,
			BMI160_REG_GYR_RANGE_250_DEG_PER_SEC));

	h->resol.acc = BMI160_ACC_RESOL_LSB_PER_G_RANGE_2G;
	h->resol.gyr = BMI160_GYR_RESOL_dLSB_PER_DPS_RANGE_250_DPS;

	return BMI160_STATUS_OK;
}

void BMI160_read_data(struct BMI160_Handle *h)
{
	BMI160_read(h, BMI160_REG_DATA_8, 15);
	for (int i = 0; i < 3; ++i) {
		int i1 = i + 1;
		h->data.raw.gyr[i] = (h->rx[i1*2  ] << 8) | h->rx[i*2+1  ];
		h->data.raw.acc[i] = (h->rx[i1*2+6] << 8) | h->rx[i*2+1+6];
		h->data.gyr_deg[i] = h->data.raw.gyr[i] / ((float)h->resol.gyr / 10);
		h->data.acc_g  [i] = h->data.raw.acc[i] / ((float)h->resol.acc /  1);
		h->data.gyr_rad[i] = h->data.gyr_deg[i] * DEG_TO_RADF;
		h->data.acc_mss[i] = h->data.acc_g  [i] * BMI160_STD_G_CONSTF;
	}
}


void BMI160_DataSheet_offset(struct BMI160_Handle *h)
{
	const uint8_t reg_offset_6 = BMI160_REG_OFFSET_6_GYR_OFF_EN
		| BMI160_REG_OFFSET_6_ACC_OFF_EN;
	BMI160_set_reg(h, BMI160_REG_OFFSET_6, reg_offset_6);
	BMI160_get_reg(h, BMI160_REG_FOC_CONF, NULL);
//data_t = 0x7D; датчик чипом вверх
//data_t = 0x7E; датчик чипом вниз
	BMI160_set_reg(h, BMI160_REG_FOC_CONF, 0x7D);
	BMI160_get_reg(h, BMI160_REG_FOC_CONF, NULL);
	BMI160_wait(h, 5);
	BMI160_get_reg(h, BMI160_REG_OFFSET_0, NULL);
	BMI160_set_reg(h, BMI160_REG_CMD, BMI160_REG_CMD_START_FOC);
	BMI160_get_reg(h, BMI160_REG_STATUS, NULL);
	BMI160_wait(h, 250);
	BMI160_get_reg(h, BMI160_REG_STATUS, NULL);
	h->rx[2] = h->rx[1] & ~1 << 3;
	BMI160_get_reg(h, BMI160_REG_STATUS, NULL);
	BMI160_read(h, BMI160_REG_OFFSET_0, 8);
	BMI160_read_data(h);
}

void BMI160_set_EXTI(struct BMI160_Handle *h)
{
	BMI160_set_reg(h, BMI160_REG_FIFO_CONFIG_1, 0xC0);
	BMI160_wait(h, 1);
	BMI160_set_reg(h, BMI160_REG_FIFO_CONFIG_0, 0x03);
	BMI160_wait(h, 1);
	BMI160_set_reg(h, BMI160_REG_INT_EN_1, 0x10);
	BMI160_wait(h, 1);
	BMI160_set_reg(h, BMI160_REG_INT_OUT_CTRL, 0x0A);
	BMI160_wait(h, 1);
	BMI160_set_reg(h, BMI160_REG_INT_MAP_1, 0x80);
}
