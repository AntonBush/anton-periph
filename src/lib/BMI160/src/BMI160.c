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

static enum BMI160_Status BMI160_activate_spi(
	struct BMI160_Handle *h)
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
	return BMI160_STATUS_OK;
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
		++attempts;
	} while (id != BMI160_REG_DEFAULT_CHIP_ID && attempts < 4);
	if (id != BMI160_REG_DEFAULT_CHIP_ID) {
		return BMI160_STATUS_ERROR;
	}
	return BMI160_STATUS_OK;
}

static enum BMI160_Status BMI160_setup_normal_mode(
	struct BMI160_Handle *h)
{
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_CMD, BMI160_REG_CMD_ACC_NORMAL));
	BMI160_wait(h, 20);
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_CMD, BMI160_REG_CMD_GYR_NORMAL));
	BMI160_wait(h, 200);
	return BMI160_STATUS_OK;
}

static enum BMI160_Status BMI160_configure(
	struct BMI160_Handle *h)
{
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

static enum BMI160_Status BMI160_fast_offset_compensation(
	struct BMI160_Handle *h)
{
	const uint8_t reg_offset_6 = BMI160_REG_OFFSET_6_GYR_OFF_EN
		| BMI160_REG_OFFSET_6_ACC_OFF_EN;
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_OFFSET_6, reg_offset_6));
	const uint8_t reg_foc_conf = BMI160_REG_FOC_CONF_GYR_EN
		| BMI160_REG_FOC_CONF_ACC_X_0G
		| BMI160_REG_FOC_CONF_ACC_Y_0G
		| BMI160_REG_FOC_CONF_ACC_Z_P1G;
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_FOC_CONF, reg_foc_conf));
	BMI160_wait(h, 5);
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_CMD, BMI160_REG_CMD_START_FOC));
	BMI160_wait(h, 250);

	uint8_t status = 0, foc_rdy = 0, attempts = 0;
	do {
		if (attempts > 0) {
			BMI160_wait(h, 250 * attempts);
		}
		BMI160_RETURN_IF_NOT_OK(BMI160_get_reg(
			h, BMI160_REG_STATUS, &status));
		foc_rdy = status & BMI160_REG_STATUS_FOC_RDY;
		++attempts;
	} while (!foc_rdy && attempts < 4);
	if (!foc_rdy) {
		return BMI160_STATUS_ERROR;
	}

	BMI160_read_data(h);

	return BMI160_STATUS_OK;
}

static enum BMI160_Status BMI160_setup_int(struct BMI160_Handle *h)
{
	// Включаем FIFO для акселерометра и гироскопа
	const uint8_t fifo_config_1 = BMI160_REG_FIFO_CONFIG_1_GYR_EN
		| BMI160_REG_FIFO_CONFIG_1_ACC_EN;
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_FIFO_CONFIG_1, fifo_config_1));
	// Когда 3 * 4 байта готовы, тогда случится прерывание
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_FIFO_CONFIG_0, 3));
	// Активируем прерывание по готовности данных
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_INT_EN_1, BMI160_REG_INT_EN_1_DRDY_EN));
	// Включаем на выход INT1, высокий активный уровень, push-pull
	const uint8_t int_out_ctrl = BMI160_REG_INT_OUT_CTRL_INT1_OUTPUT_EN
		| BMI160_REG_INT_OUT_CTRL_INT1_LVL;
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_INT_OUT_CTRL, int_out_ctrl));
	// Привязываем прерывание DRDY к пину INT1
	BMI160_RETURN_IF_NOT_OK(BMI160_set_reg(
		h, BMI160_REG_INT_MAP_1, BMI160_REG_INT_MAP_1_INT1_DRDY));
	return BMI160_STATUS_OK;
}

enum BMI160_Status BMI160_Handle_init(struct BMI160_Handle *h)
{
	BMI160_RETURN_IF_NOT_OK(BMI160_activate_spi(h));
	BMI160_RETURN_IF_NOT_OK(BMI160_check_id(h));
	BMI160_RETURN_IF_NOT_OK(BMI160_setup_normal_mode(h));
	BMI160_RETURN_IF_NOT_OK(BMI160_configure(h));
	BMI160_RETURN_IF_NOT_OK(BMI160_fast_offset_compensation(h));
	BMI160_RETURN_IF_NOT_OK(BMI160_setup_int(h));
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
