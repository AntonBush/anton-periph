#include <stddef.h>

#include "BMI160/BMI160.h"
#include "BMI160/BMI160_datasheet.h"

#define BMI160_RETURN_IF_NOT_OK(status) \
{enum BMI160_Status s = (status); \
	if (s != BMI160_STATUS_OK) return s;}

static enum BMI160_Status BMI160_write_read(
	struct BMI160_Handle *h, BMI160_Size count)
{
	return h->init.write_readf(h->tx, h->rx, count);
}

static void BMI160_wait(struct BMI160_Handle *h, int ms)
{
	h->init.waitf(ms);
}

static enum BMI160_Status BMI160_get_reg(
	struct BMI160_Handle *h, uint8_t reg, uint8_t *val_ptr)
{
	h->tx[0] = BMI160_read_command(reg);
	BMI160_RETURN_IF_NOT_OK(BMI160_write_read(h, 2));
	if (val_ptr != NULL) {
		*val_ptr = h->rx[1];
	}
	return BMI160_STATUS_OK;
}

static enum BMI160_Status BMI160_set_reg(
	struct BMI160_Handle *h, uint8_t reg, uint8_t val)
{
	h->tx[0] = BMI160_write_command(reg);
	h->tx[1] = val;
	return BMI160_write_read(h, 2);
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
		h, BMI160_REG_CMD, BMI160_CMD_COMMAND_SOFTRESET));
	BMI160_wait(h, 200);
	BMI160_RETURN_IF_NOT_OK(BMI160_get_reg(
		h, BMI160_REG_DUMMY, NULL));
	BMI160_wait(h, 20);
	BMI160_RETURN_IF_NOT_OK(BMI160_check_id(h));

	return BMI160_STATUS_OK;
}
