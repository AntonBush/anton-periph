#include <stddef.h>
#include <math.h>

#include "BMP580/BMP580.h"
#include "BMP580/BMP580_datasheet.h"

#define BMP580_RETURN_IF_NOT_OK(status) \
{enum BMP580_Status s = (status); \
	if (s != BMP580_STATUS_OK) return s;}

static const float BMP580_P0_CONSTF = 101325.0f;
static const float BMP580_T0_KELVIN_CONSTF = 273.15f;

static enum BMP580_Status BMP580_write_read(
	struct BMP580_Handle *h, BMP580_Size count)
{
	return h->init.write_readf(h->tx, h->rx, count);
}

static void BMP580_wait(struct BMP580_Handle *h, int ms)
{
	h->init.waitf(ms);
}

static enum BMP580_Status BMP580_read(
	struct BMP580_Handle *h, uint8_t reg, uint8_t count)
{
	h->tx[0] = BMP580_read_command(reg);
	return BMP580_write_read(h, 1 + count);
}

static enum BMP580_Status BMP580_write(
	struct BMP580_Handle *h, uint8_t reg, uint8_t count)
{
	h->tx[0] = BMP580_write_command(reg);
	return BMP580_write_read(h, 1 + count);
}

static enum BMP580_Status BMP580_get_reg(
	struct BMP580_Handle *h, uint8_t reg, uint8_t *val_ptr)
{
	BMP580_RETURN_IF_NOT_OK(BMP580_read(h, reg, 1));
	if (val_ptr != NULL) {
		*val_ptr = h->rx[1];
	}
	return BMP580_STATUS_OK;
}

static enum BMP580_Status BMP580_set_reg(
	struct BMP580_Handle *h, uint8_t reg, uint8_t val)
{
	h->tx[1] = val;
	return BMP580_write(h, reg, 1);
}

enum BMP580_Status BMP580_Handle_init(struct BMP580_Handle *h)
{
    uint8_t chip_id = 0;
    
    BMP580_get_reg(h, BMP580_REG_CHIP_ID, NULL);
    // Даем датчику проснуться
    BMP580_wait(h, 10);
    BMP580_set_reg(h, BMP580_REG_CMD, BMP580_REG_CMD_SOFTRESET);
    BMP580_wait(h, 10);
    BMP580_get_reg(h, BMP580_REG_CHIP_ID, NULL);
    // Даем датчику проснуться
    BMP580_wait(h, 10);

    // Читаем Chip ID (у BMP580 он равен 0x50 или 0x51)
    BMP580_get_reg(h, BMP580_REG_CHIP_ID, &chip_id);
    // Ошибка: датчик не найден
    if (chip_id != BMP580_REG_DEFAULT_CHIP_ID) {
        return BMP580_STATUS_ERROR;
    }

    // Настройка ODR (частота данных) и OSR (оверсемплинг)
    // Режим по умолчанию: Continuous mode (активен сразу)
    // ODR = 50 Гц
    // 0x45 = deep_dis=0 + odr=10001 + pwr_mode=01
    // 0x45 = deep_dis=enable_deep_standby + odr=40Hz + pwr_mode=Normal 
    //BMP580_set_reg(h, BMP580_REG_ODR_CONFIG, 0x45);
    BMP580_set_reg(h, BMP580_REG_ODR_CONFIG,
        BMP580_REG_ODR_CONFIG_30_HZ
        | BMP580_REG_ODR_CONFIG_PWR_MODE_NORMAL);
    // OSR давления = 4x, температуры = 1x
    // 0x42 = 0 + press_en=1 + osr_p=000 + osr_t=010
    // 0x42 = 0 + press_en=enable_pressure + osr_p=oversampling_x1 + osr_t=oversampling_x4
    //BMP580_set_reg(h, BMP580_REG_OSR_CONFIG, 0x42);
    BMP580_set_reg(h, BMP580_REG_OSR_CONFIG,
        BMP580_REG_OSR_CONFIG_PRESS_EN
        | BMP580_REG_OSR_CONFIG_P_32X
        | BMP580_REG_OSR_CONFIG_T_32X);

    // drdy_data_reg_en:(bit offset: 0) Data Ready
    // fifo_full_en:(bit offset: 1) FIFO Full (FIFO_FULL)
    // fifo_ths_en:(bit offset: 2) FIFO Threshold/Watermark (FIFO_THS)
    // oor_p_en:(bit offset: 3) Pressure data out-of-range (OOR_P)
    // reserved_7_4:(bit offset: 4) reserved
    // 0x00: Disable INT. Except the POR and Software_reset completion
    BMP580_set_reg(h, BMP580_REG_INT_SOURCE, BMP580_REG_INT_SOURCE_DRDY);
    //   7 6 5 4   |    3   |   2    |    1    |    0
    // pad_int_drv | int_en | int_od | int_pol | int_mode
    // int_mode: 0-pulsed, 1-latched
    // int_polarity: 0-active_low, 1-active_high
    // int_od: 0-push_pull, 1-open_drain
    // int_en: 0-disabled, 1-enabled
    // pad_int_drv: Pad drive strength for INT (MSB should be set in INT open drain config only.)
    // Note: these register fields should be read-back only after waiting at least 1¬μs after they have been written
    BMP580_wait(h, 10);
    BMP580_set_reg(h, BMP580_REG_INT_CONFIG,
        (0b0111 << 4)
        | BMP580_REG_INT_CONFIG_EN
        | BMP580_REG_INT_CONFIG_ACTIVE_HIGH
        | BMP580_REG_INT_CONFIG_LATCHED
    );
    BMP580_wait(h, 10);

    h->press_0 = BMP580_P0_CONSTF;

    return BMP580_STATUS_OK;
}

void BMP580_read_data(struct BMP580_Handle *h)
{
    // Читаем подряд 6 байт (3 для давления, 3 для температуры)
    BMP580_read(h, BMP580_REG_TEMP_DATA_XLSB, 6);
    BMP580_process_data(h);
    BMP580_read(h, BMP580_REG_INT_STATUS, 1);
}

void BMP580_async_read_data(struct BMP580_Handle *h)
{
    ;
}

void BMP580_process_data(struct BMP580_Handle *h)
{
    // Сборка 24-битных значений
    h->data.raw.temp  = (h->rx[3] << 16) | (h->rx[2] << 8) | h->rx[1];
    h->data.raw.press = (h->rx[6] << 16) | (h->rx[5] << 8) | h->rx[4];

    // Перевод в реальные физические величины по формулам Bosch
    // Градусы Цельсия
    h->data.temp  = h->data.raw.temp  / 65536.0f;
    // Паскали
    h->data.press = h->data.raw.press /    64.0f;
    // Метры
    h->data.altitude = (powf(h->press_0 / h->data.press, 1 / 5.257f) - 1)
        * (h->data.temp + BMP580_T0_KELVIN_CONSTF)
        / 0.0065f;
}
