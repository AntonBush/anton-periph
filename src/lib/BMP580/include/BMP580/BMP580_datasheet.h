#pragma once

enum BMP580_CommandMode {
	BMP580_COMMAND_MODE_READ  = 0x80,
	BMP580_COMMAND_MODE_WRITE = 0x7F,
};

enum BMP580_Reg {
    BMP580_REG_OSR_EFF = 0x38,
    BMP580_REG_ODR_CONFIG = 0x37,
    BMP580_REG_OSR_CONFIG = 0x36,

    BMP580_REG_PRESS_DATA_MSB  = 0x22,
    BMP580_REG_PRESS_DATA_LSB  = 0x21,
    BMP580_REG_PRESS_DATA_XLSB = 0x20,

    BMP580_REG_TEMP_DATA_MSB  = 0x1F,
    BMP580_REG_TEMP_DATA_LSB  = 0x1E,
    BMP580_REG_TEMP_DATA_XLSB = 0x1D,

    BMP580_REG_CHIP_STATUS = 0x11,
    BMP580_REG_CHIP_ID     = 0x01,
};

enum BMP580_RegDefault {
    BMP580_REG_DEFAULT_CHIP_ID = 0x50
};

static inline uint8_t BMP580_read_command(uint8_t reg_addr)
{
	return reg_addr | BMP580_COMMAND_MODE_READ;
}

static inline uint8_t BMP580_write_command(uint8_t reg_addr)
{
	return reg_addr & BMP580_COMMAND_MODE_WRITE;
}
