#include "bno055.h"
#include "sx_delay.h"
#include "logger.h"

static const char* TAG = "IMU";

#define REG_CHIP_ID          0x00U
#define REG_PAGE_ID          0x07U
#define REG_ACC_DATA_X_LSB   0x08U
#define REG_MAG_DATA_X_LSB   0x0EU
#define REG_GYR_DATA_X_LSB   0x14U
#define REG_EUL_HEADING_LSB  0x1AU
#define REG_QUA_DATA_W_LSB   0x20U
#define REG_LIA_DATA_X_LSB   0x28U
#define REG_GRV_DATA_X_LSB   0x2EU
#define REG_CALIB_STAT       0x35U
#define REG_SYS_STATUS       0x39U
#define REG_SYS_ERR          0x3AU
#define REG_UNIT_SEL         0x3BU
#define REG_OPR_MODE         0x3DU
#define REG_PWR_MODE         0x3EU
#define REG_SYS_TRIGGER      0x3FU
#define REG_ACC_OFFSET_X_LSB 0x55U
#define REG_MAG_OFFSET_X_LSB 0x5BU
#define REG_GYR_OFFSET_X_LSB 0x61U
#define REG_ACC_RADIUS_LSB   0x67U
#define REG_MAG_RADIUS_LSB   0x69U

#define DELAY_POWER_ON_MS    10U
#define DELAY_RESET_MS       650U
#define DELAY_MODE_SWITCH_MS 20U

static int _reg_write(bno055_t *dev, uint8_t reg, uint8_t val)
{
    return sx_i2c_mem_write(dev->i2c, dev->dev_addr, reg, 1, &val, 1);
}

static int _reg_read(bno055_t *dev, uint8_t reg, uint8_t *buf, uint32_t len)
{
    return sx_i2c_mem_read(dev->i2c, dev->dev_addr, reg, 1, buf, len);
}

static int _read_vec3(bno055_t *dev, uint8_t start_reg, bno055_vec3_t *out)
{
    uint8_t buf[6];
    if (_reg_read(dev, start_reg, buf, 6) != 0) return BNO055_ERR_I2C;
    out->x = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    out->y = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    out->z = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    return BNO055_OK;
}

int bno055_power_on(bno055_t *dev)
{
    if (!dev) return BNO055_ERR_PARAM;
    if (dev->en_gpio) {
        sx_gpio_write(dev->en_gpio, SX_GPIO_LOW);
        sx_delay_ms(DELAY_POWER_ON_MS);
    }
    return BNO055_OK;
}

int bno055_power_off(bno055_t *dev)
{
    if (!dev) return BNO055_ERR_PARAM;
    if (dev->en_gpio) {
        sx_gpio_write(dev->en_gpio, SX_GPIO_HIGH);
    }
    return BNO055_OK;
}

int bno055_init(bno055_t *dev, sx_i2c_t *i2c, uint16_t dev_addr,
                sx_gpio_t *en_gpio, sx_gpio_t *reset_gpio)
{
    if (!dev || !i2c) return BNO055_ERR_PARAM;

    dev->i2c         = i2c;
    dev->dev_addr    = dev_addr;
    dev->en_gpio     = en_gpio;
    dev->reset_gpio  = reset_gpio;
    dev->initialized = false;

    if (dev->reset_gpio) {
        sx_gpio_write(dev->reset_gpio, SX_GPIO_LOW);
        sx_delay_ms(10);
        sx_gpio_write(dev->reset_gpio, SX_GPIO_HIGH);
        sx_delay_ms(DELAY_RESET_MS);
    } else {
        if (_reg_write(dev, REG_SYS_TRIGGER, 0x20) != 0) return BNO055_ERR_I2C;
        sx_delay_ms(DELAY_RESET_MS);
    }

    uint8_t chip_id = 0;
    if (_reg_read(dev, REG_CHIP_ID, &chip_id, 1) != 0) return BNO055_ERR_I2C;
    if (chip_id != BNO055_CHIP_ID) return BNO055_ERR_CHIP_ID;

    if (_reg_write(dev, REG_PAGE_ID, 0x00) != 0) return BNO055_ERR_I2C;
    if (_reg_write(dev, REG_PWR_MODE, BNO055_PWR_MODE_NORMAL) != 0) return BNO055_ERR_I2C;
    if (_reg_write(dev, REG_UNIT_SEL, 0x00) != 0) return BNO055_ERR_I2C;
    if (_reg_write(dev, REG_SYS_TRIGGER, 0x80) != 0) return BNO055_ERR_I2C;
    sx_delay_ms(10);

    if (bno055_set_opr_mode(dev, BNO055_OPR_MODE_NDOF) != 0) return BNO055_ERR_I2C;
    dev->initialized = true;
    log_info(TAG, "IMU init done");
    return BNO055_OK;
}

int bno055_set_opr_mode(bno055_t *dev, bno055_opr_mode_t mode)
{
    if (!dev) return BNO055_ERR_PARAM;
    if (_reg_write(dev, REG_OPR_MODE, (uint8_t)mode) != 0) return BNO055_ERR_I2C;
    sx_delay_ms(DELAY_MODE_SWITCH_MS);
    return BNO055_OK;
}

int bno055_set_pwr_mode(bno055_t *dev, bno055_pwr_mode_t mode)
{
    if (!dev) return BNO055_ERR_PARAM;
    if (_reg_write(dev, REG_OPR_MODE, BNO055_OPR_MODE_CONFIGMODE) != 0) return BNO055_ERR_I2C;
    sx_delay_ms(DELAY_MODE_SWITCH_MS);
    if (_reg_write(dev, REG_PWR_MODE, (uint8_t)mode) != 0) return BNO055_ERR_I2C;
    return BNO055_OK;
}

int bno055_get_accel(bno055_t *dev, bno055_vec3_t *out)
{
    if (!dev || !out) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;
    return _read_vec3(dev, REG_ACC_DATA_X_LSB, out);
}

int bno055_get_mag(bno055_t *dev, bno055_vec3_t *out)
{
    if (!dev || !out) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;
    return _read_vec3(dev, REG_MAG_DATA_X_LSB, out);
}

int bno055_get_gyro(bno055_t *dev, bno055_vec3_t *out)
{
    if (!dev || !out) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;
    return _read_vec3(dev, REG_GYR_DATA_X_LSB, out);
}

int bno055_get_linear_accel(bno055_t *dev, bno055_vec3_t *out)
{
    if (!dev || !out) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;
    return _read_vec3(dev, REG_LIA_DATA_X_LSB, out);
}

int bno055_get_gravity(bno055_t *dev, bno055_vec3_t *out)
{
    if (!dev || !out) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;
    return _read_vec3(dev, REG_GRV_DATA_X_LSB, out);
}

int bno055_get_euler(bno055_t *dev, bno055_euler_t *out)
{
    if (!dev || !out) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;
    uint8_t buf[6];
    if (_reg_read(dev, REG_EUL_HEADING_LSB, buf, 6) != 0) return BNO055_ERR_I2C;
    out->heading = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    out->roll    = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    out->pitch   = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    return BNO055_OK;
}

int bno055_get_quat(bno055_t *dev, bno055_quat_t *out)
{
    if (!dev || !out) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;
    uint8_t buf[8];
    if (_reg_read(dev, REG_QUA_DATA_W_LSB, buf, 8) != 0) return BNO055_ERR_I2C;
    out->w = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    out->x = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    out->y = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    out->z = (int16_t)(buf[6] | ((uint16_t)buf[7] << 8));
    return BNO055_OK;
}

int bno055_get_calib_stat(bno055_t *dev, bno055_calib_stat_t *out)
{
    if (!dev || !out) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;
    uint8_t val;
    if (_reg_read(dev, REG_CALIB_STAT, &val, 1) != 0) return BNO055_ERR_I2C;
    out->sys   = (val >> 6) & 0x03U;
    out->gyro  = (val >> 4) & 0x03U;
    out->accel = (val >> 2) & 0x03U;
    out->mag   =  val       & 0x03U;
    return BNO055_OK;
}

int bno055_get_calib_data(bno055_t *dev, bno055_calib_data_t *out)
{
    if (!dev || !out) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;

    if (_reg_write(dev, REG_OPR_MODE, BNO055_OPR_MODE_CONFIGMODE) != 0) return BNO055_ERR_I2C;
    sx_delay_ms(DELAY_MODE_SWITCH_MS);

    uint8_t buf[22];
    if (_reg_read(dev, REG_ACC_OFFSET_X_LSB, buf, 22) != 0) return BNO055_ERR_I2C;

    out->acc_x      = (int16_t)(buf[0]  | ((uint16_t)buf[1]  << 8));
    out->acc_y      = (int16_t)(buf[2]  | ((uint16_t)buf[3]  << 8));
    out->acc_z      = (int16_t)(buf[4]  | ((uint16_t)buf[5]  << 8));
    out->mag_x      = (int16_t)(buf[6]  | ((uint16_t)buf[7]  << 8));
    out->mag_y      = (int16_t)(buf[8]  | ((uint16_t)buf[9]  << 8));
    out->mag_z      = (int16_t)(buf[10] | ((uint16_t)buf[11] << 8));
    out->gyr_x      = (int16_t)(buf[12] | ((uint16_t)buf[13] << 8));
    out->gyr_y      = (int16_t)(buf[14] | ((uint16_t)buf[15] << 8));
    out->gyr_z      = (int16_t)(buf[16] | ((uint16_t)buf[17] << 8));
    out->acc_radius = (int16_t)(buf[18] | ((uint16_t)buf[19] << 8));
    out->mag_radius = (int16_t)(buf[20] | ((uint16_t)buf[21] << 8));

    return BNO055_OK;
}

int bno055_set_calib_data(bno055_t *dev, const bno055_calib_data_t *in)
{
    if (!dev || !in) return BNO055_ERR_PARAM;
    if (!dev->initialized) return BNO055_ERR_NOT_INIT;

    if (_reg_write(dev, REG_OPR_MODE, BNO055_OPR_MODE_CONFIGMODE) != 0) return BNO055_ERR_I2C;
    sx_delay_ms(DELAY_MODE_SWITCH_MS);

    uint8_t buf[22];
    buf[0]  = (uint8_t)(in->acc_x      & 0xFF);
    buf[1]  = (uint8_t)(in->acc_x      >> 8);
    buf[2]  = (uint8_t)(in->acc_y      & 0xFF);
    buf[3]  = (uint8_t)(in->acc_y      >> 8);
    buf[4]  = (uint8_t)(in->acc_z      & 0xFF);
    buf[5]  = (uint8_t)(in->acc_z      >> 8);
    buf[6]  = (uint8_t)(in->mag_x      & 0xFF);
    buf[7]  = (uint8_t)(in->mag_x      >> 8);
    buf[8]  = (uint8_t)(in->mag_y      & 0xFF);
    buf[9]  = (uint8_t)(in->mag_y      >> 8);
    buf[10] = (uint8_t)(in->mag_z      & 0xFF);
    buf[11] = (uint8_t)(in->mag_z      >> 8);
    buf[12] = (uint8_t)(in->gyr_x      & 0xFF);
    buf[13] = (uint8_t)(in->gyr_x      >> 8);
    buf[14] = (uint8_t)(in->gyr_y      & 0xFF);
    buf[15] = (uint8_t)(in->gyr_y      >> 8);
    buf[16] = (uint8_t)(in->gyr_z      & 0xFF);
    buf[17] = (uint8_t)(in->gyr_z      >> 8);
    buf[18] = (uint8_t)(in->acc_radius & 0xFF);
    buf[19] = (uint8_t)(in->acc_radius >> 8);
    buf[20] = (uint8_t)(in->mag_radius & 0xFF);
    buf[21] = (uint8_t)(in->mag_radius >> 8);

    if (sx_i2c_mem_write(dev->i2c, dev->dev_addr,
                         REG_ACC_OFFSET_X_LSB, 1,
                         buf, 22) != 0) return BNO055_ERR_I2C;

    return BNO055_OK;
}