#ifndef BNO055_H
#define BNO055_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sx_i2c.h"
#include "sx_gpio.h"
#include <stdint.h>
#include <stdbool.h>

#define BNO055_I2C_ADDR_DEFAULT   (0x29U << 1)
#define BNO055_I2C_ADDR_ALT       (0x28U << 1)

#define BNO055_CHIP_ID            0xA0U

#define BNO055_OK                 0
#define BNO055_ERR_I2C            (-1)
#define BNO055_ERR_NOT_INIT       (-2)
#define BNO055_ERR_PARAM          (-3)
#define BNO055_ERR_CHIP_ID        (-4)

typedef enum {
    BNO055_OPR_MODE_CONFIGMODE   = 0x00,
    BNO055_OPR_MODE_ACCONLY      = 0x01,
    BNO055_OPR_MODE_MAGONLY      = 0x02,
    BNO055_OPR_MODE_GYROONLY     = 0x03,
    BNO055_OPR_MODE_ACCMAG       = 0x04,
    BNO055_OPR_MODE_ACCGYRO      = 0x05,
    BNO055_OPR_MODE_MAGGYRO      = 0x06,
    BNO055_OPR_MODE_AMG          = 0x07,
    BNO055_OPR_MODE_IMU          = 0x08,
    BNO055_OPR_MODE_COMPASS      = 0x09,
    BNO055_OPR_MODE_M4G          = 0x0A,
    BNO055_OPR_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_OPR_MODE_NDOF         = 0x0C,
} bno055_opr_mode_t;

typedef enum {
    BNO055_PWR_MODE_NORMAL   = 0x00,
    BNO055_PWR_MODE_LOWPOWER = 0x01,
    BNO055_PWR_MODE_SUSPEND  = 0x02,
} bno055_pwr_mode_t;

typedef struct {
    int16_t x, y, z;
} bno055_vec3_t;

typedef struct {
    int16_t w, x, y, z;
} bno055_quat_t;

typedef struct {
    int16_t heading;
    int16_t roll;
    int16_t pitch;
} bno055_euler_t;

typedef struct {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} bno055_calib_stat_t;

typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t gyr_x, gyr_y, gyr_z;
    int16_t acc_radius;
    int16_t mag_radius;
} bno055_calib_data_t;

typedef struct {
    sx_i2c_t  *i2c;
    uint16_t   dev_addr;
    sx_gpio_t *en_gpio;
    sx_gpio_t *reset_gpio;
    bool       initialized;
} bno055_t;

int bno055_power_on(bno055_t *dev);
int bno055_power_off(bno055_t *dev);

int bno055_init(bno055_t *dev, sx_i2c_t *i2c, uint16_t dev_addr,
                sx_gpio_t *en_gpio, sx_gpio_t *reset_gpio);

int bno055_set_opr_mode(bno055_t *dev, bno055_opr_mode_t mode);
int bno055_set_pwr_mode(bno055_t *dev, bno055_pwr_mode_t mode);

int bno055_get_accel(bno055_t *dev, bno055_vec3_t *out);
int bno055_get_mag(bno055_t *dev, bno055_vec3_t *out);
int bno055_get_gyro(bno055_t *dev, bno055_vec3_t *out);
int bno055_get_linear_accel(bno055_t *dev, bno055_vec3_t *out);
int bno055_get_gravity(bno055_t *dev, bno055_vec3_t *out);
int bno055_get_euler(bno055_t *dev, bno055_euler_t *out);
int bno055_get_quat(bno055_t *dev, bno055_quat_t *out);

int bno055_get_calib_stat(bno055_t *dev, bno055_calib_stat_t *out);
int bno055_get_calib_data(bno055_t *dev, bno055_calib_data_t *out);
int bno055_set_calib_data(bno055_t *dev, const bno055_calib_data_t *in);

#ifdef __cplusplus
}
#endif
#endif