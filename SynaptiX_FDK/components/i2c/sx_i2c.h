#ifndef SX_I2C_H
#define SX_I2C_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct sx_i2c sx_i2c_t;

typedef struct sx_i2c_ops{
    int (*write)(sx_i2c_t *i2c, uint16_t dev_addr, const uint8_t *data, uint32_t len);
    int (*read)(sx_i2c_t *i2c, uint16_t dev_addr, uint8_t *data, uint32_t len);
    int (*mem_write)(sx_i2c_t *i2c, uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size, const uint8_t *data, uint32_t len);
    int (*mem_read)(sx_i2c_t *i2c, uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size, uint8_t *data, uint32_t len);
    int (*is_device_ready)(sx_i2c_t *i2c, uint16_t dev_addr, uint8_t num_trial, uint32_t timeout);
} sx_i2c_ops_t;

struct sx_i2c{
    sx_i2c_ops_t *ops;
    void *pDriver;
};

static inline void sx_i2c_init(sx_i2c_t *i2c, sx_i2c_ops_t *ops, void *pDriver){
    i2c->ops = ops;
    i2c->pDriver = pDriver;
}

static inline int sx_i2c_write(sx_i2c_t *i2c, uint16_t dev_addr, const uint8_t *data, uint32_t len){
    if(i2c->ops && i2c->ops->write)
        return i2c->ops->write(i2c, dev_addr, data, len);
    return -1;
}

static inline int sx_i2c_read(sx_i2c_t *i2c, uint16_t dev_addr, uint8_t *data, uint32_t len){
    if(i2c->ops && i2c->ops->read)
        return i2c->ops->read(i2c, dev_addr, data, len);
    return -1;
}

static inline int sx_i2c_mem_write(sx_i2c_t *i2c, uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size, const uint8_t *data, uint32_t len){
    if(i2c->ops && i2c->ops->mem_write)
        return i2c->ops->mem_write(i2c, dev_addr, mem_addr, mem_addr_size, data, len);
    return -1;
}

static inline int sx_i2c_mem_read(sx_i2c_t *i2c, uint16_t dev_addr, uint16_t mem_addr, uint16_t mem_addr_size, uint8_t *data, uint32_t len){
    if(i2c->ops && i2c->ops->mem_read)
        return i2c->ops->mem_read(i2c, dev_addr, mem_addr, mem_addr_size, data, len);
    return -1;
}

static inline int sx_i2c_is_device_ready(sx_i2c_t *i2c, uint16_t dev_addr, uint8_t num_trial, uint32_t timeout){
    if(i2c->ops && i2c->ops->is_device_ready)
        return i2c->ops->is_device_ready(i2c, dev_addr, num_trial, timeout);
    return -1;
}

static inline void sx_i2c_scan(sx_i2c_t *i2c){
    int found = 0;
    for(uint8_t addr = 0x08; addr < 0x78; addr++){
        if(sx_i2c_is_device_ready(i2c, (uint16_t)(addr << 1), 1, 10) == 0){
            found++;
        }
    }
    (void)found;
}

extern sx_i2c_ops_t sx_i2c_ops;

#ifdef __cplusplus
}
#endif
#endif // SX_I2C_H
