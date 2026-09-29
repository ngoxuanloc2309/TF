#ifndef SX_GPIO_H
#define SX_GPIO_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum SX_GPIO_VALUE{
    SX_GPIO_LOW = 0,
    SX_GPIO_HIGH = 1
} SX_GPIO_VALUE;

typedef struct sx_gpio sx_gpio_t;

typedef struct {
    void    *port;
    uint16_t pin;
} sx_gpio_pin_t;

typedef struct sx_gpio_ops{
    int (*write)(sx_gpio_t *gpio, SX_GPIO_VALUE value);
    int (*read)(sx_gpio_t *gpio, SX_GPIO_VALUE *value);
    int (*toggle)(sx_gpio_t *gpio);
} sx_gpio_ops_t;

typedef struct sx_gpio{
    sx_gpio_ops_t *ops;
    void *pDriver;
    SX_GPIO_VALUE value;
} sx_gpio_t;

static inline void sx_gpio_init(sx_gpio_t *gpio, sx_gpio_ops_t *ops, void *pDriver){
    gpio->ops = ops;
    gpio->pDriver = pDriver;
    gpio->value = SX_GPIO_LOW;
}

static inline int sx_gpio_write(sx_gpio_t *gpio, SX_GPIO_VALUE value){
    if(gpio->ops && gpio->ops->write){
        int ret = gpio->ops->write(gpio, value);
        if(ret == 0){
            gpio->value = value;
        }
        return ret;
    }
    return -1;
}

static inline int sx_gpio_read(sx_gpio_t *gpio, SX_GPIO_VALUE *value){
    if(gpio->ops && gpio->ops->read){
        return gpio->ops->read(gpio, value);
    }
    return -1;
}

static inline int sx_gpio_toggle(sx_gpio_t *gpio){
    if(gpio->ops && gpio->ops->toggle){
        int ret = gpio->ops->toggle(gpio);
        if(ret == 0){
            gpio->value = (gpio->value == SX_GPIO_LOW) ? SX_GPIO_HIGH : SX_GPIO_LOW;
        }
        return ret;
    }
    return -1;
}

extern sx_gpio_ops_t sx_gpio_ops;


#ifdef __cplusplus
}
#endif
#endif // SX_GPIO_H