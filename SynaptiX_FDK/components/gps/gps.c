#include "gps.h"
#include <string.h>
#include "logger.h"
#include "sx_delay.h"

static const char *TAG = "GPS";

static void gps_callback_task(sx_gps_t *gps, char *message, void *arg);

void gps_init(sx_gps_t *_gps, sx_uart_config_t *_uart_cfg, sx_gpio_ops_t *_gpio_ops, void *_pwr_arg, void *_rst_arg)
{
    sx_uart_init(&_gps->comm, _uart_cfg, GPS_BUFF_MAX_SIZE, GPS_BUFF_MAX_SIZE);
    sx_gpio_init(&_gps->pwr, _gpio_ops, _pwr_arg);
    // sx_gpio_init(&_gps->rst, _gpio_ops, _rst_arg);
    sx_gpio_write(&_gps->pwr, SX_GPIO_LOW);
    // sx_gpio_write(&_gps->rst, SX_GPIO_HIGH);
    memset(_gps->buff, 0, GPS_BUFF_MAX_SIZE);
    _gps->state = GPS_IDLE;
    _gps->buff_id = 0;
    gps_set_callback_handle(_gps, gps_callback_task, _gps);
    sx_delay_ms(1000);
    log_debug(TAG, "Initializing");
}

void gps_set_callback_handle(sx_gps_t *_gps, gps_callback _cb, void *_arg)
{
    _gps->callback = _cb;
    _gps->arg = _arg;
}

void gps_reset(sx_gps_t *_gps)
{
    // sx_gpio_write(&_gps->rst, SX_GPIO_LOW);
    // sx_delay_ms(100);
    // sx_gpio_write(&_gps->rst, SX_GPIO_HIGH);
}
void gps_power_on(sx_gps_t *_gps)
{
    //sx_gpio_write(&_gps->pwr, SX_GPIO_HIGH);
    sx_gpio_write(&_gps->pwr, SX_GPIO_LOW);
}
void gps_power_off(sx_gps_t *_gps)
{
    //sx_gpio_write(&_gps->pwr, SX_GPIO_LOW);
    sx_gpio_write(&_gps->pwr, SX_GPIO_HIGH);
}

void gps_process(sx_gps_t *_gps, uint32_t _timestamp)
{
    _gps->time += _timestamp;
    switch (_gps->state)
    {
    case GPS_IDLE:
        /* code */
        while(sx_uart_available(&_gps->comm) > 0){
            if (1 == sx_uart_read(&_gps->comm, &_gps->buff[0], 1, 10))
            {
                if (_gps->buff[0] == '$')
                {
                    _gps->buff_id = 1;
                    _gps->state = GPS_READ_MESSAGE;
                    break;
                }
            }
        }
        break;
    case GPS_READ_MESSAGE:
        /* code */
        uint8_t byte;
        while (sx_uart_available(&_gps->comm) > 0)
        {
            if (1 != sx_uart_read(&_gps->comm, &byte, 1, 1))
                break;

            _gps->buff[_gps->buff_id] = byte;

            // Detect \r\n
            if (_gps->buff_id > 0 &&
                _gps->buff[_gps->buff_id - 1] == '\r' &&
                byte == '\n')
            {
                _gps->buff[_gps->buff_id + 1] = '\0';
                _gps->state = GPS_NEW_MESSAGE;
                break;
            }

            _gps->buff_id++;

            if (_gps->buff_id >= GPS_BUFF_MAX_SIZE - 2)
            {
                _gps->buff_id = 0;
                _gps->state = GPS_IDLE;
                break;
            }
        }
        break;
    case GPS_NEW_MESSAGE:
        /* code */
        if (_gps->callback)
        {
            _gps->callback(_gps, _gps->buff, _gps->arg);
        }
        _gps->state = GPS_IDLE;
        // log_info(TAG, "%s", _gps->buff);
        memset(_gps->buff, 0, _gps->buff_id);
        _gps->buff_id = 0;
        break;
    default:
        break;
    }
}

// static void gps_callback_task(sx_gps_t *gps, char *message, void *arg)
// {
//     gps->_sentence_id = minmea_sentence_id((const char *)message, false);
//     switch (gps->_sentence_id)
//     {
//     case MINMEA_SENTENCE_RMC:
//     {
//         struct minmea_sentence_rmc rmc;
//         if (minmea_parse_rmc(&rmc, (char *)message))
//         {
//             if (!rmc.valid)
//             {
//                 log_warn(TAG, "RMC sentence is not valid");
//                 break;
//             }
//             gps->latitude = minmea_tocoord(&rmc.latitude);
//             gps->longtitude = minmea_tocoord(&rmc.longitude);
//             log_info(TAG, "RMC: lat %f lon %f", gps->latitude, gps->longtitude);
//             if (rmc.time.hours != -1 && rmc.date.year != -1)
//             {
//                 minmea_getdatetime(&gps->tim, &rmc.date, &rmc.time);
//                 // UTC+7
//                 gps->tim.tm_hour += 7;
//                 if (gps->tim.tm_hour >= 24)
//                 {
//                     gps->tim.tm_hour -= 24;
//                     gps->tim.tm_mday += 1;
//                 }
//                 log_info(TAG, "RMC: lat=%.6f lon=%.6f time=%02d:%02d:%02d date=%02d/%02d/%04d",
//                          gps->latitude, gps->longtitude,
//                          gps->tim.tm_hour, gps->tim.tm_min, gps->tim.tm_sec,
//                          gps->tim.tm_mday, gps->tim.tm_mon + 1, gps->tim.tm_year + 1900);
//             }
//         }
//         else
//         {
//             log_warn(TAG, "RMC sentence is not valid");
//         }
//     }
//     break;
//     default:
//         break;
//     }
// }


static void gps_callback_task(sx_gps_t *gps, char *message, void *arg)
{
    gps->_sentence_id = minmea_sentence_id((const char *)message, false);
    switch (gps->_sentence_id)
    {
    case MINMEA_SENTENCE_RMC:
    {
        struct minmea_sentence_rmc rmc;
        if (minmea_parse_rmc(&rmc, (char *)message))
        {
            if (!rmc.valid)
            {
                log_warn(TAG, "RMC sentence is not valid");
                gps->latitude = 0.0f;
                gps->longtitude = 0.0f;
                break;
            }
            gps->latitude  = minmea_tocoord(&rmc.latitude);
            gps->longtitude = minmea_tocoord(&rmc.longitude);
            gps->speed     = minmea_tofloat(&rmc.speed);  // knots
            if (rmc.time.hours != -1 && rmc.date.year != -1)
            {
                minmea_getdatetime(&gps->tim, &rmc.date, &rmc.time);
                // UTC+7
                gps->tim.tm_hour += 7;
                if (gps->tim.tm_hour >= 24)
                {
                    gps->tim.tm_hour -= 24;
                    gps->tim.tm_mday += 1;
                }
                log_info(TAG, "RMC: lat=%.6f lon=%.6f spd=%.2f time=%02d:%02d:%02d date=%02d/%02d/%04d",
                         gps->latitude, gps->longtitude, gps->speed,
                         gps->tim.tm_hour, gps->tim.tm_min, gps->tim.tm_sec,
                         gps->tim.tm_mday, gps->tim.tm_mon + 1, gps->tim.tm_year + 1900);
            }
        }
        else
        {
            log_warn(TAG, "RMC parse failed");
        }
    }
    break;
    case MINMEA_SENTENCE_GGA:
    {
        struct minmea_sentence_gga gga;
        if (minmea_parse_gga(&gga, (char *)message))
        {
            gps->altitude   = minmea_tofloat(&gga.altitude);
            gps->satellites = gga.satellites_tracked;
            log_info(TAG, "GGA: alt=%.2f sat=%d", gps->altitude, gps->satellites);
        }
        else
        {
            log_warn(TAG, "GGA parse failed");
        }
    }
    break;
    default:
        break;
    }
}
