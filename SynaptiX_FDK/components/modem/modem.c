#include "modem.h"
#include "logger.h"
#include "sx_delay.h"

static const char *TAG = "MODEM";

void modem_init(modem_t *modem){
    modem->isBusy = 0;
    modem->isReady = 0;
    modem->resID = 0;
    modem->cmd = NULL;
    log_debug(TAG,"Initializing");

}

int modem_send_command(modem_t *modem, modem_command_t *cmd, uint32_t timeout){
    if(modem->isBusy) return -1;
    log_debug(TAG,"Send command %s",cmd->cmd);
    modem->cmd = cmd;
    modem->resID = 0;
    modem->isBusy = 1;
    modem->timeOut = timeout;
    modem->buff_id = 0;
    sx_uart_flush(&modem->uart);
    sx_uart_write(&modem->uart, (const uint8_t *)cmd->cmd, strlen(cmd->cmd));
    return 0;
}

void modem_poll(modem_t *modem, uint32_t timeStamp){
    static uint32_t s_time = 0;
    if (!modem->isBusy) return;

    int available = sx_uart_available(&modem->uart);
    
    if(available > 0 && (modem->buff_id + available) < MODEM_RX_BUFFER_SIZE){
        
        int read = sx_uart_read(&modem->uart, (uint8_t *)modem->buff + modem->buff_id, available, 10);
        if(read > 0){
            log_debug(TAG, "Read : %d bytes", read);
            log_debug(TAG,"Data : %s",modem->buff+modem->buff_id);
            log_print_hex(LOGGER_DEBUG,TAG,modem->buff+modem->buff_id,read);
            modem->buff_id += read;
            s_time = 0; 

            if(modem->cmd->res_success && strstr(modem->buff, modem->cmd->res_success)){
                modem->isBusy = 0;
                s_time = 0;
                log_debug(TAG, "Command success: [%s]", modem->buff);
                if(modem->cmd->callback)
                    modem->cmd->callback(modem, modem->buff, MODEM_RESPONSE_SUCCESS, modem->cmd->arg);
                return;
            }
            else if(modem->cmd->res_fail && strstr(modem->buff, modem->cmd->res_fail)){
                modem->isBusy = 0;
                s_time = 0;
                log_debug(TAG, "Command fail: [%s]", modem->buff);
                if(modem->cmd->callback)
                    modem->cmd->callback(modem, modem->buff, MODEM_RESPONSE_FAIL, modem->cmd->arg);
                return;
            }
        }
    }

    s_time += timeStamp;
    if(s_time >= modem->timeOut){
        modem->isBusy = 0;
        s_time = 0;
        log_error(TAG, "TIMEOUT response: [%s]", (modem->buff_id > 0) ? modem->buff : "NULL");
        if(modem->cmd->callback)
            modem->cmd->callback(modem, NULL, MODEM_RESPONSE_TIMEOUT, modem->cmd->arg);
    }
}
