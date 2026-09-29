#include "at_command.h"

int at_init(AT_Implementation_t *at_impl, AT_Command_t *commands, size_t num_commands){
    if(at_impl == NULL || commands == NULL || num_commands == 0){
        return -1; // Invalid parameters
    }

    at_impl->commands = commands;
    at_impl->num_commands = num_commands;
    at_impl->rx_buffer_index = 0;
    at_impl->response_callback = NULL;

    return 0; // Success
}

int at_process_input(AT_Implementation_t *at_impl, const char *input, size_t input_length)
{
    if (at_impl == NULL || input == NULL || input_length == 0) return -1;

    for (size_t i = 0; i < input_length; i++) {
        char c = input[i];

        if (c == '\r' || c == '\n') {
            if (at_impl->rx_buffer_index > 0) {
                at_impl->rx_buffer[at_impl->rx_buffer_index] = '\0';

                char *eq  = strchr((char *)at_impl->rx_buffer, '=');
                char *q   = strchr((char *)at_impl->rx_buffer, '?');

                char cmd_name[AT_RX_BUFFER_SIZE];
                const char *param   = NULL;
                char        at_type = 0;  

                if (eq != NULL) {
                    /* AT+CMD=param */
                    size_t name_len = (size_t)(eq - (char *)at_impl->rx_buffer);
                    strncpy(cmd_name, (char *)at_impl->rx_buffer, name_len);
                    cmd_name[name_len] = '\0';
                    param   = eq + 1;
                    at_type = '=';
                } else if (q != NULL) {
                    /* AT+CMD? */
                    size_t name_len = (size_t)(q - (char *)at_impl->rx_buffer);
                    strncpy(cmd_name, (char *)at_impl->rx_buffer, name_len);
                    cmd_name[name_len] = '\0';
                    at_type = '?';
                } else {
                    /* AT+CMD (execute) */
                    strncpy(cmd_name, (char *)at_impl->rx_buffer, AT_RX_BUFFER_SIZE - 1);
                    cmd_name[AT_RX_BUFFER_SIZE - 1] = '\0';
                    at_type = 0;
                }

                for (size_t j = 0; j < at_impl->num_commands; j++) {
                    if (strcmp(cmd_name, at_impl->commands[j].command) == 0) {
                        switch (at_type) {
                            case '?':
                                if (at_impl->commands[j].handler.question_handler)
                                    at_impl->commands[j].handler.question_handler(&at_impl->commands[j]);
                                break;
                            case '=':
                                if (at_impl->commands[j].handler.set_handler)
                                    at_impl->commands[j].handler.set_handler(&at_impl->commands[j], param);
                                break;
                            default:
                                if (at_impl->commands[j].handler.execute_handler)
                                    at_impl->commands[j].handler.execute_handler(&at_impl->commands[j]);
                                break;
                        }
                        break;
                    }
                }

                /* ── Reset buffer ── */
                at_impl->rx_buffer_index = 0;
            }
        } else {
            if (at_impl->rx_buffer_index < AT_RX_BUFFER_SIZE - 1)
                at_impl->rx_buffer[at_impl->rx_buffer_index++] = c;
            else
                at_impl->rx_buffer_index = 0;
        }
    }
    return 0;
}
