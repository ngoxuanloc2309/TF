#ifndef AT_COMMAND_H
#define AT_COMMAND_H


#if __cplusplus 
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define AT_RX_BUFFER_SIZE 256

#define AT_END_OF_COMMAND "\r\n"

#define AT_TYPE_QUESTION "?"
#define AT_TYPE_SET "="
#define AT_TYPE_EXECUTE ""

typedef struct AT_Command AT_Command_t;

typedef int (*at_question_response_handler_t)(AT_Command_t *cmd);
typedef int (*at_set_response_handler_t)(AT_Command_t *cmd,const char *param);
typedef int (*at_execute_response_handler_t)(AT_Command_t *cmd);

typedef struct AT_Handler{
    at_question_response_handler_t question_handler;
    at_set_response_handler_t set_handler;
    at_execute_response_handler_t execute_handler;
} AT_Handler_t;

struct AT_Command{
    char *command;
    AT_Handler_t handler;
};

typedef void (*at_response_callback_t)(const char *response);

typedef struct AT_Implementation{
    AT_Command_t *commands;
    size_t num_commands;

    uint8_t rx_buffer[AT_RX_BUFFER_SIZE];
    size_t rx_buffer_index;
    at_response_callback_t response_callback;

} AT_Implementation_t;


int at_init(AT_Implementation_t *at_impl, AT_Command_t *commands, size_t num_commands);
int at_process_input(AT_Implementation_t *at_impl, const char *input, size_t input_length);


#if __cplusplus 
}
#endif


#endif