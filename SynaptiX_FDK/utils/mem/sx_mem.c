#include "sx_mem.h"
#include <stdlib.h>

sx_mem_ops_t g_current_mem_ops = {
    .malloc = malloc,
    .free = free,
    .calloc = calloc
};

void sx_mem_set_ops(sx_mem_ops_t *ops){
    g_current_mem_ops.calloc = ops->calloc ? ops->calloc : calloc;
    g_current_mem_ops.free = ops->free ? ops->free : free;
    g_current_mem_ops.malloc = ops->malloc ? ops->malloc : malloc;
}
void *sx_malloc(size_t size){
    return g_current_mem_ops.malloc(size);
}
void sx_free(void *ptr){
    g_current_mem_ops.free(ptr);
}
void *sx_calloc(size_t num, size_t size){
    return g_current_mem_ops.calloc(num, size);
}