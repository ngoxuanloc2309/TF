#ifndef SX_MEM_H
#define SX_MEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

typedef void * (*sx_malloc_fn)(size_t _size);
typedef void (*sx_free_fn)(void *_ptr);
typedef void * (*sx_calloc_fn)(size_t _num, size_t _size);

typedef struct sx_mem_ops {
    sx_malloc_fn malloc;
    sx_free_fn free;
    sx_calloc_fn calloc;
} sx_mem_ops_t;

void sx_mem_set_ops(sx_mem_ops_t *_ops);
void *sx_malloc(size_t _size);
void sx_free(void *_ptr);
void *sx_calloc(size_t _num, size_t _size);

#ifdef __cplusplus
}
#endif

#endif /* SX_MEM_H */