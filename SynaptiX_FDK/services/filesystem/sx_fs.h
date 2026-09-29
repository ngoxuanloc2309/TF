#ifndef __SX_FS_H
#define __SX_FS_H

#ifdef __cplusplus
extern "C"{
#endif

#include "file_io.h"
#include "sx_external_flash.h"

int sx_fs_init(sx_ext_flash_t *flash);

#ifdef __cplusplus
}
#endif

#endif