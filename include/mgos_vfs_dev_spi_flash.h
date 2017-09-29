/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#ifndef CS_MOS_LIBS_VFS_DEV_SPI_FLASH_SRC_MGOS_VFS_DEV_SPI_FLASH_H_
#define CS_MOS_LIBS_VFS_DEV_SPI_FLASH_SRC_MGOS_VFS_DEV_SPI_FLASH_H_

#include <stdbool.h>

#define MGOS_VFS_DEV_TYPE_SPI_FLASH "spi_flash"

#ifdef __cplusplus
extern "C" {
#endif

bool mgos_vfs_dev_spi_flash_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CS_MOS_LIBS_VFS_DEV_SPI_FLASH_SRC_MGOS_VFS_DEV_SPI_FLASH_H_ */
