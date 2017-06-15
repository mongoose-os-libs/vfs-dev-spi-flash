/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#include "mgos_vfs_dev_spi_flash.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "common/cs_dbg.h"

#include "frozen/frozen.h"
#include "mongoose/mongoose.h"

#include "fw/src/mgos_hal.h"
#include "fw/src/mgos_spi.h"
#include "fw/src/mgos_utils.h"
#include "fw/src/mgos_vfs_dev.h"

struct dev_data {
  struct mgos_spi *spi;
  int cs;
  int freq;
  int mode;
  size_t size;
  uint32_t read_op : 8;
  uint32_t read_op_nwb : 8;
  uint32_t erase_sector_op : 8;
  uint32_t wip_mask : 8;
};

#define SPI_FLASH_OP_PROGRAM_PAGE 0x02
#define SPI_FLASH_OP_WRDI 0x04
#define SPI_FLASH_OP_RDSR 0x05
#define SPI_FLASH_OP_WREN 0x06
#define SPI_FLASH_OP_READ_FAST 0x0b
#define SPI_FLASH_OP_ERASE_SECTOR 0x20
#define SPI_FLASH_OP_READ_SFDP 0x5a
#define SPI_FLASH_OP_READ_JEDEC_ID 0x9f

#define SPI_FLASH_PAGE_SIZE 0x100
#define SPI_FLASH_SECTOR_SIZE 0x1000
#define SPI_FLASH_DEFAULT_WIP_MASK 0x01

#define SFDP_MAGIC 0x50444653 /* "SFDP" (LE) */
#define SFDP_PT0_LEN 9        /* DWORDs */

/* Note: structs below assume LE architecture */

/* SFDP is documented in JEDEC 216 */
struct __attribute__((packed)) sfdp_header {
  uint32_t magic;
  struct {
    uint32_t minor_rev : 8;
    uint32_t major_rev : 8;
    uint32_t nph : 8;
    uint32_t unused_ff : 8;
  };
};

struct __attribute__((packed)) sfdp_parameter_header {
  struct {
    uint32_t mfg_id : 8;
    uint32_t minor_rev : 8;
    uint32_t major_rev : 8;
    uint32_t len_dw : 8;
  };
  struct {
    uint32_t addr : 24;
    uint32_t unused_ff : 8;
  };
};

struct __attribute__((packed)) sfdp_pt0 {
  struct {
    uint32_t erase_size : 2;  /* 01: 4K; 00,10: rsvd; 11: no 4K erase. */
    uint32_t write_gran : 1;  /* Write granularity: 0: < 64bytes; 1: >= 64 */
    uint32_t wren_vsr : 1;    /* 1: WREN required to write VSR */
    uint32_t wren_vsr_op : 1; /* 0: 0x50 to write to VSR; 1: 0x06 */
    uint32_t unused_111 : 3;  /* Unused, contain 111. */
    uint32_t erase_4k_op : 8; /* Opcode to use for 4K erase. */
    uint32_t fr_1_1_2 : 1;    /* Supports 1-1-2 Fast Read. */
    uint32_t addr_bytes : 2;  /* 00: 3 only; 01: 3 or 4; 10: 4 only; 11: rsvd */
    uint32_t dtr : 1;         /* 1: Supportes Double Transfer Rate (DTR) */
    uint32_t fr_1_2_2 : 1;    /* 1: 1-2-2 Fast Read is supported. */
    uint32_t fr_1_4_4 : 1;    /* 1: 1-4-4 Fast Read is supported. */
    uint32_t fr_1_1_4 : 1;    /* 1: 1-1-4 Fast Read is supported. */
    uint32_t unused_ff : 8;   /* Unused, contains 0xff. */
  };
  struct {
    uint32_t density : 31;
    uint32_t over_2g : 1; /* 0: density is # of bits; 1: density is 2^N */
  };
  struct {
    uint32_t fr_1_4_4_nws : 5; /* Number of dummy cycles for 1-4-4 read */
    uint32_t fr_1_4_4_nmb : 3; /* Number of mode bits for 1-4-4 read */
    uint32_t fr_1_4_4_op : 8;  /* 1-4-4 read opcode */
    uint32_t fr_1_1_4_nws : 5; /* Number of dummy cycles for 1-1-4 read */
    uint32_t fr_1_1_4_nmb : 3; /* Number of mode bits for 1-1-4 read */
    uint32_t fr_1_1_4_op : 8;  /* 1-1-4 read opcode */
  };
  struct {
    uint32_t fr_1_1_2_nws : 5; /* Number of dummy cycles for 1-1-2 read */
    uint32_t fr_1_1_2_nmb : 3; /* Number of mode bits for 1-1-2 read */
    uint32_t fr_1_1_2_op : 8;  /* 1-1-2 read opcode */
    uint32_t fr_1_2_2_nws : 5; /* Number of dummy cycles for 1-2-2 read */
    uint32_t fr_1_2_2_nmb : 3; /* Number of mode bits for 1-2-2 read */
    uint32_t fr_1_2_2_op : 8;  /* 1-2-2 read opcode */
  };
  struct {
    uint32_t fr_2_2_2 : 1; /* 1: 2-2-2 Fast Read is supported */
    uint32_t rsvd_111 : 3; /* Reserved, set to 111. */
    uint32_t fr_4_4_4 : 1; /* 1: 4-4-4 Fast Read is supported */
    uint32_t rsvd_27 : 27; /* Reserved, set to all 1s. */
  };
  struct {
    uint32_t rsvd_16_1 : 16;   /* Reserved, set to all 1s. */
    uint32_t fr_2_2_2_nws : 5; /* Number of dummy cycles for 2-2-2 read */
    uint32_t fr_2_2_2_nmb : 3; /* Number of mode bits for 2-2-2 read */
    uint32_t fr_2_2_2_op : 8;  /* 2-2-2 read opcode */
  };
  struct {
    uint32_t rsvd_16_2 : 16;   /* Reserved, set to all 1s. */
    uint32_t fr_4_4_4_nws : 5; /* Number of dummy cycles for 4-4-4 read */
    uint32_t fr_4_4_4_nmb : 3; /* Number of mode bits for 4-4-4 read */
    uint32_t fr_4_4_4_op : 8;  /* 4-4-4 read opcode */
  };
  struct {
    uint32_t erase_st1_size : 8; /* Erase sector type 1 size (2^N) */
    uint32_t erase_st1_op : 8;   /* Erase sector type 1 opcode */
    uint32_t erase_st2_size : 8; /* Erase sector type 2 size (2^N) */
    uint32_t erase_st2_op : 8;   /* Erase sector type 2 opcode */
  };
  struct {
    uint32_t erase_st3_size : 8; /* Erase sector type 3 size (2^N) */
    uint32_t erase_st3_op : 8;   /* Erase sector type 3 opcode */
    uint32_t erase_st4_size : 8; /* Erase sector type 4 size (2^N) */
    uint32_t erase_st4_op : 8;   /* Erase sector type 4 opcode */
  };
};

static bool spi_flash_op(struct dev_data *dd, size_t tx_len,
                         const void *tx_data, int dummy_len, size_t rx_len,
                         void *rx_data) {
  struct mgos_spi_txn txn = {.cs = dd->cs, .mode = dd->mode, .freq = dd->freq};
  txn.hd.tx_len = tx_len;
  txn.hd.tx_data = tx_data;
  txn.hd.dummy_len = dummy_len;
  txn.hd.rx_len = rx_len;
  txn.hd.rx_data = rx_data;
  return mgos_spi_run_txn(dd->spi, false /* fd */, &txn);
}

static bool spi_flash_simple_op(struct dev_data *dd, uint8_t op, int dummy_len,
                                size_t rx_len, void *rx_data) {
  return spi_flash_op(dd, 1, &op, dummy_len, rx_len, rx_data);
}

static uint8_t spi_flash_rdsr(struct dev_data *dd) {
  uint8_t res = 0;
  spi_flash_simple_op(dd, SPI_FLASH_OP_RDSR, 0, 1, &res);
  return res;
}

static bool spi_flash_wren(struct dev_data *dd) {
  return spi_flash_simple_op(dd, SPI_FLASH_OP_WREN, 0, 0, NULL);
}

static bool spi_flash_wait_idle(struct dev_data *dd) {
  uint8_t st;
  do {
    st = spi_flash_rdsr(dd);
  } while (st & dd->wip_mask);
  /* TODO(rojer): feed WDT and timeout */
  return true;
}

/* Use SFDP to detect flash params. */
static bool mgos_vfs_dev_spi_flash_detect(struct dev_data *dd) {
  bool ret = false;
  uint32_t pt0_addr = 0;

  /* Start with conservative settings */
  dd->read_op = SPI_FLASH_OP_READ_FAST;
  dd->read_op_nwb = 1;

  uint8_t jid[3] = {0, 0, 0};
  if (!spi_flash_simple_op(dd, SPI_FLASH_OP_READ_JEDEC_ID, 0, 3, jid)) {
    LOG(LL_ERROR, ("Failed to read JEDEC ID"));
    goto out_err;
  }
  if (jid[0] == 0 || jid[0] == 0xff) {
    LOG(LL_ERROR, ("Invalid JEDEC ID"));
    goto out_err;
  }
  LOG(LL_DEBUG, ("JEDEC ID: %02x %02x %02x", jid[0], jid[1], jid[2]));

  { /* Retrieve SFDP header and parameter table pointer. */
    uint32_t tx_data = htonl(SPI_FLASH_OP_READ_SFDP << 24);
    uint32_t rx_data[4];
    if (!spi_flash_op(dd, 4, &tx_data, 1, sizeof(rx_data), rx_data)) {
      LOG(LL_DEBUG, ("Failed to read SFDP info"));
      goto out_nosfdp;
    }
    struct sfdp_header *sh = (struct sfdp_header *) rx_data;
    if (sh->magic != SFDP_MAGIC) {
      LOG(LL_DEBUG, ("Invalid SFDP magic (got 0x%08x)", sh->magic));
      goto out_nosfdp;
    }
    if (sh->major_rev != 1) {
      LOG(LL_ERROR, ("Unsupported SFDP version %d", (int) sh->major_rev));
      goto out_nosfdp;
    }
    struct sfdp_parameter_header *pt0h =
        (struct sfdp_parameter_header *) &rx_data[2];
    if (pt0h->len_dw != SFDP_PT0_LEN) {
      LOG(LL_ERROR, ("Invalid SFDP PT0 length (%d)", (int) pt0h->len_dw));
      goto out_nosfdp;
    }
    pt0_addr = pt0h->addr;
  }
  { /* Get Parameter Table 0. */
    uint32_t tx_data = htonl((SPI_FLASH_OP_READ_SFDP << 24) | pt0_addr);
    uint32_t rx_data[SFDP_PT0_LEN];
    if (!spi_flash_op(dd, 4, &tx_data, 1, sizeof(rx_data), rx_data)) {
      LOG(LL_ERROR, ("Failed to read SFDP params"));
      goto out_nosfdp;
    }
    struct sfdp_pt0 *pt0 = (struct sfdp_pt0 *) rx_data;
    if (dd->size == 0) {
      dd->size = (pt0->over_2g ? (1 << pt0->density) : (pt0->density + 1)) / 8;
    }
    LOG(LL_DEBUG,
        ("Read modes :%s%s%s%s%s%s", (pt0->fr_1_1_2 ? " 1-1-2" : ""),
         (pt0->fr_1_2_2 ? " 1-2-2" : ""), (pt0->fr_1_4_4 ? " 1-4-4" : ""),
         (pt0->fr_1_1_4 ? " 1-1-4" : ""), (pt0->fr_2_2_2 ? " 2-2-2" : ""),
         (pt0->fr_4_4_4 ? " 4-4-4" : "")));
    LOG(LL_DEBUG, ("Erase sizes: %d(0x%x) %d(0x%x) %d(0x%x) %d(0x%x) %d",
                   (int) (pt0->erase_st1_op ? 1 << pt0->erase_st1_size : 0),
                   (int) pt0->erase_st1_op,
                   (int) (pt0->erase_st2_op ? 1 << pt0->erase_st2_size : 0),
                   (int) pt0->erase_st2_op,
                   (int) (pt0->erase_st3_op ? 1 << pt0->erase_st3_size : 0),
                   (int) pt0->erase_st3_op,
                   (int) (pt0->erase_st4_op ? 1 << pt0->erase_st4_size : 0),
                   (int) pt0->erase_st4_op, pt0->write_gran));
    dd->erase_sector_op = pt0->erase_4k_op;
    /* TODO(rojer): double and quad reads, if both chip and SPI support them. */
  }

out_nosfdp:
  /* Ok, we don't have SFDP. Let's see if JEDEC ID byte 2 looks like size. */
  if (dd->size == 0) {
    if (jid[2] > 10) {
      dd->size = (1 << jid[2]);
    } else {
      LOG(LL_ERROR, ("Size not specified and could not be detected"));
      goto out_err;
    }
  }
  LOG(LL_INFO,
      ("Chip ID: %02x %02x, size: %d", jid[0], jid[1], (int) dd->size));
  ret = true;

out_err:
  return ret;
}

static bool mgos_vfs_dev_spi_flash_init2(struct mgos_vfs_dev *dev,
                                         const char *opts) {
  bool ret = false;
  unsigned int wip_mask = SPI_FLASH_DEFAULT_WIP_MASK;
  struct dev_data *dd = (struct dev_data *) calloc(1, sizeof(*dd));
  if (dd == NULL) goto out;
  dd->spi = mgos_spi_get_global();
  if (dd->spi == NULL) {
    LOG(LL_INFO, ("SPI is disabled"));
    goto out;
  }
  dd->cs = -1;
  json_scanf(opts, strlen(opts),
             "{cs: %d, freq: %d, mode: %d, size: %d, wip_mask: %u}", &dd->cs,
             &dd->freq, &dd->mode, &dd->size, &wip_mask);
  if (dd->freq <= 0) goto out;
  dd->wip_mask = wip_mask;
  if (!mgos_vfs_dev_spi_flash_detect(dd)) goto out;
  dev->dev_data = dd;
  ret = true;

out:
  if (!ret) free(dd);
  return ret;
}

static bool mgos_vfs_dev_spi_flash_read(struct mgos_vfs_dev *dev, size_t offset,
                                        size_t len, void *dst) {
  bool ret = true;
  struct dev_data *dd = (struct dev_data *) dev->dev_data;
  if (offset + len > dd->size) ret = false;
  if (ret && len > 0) {
    uint32_t tx_data = htonl((dd->read_op << 24) | offset);
    ret = ret && spi_flash_wait_idle(dd);
    ret = ret && spi_flash_op(dd, 4, &tx_data, dd->read_op_nwb, len, dst);
  }
  LOG(LL_VERBOSE_DEBUG, ("%p read %u @ 0x%x -> %d", dev, (unsigned int) len,
                         (unsigned int) offset, ret));
  return ret;
}

static bool mgos_vfs_dev_spi_flash_write(struct mgos_vfs_dev *dev,
                                         size_t offset, size_t len,
                                         const void *src) {
  bool ret = true;
  struct dev_data *dd = (struct dev_data *) dev->dev_data;
  if (offset + len > dd->size) ret = false;
  ret = ret && spi_flash_wait_idle(dd);
  const uint8_t *data = (const uint8_t *) src;
  size_t off = offset, l = len;
  while (ret && l > 0) {
    uint32_t tx_data[9] = {htonl((SPI_FLASH_OP_PROGRAM_PAGE << 24) | off)};
    size_t write_len = MIN(l, 32);
    if (off % SPI_FLASH_PAGE_SIZE != 0) {
      size_t page_start = (off & ~(SPI_FLASH_PAGE_SIZE - 1));
      size_t page_remain = (SPI_FLASH_PAGE_SIZE - (off - page_start));
      write_len = MIN(write_len, page_remain);
    }
    memcpy(tx_data + 1, data, write_len);
    ret = ret && spi_flash_wren(dd);
    ret = ret && spi_flash_op(dd, 4 + write_len, &tx_data, 0, 0, NULL);
    ret = ret && spi_flash_wait_idle(dd);
    l -= write_len;
    data += write_len;
    off += write_len;
  }
  LOG(LL_VERBOSE_DEBUG, ("%p write %u @ 0x%x -> %d", dev, (unsigned int) len,
                         (unsigned int) offset, ret));
  return ret;
}

static bool mgos_vfs_dev_spi_flash_erase(struct mgos_vfs_dev *dev,
                                         size_t offset, size_t len) {
  bool ret = true;
  struct dev_data *dd = (struct dev_data *) dev->dev_data;
  if (offset % SPI_FLASH_SECTOR_SIZE != 0 || len % SPI_FLASH_SECTOR_SIZE != 0 ||
      offset >= dd->size) {
    ret = false;
  }
  ret = ret && spi_flash_wait_idle(dd);
  while (ret && len > 0) {
    uint32_t tx_data = htonl((dd->erase_sector_op << 24) | offset);
    ret = ret && spi_flash_wren(dd);
    ret = ret && spi_flash_op(dd, 4, &tx_data, 0, 0, NULL);
    ret = ret && spi_flash_wait_idle(dd);
    len -= SPI_FLASH_SECTOR_SIZE;
    offset += SPI_FLASH_SECTOR_SIZE;
  }
  LOG(LL_VERBOSE_DEBUG, ("%p erase %u @ 0x%x -> %d", dev, (unsigned int) len,
                         (unsigned int) offset, ret));
  return ret;
}

static size_t mgos_vfs_dev_spi_flash_get_size(struct mgos_vfs_dev *dev) {
  struct dev_data *dd = (struct dev_data *) dev->dev_data;
  return dd->size;
}

static bool mgos_vfs_dev_spi_flash_close(struct mgos_vfs_dev *dev) {
  /* Nothing to do. */
  (void) dev;
  return true;
}

static const struct mgos_vfs_dev_ops mgos_vfs_dev_spi_flash_ops = {
    .init = mgos_vfs_dev_spi_flash_init2,
    .read = mgos_vfs_dev_spi_flash_read,
    .write = mgos_vfs_dev_spi_flash_write,
    .erase = mgos_vfs_dev_spi_flash_erase,
    .get_size = mgos_vfs_dev_spi_flash_get_size,
    .close = mgos_vfs_dev_spi_flash_close,
};

bool mgos_vfs_dev_spi_flash_init(void) {
  return mgos_vfs_dev_register_type(MGOS_VFS_DEV_TYPE_SPI_FLASH,
                                    &mgos_vfs_dev_spi_flash_ops);
}
