/**
  ******************************************************************************
  * @file    sd_fatfs.h
  * @brief   Minimal FAT32 append-only writer for the PumpLogger.
  *
  *          Lives directly on top of HAL_SD (polled, no DMA, no ISR). Just
  *          enough FAT32 to:
  *            - mount a FAT32 SD card (1st partition or whole-disk)
  *            - enumerate root short-name 8.3 entries to find next free NNN
  *            - open a file for append (creates with empty body)
  *            - append blocks of bytes; auto-allocates clusters
  *            - flush directory entries so file sizes are visible to a host
  *              after an ungraceful power-off
  *
  *          NOT supported (Phase 3 doesn't need them, Phase 5 will swap in
  *          a real FatFs library):
  *            - file read / delete (BLE FileSync side)
  *            - LFN (long file names)
  *            - subdirectories beyond root
  *            - FAT12 / FAT16
  *            - cluster fragmentation reclamation
  *
  *          Per-file state is a tiny PL_File handle the caller hangs onto.
  ******************************************************************************
  */
#ifndef PL_SD_FATFS_H
#define PL_SD_FATFS_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
  PL_FX_OK = 0,
  PL_FX_ERR_INIT,
  PL_FX_ERR_NO_FAT32,
  PL_FX_ERR_IO,
  PL_FX_ERR_FULL,
  PL_FX_ERR_NOT_FOUND,
  PL_FX_ERR_INVAL
} pl_fx_status_t;

/* Opaque-ish file handle. All fields are private to sd_fatfs.c. */
typedef struct PL_File {
  uint8_t   used;              /* 0 = closed slot */
  char      name83[11];        /* 8.3 padded with spaces */
  uint32_t  first_cluster;
  uint32_t  cur_cluster;
  uint32_t  cur_offset;        /* offset within current cluster, bytes */
  uint32_t  size;              /* current file size in bytes */
  uint32_t  dir_block;         /* LBA of directory sector containing this entry */
  uint16_t  dir_entry_idx;     /* index of dir entry within that sector (0..15) */
  uint8_t   sector_buf[512];   /* current write-back data sector */
  uint8_t   sector_dirty;
  uint32_t  sector_lba;        /* LBA of buffered sector (0 = none) */
  uint32_t  rd_pos;            /* absolute read cursor (read-mode files only) */
} PL_File;

/* Mount the SD card and parse the FAT32 layout. Must be called before any
   open/append/flush. Returns PL_FX_OK or one of the error codes. */
pl_fx_status_t SDFat_Mount(void);

/* True iff Mount succeeded. */
int SDFat_IsMounted(void);

/* Choose the next free NNN s.t. SensNNN.csv / GpsNNN.csv / BatNNN.csv do not
   exist on the card. Returns 0..999, or -1 on error. */
int  SDFat_NextSessionNumber(void);

/* Open (creating if missing) `name83` for append. Returns PL_FX_OK on
   success and fills *out. `name` is a regular 8.3 string like "SENS001.CSV". */
pl_fx_status_t SDFat_OpenAppend(PL_File *out, const char *name);

/* Append `data[len]` to the end of the file. May span sectors and clusters.
   Returns PL_FX_OK or an error. */
pl_fx_status_t SDFat_Append(PL_File *file, const void *data, uint32_t len);

/* Flush any cached sector + write back the directory entry so the
   externally-visible size matches what we've appended. Cheap to call once a
   second. */
pl_fx_status_t SDFat_Flush(PL_File *file);

/* Close — flushes + marks the slot free. */
pl_fx_status_t SDFat_Close(PL_File *file);

/* Total free space approximation (bytes). 0 if not mounted. */
uint64_t SDFat_FreeBytes(void);

/* ---- BLE FileSync support (Phase 5) ------------------------------------- */

/* Root-directory enumeration. The callback gets the 8.3 name (already
   converted to a NUL-terminated "NAME.EXT" string, no padding) and the
   file size in bytes. Return non-zero from the callback to stop early. */
typedef int (*SDFat_ListCb)(const char *name, uint32_t size, void *user);
pl_fx_status_t SDFat_ListRoot(SDFat_ListCb cb, void *user);

/* Open an existing file for reading. `name` is a regular 8.3 string. Returns
   PL_FX_ERR_NOT_FOUND if it doesn't exist. */
pl_fx_status_t SDFat_OpenRead(PL_File *out, const char *name);

/* Read up to `len` bytes from the current read cursor into `buf`. *got is
   set to the number actually read (0 at EOF). */
pl_fx_status_t SDFat_Read(PL_File *file, void *buf, uint32_t len, uint32_t *got);

/* Seek the read cursor to an absolute byte offset. */
pl_fx_status_t SDFat_Seek(PL_File *file, uint32_t offset);

/* Delete a file: free its FAT cluster chain and mark the dir entry 0xE5. */
pl_fx_status_t SDFat_Delete(const char *name);

#endif /* PL_SD_FATFS_H */
