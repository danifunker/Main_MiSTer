// Sun SCSI family (SunSparcStation: the SPARCstation 5 and 20) -- glue
// between the common code and the sun support modules. All hooks self-gate
// on the family.

#ifndef SUN_H
#define SUN_H

#include <stdint.h>
#include "../../file_io.h"

// Cores sharing the Sun SCSI target layout and its HPS services.
char is_sun_scsi_family();

// hps_io slots (SunSparcStation.sv: SC0-SC2). One shared family layout.
#define SUN_DISK_SLOTS 2   // hard disks: slots 0 and 1
#define SUN_CDROM_SLOT 2

// user_io_file_mount hook: flushes the slot's write buffer, and runs CD image
// translation on the CD slot. Returns the new mount result (0 = fail the
// mount); on a translated mount clears *writable and sets f->size to the
// virtual disc size.
int sun_mount_hook(int index, const char *name, fileTYPE *f, int *writable);

// user_io_file_mount, image removed.
void sun_unmount(int index);

// Once per user_io_poll: the write buffer's idle and age flushes.
void sun_poll();

// Slot service for the sun devices, SPI transfer included.
// Returns 0 = not ours (generic path serves it), 1 = serviced, -1 = ours but
// unsupported op (caller breaks the sector-service loop).
int sun_sd_service(int disk, fileTYPE *f, int op, uint64_t lba, int sz, int ack);

#endif
