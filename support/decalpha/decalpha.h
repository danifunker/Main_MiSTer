// DEC AlphaStation 200 4/100 (DECAlphaStation200): the user_io hooks, each gated on the core.
// Slots (the core's hps_io block devices): 0 disk SCSI ID 0 (dka0), 1 disk ID 1 (dka100),
// 2 CD-ROM ID 4 (dka400). The CD layer and the write buffer follow support/hp9k and
// support/ibmrs6k; docs/design/cdrom.md in the core's repository describes the interface.

#ifndef DECALPHA_H
#define DECALPHA_H

#include <stdint.h>
#include "../../file_io.h"

char is_decalpha();

// the CD-ROM's slot (SC2, SCSI ID 4)
int decalpha_cdrom_slot();

// bit n: slot n is behind the hard-disk write buffer (SC0 and SC1, SCSI IDs 0 and 1)
#define DECALPHA_DISK_SLOTS 2
int decalpha_buffered_disks();

// Returns 0 to fail the mount; a translated CD clears *writable and sets f->size.
int decalpha_mount_hook(int index, const char *name, fileTYPE *f, int *writable);
void decalpha_unmount(int index);

void decalpha_poll();

// 0 = not ours, 1 = serviced, -1 = ours but unsupported (caller breaks the loop).
int decalpha_sd_service(int disk, fileTYPE *f, int op, uint64_t lba, int sz, int ack);

#endif
