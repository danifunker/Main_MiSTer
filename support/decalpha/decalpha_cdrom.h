// DEC AlphaStation 200 CD-ROM slot: the disc as 2048-byte sectors (disc LBA n = slot bytes 2048n),
// plus the windows the core's SCSI target reads: the track table at DECALPHA_CDROM_TOC_BLK and
// audio frame f (2352 bytes of samples, then 208 zeros) at DECALPHA_CDROM_AUDIO_BLK + 5f.

#ifndef DECALPHA_CDROM_H
#define DECALPHA_CDROM_H

#include <stdint.h>

#define DECALPHA_CDROM_AUDIO_BLK 0x80000000ULL
#define DECALPHA_CDROM_TOC_BLK   0xC0000000ULL

enum
{
	DECALPHA_CDROM_PASSTHRU = 0, // flat image: the generic path serves it
	DECALPHA_CDROM_HANDLED  = 1, // decalpha_cdrom_fill serves it
	DECALPHA_CDROM_REJECT   = 2, // fail the mount
};

int decalpha_cdrom_mount(int index, const char *name);
uint64_t decalpha_cdrom_size(int index);
int decalpha_cdrom_active(int index);

// lba in 512-byte units; sz a multiple of 512, up to UIO_BUFFER_SIZE.
void decalpha_cdrom_fill(int index, uint64_t lba, uint8_t *buf, int sz);
void decalpha_cdrom_unmount(int index);

// 1 (buf filled) when lba is in the TOC or audio window, else 0.
int decalpha_cdrom_window(int index, uint64_t lba, uint8_t *buf, int sz);

#endif
