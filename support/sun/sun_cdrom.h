// Sun CD-ROM slot: serves CUE/CHD/raw-sector images as a flat disc of
// 2048-byte sectors; flat 2048 images pass through the generic sd path.
// Data only: the core has no CD-DA path, so there is no TOC or audio window.

#ifndef SUN_CDROM_H
#define SUN_CDROM_H

#include <stdint.h>

enum
{
	SUN_CDROM_PASSTHRU = 0, // flat 2048-byte image: use the generic path
	SUN_CDROM_HANDLED  = 1, // translation active: serve via sun_cdrom_fill
	SUN_CDROM_REJECT   = 2, // unusable image (bad cue/chd): fail the mount
};

int sun_cdrom_mount(int index, const char *name);
uint64_t sun_cdrom_size(int index);

// True when translation is live on this index (sun_cdrom_fill serves it).
int sun_cdrom_active(int index);

// lba in 512-byte units of the virtual disc; sz a multiple of 512, up to
// UIO_BUFFER_SIZE (the core asks for up to 32 blocks at a time).
void sun_cdrom_fill(int index, uint64_t lba, uint8_t *buf, int sz);
void sun_cdrom_unmount(int index);

#endif
