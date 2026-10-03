// Sun SCSI family glue -- the user_io hooks (see sun.h).

#include <stdint.h>
#include <string.h>
#include <strings.h>

#include "../../user_io.h"
#include "../../spi.h"
#include "../../file_io.h"
#include "sun.h"
#include "sun_disk.h"
#include "sun_cdrom.h"

static char is_core_named(const char *n)
{
	size_t len = strlen(n);
	return !strncasecmp(user_io_get_core_name(0), n, len)
	    || !strncasecmp(user_io_get_core_name(1), n, len);
}

char is_sun_scsi_family()
{
	return is_core_named("SunSparcStation");
}

// CD image translation on the CD-ROM slot: CUE/CHD/raw-2352 become a flat
// 2048-byte-sector virtual disc; a flat ISO stays on the generic path.
int sun_mount_hook(int index, const char *name, fileTYPE *f, int *writable)
{
	if (!is_sun_scsi_family()) return 1;

	sun_disk_flush(index);
	if (index != SUN_CDROM_SLOT) return 1;

	int r = sun_cdrom_mount(index, name);
	if (r == SUN_CDROM_HANDLED)
	{
		*writable = 0;
		f->size = (int64_t)sun_cdrom_size(index);   // core sees the virtual disc
	}
	else if (r == SUN_CDROM_REJECT)
	{
		FileClose(f);
		return 0;
	}
	return 1;
}

void sun_unmount(int index)
{
	if (!is_sun_scsi_family()) return;
	sun_disk_flush(index);
	sun_cdrom_unmount(index);
}

void sun_poll()
{
	sun_disk_poll();
}

int sun_sd_service(int disk, fileTYPE *f, int op, uint64_t lba, int sz, int ack)
{
	if (!op || !is_sun_scsi_family()) return 0;
	if (sun_disk_service(disk, f, op, lba, sz, ack)) return 1;

	// CD slot with translation active: read-only views of the virtual disc
	// (the core ties sd_wr off). Flat images stay on the generic path.
	if (sun_cdrom_active(disk))
	{
		static uint8_t buf[UIO_BUFFER_SIZE];
		if (!(op & 1) || sz > (int)sizeof(buf)) return -1;

		sun_cdrom_fill(disk, lba, buf, sz);
		EnableIO();
		spi_w(UIO_SECTOR_RD | ack);
		spi_block_write(buf, user_io_get_width(), sz);
		DisableIO();
		return 1;
	}

	return 0;
}
